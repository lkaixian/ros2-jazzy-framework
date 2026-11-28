"""Priority mux + watchdog + clamp in front of the robot's cmd_vel.

Inputs are listed highest priority first. The first input that is fresh wins. When every
input goes stale the mux publishes a zero Twist once, then stays silent.

Bumper/cliff events latch a stop: while latched the mux publishes zero Twist at the loop rate
and ignores every input. The latch clears only through the `safety_reset` service, which refuses
while a sensor is still active, and it discards all buffered commands so a reset never replays
an old command (it restores permission, not motion). This is a software gate: it is not a
hardware e-stop and is unverified on a real Kobuki.

`estop_topic` (std_msgs/Bool, True = button pressed or circuit open) latches the same stop. It is
fail-closed: no message for `estop_timeout_sec` (reader dead, cable cut) counts as pressed. The
executor halt and the arm hold follow from the `safety/stop_state` latch (robot_mode -> halt).
"""
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


def clamp(x, lim):
    return max(-lim, min(lim, x))


def slew(cur, target, step):
    """Move cur toward target by at most step."""
    return cur + max(-step, min(step, target - cur))


def select(last, now, timeout):
    """last: list of (stamp or None, Twist) in priority order -> chosen index or None."""
    for i, (stamp, _) in enumerate(last):
        if stamp is not None and now - stamp <= timeout:
            return i
    return None


def estop_hit(pressed, stamp, now, timeout):
    """-> (hit, reason). Pressed, never heard, or silent for `timeout` seconds all count as a stop."""
    if stamp is None or now - stamp > timeout:
        return True, 'e-stop: no signal from the button reader'
    return bool(pressed), 'e-stop pressed'


class StopLatch:
    """Latching stop. Active sensors are tracked so a reset is refused while any is still hit."""

    def __init__(self):
        self.latched = False
        self.reason = ''
        self.active = set()

    def event(self, key, hit, reason=''):
        """key: hashable sensor id, hit: True when pressed / at a cliff. Returns True on a new latch."""
        if hit:
            self.active.add(key)
            new = not self.latched
            self.latched = True
            if new or not self.reason:
                self.reason = reason or str(key)
            return new
        self.active.discard(key)
        return False

    def reset(self):
        """-> (ok, message). A hit sensor or an unavailable safety input blocks the reset."""
        if not self.latched:
            return True, 'not latched'
        if self.active:
            return False, f'still active: {sorted(map(str, self.active))}'
        self.latched = False
        self.reason = ''
        return True, 'reset'


LEFT, CENTER, RIGHT = 0, 1, 2     # BumperEvent.bumper


class BumpRecovery:
    """After a bumper latch: wait `settle` s for the other bumpers, back off, turn away.

    One side only -> obstacle on that side, turn the other way a little. Centre, or more than one
    bumper -> a wall ahead, turn a quarter circle. Open loop and timed (no odometry here), so the
    distances are nominal. The latch stays set when it ends: a human or `safety_reset` decides.
    """

    def __init__(self, back_speed=0.1, back_dist=0.1, turn_rate=0.5, settle=0.3, log=lambda s: None):
        self.back_speed, self.back_dist, self.turn_rate, self.settle = (
            back_speed, back_dist, turn_rate, settle)
        self.log = log
        self.phase = None             # None | 'settle' | 'run'
        self.ids = set()

    def begin(self, now, bumper):
        self.phase, self.t0, self.ids = 'settle', now, {bumper}

    def cancel(self):
        self.phase = None

    def hit(self, bumper):
        """A press while settling joins the verdict; any later press (or a cliff) ends recovery."""
        if self.phase == 'settle':
            self.ids.add(bumper)
        else:
            self.cancel()

    def _plan(self):
        if self.ids == {LEFT}:
            name, sign, angle = 'left', -1.0, 0.5
        elif self.ids == {RIGHT}:
            name, sign, angle = 'right', 1.0, 0.5
        else:
            name, sign, angle = 'wall', 1.0, 1.57     # shortcut: always turns left, pick by scan if needed
        self.t_back = self.back_dist / self.back_speed
        self.t_turn = angle / self.turn_rate
        self.wz = sign * self.turn_rate
        self.phase = 'run'
        self.log(f'bump recovery: {name} -> back {self.back_dist} m, turn {sign * angle:+.2f} rad')

    def cmd(self, now):
        """-> (linear.x, angular.z), or None when not running."""
        if self.phase is None:
            return None
        t = now - self.t0
        if t < self.settle:
            return 0.0, 0.0
        if self.phase == 'settle':
            self._plan()
        t -= self.settle
        if t < self.t_back:
            return -self.back_speed, 0.0
        if t < self.t_back + self.t_turn:
            return 0.0, self.wz
        self.phase = None
        return None


class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux_node')
        self.declare_parameter('inputs', ['odom_extension/cmd_vel', 'pid_extension/cmd_vel',
                                          'nav2/cmd_vel'])
        self.declare_parameter('output', 'cmd_vel')
        self.declare_parameter('stamped', False)
        self.declare_parameter('timeout_sec', 0.5)
        self.declare_parameter('max_linear', 0.3)
        self.declare_parameter('max_angular', 0.8)
        self.declare_parameter('rate_hz', 20.0)
        # Ramp for commanded changes (a step 0 -> 0.2 m/s rocks the robot and smears the map).
        # Watchdog and latch stops are never ramped. 0 = off.
        self.declare_parameter('accel_linear', 0.5)     # m/s^2
        self.declare_parameter('accel_angular', 1.5)    # rad/s^2
        # Kobuki bumper/cliff latch. Empty topic = that sensor is not watched.
        self.declare_parameter('safety_events_enabled', True)
        self.declare_parameter('bumper_topic', 'events/bumper')
        self.declare_parameter('bumper_type', 'kobuki_ros_interfaces/msg/BumperEvent')
        self.declare_parameter('cliff_topic', 'events/cliff')
        self.declare_parameter('cliff_type', 'kobuki_ros_interfaces/msg/CliffEvent')
        # Bumper-only latch: back off and turn away by itself. Off until tried on the real robot.
        self.declare_parameter('bump_recovery_enabled', False)
        self.declare_parameter('bump_back_dist', 0.10)
        # E-stop button as a software stop (a hardware cut is still needed). Empty topic = not watched.
        self.declare_parameter('estop_topic', '')
        self.declare_parameter('estop_timeout_sec', 1.0)
        self.declare_parameter('estop_sound_topic', 'commands/sound')   # Kobuki beeps (ERROR sound) when the button is pressed; '' = silent
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        self.stamped = p('stamped')
        self.last = []
        for i, topic in enumerate(p('inputs')):
            self.last.append((None, Twist()))
            self.create_subscription(Twist, topic, lambda m, i=i: self._on(i, m), 10)
        self.pub = self.create_publisher(TwistStamped if self.stamped else Twist,
                                         p('output'), 10)
        self.active = None
        self.cur = Twist()              # what was last published, the start of the next ramp step
        self.stop = StopLatch()
        self.recovery = BumpRecovery(back_dist=p('bump_back_dist'), log=self.get_logger().info)
        self.recovery_enabled = p('bump_recovery_enabled')
        self.state_pub = self.create_publisher(String, 'safety/stop_state', QoSProfile(      # latched: a reader that starts later (the e-stop node) still learns the mux is stopped
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger, 'safety_reset', self._reset)
        if p('safety_events_enabled'):
            # hit-state field and value that means "stop" for each Kobuki event message
            self._watch('bumper', p('bumper_topic'), p('bumper_type'), 'bumper', 'state')
            self._watch('cliff', p('cliff_topic'), p('cliff_type'), 'sensor', 'state')
        else:
            self.get_logger().warn('safety_events_enabled=false: bumper/cliff do NOT stop the robot')
        self.estop_t, self.estop_pressed, self.beep = None, True, None
        if p('estop_topic'):
            self.create_subscription(Bool, p('estop_topic'), self._on_estop, 10)
            self.beep = self._sound_publisher(p('estop_sound_topic'))
        else:
            self.get_logger().warn('estop_topic empty: no e-stop button is watched')
        self.create_timer(1.0 / p('rate_hz'), self._tick)

    def _sound_publisher(self, topic):
        if not topic:
            return None
        try:
            return self.create_publisher(get_message('kobuki_ros_interfaces/msg/Sound'), topic, 10)
        except (ImportError, AttributeError, ValueError) as e:
            self.get_logger().warn(f'no e-stop beep: cannot load kobuki_ros_interfaces/msg/Sound ({e})')
            return None

    def _on_estop(self, m):
        self.estop_t, self.estop_pressed = time.monotonic(), m.data
        self._estop_check()

    def _estop_check(self):
        if not self.get_parameter('estop_topic').value:
            return
        hit, why = estop_hit(self.estop_pressed, self.estop_t, time.monotonic(),
                             self.get_parameter('estop_timeout_sec').value)
        was = 'estop' in self.stop.active
        self.stop.event('estop', hit, why)
        if hit and not was:
            self.recovery.cancel()                  # no blind manoeuvre after an e-stop
            self._drop_commands()
            self.get_logger().error(f'STOP latched: {why}')
            if self.beep and why == 'e-stop pressed':          # not for a lost signal: that would beep at every start
                self.beep.publish(self.beep.msg_type(value=4))   # 4 = Sound.ERROR
            self._publish_state()

    def _watch(self, kind, topic, type_name, id_field, state_field):
        if not topic:
            return
        try:
            msg_type = get_message(type_name)
        except (ImportError, AttributeError, ValueError) as e:
            # fail closed: a safety input we cannot read keeps the robot stopped
            self.get_logger().error(f'{kind}: cannot load {type_name} ({e}); staying stopped')
            self.stop.event(f'{kind}-unavailable', True, f'{kind} message type unavailable')
            return
        self.create_subscription(
            msg_type, topic,
            lambda m, k=kind, i=id_field, st=state_field: self._on_event(
                k, getattr(m, i), getattr(m, st) == 1), 10)

    def _on_event(self, kind, ident, hit):
        new = self.stop.event((kind, ident), hit, f'{kind} {ident}')
        if hit and kind != 'bumper':
            self.recovery.cancel()                  # a cliff never gets a blind manoeuvre
        elif hit and not new:
            self.recovery.hit(ident)
        if new and hit:
            self.get_logger().error(f'STOP latched: {kind} {ident}')
            self._drop_commands()
            if kind == 'bumper' and self.recovery_enabled:
                self.recovery.begin(time.monotonic(), ident)
        self._publish_state()

    def _drop_commands(self):
        self.last = [(None, Twist()) for _ in self.last]
        self.active = None

    def _reset(self, _req, resp):
        resp.success, resp.message = self.stop.reset()
        if resp.success:
            self.recovery.cancel()
            self._drop_commands()
            self.get_logger().warn('stop latch reset: waiting for fresh commands')
        else:
            self.get_logger().warn(f'reset refused: {resp.message}')
        self._publish_state()
        return resp

    def _publish_state(self):
        m = String()
        m.data = f'latched: {self.stop.reason}' if self.stop.latched else 'armed'
        self.state_pub.publish(m)

    def _on(self, i, msg):
        self.last[i] = (time.monotonic(), msg)

    def _out(self, tw):
        if self.stamped:
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.twist = tw
            self.pub.publish(m)
        else:
            self.pub.publish(tw)

    def _stop(self):
        self.cur = Twist()
        self._out(self.cur)

    def _drive(self, tgt):
        """Publish tgt, ramped from the last output by accel_* (0 = no ramp)."""
        dt = 1.0 / self.get_parameter('rate_hz').value
        al = self.get_parameter('accel_linear').value * dt
        aa = self.get_parameter('accel_angular').value * dt
        out = Twist()
        out.linear.x = slew(self.cur.linear.x, tgt.linear.x, al) if al else tgt.linear.x
        out.linear.y = slew(self.cur.linear.y, tgt.linear.y, al) if al else tgt.linear.y
        out.angular.z = slew(self.cur.angular.z, tgt.angular.z, aa) if aa else tgt.angular.z
        self.cur = out
        self._out(out)

    def _tick(self):
        self._estop_check()
        if self.stop.latched:
            move = self.recovery.cmd(time.monotonic())
            if move:
                tgt = Twist()
                tgt.linear.x, tgt.angular.z = move
                self._drive(tgt)
            else:
                self._stop()
            self.active = None
            return
        idx = select(self.last, time.monotonic(), self.get_parameter('timeout_sec').value)
        if idx is None:
            if self.active is not None:     # fresh -> stale: stop once
                self._stop()
                self.active = None
            return
        if idx != self.active:
            self.get_logger().info(f'cmd_vel source: input {idx}')
        self.active = idx
        src = self.last[idx][1]
        tgt = Twist()
        ml, ma = self.get_parameter('max_linear').value, self.get_parameter('max_angular').value
        tgt.linear.x = clamp(src.linear.x, ml)
        tgt.linear.y = clamp(src.linear.y, ml)
        tgt.angular.z = clamp(src.angular.z, ma)
        self._drive(tgt)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
