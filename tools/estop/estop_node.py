#!/usr/bin/env python3
"""E-stop listener: ESP32 on a dedicated serial port -> std_msgs/Bool `estop` (True = stopped) at 10 Hz.

Runs as its own process (systemd unit robocup-estop), outside the launch: stopping or killing the stack never kills it.
If this process dies the mux hears silence and stays stopped (fail-closed). Serial lost, bad lines or
silence > `silence_sec` all read as pressed. After the button has been released for `release_hold_sec` it
calls the mux's `safety_reset` (restores permission only: buffered commands are dropped, the executor stays halted).

    estop_node.py --port /dev/serial/by-id/usb-...   (see tools/estop/README in the unit file)
"""
import argparse
import time

import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


def mux_wants_reset(text):
    """True when the mux says it is stopped for the e-stop ('latched: e-stop: no signal ...' or '... pressed'): the button node must then reset it again once the button reads released."""
    return text.startswith('latched') and 'e-stop' in text


class EstopState:
    """Pure logic. feed() each serial line, tick() each cycle -> (stopped, want_reset)."""

    def __init__(self, silence_sec=0.5, release_hold_sec=1.0):
        self.silence, self.hold = silence_sec, release_hold_sec
        self.last_ok = self.armed_since = None
        self.need_reset = True            # at start the mux is latched (no signal yet)

    def feed(self, line, now):
        if line == 'E0':
            self.last_ok = now
            self.armed_since = self.armed_since if self.armed_since is not None else now
        elif line == 'E1':
            self.last_ok = now
            self.armed_since, self.need_reset = None, True
        # anything else: ignored, so garbage never counts as "released"

    def tick(self, now):
        if self.last_ok is None or now - self.last_ok > self.silence:
            self.armed_since, self.need_reset = None, True
        stopped = self.armed_since is None or now - self.armed_since < self.hold
        return stopped, (not stopped and self.need_reset)


class EstopNode(Node):
    def __init__(self, port):
        super().__init__('estop_node')
        self.port, self.ser, self.buf, self.retry = port, None, b'', 0.0
        self.state = EstopState()
        self.pub = self.create_publisher(Bool, 'estop', 10)
        self.reset = self.create_client(Trigger, 'safety_reset')
        self.pending = None
        # the mux latches 'e-stop: no signal' when it starts before this node has spoken, and keeps the latch after we have already reset an earlier one:
        # hear its (latched) state and ask for a reset again whenever it is stopped for the e-stop and the button reads released
        self.create_subscription(String, 'safety/stop_state', self._on_mux, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(0.05, self._tick)
        self.create_timer(0.1, self._publish)
        self.stopped = True

    def _read(self, now):
        if self.ser is None:
            if now < self.retry:
                return
            self.retry = now + 1.0
            try:
                ser = serial.Serial(None, 115200, timeout=0)
                ser.dtr = ser.rts = False        # on this board RTS is wired to the ESP32 EN pin: asserted = held in reset = silent = e-stop reads pressed
                ser.port = self.port
                ser.open()
                self.ser = ser
                self.get_logger().info(f'opened {self.port}')
            except (serial.SerialException, OSError):
                return
        try:
            self.buf += self.ser.read(256)
        except (serial.SerialException, OSError):
            self.get_logger().error('serial lost: e-stop reads as pressed')
            try:
                self.ser.close()
            except Exception:  # noqa: BLE001
                pass
            self.ser, self.buf = None, b''
            return
        *lines, self.buf = self.buf.split(b'\n')
        self.buf = self.buf[-16:]
        for ln in lines:
            self.state.feed(ln.strip().decode(errors='replace'), now)

    def _on_mux(self, m):
        if mux_wants_reset(m.data):
            self.state.need_reset = True

    def _tick(self):
        now = time.monotonic()
        self._read(now)
        was = self.stopped
        self.stopped, want = self.state.tick(now)
        if self.stopped != was:
            self.get_logger().warn('E-STOP PRESSED' if self.stopped else 'e-stop released: waiting for the mux to see it')
        if want and (self.pending is None or self.pending.done()) and self.reset.service_is_ready():
            self.pending = self.reset.call_async(Trigger.Request())
            self.pending.add_done_callback(self._reset_done)

    def _reset_done(self, fut):
        r = fut.result()
        if r is not None and r.success:
            self.state.need_reset = False
            self.get_logger().info(f'safety_reset: {r.message}')
        else:
            self.get_logger().warn(f'safety_reset refused, will retry: {None if r is None else r.message}')

    def _publish(self):
        self.pub.publish(Bool(data=self.stopped))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', required=True, help='use /dev/serial/by-id/..., not ttyUSB0 (numbers move)')
    args, ros = ap.parse_known_args()
    rclpy.init(args=ros)
    try:
        while rclpy.ok():           # never stays down: any crash restarts the watcher (the mux stops the robot meanwhile, it hears no signal)
            node = EstopNode(args.port)
            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                break
            except Exception as e:  # noqa: BLE001
                print(f'estop_node crashed, restarting: {e!r}', flush=True)
                time.sleep(0.5)
            finally:
                node.destroy_node()
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
