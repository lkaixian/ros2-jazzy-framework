"""Base class for extensions that actually execute BT nodes (action server per extension)."""
import math
import threading
import time

import rclpy
from bt_common.extension_node import ExtensionNode
from bt_pipeline_msgs.action import ExecuteNode
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class PortError(ValueError):
    pass


def port_float(ports, key, default=None, lo=None, hi=None):
    v = ports.get(key)
    if v is None or v == '':
        if default is None:
            raise PortError(f"missing port '{key}'")
        return default
    try:
        f = float(v)
    except ValueError:
        raise PortError(f"port '{key}' must be a number, got {v!r}") from None
    if not math.isfinite(f):
        raise PortError(f"port '{key}' must be finite")
    if (lo is not None and f < lo) or (hi is not None and f > hi):
        raise PortError(f"port '{key}'={f} outside [{lo}, {hi}]")
    return f


def port_str(ports, key, default=None):
    v = ports.get(key)
    if v is None or v == '':
        if default is None:
            raise PortError(f"missing port '{key}'")
        return default
    return v


def port_bool(ports, key, default=False):
    v = ports.get(key)
    if v is None or v == '':
        return default
    if v.lower() in ('true', '1', 'yes'):
        return True
    if v.lower() in ('false', '0', 'no'):
        return False
    raise PortError(f"port '{key}' must be true/false, got {v!r}")


class Ctl:
    """Handed to handlers: cancellation, feedback and cancel-aware sleeping."""

    def __init__(self, goal_handle):
        self.gh = goal_handle

    @property
    def cancelled(self):
        return self.gh.is_cancel_requested

    def feedback(self, text):
        self.gh.publish_feedback(ExecuteNode.Feedback(status=text))

    def sleep(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            if self.cancelled:
                return False
            time.sleep(min(0.01, max(0.0, end - time.monotonic())))
        return not self.cancelled


def call_action(client, goal, ctl, timeout_sec, server_wait=2.0):
    """Send a goal and wait for its result, honouring ctl cancellation and a timeout.

    Returns (result_or_None, error_string).
    """
    if not client.wait_for_server(timeout_sec=server_wait):
        return None, 'action server unavailable'
    accepted = threading.Event()
    box = {}

    def on_goal(fut):
        box['gh'] = fut.result()
        accepted.set()
    client.send_goal_async(goal).add_done_callback(on_goal)
    if not accepted.wait(5.0) or not box['gh'].accepted:
        return None, 'goal rejected or no response'
    gh = box['gh']
    finished = threading.Event()
    gh.get_result_async().add_done_callback(lambda f: (box.__setitem__('res', f.result()),
                                                       finished.set()))
    end = time.monotonic() + timeout_sec
    while not finished.wait(0.02):
        if ctl.cancelled or time.monotonic() > end:
            gh.cancel_goal_async()
            return None, 'cancelled' if ctl.cancelled else 'timeout'
    return box['res'].result, ''


def call_service(client, request, ctl, timeout_sec, wait_sec=2.0):
    """Call a service from a handler, honouring cancellation. Returns (response, error)."""
    if not client.wait_for_service(timeout_sec=wait_sec):
        return None, f'service {client.srv_name} unavailable'
    done, box = threading.Event(), {}

    def on_done(fut):
        try:
            box['r'] = fut.result()
        except Exception as e:  # noqa: BLE001
            box['e'] = f'{type(e).__name__}: {e}'
        done.set()
    client.call_async(request).add_done_callback(on_done)
    end = time.monotonic() + timeout_sec
    while not done.wait(0.02):
        if ctl.cancelled:
            return None, 'cancelled'
        if time.monotonic() > end:
            return None, 'timeout'
    return box.get('r'), box.get('e', '')


def unpack(r):
    """(ok, msg) or (ok, msg, {out_name: value}) -> (ok, msg, outputs); values become strings."""
    ok, msg, *rest = r
    return ok, msg, {k: str(v) for k, v in (rest[0] if rest else {}).items()}


class ExtensionExecutor(ExtensionNode):
    """Subclasses fill self.handlers: {node_id: fn(ports: dict, ctl: Ctl) -> (ok, message[, outputs])}.
    outputs = {"out_x": value} is written to the BT blackboard when the node's XML names that out_ port.

    Action-category nodes are exclusive (a second goal is rejected while one runs);
    Condition nodes are always allowed so they can be polled during an action.
    """

    def __init__(self, package):
        super().__init__(package)
        self.handlers = {}
        self._cat = {n.id: n.category for n in self.manifest.nodes}
        self._lock = threading.Lock()
        self._action_busy = False
        self._server = ActionServer(
            self, ExecuteNode, f'{package}/execute', execute_callback=self._execute,
            goal_callback=self._on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())

    def _on_goal(self, req):
        if req.node_id not in self.handlers:
            self.get_logger().warn(f'unknown node {req.node_id!r}: rejected')
            return GoalResponse.REJECT
        if self._cat.get(req.node_id) == 'Action':
            with self._lock:
                if self._action_busy:
                    self.get_logger().warn(f'{req.node_id}: another action is running; rejected')
                    return GoalResponse.REJECT
                self._action_busy = True
        return GoalResponse.ACCEPT

    def _execute(self, gh):
        req = gh.request
        ports = dict(zip(req.port_names, req.port_values))
        result = ExecuteNode.Result()
        outs = {}
        try:
            ok, msg, outs = unpack(self.handlers[req.node_id](ports, Ctl(gh)))
        except PortError as e:
            ok, msg = False, f'bad port: {e}'
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'{req.node_id} failed: {type(e).__name__}: {e}')
            ok, msg = False, f'{type(e).__name__}: {e}'
        finally:
            if self._cat.get(req.node_id) == 'Action':
                with self._lock:
                    self._action_busy = False
        result.success, result.message = bool(ok), msg
        result.output_names, result.output_values = list(outs), list(outs.values())
        if gh.is_cancel_requested:
            result.success, result.message = False, 'cancelled'
            gh.canceled()
        elif ok:
            gh.succeed()
        else:
            gh.abort()
        return result


def run_executor(node_cls, args=None):
    rclpy.init(args=args)
    node = node_cls()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
