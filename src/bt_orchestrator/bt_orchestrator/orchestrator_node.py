import threading

import rclpy
from bt_pipeline_msgs.msg import BtPlan
from bt_pipeline_msgs.srv import GenerateXml, ValidateTree
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


class OrchestratorNode(Node):
    """bt/task_spec (JSON from llm_bridge) -> SLM -> validator, retrying with errors.

    Only a validated tree is published with valid=true; after max_attempts the last
    failing attempt is published with valid=false so consumers can refuse it.
    """

    def __init__(self):
        super().__init__('orchestrator_node')
        self.declare_parameter('max_attempts', 3)
        self.declare_parameter('call_timeout_sec', 60.0)
        cb = ReentrantCallbackGroup()
        self.slm = self.create_client(GenerateXml, 'bt_slm/generate', callback_group=cb)
        self.val = self.create_client(ValidateTree, 'bt_validator/validate',
                                      callback_group=cb)
        self.plan_pub = self.create_publisher(BtPlan, 'bt/plan', 10)
        self.create_subscription(String, 'bt/task_spec', self._on_spec, 10,
                                 callback_group=cb)
        self.lock = threading.Lock()   # one plan at a time

    def _call(self, client, req):
        done = threading.Event()
        fut = client.call_async(req)
        fut.add_done_callback(lambda _: done.set())
        if not done.wait(self.get_parameter('call_timeout_sec').value):
            fut.cancel()
            raise TimeoutError(f'{client.srv_name} timed out')
        return fut.result()

    def _on_spec(self, msg):
        with self.lock:
            plan = BtPlan(task_spec=msg.data, valid=False)
            plan.header.stamp = self.get_clock().now().to_msg()
            try:
                self._run(plan)
            except Exception as e:  # noqa: BLE001
                plan.errors = [f'pipeline error: {type(e).__name__}: {e}']
                self.get_logger().error(plan.errors[0])
            self.plan_pub.publish(plan)
            self.get_logger().info(
                f'plan valid={plan.valid} attempts={plan.attempts} errors={len(plan.errors)}')

    def _run(self, plan):
        for attempt in range(1, self.get_parameter('max_attempts').value + 1):
            plan.attempts = attempt
            g = self._call(self.slm, GenerateXml.Request(
                task_spec=plan.task_spec, previous_xml=plan.xml,
                previous_errors=list(plan.errors)))
            if not g.ok:
                plan.errors = [f'SLM failed: {g.error}']
                continue
            plan.xml = g.xml
            v = self._call(self.val, ValidateTree.Request(xml=plan.xml))
            plan.errors = list(v.errors)
            if v.valid:
                plan.valid = True
                return


def main(args=None):
    rclpy.init(args=args)
    node = OrchestratorNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
