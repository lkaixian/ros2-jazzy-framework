import rclpy
from bt_pipeline_msgs.msg import BtPlan
from rclpy.node import Node
from std_msgs.msg import String


def describe(plan, speak_errors=True, max_errors=2):
    if plan.valid:
        return 'Plan ready.'
    msg = 'I could not make a valid plan.'
    if speak_errors and plan.errors:
        msg += ' ' + ' '.join(plan.errors[:max_errors])
    return msg


class PlanAnnouncer(Node):
    """bt/plan -> tts/say, so the operator hears whether the task was accepted."""

    def __init__(self):
        super().__init__('plan_announcer_node')
        self.declare_parameter('speak_errors', True)
        self.declare_parameter('max_errors_spoken', 2)
        self.pub = self.create_publisher(String, 'tts/say', 10)
        self.create_subscription(BtPlan, 'bt/plan', self._on_plan, 10)

    def _on_plan(self, plan):
        self.pub.publish(String(data=describe(
            plan, self.get_parameter('speak_errors').value,
            self.get_parameter('max_errors_spoken').value)))


def main(args=None):
    rclpy.init(args=args)
    node = PlanAnnouncer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
