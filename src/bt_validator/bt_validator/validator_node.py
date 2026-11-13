import rclpy
from bt_common.catalog import Catalog
from bt_common.extension_node import MANIFEST_QOS
from bt_pipeline_msgs.msg import ExtensionManifest
from bt_pipeline_msgs.srv import ValidateTree
from rclpy.node import Node

from bt_validator.validate import validate_xml


class ValidatorNode(Node):
    def __init__(self):
        super().__init__('bt_validator_node')
        self.declare_parameter('max_depth', 12)
        self.declare_parameter('max_nodes', 64)
        self.catalog = Catalog()
        self.create_subscription(ExtensionManifest, 'extensions/manifest',
                                 self._on_manifest, MANIFEST_QOS)
        self.create_service(ValidateTree, 'bt_validator/validate', self._on_validate)

    def _on_manifest(self, msg):
        self.catalog.add_manifest_msg(msg)
        self.get_logger().info(f'extension registered: {msg.extension}')

    def _on_validate(self, req, res):
        res.errors = validate_xml(req.xml, self.catalog,
                                  self.get_parameter('max_depth').value,
                                  self.get_parameter('max_nodes').value)
        res.valid = not res.errors
        return res


def main(args=None):
    rclpy.init(args=args)
    node = ValidatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
