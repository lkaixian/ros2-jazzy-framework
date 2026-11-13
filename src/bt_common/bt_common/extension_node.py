"""Base for *_extension packages: advertises BT nodes via a latched manifest."""
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from bt_common.catalog import _from_dict
from bt_pipeline_msgs.msg import BtNodeSpec, ExtensionManifest
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

MANIFEST_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)


class ExtensionNode(Node):
    def __init__(self, package):
        super().__init__(package + '_node')
        default = os.path.join(get_package_share_directory(package), 'config',
                               package + '.yaml')
        self.declare_parameter('manifest_file', default)
        with open(self.get_parameter('manifest_file').value) as f:
            data = yaml.safe_load(f)
        _from_dict(data)      # fail fast on port names BehaviorTree.CPP would reject
        msg = ExtensionManifest(extension=data['extension'])
        for nid, s in data['nodes'].items():
            msg.nodes.append(BtNodeSpec(
                id=nid, category=s['category'], required_ports=list(s.get('required', [])),
                optional_ports=list(s.get('optional', [])),
                description=s.get('description', '')))
        self._manifest_pub = self.create_publisher(ExtensionManifest, 'extensions/manifest',
                                         MANIFEST_QOS)
        self.manifest = msg
        self.declare_parameter('republish_sec', 3.0)
        self._manifest_pub.publish(msg)
        # Latched delivery alone can lose the race with DDS discovery when many processes
        # start together; consumers register idempotently, so periodic re-publish is safe.
        self.create_timer(self.get_parameter('republish_sec').value,
                          lambda: self._manifest_pub.publish(self.manifest))
        self.get_logger().info(f'{msg.extension}: advertising {len(msg.nodes)} BT nodes')


def run_extension(package, args=None):
    rclpy.init(args=args)
    node = ExtensionNode(package)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
