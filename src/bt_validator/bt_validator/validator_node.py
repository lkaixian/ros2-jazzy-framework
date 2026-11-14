import json

import rclpy
from bt_common.catalog import Catalog, format_listing
from bt_common.macros import (callable_macros, describe, load_macros, load_whitelist, MacroError,
                              spec_schema)
from bt_common.extension_node import MANIFEST_QOS
from bt_pipeline_msgs.msg import ExtensionManifest
from bt_pipeline_msgs.srv import GetCatalog, ValidateTree
from rclpy.node import Node
from std_msgs.msg import String

from bt_validator.validate import validate_xml

LABEL_KEY = ('DetectObject', 'label')


class ValidatorNode(Node):
    """Validates BT XML and is the single source of truth for what the planners may use:
    core + extension nodes (bt_validator/catalog) and the detector's object labels."""

    def __init__(self):
        super().__init__('bt_validator_node')
        self.declare_parameter('max_depth', 12)
        self.declare_parameter('max_nodes', 64)
        self.declare_parameter('macros_file', '')     # '' = framework_bringup/param/macros.yaml
        self.declare_parameter('labels', '')       # comma-separated; used until a detector announces
        self.catalog = Catalog()
        try:
            path = self.get_parameter('macros_file').value or None
            self.macros = callable_macros(load_macros(path), load_whitelist(path))   # LLM-visible macros only
        except (MacroError, OSError) as e:
            self.get_logger().error(f'macros not loaded: {e}')
            self.macros = {}
        self.vocab = {}
        labels = [s.strip() for s in self.get_parameter('labels').value.split(',') if s.strip()]
        if labels:
            self.vocab[LABEL_KEY] = set(labels)
        self.create_subscription(ExtensionManifest, 'extensions/manifest',
                                 self._on_manifest, MANIFEST_QOS)
        self.create_subscription(String, 'object_detection/classes', self._on_classes,
                                 MANIFEST_QOS)
        self.create_service(ValidateTree, 'bt_validator/validate', self._on_validate)
        self.create_service(GetCatalog, 'bt_validator/catalog', self._on_catalog)

    def _on_manifest(self, msg):
        self.catalog.add_manifest_msg(msg)
        self.get_logger().info(f'extension registered: {msg.extension}',
                               throttle_duration_sec=30.0)

    def _on_classes(self, msg):
        try:
            classes = [str(c) for c in json.loads(msg.data)['classes']]
        except (ValueError, KeyError, TypeError) as e:
            self.get_logger().error(f'bad object_detection/classes message: {e}')
            return
        if set(classes) != self.vocab.get(LABEL_KEY):
            self.get_logger().info(f'detector classes: {len(classes)} labels')
        self.vocab[LABEL_KEY] = set(classes)

    def _on_validate(self, req, res):
        res.errors = validate_xml(req.xml, self.catalog,
                                  self.get_parameter('max_depth').value,
                                  self.get_parameter('max_nodes').value, self.vocab)
        res.valid = not res.errors
        return res

    def _on_catalog(self, _req, res):
        res.listing = format_listing(self.catalog.all().values(), with_desc=True)
        if self.macros:
            res.listing += ('\n\nMacros: a step {"macro": "<name>", "args": {...}} expands to several '
                            'nodes - prefer a macro when one fits (shorter, faster):\n' +
                            describe(self.macros))
        res.schema = json.dumps(spec_schema(list(self.catalog.all().values()), self.macros))
        res.labels = sorted(self.vocab.get(LABEL_KEY, ()))
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
