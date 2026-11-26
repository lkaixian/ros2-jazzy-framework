import json
import os

import yaml

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
        self.declare_parameter('places_file', '')      # '' = framework_bringup/param/places.yaml; its names constrain the LLM's place slots
        self.declare_parameter('extensions_file', '')  # '' = param/extensions.yaml: the arm places taught there (arm_extension_node.places) constrain put_place
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

    def _param_yaml(self, param, default):
        path = self.get_parameter(param).value
        if not path:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory('framework_bringup'), 'param', default)
        try:
            return yaml.safe_load(open(path)) or {}
        except (OSError, yaml.YAMLError) as e:
            self.get_logger().warn(f'{path} not loaded ({e}): its slots stay free text')
            return {}

    def _places(self):
        return list(self._param_yaml('places_file', 'places.yaml').get('places') or {})

    def _arm_places(self):
        taught = ((self._param_yaml('extensions_file', 'extensions.yaml').get('arm_extension_node') or {}).get('ros__parameters') or {}).get('places') or []
        return [t.split(':', 1)[0] for t in taught if ':' in t]

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
        labels = sorted(self.vocab.get(LABEL_KEY, ()))
        enums = {'nav': self._places(), 'arm': self._arm_places(), 'label': labels}
        res.schema = json.dumps(spec_schema(list(self.catalog.all().values()), self.macros, enums))
        res.labels = labels
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
