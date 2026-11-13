import json

import rclpy
from bt_common.catalog import Catalog
from bt_common.extension_node import MANIFEST_QOS
from bt_validator.validate import validate_xml
from bt_pipeline_msgs.msg import ExtensionManifest
from bt_pipeline_msgs.srv import GenerateXml
from rclpy.node import Node

from bt_slm.fallback import build_xml
from bt_slm.prompt import prompt_for


class SlmNode(Node):
    def __init__(self):
        super().__init__('bt_slm_node')
        self.declare_parameter('model_path', '')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('max_new_tokens', 768)
        self.declare_parameter('temperature', 0.0)
        # the rule-based generator is exact for every spec in the grammar and ~25000x faster than the
        # model (0.06 ms vs 1.5 s on a GTX 1650), so the model only runs when rules cannot do it
        self.declare_parameter('rules_first', True)
        self.catalog = Catalog()
        self.tok = self.model = None
        self._load()
        self.create_subscription(ExtensionManifest, 'extensions/manifest',
                                 lambda m: self.catalog.add_manifest_msg(m), MANIFEST_QOS)
        self.create_service(GenerateXml, 'bt_slm/generate', self._on_generate)

    def _load(self):
        path = self.get_parameter('model_path').value
        if not path:
            self.get_logger().warn('model_path empty: using rule-based fallback generator')
            return
        try:
            from transformers import AutoModelForCausalLM, AutoTokenizer
            self.tok = AutoTokenizer.from_pretrained(path)
            self.model = AutoModelForCausalLM.from_pretrained(path).to(
                self.get_parameter('device').value).eval()
            self.get_logger().info(f'SLM loaded from {path}')
        except Exception as e:  # noqa: BLE001 - any load failure -> fallback, loudly
            self.get_logger().error(f'SLM load failed ({e}); using fallback generator')
            self.tok = self.model = None

    def _generate(self, req):
        if self.model is None:
            return build_xml(req.task_spec, set(self.catalog.all()), req.previous_errors)
        if self.get_parameter('rules_first').value and not req.previous_errors:
            try:
                xml = build_xml(req.task_spec)
                if not validate_xml(xml, self.catalog):
                    return xml
            except Exception:  # noqa: BLE001 - out of grammar / unknown node: the model may cope
                pass
        import torch
        from bt_slm.prompt import extract_xml
        prompt = prompt_for(req.task_spec, self.catalog, req.previous_xml,
                            list(req.previous_errors))
        ids = self.tok(prompt, return_tensors='pt').to(self.model.device)
        temp = self.get_parameter('temperature').value
        with torch.no_grad():
            out = self.model.generate(
                **ids, max_new_tokens=self.get_parameter('max_new_tokens').value,
                do_sample=temp > 0, temperature=temp if temp > 0 else None,
                pad_token_id=self.tok.eos_token_id)
        return extract_xml(self.tok.decode(out[0][ids['input_ids'].shape[1]:],
                                           skip_special_tokens=True))

    def _on_generate(self, req, res):
        try:
            json.loads(req.task_spec)
            res.xml = self._generate(req)
            res.ok = True
        except Exception as e:  # noqa: BLE001
            res.ok, res.error = False, f'{type(e).__name__}: {e}'
        return res


def main(args=None):
    rclpy.init(args=args)
    node = SlmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
