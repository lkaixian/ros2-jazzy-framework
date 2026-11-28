#!/usr/bin/env python3
"""One-off: OSNet-AIN state dict (LibreYOLO/LibreReID-osnet on Hugging Face, MIT) -> ONNX for person_detection_node's reid_model.

  python3 tools/reid/export_osnet.py WEIGHTS.pt OSNET_AIN.py models/reid/osnet_ain_x1_0.onnx

OSNET_AIN.py is torchreid/reid/models/osnet_ain.py from the torchreid sdist (imports torch only; read it before running it);
torchreid itself is not installed. Input 1x3x256x128 RGB, ImageNet-normalised; output 512-d feature (the runtime L2-normalises it).
"""
import importlib.util
import sys

import torch

weights, arch, out = sys.argv[1:4]
spec = importlib.util.spec_from_file_location('osnet_ain', arch)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
net = mod.osnet_ain_x1_0(pretrained=False)
miss, extra = net.load_state_dict(torch.load(weights, map_location='cpu', weights_only=True), strict=False)
assert not extra and all(k.startswith('classifier') for k in miss), (miss, extra)   # only the stripped head may be absent
net.eval()
torch.onnx.export(net, torch.zeros(1, 3, 256, 128), out, input_names=['image'], output_names=['feature'],
                  dynamic_axes={'image': {0: 'n'}, 'feature': {0: 'n'}}, opset_version=17,
                  dynamo=False)   # the default exporter needs the onnxscript package; the TorchScript one does not
print('wrote', out)
