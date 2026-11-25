import pytest


def test_node_module_imports_and_declares_every_catalog_node():
    pytest.importorskip('dynamixel_interfaces')        # the driver's messages are not in CI
    import yaml
    from pathlib import Path

    import arm_extension.arm_extension_node as m
    cfg = yaml.safe_load((Path(m.__file__).parents[1] / 'config' / 'arm_extension.yaml').read_text())
    src = Path(m.__file__).read_text()
    assert all(f"'{n}'" in src for n in cfg['nodes'])
