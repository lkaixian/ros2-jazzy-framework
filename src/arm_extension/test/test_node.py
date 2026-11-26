import pytest


def test_node_module_imports_and_declares_every_catalog_node():
    pytest.importorskip('dynamixel_interfaces')        # the driver's messages are not in CI
    import yaml
    from pathlib import Path

    import arm_extension.arm_extension_node as m
    cfg = yaml.safe_load((Path(m.__file__).parents[1] / 'config' / 'arm_extension.yaml').read_text())
    src = Path(m.__file__).read_text()
    assert all(f"'{n}'" in src for n in cfg['nodes'])


def test_wheel_only_joint_states_do_not_wipe_the_arm_joints():
    pytest.importorskip('dynamixel_interfaces')
    from types import SimpleNamespace as NS

    import arm_extension.arm_extension_node as m
    n = NS(js={}, js_t=0.0)
    arm = NS(name=['joint1', 'joint2', 'joint3', 'joint4', 'gripper_left_joint'], position=[0.0, 1.0, 2.0, 3.0, 4.0])
    wheels = NS(name=['wheel_left_joint', 'wheel_right_joint'], position=[5.0, 6.0])
    m.ArmExtension._on_js(n, arm)
    t = n.js_t
    m.ArmExtension._on_js(n, wheels)
    assert n.js['joint2'] == 1.0 and n.js_t == t
