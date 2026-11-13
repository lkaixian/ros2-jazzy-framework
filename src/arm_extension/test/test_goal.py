import math

from arm_extension.moveit_goal import build_goal, quat_from_rpy


def test_position_only_has_no_orientation_constraint():
    g = build_goal('arm', 'tool0', 'base_link', (0.3, 0.1, 0.4))
    c = g.request.goal_constraints[0]
    assert len(c.position_constraints) == 1 and not c.orientation_constraints
    pc = c.position_constraints[0]
    assert pc.link_name == 'tool0' and pc.header.frame_id == 'base_link'
    assert pc.constraint_region.primitive_poses[0].position.z == 0.4
    assert g.request.group_name == 'arm' and not g.planning_options.plan_only


def test_orientation_and_scaling():
    g = build_goal('arm', 'tool0', 'base_link', (0, 0, 0.3), rpy=(0, 0, math.pi / 2),
                   vel_scale=0.05)
    oc = g.request.goal_constraints[0].orientation_constraints[0]
    assert abs(oc.orientation.z - math.sin(math.pi / 4)) < 1e-9
    assert g.request.max_velocity_scaling_factor == 0.05


def test_quaternion_unit_norm():
    q = quat_from_rpy(0.3, -0.7, 2.0)
    assert abs(sum(v * v for v in q) - 1.0) < 1e-12
