"""Build a MoveGroup goal for a Cartesian pose target (model-agnostic)."""
import math

from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (BoundingVolume, Constraints, MotionPlanRequest,
                             OrientationConstraint, PositionConstraint)
from shape_msgs.msg import SolidPrimitive


def quat_from_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r / 2), math.sin(r / 2), math.cos(p / 2),
                              math.sin(p / 2), math.cos(y / 2), math.sin(y / 2))
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def build_goal(group, ee_link, frame, xyz, rpy=None, vel_scale=0.1, acc_scale=0.1,
               planning_time=5.0, pos_tol=0.005, ori_tol=0.05, attempts=3, plan_only=False):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = xyz
    if rpy is not None:
        q = quat_from_rpy(*rpy)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
    else:
        pose.orientation.w = 1.0
    c = Constraints()
    pc = PositionConstraint()
    pc.header.frame_id, pc.link_name, pc.weight = frame, ee_link, 1.0
    sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[pos_tol])
    pc.constraint_region = BoundingVolume(primitives=[sphere], primitive_poses=[pose])
    c.position_constraints.append(pc)
    if rpy is not None:
        oc = OrientationConstraint()
        oc.header.frame_id, oc.link_name, oc.weight = frame, ee_link, 1.0
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = \
            oc.absolute_z_axis_tolerance = ori_tol
        c.orientation_constraints.append(oc)
    req = MotionPlanRequest(group_name=group, num_planning_attempts=attempts,
                            allowed_planning_time=planning_time,
                            max_velocity_scaling_factor=vel_scale,
                            max_acceleration_scaling_factor=acc_scale)
    req.goal_constraints.append(c)
    goal = MoveGroup.Goal(request=req)
    goal.planning_options.plan_only = plan_only
    return goal
