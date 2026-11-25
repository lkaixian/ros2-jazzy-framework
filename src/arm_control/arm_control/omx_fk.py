"""OpenMANIPULATOR-X forward kinematics from the installed URDF. No ROS node, no MoveIt: pure maths on the xacro output.

Frame = the URDF `world` (the arm's base plate: x forward, y left, z up, metres). Joint angles in rad, as the ros2_control driver
reports them (raw tick 2048 = 0 rad).
"""
import math
import subprocess
import xml.etree.ElementTree as ET

import numpy as np

XACRO = '/opt/ros/jazzy/share/open_manipulator_description/urdf/open_manipulator_x/open_manipulator_x.urdf.xacro'
TICK = 2 * math.pi / 4096


def _rot(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


class Arm:
    def __init__(self, urdf_text=None):
        text = urdf_text or subprocess.run(['xacro', XACRO, 'use_mock_hardware:=true'], capture_output=True, text=True,
                                           check=True).stdout
        self.text = text
        root = ET.fromstring(text)
        self.child = {j.find('child').get('link'): j for j in root.findall('joint')}

    def pose(self, link, q):
        """4x4 transform of `link` in the arm base frame for joint angles q = {'joint1': ..., ...}."""
        chain, cur = [], link
        while cur in self.child:
            chain.append(self.child[cur])
            cur = self.child[cur].find('parent').get('link')
        m = np.eye(4)
        for j in reversed(chain):
            o = j.find('origin')
            t = np.eye(4)
            if o is not None:
                t[:3, :3] = _rot(*[float(v) for v in (o.get('rpy') or '0 0 0').split()])
                t[:3, 3] = [float(v) for v in (o.get('xyz') or '0 0 0').split()]
            ang = q.get(j.get('name'), 0.0)
            if j.get('type') in ('revolute', 'continuous') and ang:
                ax = np.array([float(v) for v in j.find('axis').get('xyz').split()])
                ax /= np.linalg.norm(ax)
                k = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
                r = np.eye(4)
                r[:3, :3] = np.eye(3) + math.sin(ang) * k + (1 - math.cos(ang)) * k @ k
                t = t @ r
            m = m @ t
        return m

    def xyz(self, link, q):
        return self.pose(link, q)[:3, 3]


def ticks_to_rad(ticks):
    return (ticks - 2048) * TICK
