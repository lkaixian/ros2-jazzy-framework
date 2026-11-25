"""Static (gravity-only) joint torque of the OpenMANIPULATOR-X for any pose, and a cost that grows as it nears the overload limit.

Pure maths on the URDF masses (omx_fk.Arm): no ROS, no hardware. Torque is about each joint axis in N*m. It ignores acceleration,
friction, cable pull and anything not in the URDF except the `payload` point mass you give it (OAK + bracket on the gripper),
so treat utilisation as an ESTIMATE until it is compared with the servos' measured current (/dynamic_joint_states).
"""
import math
import xml.etree.ElementTree as ET

import numpy as np
from arm_control.omx_fk import Arm

G = 9.81
JOINTS = ('joint1', 'joint2', 'joint3', 'joint4')
CHAIN = ('link1', 'link2', 'link3', 'link4', 'link5')   # link_n is moved by joint n; link5 carries the gripper


class TorqueModel:
    def __init__(self, arm=None, payload_kg=0.15, payload_link='end_effector_link'):
        """payload_kg: guess for OAK-D Lite (61 g) + bracket + share of cable; replace with a weighed value."""
        self.arm = arm or Arm()
        self.payload, self.payload_link = payload_kg, payload_link
        root = ET.fromstring(self.arm.text)
        self.mass, self.com = {}, {}
        for link in root.findall('link'):
            i = link.find('inertial')
            if i is not None:
                self.mass[link.get('name')] = float(i.find('mass').get('value'))
                o = i.find('origin')
                self.com[link.get('name')] = [float(v) for v in (o.get('xyz') if o is not None else '0 0 0').split()]
        self.axis = {j.get('name'): np.array([float(v) for v in j.find('axis').get('xyz').split()])
                     for j in root.findall('joint') if j.find('axis') is not None}

    def torques(self, q):
        """{joint: gravity torque in N*m} for joint angles q = {'joint1': rad, ...}."""
        pts = []        # (mass, world position) of every body that gravity pulls on
        for link, m in self.mass.items():
            if m > 0.005:
                pose = self.arm.pose(link, q)
                pts.append((link, m, pose[:3, :3] @ self.com[link] + pose[:3, 3]))
        pts.append((self.payload_link, self.payload, self.arm.xyz(self.payload_link, q)))
        out = {}
        for n, joint in enumerate(JOINTS, start=1):
            pose = self.arm.pose(CHAIN[n - 1], q)
            axis, origin = pose[:3, :3] @ self.axis[joint], pose[:3, 3]
            tau = 0.0
            for link, m, p in pts:
                moved = link == self.payload_link or (link in CHAIN and CHAIN.index(link) >= n - 1) or link.startswith('gripper')
                if moved:
                    tau += float(np.dot(np.cross(p - origin, [0, 0, -m * G]), axis))
            out[joint] = tau
        return out


def utilisation(tau, limit):
    """|torque| as a fraction of `limit` (N*m) for the joints that carry gravity load (2, 3, 4)."""
    return {j: abs(t) / limit[j] for j, t in tau.items() if j != 'joint1'}


def cost(u, kind='poly', soft=0.4, hard=0.8):
    """0 below `soft`, rising to 1 at `hard`, inf at/above it. kind: linear | poly (quadratic) | exp."""
    if u >= hard:
        return math.inf
    if u <= soft:
        return 0.0
    x = (u - soft) / (hard - soft)
    return {'linear': x, 'poly': x * x, 'exp': (math.exp(3 * x) - 1) / (math.exp(3) - 1)}[kind]
