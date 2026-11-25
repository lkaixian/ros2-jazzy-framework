#!/usr/bin/env python3
"""Fake arm / pose / social / nav extensions for arm_task_test.py: the REAL manifests (so the validator and executor see the real nodes) and
scripted behaviour, no robot, no arm. NextObject runs the real selection code and the real param/object_categories.yaml on a scripted list of
objects; ExtractAnswer runs the real answer parser. Every call is published in order as JSON on `fake/calls`; `fake/world` sets the objects,
the spoken answer and which node names fail."""
import json
import os
import sys

import rclpy
from bt_common.executor_node import ExtensionExecutor, port_str
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'src')
sys.path.insert(0, os.path.join(ROOT, 'pose_extension'))
sys.path.insert(0, os.path.join(ROOT, 'social_extension'))
from pose_extension.categories import load_table, pick_next  # noqa: E402
from social_extension.answers import extract_place  # noqa: E402

TABLE = load_table(os.path.join(ROOT, 'framework_bringup', 'param', 'object_categories.yaml'))
W = {'objects': [], 'answer': 'put it in the bin please', 'fail': {}, 'calls': [], 'last': ''}


class Obj:
    def __init__(self, label):
        self.label, self.confidence, self.x_min, self.y_min, self.x_max, self.y_max = label, 0.9, 0, 0, 100, 100


def record(pub, node, ports):
    W['calls'].append([node, {k: v for k, v in ports.items()}])
    pub.publish(String(data=json.dumps(W['calls'])))


def scripted(pub, node, ok_msg='fake: done'):
    def handler(ports, ctl):
        record(pub, node, ports)
        if W['fail'].get(node, 0) > 0:
            W['fail'][node] -= 1
            return False, f'fake: {node} failed'
        return True, ok_msg
    return handler


class Fake(ExtensionExecutor):
    def __init__(self, name, nodes):
        super().__init__(name)
        self.pub = self.create_publisher(String, 'fake/calls', 10)
        self.handlers = {n: scripted(self.pub, n) for n in nodes}


class FakeArm(Fake):
    def __init__(self):
        super().__init__('arm_extension', ['PickObject', 'PlaceObject', 'ArmPark', 'OpenGripper', 'CloseGripper', 'MoveArmToPose'])
        self.create_subscription(String, 'fake/world', self._world, 10)

    def _world(self, m):
        d = json.loads(m.data)
        W['calls'].clear()
        W.update(objects=d.get('objects', []), answer=d.get('answer', W['answer']), fail=d.get('fail', {}), last='')


class FakePose(Fake):
    def __init__(self):
        super().__init__('pose_extension', [])
        self.handlers = {'NextObject': self._next}

    def _next(self, ports, ctl):
        record(self.pub, 'NextObject', ports)
        o, e = pick_next([Obj(x) for x in W['objects']], TABLE.get(port_str(ports, 'task'), {}), W['last'])
        if o is None:
            return False, 'fake: nothing in view'
        W['objects'].remove(o.label)             # picked up: it is gone from the table
        W['last'] = o.label
        return True, o.label, {'out_label': o.label, 'out_nav': e['nav'], 'out_put': e['put'], 'out_group': e['group']}


class FakeSocial(Fake):
    def __init__(self):
        super().__init__('social_extension', ['Speak', 'WaitForPhrase'])
        h = self.handlers
        h['Speak'] = lambda ports, ctl: (record(self.pub, 'Speak', {'text': port_str(ports, 'text'), 'person': ports.get('person', ''),
                                                                    'detail': ports.get('detail', '')}) or True, 'fake: spoke')
        h['WaitForTranscript'] = lambda ports, ctl: (record(self.pub, 'WaitForTranscript', ports) or True, 'heard', {'out_text': W['answer']})
        h['ExtractAnswer'] = self._extract

    def _extract(self, ports, ctl):
        record(self.pub, 'ExtractAnswer', ports)
        v = extract_place(port_str(ports, 'text'))
        return (True, v, {'out_value': v}) if v else (False, 'no place')


class FakeNav(Fake):
    def __init__(self):
        super().__init__('nav_extension', ['NavigateToPlace', 'ClearCostmaps'])


def main():
    rclpy.init()
    ex = MultiThreadedExecutor(num_threads=8)
    for n in (FakeArm(), FakePose(), FakeSocial(), FakeNav()):
        ex.add_node(n)
    try:
        ex.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
