#!/usr/bin/env python3
"""Fake nav + social extensions for recovery_test.py: real manifests and action servers, no robot.

NavigateToPlace fails for a place while `fake/fail` ({"door": 2} = the next 2 goals to `door` fail: one go_to_place run makes 2 goals, it retries itself) says so and counts every goal on
`fake/calls`; Speak publishes its text on `fake/spoken`."""
import json

import rclpy
from bt_common.executor_node import ExtensionExecutor, port_str
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String


class FakeNav(ExtensionExecutor):
    def __init__(self):
        super().__init__('nav_extension')
        self.fail, self.calls = {}, {}
        self.create_subscription(String, 'fake/fail', lambda m: self.fail.update(json.loads(m.data)), 10)
        self.pub = self.create_publisher(String, 'fake/calls', 10)
        self.handlers = {'NavigateToPlace': self._go, 'ClearCostmaps': lambda ports, ctl: (True, 'fake: cleared')}

    def _go(self, ports, ctl):
        place = port_str(ports, 'place')
        self.calls[place] = self.calls.get(place, 0) + 1
        self.pub.publish(String(data=json.dumps(self.calls)))
        if self.fail.get(place, 0) > 0:
            self.fail[place] -= 1
            return False, f'fake: {place} is blocked'
        return True, f'fake: reached {place}'


class FakeSocial(ExtensionExecutor):
    def __init__(self):
        super().__init__('social_extension')
        self.pub = self.create_publisher(String, 'fake/spoken', 10)
        self.handlers = {'Speak': lambda ports, ctl: (self.pub.publish(String(data=port_str(ports, 'text'))) or True, 'fake: spoke')}


def main():
    rclpy.init()
    ex = MultiThreadedExecutor(num_threads=6)
    nodes = [FakeNav(), FakeSocial()]
    for n in nodes:
        ex.add_node(n)
    try:
        ex.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
