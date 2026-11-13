import os
import threading
import time

from ament_index_python.packages import get_package_share_directory
from bt_common.executor_node import (call_service, ExtensionExecutor, port_bool, port_float,
                                     port_str, run_executor)
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap, SaveMap
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from nav_extension import nav_logic as nl


def default_places_file():
    try:
        return os.path.join(get_package_share_directory('framework_bringup'), 'param',
                            'places.yaml')
    except Exception:  # noqa: BLE001 - package not installed: no default places
        return ''


class NavExtension(ExtensionExecutor):
    """BT nodes over Nav2: NavigateToPose, NavigateToPlace, SetInitialPose, ClearCostmaps,
    SaveMap, IsNearPose. Needs navigation.launch.py (Nav2 active) for most of them."""

    def __init__(self):
        super().__init__('nav_extension')
        for k, v in {'nav2_action': 'navigate_to_pose', 'base_frame': 'base',
                     'global_frame': 'map', 'places_file': '', 'map_dir': '~/maps',
                     'default_timeout_sec': 180.0, 'tf_timeout_sec': 0.5,
                     'initial_pose_topic': 'initialpose',
                     'saver_service': 'map_saver/save_map',
                     'clear_local_service': 'local_costmap/clear_entirely_local_costmap',
                     'clear_global_service': 'global_costmap/clear_entirely_global_costmap'
                     }.items():
            self.declare_parameter(k, v)
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, p('nav2_action'))
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, p('initial_pose_topic'), 10)
        self.clear_local = self.create_client(ClearEntireCostmap, p('clear_local_service'))
        self.clear_global = self.create_client(ClearEntireCostmap, p('clear_global_service'))
        self.saver = self.create_client(SaveMap, p('saver_service'))
        self.places = {}
        path = p('places_file') or default_places_file()
        try:
            self.places = nl.load_places(path)
            self.get_logger().info(f'{len(self.places)} places from {path}: '
                                   f'{nl.places_listing(self.places)}')
        except nl.PlacesError as e:
            self.get_logger().error(f'NavigateToPlace has no places: {e}')
        # tell the LLM which place names exist (the manifest is re-published periodically)
        for spec in self.manifest.nodes:
            if spec.id == 'NavigateToPlace':
                spec.description += f' Known places: {nl.places_listing(self.places)}.'
        self.handlers = {'NavigateToPose': self._navigate_pose,
                         'NavigateToPlace': self._navigate_place,
                         'SetInitialPose': self._set_initial_pose,
                         'ClearCostmaps': self._clear_costmaps,
                         'SaveMap': self._save_map,
                         'IsNearPose': self._is_near}

    # ------------------------------------------------------------------ helpers
    def robot_pose(self, frame=None):
        """(x, y, yaw) of the base frame in `frame` (default global), or None."""
        frame = frame or self.get_parameter('global_frame').value
        try:
            t = self.tf_buffer.lookup_transform(
                frame, self.get_parameter('base_frame').value, Time(),
                timeout=Duration(seconds=self.get_parameter('tf_timeout_sec').value))
        except TransformException:
            return None
        r = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                nl.quat_to_yaw(r.x, r.y, r.z, r.w))

    def _send_goal(self, goal, ctl, timeout):
        """(status, result, error). Cancels the Nav2 goal on BT halt or timeout."""
        if not self.nav.wait_for_server(timeout_sec=2.0):
            return None, None, ('navigate_to_pose action server unavailable '
                                '(is Nav2 running and active?)')
        accepted, box = threading.Event(), {}

        def on_response(fut):
            box['gh'] = fut.result()
            accepted.set()

        def on_feedback(msg):
            try:
                ctl.feedback(f'{msg.feedback.distance_remaining:.2f} m to go')
            except Exception:  # noqa: BLE001 - feedback is best effort
                pass
        self.nav.send_goal_async(goal, feedback_callback=on_feedback).add_done_callback(on_response)
        if not accepted.wait(5.0):
            return None, None, 'no response from Nav2'
        gh = box['gh']
        if not gh.accepted:
            return None, None, 'goal rejected by Nav2 (not active yet, or goal unreachable)'
        finished = threading.Event()
        gh.get_result_async().add_done_callback(
            lambda f: (box.__setitem__('res', f.result()), finished.set()))
        end = time.monotonic() + timeout
        while not finished.wait(0.05):
            if ctl.cancelled or time.monotonic() > end:
                gh.cancel_goal_async()          # make Nav2 stop the robot
                finished.wait(3.0)
                return None, None, 'cancelled' if ctl.cancelled else 'timeout'
        res = box['res']
        return res.status, res.result, ''

    def _go(self, x, y, yaw, frame, timeout, ctl):
        if yaw is None:
            yaw = nl.goal_yaw(self.robot_pose(), (x, y))
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        q = nl.yaw_to_quat(yaw)
        (goal.pose.pose.orientation.x, goal.pose.pose.orientation.y,
         goal.pose.pose.orientation.z, goal.pose.pose.orientation.w) = q
        status, result, err = self._send_goal(goal, ctl, timeout)
        if err:
            return False, err
        return nl.nav_result(status, result.error_code, result.error_msg)

    def _timeout(self, ports):
        default = self.get_parameter('default_timeout_sec').value
        return port_float(ports, 'timeout_ms', default * 1000.0, lo=1000.0, hi=3600000.0) / 1000.0

    # ------------------------------------------------------------------ handlers
    def _navigate_pose(self, ports, ctl):
        x, y = port_float(ports, 'x'), port_float(ports, 'y')
        yaw = port_float(ports, 'yaw') if ports.get('yaw') else None
        frame = ports.get('frame') or self.get_parameter('global_frame').value
        return self._go(x, y, yaw, frame, self._timeout(ports), ctl)

    def _navigate_place(self, ports, ctl):
        place = port_str(ports, 'place')
        if place not in self.places:
            return False, f'unknown place {place!r}; known: {nl.places_listing(self.places)}'
        x, y, yaw = self.places[place]
        return self._go(x, y, yaw, self.get_parameter('global_frame').value,
                        self._timeout(ports), ctl)

    def _set_initial_pose(self, ports, ctl):
        x, y = port_float(ports, 'x'), port_float(ports, 'y')
        yaw = port_float(ports, 'yaw', 0.0)
        m = PoseWithCovarianceStamped()
        m.header.frame_id = self.get_parameter('global_frame').value
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        (m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z,
         m.pose.pose.orientation.w) = nl.yaw_to_quat(yaw)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.25      # same as rviz "2D Pose Estimate"
        m.pose.covariance[35] = 0.0685
        end = time.monotonic() + 3.0
        while self.init_pub.get_subscription_count() == 0 and time.monotonic() < end:
            if not ctl.sleep(0.1):
                return False, 'cancelled'
        if self.init_pub.get_subscription_count() == 0:
            return False, 'nobody listens on the initial pose topic (is AMCL running?)'
        self.init_pub.publish(m)
        return True, f'initial pose ({x:.2f}, {y:.2f}, {yaw:.2f}) sent to AMCL'

    def _clear_costmaps(self, ports, ctl):
        for name, client in (('local', self.clear_local), ('global', self.clear_global)):
            res, err = call_service(client, ClearEntireCostmap.Request(request=Empty()), ctl, 5.0)
            if res is None:
                return False, f'{name} costmap: {err}'
        return True, 'costmaps cleared'

    def _save_map(self, ports, ctl):
        file = port_str(ports, 'file')
        base = nl.map_base_path(self.get_parameter('map_dir').value, file)
        if not port_bool(ports, 'overwrite', False) and (os.path.exists(base + '.yaml') or
                                                         os.path.exists(base + '.pgm')):
            return False, f'{base}.yaml exists (pass overwrite="true" to replace it)'
        os.makedirs(os.path.dirname(base), exist_ok=True)
        req = SaveMap.Request(map_topic='map', map_url=base, image_format='pgm',
                              map_mode='trinary', free_thresh=0.25, occupied_thresh=0.65)
        res, err = call_service(self.saver, req, ctl, 20.0)
        if res is None:
            return False, err
        return bool(res.result), (f'map saved to {base}.yaml' if res.result
                                  else 'map_saver reported failure (is a /map being published?)')

    def _is_near(self, ports, ctl):
        x, y = port_float(ports, 'x'), port_float(ports, 'y')
        tol = port_float(ports, 'tolerance', 0.3, lo=0.01, hi=50.0)
        frame = ports.get('frame') or self.get_parameter('global_frame').value
        pose = self.robot_pose(frame)
        if pose is None:
            return False, f'no TF {frame} -> {self.get_parameter("base_frame").value}'
        d = nl.distance(pose, (x, y))
        return d <= tol, f'{d:.2f} m from ({x:.2f}, {y:.2f})'


def main(args=None):
    run_executor(NavExtension, args)


if __name__ == '__main__':
    main()
