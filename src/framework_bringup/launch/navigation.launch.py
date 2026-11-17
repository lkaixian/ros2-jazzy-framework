"""Navigation: static TFs, SLAM or AMCL localization, Nav2, map saver.

  # localize on a saved map and navigate (default: slam:=none, map:=sample test_room)
  ros2 launch framework_bringup navigation.launch.py map:=$HOME/maps/my_map.yaml
  # build a map with slam_toolbox (default 2D SLAM) while navigating
  ros2 launch framework_bringup navigation.launch.py slam:=slam_toolbox
  # alternatives: slam:=cartographer (2D), slam:=rtabmap (RGB-D, needs sensors.launch.py)
  # mapping only, no planner/controller:   navigation:=false

Nav2's final velocity is published on `nav2/cmd_vel`; cmd_vel_mux_node (extensions.launch.py)
clamps it, applies the watchdog and forwards it to the robot (cmd_vel_topic:=commands/velocity
on the Kobuki). Parameters: param/navigation.yaml (static TFs - PLACEHOLDERS), nav2_params.yaml,
slam_toolbox.yaml, cartographer/kobuki_2d.lua, rtabmap.yaml. A launch argument overrides a file.
"""
import os

from ament_index_python.packages import get_package_share_directory as share
from bt_common.launch_utils import (declare_overrides, declare_params_dir, flag, given, load_yaml,
                                    param_path)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.events import matches_action
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

SLAMS = ('none', 'slam_toolbox', 'cartographer', 'rtabmap')
NAV2_NODES = ['controller_server', 'smoother_server', 'planner_server', 'behavior_server',
              'velocity_smoother', 'collision_monitor', 'bt_navigator', 'waypoint_follower']


def _static_tfs(nav_cfg):
    out = []
    for key in ('footprint_to_base', 'base_to_laser', 'base_to_camera'):
        if key not in nav_cfg['static_tf']:
            continue       # footprint_to_base is optional (see navigation.yaml)
        t = nav_cfg['static_tf'][key]
        args = []
        for flag_name, k in (('--x', 'x'), ('--y', 'y'), ('--z', 'z'), ('--yaw', 'yaw'),
                             ('--pitch', 'pitch'), ('--roll', 'roll')):
            args += [flag_name, str(t[k])]
        args += ['--frame-id', t['parent'], '--child-frame-id', t['child']]
        out.append(Node(package='tf2_ros', executable='static_transform_publisher',
                        name=f'static_tf_{key}', arguments=args, output='log'))
    return out


def _initial_pose(context):
    v = given(context, 'initial_pose')
    if v is None:
        return {}
    try:
        x, y, yaw = (float(s) for s in v.split(','))
    except ValueError:
        raise ValueError("initial_pose must be 'x,y,yaw' (map frame, metres/radians), got "
                         f'{v!r}') from None
    return {'set_initial_pose': True, 'initial_pose': {'x': x, 'y': y, 'z': 0.0, 'yaw': yaw}}


def _manager(name, nodes, sim, bond_timeout):
    return Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name=name,
                output='screen', parameters=[{'use_sim_time': sim, 'autostart': True,
                                              'node_names': nodes,
                                              'bond_timeout': bond_timeout}])


def _slam_toolbox(context, sim):
    node = LifecycleNode(
        package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
        namespace='', output='screen',
        parameters=[param_path(context, 'slam_toolbox.yaml'),
                    {'use_lifecycle_manager': False, 'use_sim_time': sim}])
    configure = EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(node),
                                            transition_id=Transition.TRANSITION_CONFIGURE))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=node, start_state='configuring', goal_state='inactive',
        entities=[LogInfo(msg='[navigation] slam_toolbox activating'),
                  EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(node),
                                              transition_id=Transition.TRANSITION_ACTIVATE))]))
    return [node, configure, activate]


def _cartographer(context, sim):
    cfg_dir = os.path.join(os.path.dirname(param_path(context, 'navigation.yaml')), 'cartographer')
    return [
        Node(package='cartographer_ros', executable='cartographer_node', name='cartographer_node',
             output='screen', parameters=[{'use_sim_time': sim}],
             arguments=['-configuration_directory', cfg_dir, '-configuration_basename',
                        'kobuki_2d.lua'],
             remappings=[('scan', 'scan'), ('odom', 'odom')]),
        Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
             name='cartographer_occupancy_grid_node', output='screen',
             parameters=[{'use_sim_time': sim}],
             arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']),
    ]


def _rtabmap(context, sim):
    rs = load_yaml(param_path(context, 'realsense.yaml'))
    cam = f"/{rs.get('camera_namespace', 'camera')}/{rs.get('camera_name', 'camera')}"
    cfg = load_yaml(param_path(context, 'rtabmap.yaml'))
    rtab = dict(cfg['rtabmap']['ros__parameters'], use_sim_time=sim)
    rtab['database_path'] = os.path.expanduser(str(rtab['database_path']))
    sync = dict(cfg['rgbd_sync']['ros__parameters'], use_sim_time=sim)
    new_map = flag(context, 'rtabmap_new_map')
    return [
        Node(package='rtabmap_sync', executable='rgbd_sync', name='rgbd_sync', output='screen',
             parameters=[sync],
             remappings=[('rgb/image', f'{cam}/color/image_raw'),
                         ('depth/image', f'{cam}/aligned_depth_to_color/image_raw'),
                         ('rgb/camera_info', f'{cam}/color/camera_info'),
                         ('rgbd_image', 'rgbd_image')]),
        Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
             parameters=[rtab], arguments=['-d'] if new_map else [],
             remappings=[('rgbd_image', 'rgbd_image'), ('scan', 'scan'), ('odom', 'odom')]),
    ]


def _localization(context, sim, map_yaml):
    amcl_params = [param_path(context, 'nav2_params.yaml'), {'use_sim_time': sim},
                   _initial_pose(context)]
    amcl_params = [p for p in amcl_params if p]
    return [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[{'yaml_filename': map_yaml, 'use_sim_time': sim}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=amcl_params),
    ]


# planner:=<key> swaps the global planner plugin without touching nav2_params.yaml
PLANNERS = {'navfn': 'nav2_navfn_planner::NavfnPlanner',
            'dstar_lite': 'dstar_lite_planner::DStarLite'}


def _nav2(context, sim):
    params = [param_path(context, 'nav2_params.yaml'), {'use_sim_time': sim}]
    to_nav = [('cmd_vel', 'cmd_vel_nav')]
    planner = given(context, 'planner')
    if planner is not None and planner not in PLANNERS:
        raise ValueError(f'planner:={planner!r}: choose one of {sorted(PLANNERS)}')

    def n(pkg, exe, name, remap=()):
        p = params + ([{'GridBased.plugin': PLANNERS[planner]}]
                      if name == 'planner_server' and planner else [])
        return Node(package=pkg, executable=exe, name=name, output='screen', parameters=p,
                    remappings=list(remap))
    return [
        # /scan without the beams that hit the robot's own body: costmaps and collision_monitor read /scan_filtered
        Node(package='bt_common', executable='scan_mask_node', name='scan_mask_node', output='screen',
             parameters=[{'use_sim_time': sim}]),
        n('nav2_controller', 'controller_server', 'controller_server', to_nav),
        n('nav2_smoother', 'smoother_server', 'smoother_server'),
        n('nav2_planner', 'planner_server', 'planner_server'),
        n('nav2_behaviors', 'behavior_server', 'behavior_server', to_nav),
        n('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother', to_nav),
        n('nav2_collision_monitor', 'collision_monitor', 'collision_monitor'),
        n('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        n('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower'),
    ]


def _actions(context):
    slam = (given(context, 'slam') or 'none').strip()
    if slam not in SLAMS:
        raise ValueError(f'slam:={slam!r} is not one of {SLAMS}')
    sim = flag(context, 'use_sim_time')
    nav_cfg = load_yaml(param_path(context, 'navigation.yaml'))
    bond = float(nav_cfg.get('navigation', {}).get('bond_timeout', 10.0))
    out = []
    if flag(context, 'publish_static_tf'):
        out += _static_tfs(nav_cfg)
    if slam == 'slam_toolbox':
        out += _slam_toolbox(context, sim)
    elif slam == 'cartographer':
        out += _cartographer(context, sim)
    elif slam == 'rtabmap':
        out += _rtabmap(context, sim)
    else:
        map_yaml = given(context, 'map') or nav_cfg.get('navigation', {}).get('default_map') or \
            os.path.join(share('framework_bringup'), 'maps', 'test_room.yaml')
        map_yaml = os.path.expanduser(map_yaml)
        if not os.path.isfile(map_yaml):
            raise FileNotFoundError(f'map file {map_yaml} not found (pass map:=<file.yaml> or '
                                    f'slam:=slam_toolbox)')
        out.append(LogInfo(msg=f'[navigation] localizing (AMCL) on {map_yaml}'))
        out += _localization(context, sim, map_yaml)
        out.append(_manager('lifecycle_manager_localization', ['map_server', 'amcl'], sim, bond))
    # map_saver lets the SaveMap BT node / `ros2 run nav2_map_server map_saver_cli` write the map
    out.append(Node(package='nav2_map_server', executable='map_saver_server', name='map_saver',
                    output='screen', parameters=[param_path(context, 'nav2_params.yaml'),
                                                 {'use_sim_time': sim}]))
    out.append(_manager('lifecycle_manager_map_saver', ['map_saver'], sim, bond))
    if flag(context, 'navigation'):
        out += _nav2(context, sim)
        out.append(_manager('lifecycle_manager_navigation', NAV2_NODES, sim, bond))
    return out


def generate_launch_description():
    return LaunchDescription([
        declare_params_dir(),
        DeclareLaunchArgument('navigation', default_value='true',
                              description='false = SLAM/localization only, no planner/controller'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('publish_static_tf', default_value='true',
                              description='base->laser and base->camera_link from navigation.yaml'),
        DeclareLaunchArgument('rtabmap_new_map', default_value='true',
                              description='slam:=rtabmap: delete the old database on start'),
        *declare_overrides('slam', 'map', 'initial_pose', 'planner'),
        OpaqueFunction(function=_actions),
    ])
