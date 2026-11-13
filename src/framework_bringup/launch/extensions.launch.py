"""Extension packages: each advertises its BT nodes on extensions/manifest."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node

EXTENSIONS = ('arm', 'pose', 'odom', 'pid')


def generate_launch_description():
    actions = [DeclareLaunchArgument(f'enable_{e}', default_value='true')
               for e in EXTENSIONS]
    actions += [Node(package=f'{e}_extension', executable=f'{e}_extension_node',
                     condition=IfCondition(LC(f'enable_{e}')), output='screen')
                for e in EXTENSIONS]
    return LaunchDescription(actions)
