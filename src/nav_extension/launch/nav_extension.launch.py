"""Navigation BT nodes (needs navigation.launch.py running). Parameters: param/extensions.yaml."""
from bt_common.launch_utils import declare_params_dir, make_node
from launch import LaunchDescription
from launch.actions import OpaqueFunction


def _nodes(context):
    return [make_node(context, 'nav_extension', 'nav_extension_node', 'nav_extension_node',
                      'extensions.yaml')]


def generate_launch_description():
    return LaunchDescription([declare_params_dir(), OpaqueFunction(function=_nodes)])
