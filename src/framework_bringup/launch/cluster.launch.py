"""Cluster platform: job placement, failover and load balancing across machines.

  ros2 launch framework_bringup cluster.launch.py role:=agent     host_id:=pi      roles:=core tags:=camera,audio
  ros2 launch framework_bringup cluster.launch.py role:=both      host_id:=laptop1 roles:=compute tags:=gpu
  ros2 launch framework_bringup cluster.launch.py role:=scheduler

role: agent (default, every machine), scheduler (the orchestrator) or both. Run exactly ONE
scheduler in the whole cluster. Parameters: param/cluster.yaml, jobs: param/cluster_jobs.yaml.
"""
from bt_common.launch_utils import declare_overrides, declare_params_dir, make_node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

AGENT_ARGS = {'host_id': 'host_id', 'roles': 'roles', 'tags': 'tags'}
F = 'cluster.yaml'


def _nodes(context):
    role = LaunchConfiguration('role').perform(context).strip().lower()
    if role not in ('agent', 'scheduler', 'both'):
        raise ValueError(f"role must be agent, scheduler or both, got {role!r}")
    out = []
    if role in ('agent', 'both'):
        out.append(make_node(context, 'cluster_core', 'cluster_agent_node',
                             'cluster_agent_node', F, AGENT_ARGS))
    if role in ('scheduler', 'both'):
        out.append(make_node(context, 'cluster_core', 'cluster_scheduler_node',
                             'cluster_scheduler_node', F, {'scheduler_id': 'scheduler_id'}))
    return out


def generate_launch_description():
    return LaunchDescription([
        declare_params_dir(),
        DeclareLaunchArgument('role', default_value='agent',
                              description='agent | scheduler | both (exactly one scheduler per cluster)'),
        *declare_overrides('host_id', 'roles', 'tags', 'scheduler_id'),
        OpaqueFunction(function=_nodes),
    ])
