"""host_info: laptop/PC health of every cluster host (or one): CPU, RAM, GPU/VRAM, temperature, battery.

  ros2 run cluster_core host_info            # every host that runs a cluster_agent
  ros2 run cluster_core host_info <host_id>
Reads the `host_info/<host_id>` service each agent serves; values are the agent's latest ~2 Hz sample.
"""
import argparse
import sys
import time

import rclpy
from cluster_core.sysinfo import host_service_name
from cluster_msgs.srv import HostInfo

TYPE = 'cluster_msgs/srv/HostInfo'


def fmt(i):
    def mb(v):
        return f'{v / 1024:.1f}G' if v >= 0 else '-'
    gpu = '-' if i.gpu_mem_total_mb < 0 else (
        f'{i.gpu_util * 100:3.0f}%  {mb(i.gpu_mem_total_mb - i.gpu_mem_free_mb)}/{mb(i.gpu_mem_total_mb)}')
    batt = '-' if i.battery_pct < 0 else (
        f"{i.battery_pct:.0f}%{'+' if i.charging == 1 else '' if i.charging == 0 else '?'}")
    temp = '-' if i.temp_c < 0 else f'{i.temp_c:.0f}C'
    return (f'{i.host_id:<14} cpu {i.cpu_load * 100:3.0f}% x{i.cpu_cores:<2} '
            f'ram {mb(i.mem_total_mb - i.mem_free_mb)}/{mb(i.mem_total_mb)}  gpu {gpu:<18} '
            f'temp {temp:<5} battery {batt}')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('host', nargs='?', default='')
    a = ap.parse_args(argv)
    rclpy.init()
    n = rclpy.create_node('host_info_cli')
    try:
        end = time.monotonic() + 2.5               # let discovery find the agents
        while time.monotonic() < end:
            rclpy.spin_once(n, timeout_sec=0.1)
        want = host_service_name(a.host) if a.host else None
        names = sorted(nm for nm, ts in n.get_service_names_and_types()
                       if TYPE in ts and (want is None or nm.lstrip('/') == want))
        if not names:
            print('no host_info service found (is a cluster_agent running?)', file=sys.stderr)
            return 1
        for nm in names:
            c = n.create_client(HostInfo, nm)
            f = c.call_async(HostInfo.Request()) if c.wait_for_service(2.0) else None
            if f is None:
                print(f'{nm}: unavailable')
                continue
            rclpy.spin_until_future_complete(n, f, timeout_sec=3.0)
            print(fmt(f.result().info) if f.done() else f'{nm}: timeout')
        return 0
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
