"""cluster_status: print hosts and job placements of the running cluster.

    ros2 run cluster_core cluster_status [--wait 3]
Exit status: 0 ok, 1 something is unplaceable/failed/down, 2 no scheduler heard.
"""
import argparse
import sys
import time


def _table(headers, rows):
    widths = [max(len(str(x)) for x in col) for col in zip(headers, *rows)] if rows else [len(h) for h in headers]
    fmt = '  '.join('{:<%d}' % w for w in widths)
    lines = [fmt.format(*headers), fmt.format(*['-' * w for w in widths])]
    lines += [fmt.format(*[str(c) for c in r]) for r in rows]
    return '\n'.join(lines)


def render(status, hosts):
    """status: dict from ClusterStatus; hosts: {id: dict from NodeInfo}. Returns (text, healthy)."""
    alive, dead, excl = set(status['alive']), set(status['dead']), set(status['excluded'])
    rows = []
    for hid in sorted(set(hosts) | alive | dead | excl):
        h = hosts.get(hid, {})
        st = 'alive' if hid in alive else 'DEAD' if hid in dead else 'EXCLUDED(catalog)' if hid in excl else '?'
        gpu = h.get('gpu_mem_free_mb', -1)
        rows.append((hid, st, ','.join(h.get('roles', [])) or '-', ','.join(h.get('tags', [])) or '-',
                     f"{100 * h.get('cpu_load', 0):.0f}%/{h.get('cores', '?')}c",
                     f"{h.get('mem_free_mb', 0):.0f}MB", f'{gpu:.0f}MB' if gpu >= 0 else '-',
                     h.get('njobs', 0)))
    out = [f"scheduler {status['scheduler_id']}  epoch {status['epoch']}", '',
           _table(['HOST', 'STATE', 'ROLES', 'TAGS', 'CPU', 'MEM FREE', 'GPU FREE', 'JOBS'], rows), '']
    jrows = [(j['job_id'], j['host_id'] or '-', j['state'], j['job_state'], j['reason'])
             for j in status['jobs']]
    out += [_table(['JOB', 'HOST', 'PLACEMENT', 'RUNNING', 'REASON'], jrows)]
    if status['events']:
        out += ['', 'recent events:'] + [f'  {e}' for e in status['events'][-8:]]
    healthy = (not dead and not excl and all(j['state'] == 'placed' and j['job_state'] in ('running', 'unknown')
                                             for j in status['jobs']))
    return '\n'.join(out), healthy


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--wait', type=float, default=3.0, help='seconds to listen')
    a = ap.parse_args(argv)
    import rclpy
    from cluster_msgs.msg import ClusterStatus, NodeInfo

    from cluster_core.qos import INFO_QOS, LATCHED_QOS, TOPIC_INFO, TOPIC_STATUS
    rclpy.init()
    node = rclpy.create_node('cluster_status_cli')
    box = {'status': None, 'hosts': {}}

    def on_status(m):
        box['status'] = {
            'scheduler_id': m.scheduler_id, 'epoch': int(m.epoch), 'alive': list(m.hosts_alive),
            'dead': list(m.hosts_dead), 'excluded': list(m.hosts_excluded), 'events': list(m.events),
            'jobs': [{'job_id': j.job_id, 'host_id': j.host_id, 'state': j.state, 'reason': j.reason,
                      'job_state': j.job_state} for j in m.jobs]}

    def on_info(m):
        box['hosts'][m.host_id] = {
            'roles': list(m.roles), 'tags': list(m.tags), 'cores': m.cpu_cores,
            'cpu_load': m.cpu_load, 'mem_free_mb': m.mem_free_mb, 'gpu_mem_free_mb': m.gpu_mem_free_mb,
            'njobs': sum(1 for j in m.jobs if j.state in (1, 2))}
    node.create_subscription(ClusterStatus, TOPIC_STATUS, on_status, LATCHED_QOS)
    node.create_subscription(NodeInfo, TOPIC_INFO, on_info, INFO_QOS)
    end = time.monotonic() + a.wait
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.try_shutdown()
    if box['status'] is None:
        print('no cluster scheduler heard (same ROS_DOMAIN_ID? scheduler running?)')
        return 2
    text, healthy = render(box['status'], box['hosts'])
    print(text)
    return 0 if healthy else 1


if __name__ == '__main__':
    sys.exit(main())
