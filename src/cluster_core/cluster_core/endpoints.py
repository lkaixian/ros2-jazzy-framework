"""Services a host advertises in its heartbeat (pure helpers, no ROS)."""
import socket

RUNNING = 1     # cluster_msgs/JobStatus.RUNNING


def local_address():
    """The address other machines would reach this host on. A UDP connect() sends no packet."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('10.255.255.255', 1))
        return s.getsockname()[0]
    except OSError:
        return '127.0.0.1'
    finally:
        s.close()


def advertised(catalog, statuses, address):
    """[(job_id, kind, url, priority)] for RUNNING jobs whose catalog entry has `endpoint:`.

    A job that is starting, in BACKOFF, FAILED or stopping advertises nothing, so consumers drop
    it at once instead of waiting for the host to go silent.
    """
    out = []
    for st in statuses:
        jid = st['job_id']
        if st['state'] != RUNNING or jid not in catalog:
            continue
        ep = catalog[jid].get('endpoint')
        if ep:
            out.append((jid, ep['kind'], f"http://{address}:{ep['port']}", int(ep['priority'])))
    return out
