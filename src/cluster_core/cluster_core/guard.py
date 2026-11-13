"""guard: run an argv as a child and tear the whole process group down if our parent dies.

    python3 -m cluster_core.guard -- ros2 run pkg exe ...

The agent starts each job process through this wrapper (as a session/group leader). If the agent
is killed with SIGKILL, the guard notices the parent change within ~0.2 s and kills the group, so a
job can never keep running after its agent is gone (this is what makes failover safe: no
duplicate instance survives on a host the scheduler has written off).
"""
import os
import signal
import subprocess
import sys
import time

POLL = 0.1
TERM_GRACE = 3.0


def _killpg(sig, child):
    """Signal the whole job group, but ONLY if we lead it (the supervisor starts us with
    start_new_session). Run by hand from a shell we would otherwise signal the shell's group."""
    try:
        if os.getpgrp() == os.getpid():
            os.killpg(os.getpgrp(), sig)
        else:
            child.send_signal(sig)
    except ProcessLookupError:
        pass


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv and argv[0] == '--':
        argv = argv[1:]
    if not argv:
        print('usage: python3 -m cluster_core.guard -- COMMAND [ARGS...]', file=sys.stderr)
        return 2
    parent = os.getppid()
    try:
        child = subprocess.Popen(argv)
    except OSError as e:
        print(f'guard: cannot start {argv[0]!r}: {e}', file=sys.stderr)
        return 127
    stop = {'now': False}

    def on_term(_sig, _frm):
        stop['now'] = True
    signal.signal(signal.SIGTERM, on_term)
    signal.signal(signal.SIGINT, on_term)

    while True:
        rc = child.poll()
        if rc is not None:
            return rc if rc >= 0 else 128 - rc
        if stop['now'] or os.getppid() != parent:
            break
        time.sleep(POLL)

    # shut the whole group down (the guard ignores its own SIGTERM while doing so)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    _killpg(signal.SIGTERM, child)
    end = time.monotonic() + TERM_GRACE
    while child.poll() is None and time.monotonic() < end:
        time.sleep(POLL)
    if child.poll() is None:
        _killpg(signal.SIGKILL, child)                # also ends the guard itself
    return 1


if __name__ == '__main__':
    sys.exit(main())
