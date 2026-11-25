"""One owner of the arm's serial port at a time. Tools hold the lock for their whole run; the GUI takes it only for a short poll.

lock = acquire() returns None if another tool still holds it after `wait` seconds.
"""
import fcntl
import time

PATH = '/tmp/arm_serial.lock'


def acquire(wait=1.5):
    f = open(PATH, 'w')
    end = time.time() + wait
    while True:
        try:
            fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
            return f
        except BlockingIOError:
            if time.time() > end:
                f.close()
                return None
            time.sleep(0.05)
