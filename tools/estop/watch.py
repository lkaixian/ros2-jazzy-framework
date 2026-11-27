#!/usr/bin/env python3
"""Show the e-stop state on one line, printing only when it changes. Ctrl+C to quit.   watch.py [port]"""
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/estop'
NAMES = {'E0': 'ARMED   (contact closed)', 'E1': 'STOPPED (pressed, or wire open)'}
last = None
with serial.Serial(PORT, 115200, timeout=0.5) as s:
    while True:
        ln = s.readline().strip().decode(errors='replace')
        if ln in NAMES and ln != last:
            print(time.strftime('%H:%M:%S'), NAMES[ln], flush=True)
            last = ln
