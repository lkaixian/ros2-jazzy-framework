"""Machine metrics from /proc, /sys and (optionally) nvidia-smi. No psutil: runs on Pi/Jetson.

Parsers take text so they can be unit-tested with fake files. Every reader degrades to
"unknown" values (-1) instead of raising.
"""
import glob
import os
import subprocess
import time


def parse_proc_stat(text):
    """First 'cpu ' line of /proc/stat -> (idle_jiffies, total_jiffies)."""
    for line in text.splitlines():
        if line.startswith('cpu '):
            f = [int(x) for x in line.split()[1:]]
            if len(f) < 4:
                break
            idle = f[3] + (f[4] if len(f) > 4 else 0)          # idle + iowait
            total = sum(f[:8]) if len(f) >= 8 else sum(f)      # skip guest fields (already in user)
            return idle, total
    raise ValueError('no aggregate cpu line in /proc/stat text')


class CpuMeter:
    """Whole-machine load 0..1 from successive /proc/stat samples."""

    def __init__(self):
        self.prev = None

    def update(self, stat_text):
        idle, total = parse_proc_stat(stat_text)
        load = 0.0
        if self.prev is not None:
            d_total, d_idle = total - self.prev[1], idle - self.prev[0]
            if d_total > 0:
                load = min(1.0, max(0.0, 1.0 - d_idle / d_total))
        self.prev = (idle, total)
        return load


def parse_meminfo(text):
    """-> (total_mb, available_mb); falls back to MemFree when MemAvailable is missing."""
    kb = {}
    for line in text.splitlines():
        k, _, v = line.partition(':')
        if v.strip():
            kb[k.strip()] = int(v.split()[0])
    if 'MemTotal' not in kb:
        raise ValueError('MemTotal missing in /proc/meminfo text')
    avail = kb.get('MemAvailable', kb.get('MemFree', 0))
    return kb['MemTotal'] / 1024.0, avail / 1024.0


def parse_thermal(values):
    """List of thermal_zone temp strings/ints (millidegrees C) -> hottest in C, -1 if none."""
    best = -1.0
    for v in values:
        try:
            c = int(v) / 1000.0
        except (TypeError, ValueError):
            continue
        if -50.0 < c < 150.0:                      # discard bogus sensors
            best = max(best, c)
    return best


def parse_nvidia_smi(csv):
    """'memory.total,memory.free,utilization.gpu' csv (nounits) -> (total_mb, free_mb, util 0..1).

    Several GPUs: the one with the most free memory (the one a new job would use)."""
    best = None
    for line in csv.strip().splitlines():
        parts = [p.strip() for p in line.split(',')]
        if len(parts) < 3:
            continue
        try:
            total, free, util = float(parts[0]), float(parts[1]), float(parts[2]) / 100.0
        except ValueError:
            continue
        if best is None or free > best[1]:
            best = (total, free, util)
    return best if best else (-1.0, -1.0, -1.0)


def read_text(path):
    with open(path) as f:
        return f.read()


def read_thermal(base='/sys/class/thermal'):
    vals = []
    for p in glob.glob(os.path.join(base, 'thermal_zone*', 'temp')):
        try:
            vals.append(read_text(p).strip())
        except OSError:
            pass
    return parse_thermal(vals)


class GpuProbe:
    """nvidia-smi, cached and time-limited so a hung driver cannot stall heartbeats."""

    CMD = ['nvidia-smi', '--query-gpu=memory.total,memory.free,utilization.gpu',
           '--format=csv,noheader,nounits']

    def __init__(self, cache_sec=5.0, timeout=1.0, run=subprocess.run, clock=time.monotonic):
        self.cache_sec, self.timeout, self.run, self.clock = cache_sec, timeout, run, clock
        self.value = (-1.0, -1.0, -1.0)
        self.t = None
        self.absent = False

    def read(self):
        now = self.clock()
        if self.absent or (self.t is not None and now - self.t < self.cache_sec):
            return self.value
        self.t = now
        try:
            out = self.run(self.CMD, capture_output=True, text=True, timeout=self.timeout)
            self.value = parse_nvidia_smi(out.stdout) if out.returncode == 0 else (-1.0,) * 3
        except FileNotFoundError:
            self.absent = True                     # no NVIDIA driver here: stop asking
            self.value = (-1.0, -1.0, -1.0)
        except (subprocess.TimeoutExpired, OSError):
            self.value = (-1.0, -1.0, -1.0)
        return self.value


class Sampler:
    """Everything a heartbeat needs, from the real /proc (paths injectable for tests)."""

    def __init__(self, proc='/proc', thermal='/sys/class/thermal', gpu=None):
        self.proc, self.thermal = proc, thermal
        self.cpu = CpuMeter()
        self.gpu = gpu or GpuProbe()
        self.cores = os.cpu_count() or 1

    def sample(self):
        load = self.cpu.update(read_text(os.path.join(self.proc, 'stat')))
        total, avail = parse_meminfo(read_text(os.path.join(self.proc, 'meminfo')))
        g_total, g_free, g_util = self.gpu.read()
        return {'cores': self.cores, 'cpu_load': load, 'mem_total_mb': total,
                'mem_free_mb': avail, 'gpu_mem_total_mb': g_total, 'gpu_mem_free_mb': g_free,
                'gpu_util': g_util, 'temp_c': read_thermal(self.thermal)}
