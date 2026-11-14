import subprocess

import pytest

from cluster_core import sysinfo

STAT1 = 'cpu  1000 0 500 8000 100 0 0 0 0 0\ncpu0 1 2 3 4\nintr 5\n'
STAT2 = 'cpu  1500 0 700 8300 100 0 0 0 0 0\ncpu0 1 2 3 4\n'      # +700 busy, +300 idle
MEMINFO = 'MemTotal:       16000000 kB\nMemFree:         1000000 kB\nMemAvailable:    8000000 kB\nBuffers: 1 kB\n'


def test_cpu_meter_deltas():
    m = sysinfo.CpuMeter()
    assert m.update(STAT1) == 0.0                     # first sample: no baseline yet
    assert m.update(STAT2) == pytest.approx(0.7)      # 700 busy of 1000
    assert m.update(STAT2) == 0.0                     # no time passed: no division by zero


def test_cpu_meter_guest_fields_not_double_counted():
    m = sysinfo.CpuMeter()
    m.update('cpu 100 0 100 800 0 0 0 0 50 0\n')
    assert m.update('cpu 200 0 200 800 0 0 0 0 100 0\n') == pytest.approx(1.0)


def test_bad_proc_stat():
    with pytest.raises(ValueError):
        sysinfo.parse_proc_stat('intr 1 2 3\n')
    with pytest.raises(ValueError):
        sysinfo.parse_proc_stat('cpu 1 2\n')


def test_meminfo():
    total, avail = sysinfo.parse_meminfo(MEMINFO)
    assert total == pytest.approx(15625.0) and avail == pytest.approx(7812.5)
    # old kernels without MemAvailable fall back to MemFree
    assert sysinfo.parse_meminfo('MemTotal: 2048 kB\nMemFree: 1024 kB\n') == (2.0, 1.0)
    with pytest.raises(ValueError):
        sysinfo.parse_meminfo('MemFree: 1 kB\n')


def test_thermal_hottest_and_garbage_ignored():
    assert sysinfo.parse_thermal(['45000', '61500\n', 'x', '-274000', '999000']) == 61.5
    assert sysinfo.parse_thermal([]) == -1.0
    assert sysinfo.parse_thermal(['bad']) == -1.0


def test_nvidia_smi_parsing_and_best_gpu():
    assert sysinfo.parse_nvidia_smi('4096, 3662, 7\n') == (4096.0, 3662.0, 0.07)
    # several GPUs: the one with the most free memory
    assert sysinfo.parse_nvidia_smi('8000, 1000, 90\n8000, 6000, 5\n')[1] == 6000.0
    assert sysinfo.parse_nvidia_smi('') == (-1.0, -1.0, -1.0)
    assert sysinfo.parse_nvidia_smi('N/A, N/A, N/A') == (-1.0, -1.0, -1.0)


class Clock:
    t = 0.0

    def __call__(self):
        return self.t


def test_gpu_probe_caches_and_survives_failures():
    calls = []

    def run(*a, **k):
        calls.append(1)
        return subprocess.CompletedProcess(a, 0, stdout='4096, 3000, 10\n')
    clk = Clock()
    g = sysinfo.GpuProbe(cache_sec=5.0, run=run, clock=clk)
    assert g.read()[1] == 3000.0 and g.read()[1] == 3000.0
    assert len(calls) == 1                           # cached
    clk.t = 6.0
    g.read()
    assert len(calls) == 2


def test_gpu_probe_absent_binary_is_remembered():
    calls = []

    def run(*a, **k):
        calls.append(1)
        raise FileNotFoundError
    g = sysinfo.GpuProbe(run=run, clock=Clock())
    assert g.read() == (-1.0, -1.0, -1.0)
    g.clock.t = 100.0
    g.read()
    assert len(calls) == 1                           # does not keep forking a missing binary


def test_gpu_probe_timeout_gives_unknown():
    def run(*a, **k):
        raise subprocess.TimeoutExpired('nvidia-smi', 1)
    assert sysinfo.GpuProbe(run=run, clock=Clock()).read() == (-1.0, -1.0, -1.0)


def test_sampler_on_fake_proc_tree(tmp_path):
    (tmp_path / 'stat').write_text(STAT1)
    (tmp_path / 'meminfo').write_text(MEMINFO)
    zone = tmp_path / 'thermal' / 'thermal_zone0'
    zone.mkdir(parents=True)
    (zone / 'temp').write_text('55000\n')

    class NoGpu:
        def read(self):
            return (-1.0, -1.0, -1.0)
    s = sysinfo.Sampler(str(tmp_path), str(tmp_path / 'thermal'), NoGpu())
    a = s.sample()
    (tmp_path / 'stat').write_text(STAT2)
    b = s.sample()
    assert a['cpu_load'] == 0.0 and b['cpu_load'] == pytest.approx(0.7)
    assert b['temp_c'] == 55.0 and b['mem_free_mb'] == pytest.approx(7812.5) and b['gpu_mem_free_mb'] == -1.0


def test_sampler_real_machine_returns_sane_numbers():
    s = sysinfo.Sampler()
    s.sample()
    r = s.sample()
    assert r['cores'] >= 1 and 0.0 <= r['cpu_load'] <= 1.0 and r['mem_total_mb'] > 0


def test_parse_battery():
    assert sysinfo.parse_battery('87\n', 'Charging\n') == (87.0, 1)
    assert sysinfo.parse_battery('40', 'Discharging') == (40.0, 0)
    assert sysinfo.parse_battery('100', 'Full') == (100.0, 1)
    assert sysinfo.parse_battery('40', 'Weird') == (40.0, -1)
    assert sysinfo.parse_battery('150', 'Full') == (-1.0, -1)
    assert sysinfo.parse_battery('x', 'Full') == (-1.0, -1)


def test_read_battery_from_fake_sysfs(tmp_path):
    assert sysinfo.read_battery(str(tmp_path)) == (-1.0, -1)          # desktop: no battery
    b = tmp_path / 'BAT0'
    b.mkdir()
    (b / 'capacity').write_text('55\n')
    (b / 'status').write_text('Discharging\n')
    assert sysinfo.read_battery(str(tmp_path)) == (55.0, 0)


def test_host_service_name_is_a_valid_ros_name():
    f = sysinfo.host_service_name
    assert f('laptop-a') == 'host_info/laptop_a'
    assert f('11e1efbe9d79') == 'host_info/h11e1efbe9d79'      # container ids start with a digit (CI failure)
    assert f('') == 'host_info/host' and f('a.b c') == 'host_info/a_b_c'
