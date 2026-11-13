import importlib.util
import os

spec = importlib.util.spec_from_file_location(
    'dash', os.path.join(os.path.dirname(__file__), '..', 'scripts', 'dash.py'))
dash = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dash)


def test_classify_priority():
    c = dash.classify
    assert c(False, None) == 'OFFLINE'
    assert c(False, 1.0, busy=True) == 'OFFLINE'          # gone beats everything
    assert c(True, 5.0, busy=True) == 'ERROR'             # recent error beats busy
    assert c(True, 31.0, busy=True) == 'BUSY'             # old errors expire
    assert c(True, None, down=True) == 'ERROR'
    assert c(True, None) == 'ONLINE'


def test_paint_and_plain():
    assert dash.paint('x', '31', False) == 'x'
    assert dash.paint('x', '31', True).startswith('\033[31m')


def test_expected_nodes_come_from_check_params():
    names = dash.expected_nodes()
    assert 'llm_bridge_node' in names and 'cmd_vel_mux_node' in names
