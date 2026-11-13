import math

import pytest

from nav_extension import nav_logic as nl


def test_yaw_quaternion_roundtrip():
    for yaw in (-3.0, -1.0, 0.0, 0.7, 2.9):
        x, y, z, w = nl.yaw_to_quat(yaw)
        assert nl.quat_to_yaw(x, y, z, w) == pytest.approx(yaw)
        assert x * x + y * y + z * z + w * w == pytest.approx(1.0)


def test_bearing_and_distance():
    assert nl.bearing((0, 0), (1, 1)) == pytest.approx(math.pi / 4)
    assert nl.bearing((1, 1), (0, 1)) == pytest.approx(math.pi)
    assert nl.distance((0, 0), (3, 4)) == 5.0


def test_goal_yaw_cases():
    assert nl.goal_yaw((0, 0, 1.0), (5, 0), yaw=0.3) == 0.3                 # explicit wins
    assert nl.goal_yaw((0, 0, 1.0), (0, 5)) == pytest.approx(math.pi / 2)  # direction of travel
    assert nl.goal_yaw((2, 2, 1.25), (2.01, 2.0)) == 1.25                 # already there: keep
    assert nl.goal_yaw(None, (3, 3)) == 0.0                                 # TF unknown


def write(tmp_path, text):
    p = tmp_path / 'places.yaml'
    p.write_text(text)
    return str(p)


def test_load_places(tmp_path):
    places = nl.load_places(write(tmp_path, 'places:\n  a: {x: 1, y: 2, yaw: 0.5}\n  b-2: {x: 0, y: 0}\n'))
    assert places == {'a': (1.0, 2.0, 0.5), 'b-2': (0.0, 0.0, 0.0)}
    assert nl.places_listing(places) == 'a, b-2'
    assert nl.places_listing({}) == '(none configured)'


@pytest.mark.parametrize('text', [
    'nothing: 1\n',                                            # no places key
    'places: [1, 2]\n',                                        # wrong type
    'places:\n  bad name: {x: 1, y: 2}\n',                     # space in name
    'places:\n  ../x: {x: 1, y: 2}\n',                         # traversal-looking name
    'places:\n  a: {x: one, y: 2}\n',                          # non numeric
    'places:\n  a: {y: 2}\n',                                  # missing x
    'places:\n  a: {x: .nan, y: 2}\n',                         # not finite
    'places: {a: {x: 1, y: 2}\n',                              # broken yaml
])
def test_load_places_rejects_bad_files(tmp_path, text):
    with pytest.raises(nl.PlacesError):
        nl.load_places(write(tmp_path, text))


def test_missing_places_file_is_an_error_not_a_crash(tmp_path):
    with pytest.raises(nl.PlacesError):
        nl.load_places(str(tmp_path / 'nope.yaml'))


def test_map_path_stays_inside_map_dir(tmp_path):
    assert nl.map_base_path(str(tmp_path), 'room1') == str(tmp_path / 'room1')
    for bad in ('../escape', '/etc/passwd', 'a/b', '', 'x' * 61, 'a b', 'a.yaml'):
        with pytest.raises(ValueError):
            nl.map_base_path(str(tmp_path), bad)


def test_nav_result_mapping():
    assert nl.nav_result(4) == (True, 'arrived')
    assert nl.nav_result(5) == (False, 'cancelled')
    ok, msg = nl.nav_result(6, 105, 'Failed to find a valid path')
    assert not ok and 'aborted' in msg and 'valid path' in msg
    assert nl.nav_result(6, 7, '')[1].endswith('error code 7')
    assert not nl.nav_result(2)[0]                             # unexpected status is a failure
