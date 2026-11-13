import numpy as np
import pytest

from face_recognition.gallery import Gallery, valid_name


@pytest.mark.parametrize('name', ['alice', 'Bob_2', 'a-b', 'x' * 40])
def test_valid_names(name):
    assert valid_name(name)


@pytest.mark.parametrize('name', ['', '../etc/passwd', 'a/b', 'a b', 'x' * 41, 'unknown',
                                  'UNKNOWN', 'a.npy', 'é'])
def test_invalid_names(name):
    assert not valid_name(name)


def test_add_persists_and_matches_immediately(tmp_path):
    g = Gallery(str(tmp_path / 'faces'))
    assert g.match(np.ones(8), 0.5) == ('unknown', 0.0)
    g.add('alice', np.ones(8))
    assert g.match(np.ones(8) * 3, 0.9)[0] == 'alice'            # scale-invariant cosine
    assert (tmp_path / 'faces' / 'alice.npy').exists()
    assert Gallery(str(tmp_path / 'faces')).names == ['alice']    # survives restart


def test_orthogonal_face_is_unknown_and_reenroll_replaces(tmp_path):
    g = Gallery(str(tmp_path))
    g.add('alice', np.array([1.0, 0, 0, 0]))
    assert g.match(np.array([0, 1.0, 0, 0]), 0.6)[0] == 'unknown'
    g.add('alice', np.array([0, 1.0, 0, 0]))
    assert g.names == ['alice'] and g.match(np.array([0, 1.0, 0, 0]), 0.6)[0] == 'alice'


def test_add_rejects_path_traversal_and_empty_dir(tmp_path):
    with pytest.raises(ValueError):
        Gallery(str(tmp_path)).add('../evil', np.ones(4))
    assert not list(tmp_path.parent.glob('evil*'))
    with pytest.raises(ValueError):
        Gallery('').add('alice', np.ones(4))
