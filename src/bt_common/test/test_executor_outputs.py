from bt_common.executor_node import unpack


def test_unpack_plain_result_has_no_outputs():
    assert unpack((True, 'ok')) == (True, 'ok', {})


def test_unpack_outputs_become_strings():
    assert unpack((True, 'ok', {'out_n': 3, 'out_t': 'x'})) == (True, 'ok', {'out_n': '3', 'out_t': 'x'})
