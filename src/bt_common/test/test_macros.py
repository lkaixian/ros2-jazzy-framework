import pytest
from bt_common.macros import MAX_DEPTH, MacroError, describe, expand, load_macros

LIB = {
    'say': {'description': 'say it', 'params': {'text': {'required': True}},
            'steps': [{'node': 'Speak', 'ports': {'text': '${text}'}}], 'example_args': {}},
    'go': {'description': 'go', 'params': {'place': {'required': True}, 'speed': {'default': '0.2'}},
           'steps': [{'node': 'NavigateToPlace', 'ports': {'place': '${place}', 'speed': '${speed}'}}],
           'example_args': {}},
    'go_say': {'description': 'both', 'params': {'place': {}, 'text': {}},
               'steps': [{'macro': 'go', 'args': {'place': '${place}'}},
                         {'macro': 'say', 'args': {'text': '${text}'}}], 'example_args': {}},
    'loop': {'description': '', 'params': {}, 'steps': [{'macro': 'loop'}], 'example_args': {}},
    'a': {'description': '', 'params': {}, 'steps': [{'macro': 'b'}], 'example_args': {}},
    'b': {'description': '', 'params': {}, 'steps': [{'macro': 'a'}], 'example_args': {}},
    'typo': {'description': '', 'params': {}, 'steps': [{'node': 'Speak', 'ports': {'text': '${nope}'}}],
             'example_args': {}},
}


def spec(*steps):
    return {'goal': 'g', 'steps': list(steps)}


def test_spec_without_macros_is_returned_untouched():
    s = spec({'node': 'Speak', 'ports': {'text': 'hi'}})
    assert expand(s, LIB) is s


def test_expansion_substitutes_defaults_and_splices_into_the_parent():
    out = expand(spec({'macro': 'go', 'args': {'place': 'door'}}, {'node': 'AlwaysSuccess'}), LIB)
    assert out['steps'] == [
        {'node': 'NavigateToPlace', 'ports': {'place': 'door', 'speed': '0.2'}},
        {'node': 'AlwaysSuccess'}]


def test_nested_macros_and_macros_inside_groups():
    out = expand(spec({'group': 'Sequence', 'retry': 2,
                       'steps': [{'macro': 'go_say', 'args': {'place': 'a', 'text': 'hi'}}]}), LIB)
    inner = out['steps'][0]
    assert inner['group'] == 'Sequence' and inner['retry'] == 2
    assert [s['node'] for s in inner['steps']] == ['NavigateToPlace', 'Speak']
    assert inner['steps'][1]['ports']['text'] == 'hi'


def test_does_not_mutate_the_library_or_the_input():
    s = spec({'macro': 'say', 'args': {'text': 'x'}})
    expand(s, LIB)
    expand(s, LIB)
    assert LIB['say']['steps'][0]['ports']['text'] == '${text}' and 'macro' in s['steps'][0]


@pytest.mark.parametrize('call, text', [
    ({'macro': 'nope'}, 'unknown macro'),
    ({'macro': 'say'}, 'missing required'),
    ({'macro': 'say', 'args': {'text': 'x', 'extra': '1'}}, 'unknown args'),
    ({'macro': 'say', 'args': 'x'}, 'args must be an object'),
    ({'macro': 'say', 'args': {'text': 'x'}, 'retry': 3}, 'unexpected keys'),
    ({'macro': 'loop'}, 'calls itself'),
    ({'macro': 'a'}, 'calls itself'),
    ({'macro': 'typo'}, 'unknown parameter'),
])
def test_errors_are_specific_enough_for_the_llm_to_fix(call, text):
    with pytest.raises(MacroError, match=text):
        expand(spec(call), LIB)


def test_depth_limit():
    lib = {f'm{i}': {'description': '', 'params': {}, 'example_args': {},
                     'steps': [{'macro': f'm{i + 1}'}]} for i in range(MAX_DEPTH + 2)}
    lib[f'm{MAX_DEPTH + 2}'] = {'description': '', 'params': {}, 'example_args': {},
                                'steps': [{'node': 'AlwaysSuccess'}]}
    with pytest.raises(MacroError, match='deeper'):
        expand(spec({'macro': 'm0'}), lib)


def test_describe_marks_optional_args():
    assert describe(LIB).splitlines()[0].startswith('a() - ')
    assert 'go(place,speed?) - go' in describe(LIB)
    assert 'say(text) - say it' in describe(LIB)


def test_load_rejects_malformed_files(tmp_path):
    f = tmp_path / 'm.yaml'
    f.write_text('macros:\n  Bad-Name:\n    steps: [{node: X}]\n')
    with pytest.raises(MacroError, match='lower-case'):
        load_macros(str(f))
    f.write_text('macros:\n  ok:\n    steps: [{node: X, group: Sequence}]\n')
    with pytest.raises(MacroError, match='exactly one'):
        load_macros(str(f))
    f.write_text('macros:\n  ok:\n    steps: []\n')
    with pytest.raises(MacroError, match='non-empty'):
        load_macros(str(f))
    assert load_macros(str(tmp_path / 'missing.yaml')) == {}


def test_foreach_unrolls_once_per_value_and_expands_macros_inside():
    import pytest as _pytest
    from bt_common.macros import MacroError, expand
    lib = {'go': {'params': {'place': {'required': True}}, 'description': '', 'example_args': {},
                  'steps': [{'node': 'NavigateToPlace', 'ports': {'place': '${place}'}}]}}
    spec = {'goal': 'tour', 'steps': [{'foreach': ['a', 'b'], 'as': 'p',
                                       'steps': [{'macro': 'go', 'args': {'place': '${p}'}},
                                                 {'node': 'Speak', 'ports': {'text': 'at ${p}'}}]}]}
    got = expand(spec, lib)['steps']
    assert [s['ports'].get('place') or s['ports'].get('text') for s in got] == ['a', 'at a', 'b', 'at b']
    for bad in ({'foreach': [], 'as': 'p', 'steps': [{'node': 'X'}]},
                {'foreach': list('abcdefghi'), 'as': 'p', 'steps': [{'node': 'X'}]},
                {'foreach': ['a'], 'as': 'P!', 'steps': [{'node': 'X'}]},
                {'foreach': ['a'], 'as': 'p', 'steps': [{'node': 'X', 'ports': {'t': '${nope}'}}]}):
        with _pytest.raises(MacroError):
            expand({'goal': 'g', 'steps': [bad]}, lib)
