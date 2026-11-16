import json

import pytest

from bt_rehearsal.isolation import Observable
from bt_rehearsal.lessonbook import LessonBook, Recorder, situation_key, situations_markdown
from bt_rehearsal.llm import template_lesson


def fail(msg, node='NavigateToPlace'):
    return Observable({'objects': []}, (), {'result': 'FAILURE', 'failed_node': node, 'message': msg, 'steps': 3})


def test_key_blanks_quoted_values():
    assert situation_key(fail("unknown place 'doorway'")) == situation_key(fail("unknown place 'kitchen'"))
    assert situation_key(fail('timeout')) == 'NavigateToPlace|timeout'
    assert situation_key(Observable({}, ("[M] <Go> port 'x'='7' is not a number",))).startswith('validator|')


def test_recorder_and_markdown():
    r = Recorder()
    r.lesson('go', fail("unknown place 'a'"), 'Valid places: door.')
    r.lesson('go2', fail("unknown place 'b'"))
    assert r.seen["NavigateToPlace|unknown place '<x>'"]['count'] == 2
    md = situations_markdown(r.seen)
    assert "### key: NavigateToPlace|unknown place '<x>'" in md and 'JSON object' in md and 'Valid places: door.' in md


def test_book_lookup_fallback_and_lint(tmp_path):
    p = tmp_path / 'l.json'
    p.write_text(json.dumps({'NavigateToPlace|timeout': 'Leave out timeout_ms.'}))
    b = LessonBook(str(p))
    assert b.lesson('t', fail('timeout')) == 'Leave out timeout_ms.'
    other = fail('no path to goal')
    assert b.lesson('t', other) == template_lesson(other) and b.misses == {'NavigateToPlace|no path to goal'}
    p.write_text(json.dumps({'k': 'robot was at 1.5, 2.5'}))
    with pytest.raises(ValueError, match='lint'):
        LessonBook(str(p))
