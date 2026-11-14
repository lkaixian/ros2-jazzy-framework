"""Robot-capability policy applied to a task spec before it is planned (no ROS imports)."""
import json
import re

# Verbs that need the arm. 'take/bring me to ...' and 'take the guest ...' are escorting, which the base can do.
_ARM = re.compile(r"\b(?:take|bring|carry|fetch|deliver|put|place (?:the|a|an|it|this|that|my)|grab|pick up|hand|pour|throw|open|"
                  r"make (?:me|us)|get (?:me|us))\b", re.I)
_ESCORT = re.compile(r"\b(?:take|bring|lead|show|guide|escort|walk)\s+(?:me|us|them|him|her|"
                     r"(?:the |our |my )?(?:guests?|visitors?|person|people))\b(?!\s+(?:a|an|the|some)\b)", re.I)
REFUSAL = {'steps': [{'macro': 'say', 'args': {'text': 'Sorry, I have no arm and cannot do that'}}]}


def needs_arm(goal):
    """True when the goal asks to move, hand over or open something. shortcut: verb list on the model's restated goal, no parsing;
    add a verb here when a new arm request slips through."""
    return bool(_ARM.search(goal)) and not _ESCORT.search(goal)


def refuse_no_arm(task_spec):
    """The refusal spec (JSON text) when the spec's goal needs the arm, else None."""
    try:
        goal = str(json.loads(task_spec).get('goal', ''))
    except (ValueError, AttributeError):
        return None
    return json.dumps(dict(REFUSAL, goal=goal)) if needs_arm(goal) else None
