"""Prompt format shared by inference and fine-tuning data generation."""

END = '</root>'


def normalize_spec(task_spec):
    """Canonical compact JSON: the exact text the SLM sees in training and at inference."""
    import json
    return json.dumps(json.loads(task_spec), separators=(',', ':'))


def build_prompt(task_spec, catalog_listing, previous_xml='', errors=()):
    p = f'### NODES\n{catalog_listing}\n### TASK_SPEC\n{task_spec}\n'
    if errors:
        p += f'### PREVIOUS_XML\n{previous_xml}\n### ERRORS\n' + '\n'.join(errors) + '\n'
    return p + '### BT_XML\n'


def prompt_for(task_spec, catalog, previous_xml='', errors=()):
    """The exact prompt the SLM gets at inference (and was trained on)."""
    return build_prompt(normalize_spec(task_spec), catalog.prompt_listing(), previous_xml, errors)


def extract_xml(text):
    i = text.find('<root')
    if i < 0:
        return text.strip()
    j = text.find(END, i)
    return text[i:j + len(END)] if j >= 0 else text[i:].strip()
