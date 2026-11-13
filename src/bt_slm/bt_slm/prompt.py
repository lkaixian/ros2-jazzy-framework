"""Prompt format shared by inference and fine-tuning data generation."""

END = '</root>'


def build_prompt(task_spec, catalog_listing, previous_xml='', errors=()):
    p = f'### NODES\n{catalog_listing}\n### TASK_SPEC\n{task_spec}\n'
    if errors:
        p += f'### PREVIOUS_XML\n{previous_xml}\n### ERRORS\n' + '\n'.join(errors) + '\n'
    return p + '### BT_XML\n'


def extract_xml(text):
    i = text.find('<root')
    if i < 0:
        return text.strip()
    j = text.find(END, i)
    return text[i:j + len(END)] if j >= 0 else text[i:].strip()
