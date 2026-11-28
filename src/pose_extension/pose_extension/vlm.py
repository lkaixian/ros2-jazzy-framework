"""Describe a person with a vision-language model behind an OpenAI-style chat endpoint (llama-server running Gemma 4 E4B with its mmproj, or any vision model). ROS-free.

One image in, a fixed JSON out: clothes, hair, glasses, gender, age. About 1.2 s per look on the RTX 3080 laptop with Gemma 4 E4B Q4_0 (measured on person crops). The model can
be wrong or invent things: the node votes over looks, keeps the detectors' clothes colour when they disagree, and says nothing when the looks do not agree.
Needs a server with vision: `llama-server -hf ggml-org/gemma-4-E4B-it-GGUF:Q4_0` loads the mmproj itself (look for "loaded multimodal model" in its log).
"""
import base64
import json
import re
import urllib.error
import urllib.request

import cv2

PROMPT = ('Describe the person in this photo as JSON with exactly these keys: top_colour, top_type, bottom_colour, hair_colour, hair_style, glasses (yes or no), '
          'gender (man or woman), age_estimate (a number). Colours are one plain word. Only what you can see; use null for anything not visible. Output only the JSON.')


def parse(text):
    """The JSON the model wrote -> {'top': 'black', 'top_type': 't-shirt', 'bottom': '', 'hair': 'black', 'hair_style': 'short', 'glasses': 'yes'|'no'|'',
    'gender': 'man'|'woman'|'', 'age': int|0}; {} when there is no JSON object in the reply (code fences and chatter around it are ignored)."""
    m = re.search(r'\{.*\}', text, re.S)
    if not m:
        return {}
    try:
        d = json.loads(m.group(0))
    except ValueError:
        return {}
    word = lambda v: str(v).strip().lower() if isinstance(v, (str, int)) and str(v).strip().lower() not in ('', 'null', 'none', 'unknown', 'n/a') else ''    # noqa: E731
    gender = word(d.get('gender'))
    gender = 'man' if gender in ('man', 'male', 'boy') else 'woman' if gender in ('woman', 'female', 'girl') else ''
    glasses = word(d.get('glasses'))
    try:
        age = int(float(str(d.get('age_estimate', d.get('age', 0))).split('-')[0]))
    except ValueError:
        age = 0
    return {'top': word(d.get('top_colour')), 'top_type': word(d.get('top_type')), 'bottom': word(d.get('bottom_colour')), 'hair': word(d.get('hair_colour')),
            'hair_style': word(d.get('hair_style')), 'glasses': 'yes' if glasses in ('yes', 'true') else 'no' if glasses in ('no', 'false') else '',
            'gender': gender, 'age': age if 1 <= age <= 110 else 0}


def combine(looks, min_share=1.0):
    """Several parse() results -> one: gender only when at least `min_share` of the looks that gave one agree (default: all), age = median of the plausible ones, every other
    key its most common non-empty value. {} when no look gave anything."""
    looks = [l for l in looks if l]
    if not looks:
        return {}
    out = {}
    for k in ('top', 'top_type', 'bottom', 'hair', 'hair_style', 'glasses'):
        vals = [l[k] for l in looks if l.get(k)]
        out[k] = max(set(vals), key=vals.count) if vals else ''
    g = [l['gender'] for l in looks if l.get('gender')]
    best = max(set(g), key=g.count) if g else ''
    out['gender'] = best if g and g.count(best) / len(g) >= min_share else ''
    ages = sorted(l['age'] for l in looks if l.get('age'))
    out['age'] = ages[len(ages) // 2] if ages else 0
    return out


def sentence(a):
    """'a man of about 25 wearing a black t-shirt, with short black hair and glasses' from a combine() result; '' when nothing certain."""
    who = ('a ' + a['gender'] if a.get('gender') else 'a person') + (f' of about {a["age"]}' if a.get('age') else '')
    wear = ' '.join(w for w in (a.get('top'), a.get('top_type')) if w)
    parts = [who + (f' wearing a {wear}' if wear else '')]
    withs = [' '.join(w for w in (a.get('hair_style'), a.get('hair'), 'hair') if w)] if a.get('hair') or a.get('hair_style') else []
    if a.get('glasses') == 'yes':
        withs.append('glasses')
    if withs:
        parts.append('with ' + ' and '.join(withs))
    s = ', '.join(parts)
    return s if (a.get('gender') or a.get('top') or a.get('hair') or a.get('glasses') == 'yes') else ''


def ask(url, bgr, timeout=20.0, prompt=PROMPT):
    """One look: the model's raw reply for a BGR image. Raises RuntimeError with a readable reason (no server, no vision, timeout)."""
    ok, buf = cv2.imencode('.jpg', bgr)
    if not ok:
        raise RuntimeError('cannot encode the image')
    body = {'model': 'default', 'max_tokens': 200, 'temperature': 0.1, 'messages': [{'role': 'user', 'content': [
        {'type': 'image_url', 'image_url': {'url': 'data:image/jpeg;base64,' + base64.b64encode(buf.tobytes()).decode()}}, {'type': 'text', 'text': prompt}]}]}
    req = urllib.request.Request(url.rstrip('/') + '/v1/chat/completions', json.dumps(body).encode(), {'Content-Type': 'application/json'})
    try:
        return json.load(urllib.request.urlopen(req, timeout=timeout))['choices'][0]['message']['content']
    except urllib.error.HTTPError as e:
        raise RuntimeError(f'the vision server refused the image ({e.code}): is it started with a multimodal model (mmproj)?') from None
    except (urllib.error.URLError, OSError) as e:
        raise RuntimeError(f'no vision server at {url}: {e}') from None
    except (KeyError, ValueError, IndexError):
        raise RuntimeError('the vision server gave an unreadable reply') from None
