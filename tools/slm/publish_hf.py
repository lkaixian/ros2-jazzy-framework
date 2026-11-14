#!/usr/bin/env python3
"""Upload the fine-tuned SLM to the Hugging Face Hub (private unless you pass --public).

  hf auth login                                  # once, in your own terminal (the token never goes through this script's arguments)
  python3 tools/slm/publish_hf.py --model models/bt-slm --repo <your-user>/<model-name> --valid 0.975 --exact 0.915
  python3 tools/slm/publish_hf.py ... --public   # only when you want it visible to everyone

Writes a model card and copies the base model's LFM Open License v1.0 (required by section 4 of that licence: give recipients a copy,
say the files were changed, keep the attribution). Commercial use above 10 M USD annual revenue is restricted by that licence.
Nothing is uploaded without a valid login; `--dry-run` only prints what would be uploaded.
"""
import argparse
import os
import shutil
import sys
import tempfile
import urllib.request

BASE = 'LiquidAI/LFM2.5-230M-Base'
CARD = """---
license: other
license_name: lfm1.0
license_link: LICENSE
base_model: {base}
tags: [text-generation, fine-tuned]
---
# {title}

{chart}A small causal LM fine-tuned from [{base}](https://huggingface.co/{base}) (**modified: fine-tuned, files changed**) to turn a JSON task spec
(plus a node catalog, the previous XML and validator errors on a retry) into BehaviorTree.CPP v4 XML.

- Training data: synthetic (task spec, XML) pairs labelled by a validator; rows longer than {max_len} tokens were skipped; 1 epoch.
- Held-out score against a rule-based generator on synthetic specs: valid {valid}, exact {exact}. This shows the model learned the rules;
  it is NOT evaluated on real LLM output.
{note}{project}- License: the base model's LFM Open License v1.0 (copy in `LICENSE`); attribution to Liquid AI, Inc. retained; commercial use is
  restricted above the license's revenue threshold.
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--model', default='models/bt-slm')
    ap.add_argument('--repo', required=True, help='<user-or-org>/<name>')
    ap.add_argument('--valid', default='n/a')
    ap.add_argument('--exact', default='n/a')
    ap.add_argument('--max-len', default='768')
    ap.add_argument('--public', action='store_true')
    ap.add_argument('--chart', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'docs', 'img', 'slm-benchmark.png'),
                    help='benchmark chart shown on the card (default docs/img/slm-benchmark.png if it exists)')
    ap.add_argument('--note', default='', help='one line about this version, shown on the model card')
    ap.add_argument('--project-url', help='optional: link the model card to the project that uses it (default: no link, nothing identifying)')
    ap.add_argument('--dry-run', action='store_true')
    a = ap.parse_args()
    if not os.path.exists(os.path.join(a.model, 'model.safetensors')):
        sys.exit(f'{a.model}/model.safetensors not found')
    from huggingface_hub import HfApi, get_token
    if not a.dry_run and not get_token():
        sys.exit('no Hugging Face login: run `hf auth login` in your terminal first')
    with tempfile.TemporaryDirectory() as tmp:
        stage = os.path.join(tmp, 'stage')
        shutil.copytree(a.model, stage)
        with urllib.request.urlopen(f'https://huggingface.co/{BASE}/raw/main/LICENSE', timeout=30) as r:
            open(os.path.join(stage, 'LICENSE'), 'wb').write(r.read())
        has_chart = os.path.exists(a.chart)
        if has_chart:
            shutil.copy(a.chart, os.path.join(stage, 'benchmark.png'))
        open(os.path.join(stage, 'README.md'), 'w').write(CARD.format(
            base=BASE, title=a.repo.split('/')[-1], chart='![benchmark: held-out accuracy per run and training loss](benchmark.png)\n\n' if has_chart else '', valid=a.valid, exact=a.exact, max_len=a.max_len,
            note=f'- This version: {a.note}\n' if a.note else '',
            project=f'- Used by: {a.project_url}\n' if a.project_url else ''))
        files = sorted(os.listdir(stage))
        print(f"{'would upload' if a.dry_run else 'uploading'} to {a.repo} (new repos: {'public' if a.public else 'private'}; an existing repo keeps its visibility): {files}")
        if a.dry_run:
            return
        api = HfApi()
        api.create_repo(a.repo, private=not a.public, exist_ok=True)
        api.upload_folder(folder_path=stage, repo_id=a.repo, commit_message='Fine-tuned bt-slm weights')
    print(f'done: https://huggingface.co/{a.repo}')


if __name__ == '__main__':
    main()
