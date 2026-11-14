"""Fine-tune a small causal LM (e.g. LiquidAI/LFM2.5-230M-Base) on bt_slm datasets.

Plain PyTorch loop (no Trainer) so it does not depend on Trainer API versions. Loss is on
the completion only. Requires: pip install transformers (torch is already present).
"""
import argparse
import json
import os
import random


def encode(tok, prompt, completion, max_len):
    """-> (input_ids, labels) with prompt tokens masked (-100), or None if too long."""
    p = tok(prompt)['input_ids']
    c = tok(completion, add_special_tokens=False)['input_ids'] + [tok.eos_token_id]
    if len(p) + len(c) > max_len:
        return None
    return p + c, [-100] * len(p) + c


def collate(batch, pad_id):
    import torch
    n = max(len(i) for i, _ in batch)
    ids = torch.full((len(batch), n), pad_id, dtype=torch.long)
    lab = torch.full((len(batch), n), -100, dtype=torch.long)
    att = torch.zeros((len(batch), n), dtype=torch.long)
    for k, (i, l) in enumerate(batch):
        ids[k, :len(i)], lab[k, :len(l)], att[k, :len(i)] = (
            torch.tensor(i), torch.tensor(l), 1)
    return ids, lab, att


def load_rows(path, tok, max_len, limit=0):
    rows, skipped = [], 0
    with open(path) as f:
        for line in f:
            if limit and len(rows) + skipped >= limit:
                break
            r = json.loads(line)
            e = encode(tok, r['prompt'], r['completion'], max_len)
            if e is None:
                skipped += 1
            else:
                rows.append(e)
    if skipped:
        print(f'{path}: skipped {skipped} rows longer than {max_len} tokens')
    return rows


def evaluate(model, tok, val_path, cat, n, max_new, device):
    """Greedy-decode n validation prompts; report validator pass rate and exact match."""
    import torch
    from bt_validator.validate import validate_xml

    from bt_slm.prompt import extract_xml
    ok = exact = total = 0
    with open(val_path) as f:
        rows = [json.loads(line) for line in f][:n]
    model.eval()
    for r in rows:
        ids = tok(r['prompt'], return_tensors='pt').to(device)
        with torch.no_grad():
            out = model.generate(**ids, max_new_tokens=max_new, do_sample=False,
                                 pad_token_id=tok.eos_token_id)
        xml = extract_xml(tok.decode(out[0][ids['input_ids'].shape[1]:], skip_special_tokens=True))
        total += 1
        ok += not validate_xml(xml, cat)
        exact += xml.strip() == r['completion'].strip()
    return {'n': total, 'valid_rate': ok / max(1, total), 'exact_rate': exact / max(1, total)}


def main(args=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--data-dir', default='bt_dataset',
                    help='one dir, or several joined by commas, each optionally DIR:N to use only its first N train rows')
    ap.add_argument('--base', default='LiquidAI/LFM2.5-230M-Base')
    ap.add_argument('--out', default='bt_slm_model')
    ap.add_argument('--epochs', type=int, default=3)
    ap.add_argument('--batch-size', type=int, default=8)
    ap.add_argument('--lr', type=float, default=5e-5)
    ap.add_argument('--max-len', type=int, default=1536)
    ap.add_argument('--eval-n', type=int, default=200)
    ap.add_argument('--max-new', type=int, default=1024, help='tokens the evaluation may generate (heavy plans need 3000+)')
    ap.add_argument('--optim', choices=['adamw', 'adafactor'], default='adamw', help='adafactor: almost no optimizer state (4 GB GPUs)')
    ap.add_argument('--grad-ckpt', action='store_true', help='gradient checkpointing: slower, but long rows fit a 4 GB GPU')
    ap.add_argument('--seed', type=int, default=0)
    a = ap.parse_args(args)
    try:
        import torch
        from transformers import AutoModelForCausalLM, AutoTokenizer
    except ImportError as e:
        raise SystemExit(f'missing dependency ({e}); pip install transformers') from e
    from bt_slm.dataset import discover_catalog

    random.seed(a.seed)
    torch.manual_seed(a.seed)
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    tok = AutoTokenizer.from_pretrained(a.base)
    if tok.pad_token_id is None:
        tok.pad_token = tok.eos_token
    model = AutoModelForCausalLM.from_pretrained(a.base).to(device)
    print(f'{a.base}: {sum(p.numel() for p in model.parameters()) / 1e6:.0f}M params on {device}')
    if a.grad_ckpt:
        model.gradient_checkpointing_enable()
        model.config.use_cache = False
    dirs = [d.partition(':')[::2] for d in a.data_dir.split(',')]          # [(dir, 'N' or '')]
    rows = [r for d, n in dirs for r in load_rows(os.path.join(d, 'train.jsonl'), tok, a.max_len, int(n or 0))]
    steps = a.epochs * ((len(rows) + a.batch_size - 1) // a.batch_size)
    if a.optim == 'adafactor':
        from transformers.optimization import Adafactor
        opt = Adafactor(model.parameters(), lr=a.lr, scale_parameter=False, relative_step=False, warmup_init=False)
    else:
        opt = torch.optim.AdamW(model.parameters(), lr=a.lr, weight_decay=0.01)
    sched = torch.optim.lr_scheduler.LambdaLR(
        opt, lambda s: min(1.0, (s + 1) / 50) * max(0.0, 1 - s / max(1, steps)))
    amp = torch.autocast(device, dtype=torch.bfloat16, enabled=device == 'cuda')
    step = 0
    model.train()
    for ep in range(a.epochs):
        random.shuffle(rows)
        for i in range(0, len(rows), a.batch_size):
            ids, lab, att = (t.to(device) for t in collate(rows[i:i + a.batch_size], tok.pad_token_id))
            with amp:
                loss = model(input_ids=ids, attention_mask=att, labels=lab).loss
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            opt.step()
            sched.step()
            opt.zero_grad(set_to_none=True)
            step += 1
            if step % 10 == 0:
                print(f'epoch {ep + 1} step {step}/{steps} loss {loss.item():.4f}')
    model.save_pretrained(a.out)
    tok.save_pretrained(a.out)
    print('saved', a.out)
    model.config.use_cache = True
    for d, _ in dirs:
        print('eval', d, evaluate(model, tok, os.path.join(d, 'val.jsonl'), discover_catalog(), a.eval_n, a.max_new, device))


if __name__ == '__main__':
    main()
