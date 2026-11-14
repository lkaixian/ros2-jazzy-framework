# SLM benchmark (`bt_slm`)

![SLM benchmark: held-out accuracy per training run and the v4 training loss](img/slm-benchmark.png)
([dark version](img/slm-benchmark-dark.png); data behind it: `slm-benchmark.json`; redraw with `python3 tools/slm/plot_benchmark.py`)

Table view of the left panel:

| Run | Training rows | Catalog | `--max-len` | Valid XML | Exact match |
|---|---|---|---|---|---|
| v1 | 2,865 | early catalog | 1024 | 99.0 % | 98.5 % |
| v2 | 472 | 42 nodes | 512 | 89.5 % | 79.5 % |
| v3 | 2,392 | 42 nodes | 768 | 97.5 % | 91.5 % |
| **v4** (current, `models/bt-slm`) | 2,835 | 45 nodes + Parallel/Repeat | 768 | **99.5 %** | **93.5 %** |

**How to read it**
- *Valid XML* = the validator accepts the generated tree; *exact* = identical to the rule-based generator's tree for the same spec. Each run is scored on
  its own 200 held-out rows of synthetic specs, so the catalogs differ between runs and the bars are **not strictly comparable**.
- It shows the model learned the rules. It does **not** show how it copes with real LLM output, and the rule-based generator runs first anyway
  (`rules_first`): the SLM only handles retries and specs the rules cannot render.
- Why v2 is low: only 472 of 4000 rows fit under 512 tokens (a 4 GB GPU at batch size 1 limits the sequence length). Why v4 uses fewer rows than v1:
  rows are longer now (45-node catalog), so fewer fit under 768 tokens.
- Loss panel: batch size 1, so each logged value is a single example; the bold line is a 5-point average.
- Speed (measured earlier, GTX 1650): rule-based generator 0.06 ms, SLM about 1.5 s per spec.

Next version: add a row to `slm-benchmark.json` (and the loss points) after each fine-tune, then re-run the plot script.
