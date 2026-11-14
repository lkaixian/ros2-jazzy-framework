# LLM link, failover and models

## LLM link, failover and emergency halt
`llm_bridge` talks to `llama-server` over HTTP/TCP with one reused keep-alive connection and sends `Authorization: Bearer $LLM_API_KEY` when the variable is set (the llama-server machine exports `LLAMA_API_KEY`; the key never goes in YAML or argv). Endpoints are an ordered list: hosts that run an LLM job advertise it in their cluster heartbeat (`llm_big` = Gemma E4B on the strong PC, `llm_small` = Gemma E2B on the robot laptop, see `param/cluster_jobs.yaml`), a host that joins is used once its `/health` passes and one that goes silent is dropped after 4 s, and `server_urls` stay as static fallbacks. A per-endpoint circuit breaker skips dead servers. If **every** endpoint is down the bridge calls `bt_executor/halt` (repeated while the outage lasts) and `llm/status` goes `down`. `param/llm_knowledge.md` (robot limits, places, rules; guarded by a test against the node catalog and `places.yaml`) is appended to every planning prompt. Verified with fake servers and the real cluster agent/scheduler on one machine; a real `llama-server` and a second machine were not tried.

## Models
**Local setup that was verified (GTX 1650, 4 GB):** see `notes/RUNBOOK.md` "Local LLM + voice": prebuilt llama.cpp (CUDA 12.8), Gemma 4 E2B, `--reasoning off`, constrained JSON-schema decoding with the macro whitelist (`param/macros.yaml`), Piper TTS and faster-whisper STT. `./setup.sh --only llamacpp,voice --backend prebuilt` reproduces it.
* LLM: Gemma 4 E4B via `llama-server -hf ggml-org/gemma-4-E4B-it-GGUF` (needs a llama.cpp build with Gemma 4 support).
* SLM: `LiquidAI/LFM2.5-230M-Base`, fine-tuned on synthetic pairs labelled by the validator:
  ```
  ros2 run bt_slm make_bt_dataset --out-dir bt_dataset --n 30000        # generate + repair rows
  pip install transformers
  ros2 run bt_slm finetune_bt_slm --data-dir bt_dataset --out bt_slm_model
  ros2 launch framework_bringup framework.launch.py slm_model_path:=$PWD/bt_slm_model
  ```
  Until `slm_model_path` is set, a deterministic rule-based generator (the dataset's ground truth) is used, and with a model set it still runs first (`rules_first`). The spec grammar is sequence/fallback groups with retries plus macros; richer structure (Parallel, SubTree) needs grammar and data extensions. One fine-tune has been run (3000 synthetic rows, batch 1, 1 epoch on a GTX 1650; the default batch size does not fit 4 GB): 99 % valid / 98.5 % exact on 200 held-out rows, measured against the rule-based generator, so it shows the model learned the rules and nothing about real LLM output.
* YOLO11 weights (`yolo11n.pt`, `yolo11n-pose.pt`) download on first use; face weights must be supplied (`yolov11n-face.pt`).
