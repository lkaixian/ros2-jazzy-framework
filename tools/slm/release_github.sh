#!/bin/bash
# release_github.sh [TAG] [MODEL_DIR]: pack the SLM and attach it to a DRAFT GitHub release of this repo (nothing is public until you
# publish the draft on github.com). Needs `gh auth login`. GitHub release assets may be up to 2 GB; plain git files only 100 MB.
# The LFM Open License v1.0 of the base model is included, as the licence requires for redistribution.
set -euo pipefail
TAG="${1:-slm-$(date +%Y%m%d)}"; DIR="${2:-models/bt-slm}"
[[ -f "$DIR/model.safetensors" ]] || { echo "$DIR/model.safetensors not found" >&2; exit 2; }
TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT
cp -r "$DIR" "$TMP/bt-slm"
curl -fsSL https://huggingface.co/LiquidAI/LFM2.5-230M-Base/raw/main/LICENSE -o "$TMP/bt-slm/LICENSE"
printf 'Fine-tuned from LiquidAI/LFM2.5-230M-Base (modified: fine-tuned, files changed). LFM Open License v1.0, see LICENSE.\nUse: put these files in models/bt-slm/ of ros2-jazzy-framework or set ROBOCUP_SLM_MODEL.\n' > "$TMP/bt-slm/NOTICE.txt"
tar -C "$TMP" -czf "$TMP/bt-slm-$TAG.tar.gz" bt-slm
gh release create "$TAG" "$TMP/bt-slm-$TAG.tar.gz" --draft --title "Fine-tuned SLM $TAG" \
  --notes "Weights of the fine-tuned bt_slm model (LFM Open License v1.0 applies). Extract into models/ : tar -xzf bt-slm-$TAG.tar.gz -C models"
echo "draft release $TAG created; review and publish it on GitHub"
