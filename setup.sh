#!/usr/bin/env bash
# One-shot setup for ros2-jazzy-robocup-fw: system + ROS + Python packages, llama.cpp (LLM
# runtime), the LLM, STT and TTS models you pick for this computer (menus, with "other") and the SLM base (LFM2.5-230M-Base), then the build.
#
#   ./setup.sh                      # everything, asks before starting
#   ./setup.sh --dry-run            # print every command, change nothing
#   ./setup.sh --only build         # a single step
#   ./setup.sh --skip models,llamacpp
#   ./setup.sh --yes --backend cuda --minimal
#
# Steps (in order): apt rosdep python llamacpp choose models voice build robot   (optional, never default: slm, slmweights)
# slmweights: download a published fine-tuned SLM instead of training one:  SLM_WEIGHTS_REPO=<user>/<repo> ./setup.sh --only slmweights
# robot: Kobuki base (turtlebot2_ros2_jazzy + kobuki_*), RPLIDAR (sllidar_ros2), audio_common into ROBOT_WS (~/ros2_ws), plus udev rules. Jazzy only.
# --backend prebuilt: download the official llama.cpp CUDA 12.8 release (no CUDA toolkit, no compiling).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ALL_STEPS=(apt rosdep python llamacpp choose models voice build robot)
DRY=0; YES=0; MINIMAL=0; BACKEND=auto
ONLY=""; SKIP=""
LLAMA_DIR="${LLAMA_DIR:-$HOME/llama.cpp}"
ROBOT_WS="${ROBOT_WS:-$HOME/ros2_ws}"   # robot drivers live outside this repo
ROS_DOMAIN_ID_CHOICE="${ROS_DOMAIN_ID_CHOICE:-}"
LLM_REPO="${LLM_REPO:-}"; STT_MODEL="${STT_MODEL:-}"; STT_DEVICE="${STT_DEVICE:-}"; STT_COMPUTE="${STT_COMPUTE:-}"; TTS_ENGINE="${TTS_ENGINE:-}"   # empty = the choose step picks (menu, or the hardware default with --yes)
CHOICE="$ROOT/models/choice.env"   # what choose picked (gitignored with models/)
SLM_BASE="${SLM_BASE:-LiquidAI/LFM2.5-230M-Base}"
MIN_LLAMA_BUILD=8637          # first llama.cpp build with Gemma 4 support
LLAMA_TAG="${LLAMA_TAG:-b12375}"   # the llama.cpp release this was tested with (CUDA 12.8 prebuilt); LLAMA_TAG=latest takes the newest
PY=/usr/bin/python3           # ROS uses the system Python; never a venv/conda one
DISTRO="${DISTRO:-jazzy}"     # jazzy (Ubuntu 24.04) or humble (Ubuntu 22.04, Python 3.10); setup_humble.sh sets it
LOCK="$ROOT/requirements-lock.txt"; [[ $DISTRO == humble ]] && LOCK="$ROOT/requirements-lock-humble.txt"

usage() { sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'; cat <<EOF

Options:
  --dry-run          print commands only
  --yes, -y          do not ask for confirmation
  --only a,b         run just these steps
  --skip a,b         skip these steps
  --minimal          skip the optional heavy Python extras (STT/TTS engines)
  --backend B        llama.cpp backend: auto|cpu|cuda|vulkan|prebuilt   (default auto: cuda if nvcc exists)
  --llama-dir DIR    llama.cpp checkout/build dir (default $LLAMA_DIR); LLAMA_TAG=<bNNNN|latest> pins the llama.cpp release (default $LLAMA_TAG)
  --llm-repo REPO    GGUF repo[:quant] for llama-server -hf (skips the LLM menu)
  --stt-model M      faster-whisper size name or CTranslate2 HF repo (skips the STT menu)
  --tts-engine E     espeak | piper | kokoro (skips the TTS menu)
  --ros-domain-id N  write export ROS_DOMAIN_ID=N into env.sh (default: ask; with --yes nothing is written)
  --slm-base REPO    base model to fine-tune (default $SLM_BASE)
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run) DRY=1 ;;
    --yes|-y) YES=1 ;;
    --minimal) MINIMAL=1 ;;
    --only) ONLY="$2"; shift ;;
    --skip) SKIP="$2"; shift ;;
    --backend) BACKEND="$2"; shift ;;
    --llama-dir) LLAMA_DIR="$2"; shift ;;
    --llm-repo) LLM_REPO="$2"; shift ;;
    --stt-model) STT_MODEL="$2"; shift ;;
    --tts-engine) TTS_ENGINE="$2"; shift ;;
    --ros-domain-id) ROS_DOMAIN_ID_CHOICE="$2"; shift ;;
    --slm-base) SLM_BASE="$2"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

log()  { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33mwarning:\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31merror:\033[0m %s\n' "$*" >&2; exit 1; }
# run CMD...: echo it, and execute unless --dry-run
run()  { printf '  $ %s\n' "$*"; [[ $DRY -eq 1 ]] || "$@"; }

in_list() { [[ ",$2," == *",$1,"* ]]; }
wanted() {
  local s="$1"
  if [[ -n "$ONLY" ]]; then in_list "$s" "$ONLY"; return; fi
  [[ "$s" == slm || "$s" == slmweights ]] && return 1   # optional steps: only via --only
  ! in_list "$s" "$SKIP"
}

[[ -n "$ONLY" ]] && for s in ${ONLY//,/ }; do
  [[ " ${ALL_STEPS[*]} slm slmweights " == *" $s "* ]] || die "unknown step '$s' (steps: ${ALL_STEPS[*]} slm slmweights)"
done

# ---------------------------------------------------------------- preflight
preflight() {
  log "Preflight"
  [[ $EUID -ne 0 ]] || die "run as your normal user (sudo is used where needed)"
  [[ -f /opt/ros/$DISTRO/setup.bash ]] || die "ROS 2 $DISTRO not found at /opt/ros/$DISTRO"
  [[ -x $PY ]] || die "$PY not found"
  . /etc/os-release
  local ubuntu=24.04; [[ $DISTRO == humble ]] && ubuntu=22.04
  [[ "${VERSION_ID:-}" == "$ubuntu" ]] || warn "$DISTRO is for Ubuntu $ubuntu (this is ${PRETTY_NAME:-unknown})"
  if [[ "$(command -v python3)" != "$PY" ]]; then
    warn "'python3' on PATH is $(command -v python3), not $PY (a venv/~/pyvenv.cfg can hide system"
    warn "packages from ROS). This script and env.sh always use $PY first."
  fi
  echo "  workspace: $ROOT"
}

confirm() {
  [[ $YES -eq 1 || $DRY -eq 1 ]] && return
  echo "About to run steps:"; for s in "${ALL_STEPS[@]}" slm slmweights; do wanted "$s" && echo "  - $s"; done
  echo "This installs system packages (sudo), pip packages into ~/.local, may download several GB"
  echo "of models, and builds llama.cpp and the ROS workspace."
  read -r -p "Continue? [y/N] " a; [[ "$a" =~ ^[Yy]$ ]] || exit 1
}

# ---------------------------------------------------------------- steps
step_apt() {
  log "apt: build tools, audio, JSON/curl dev libraries, ROS helpers"
  local pk=(build-essential cmake git curl ca-certificates libcurl4-openssl-dev nlohmann-json3-dev
            alsa-utils espeak-ng python3-pip python3-yaml python3-numpy python3-opencv python3-pytest
            python3-colcon-common-extensions python3-rosdep ros-$DISTRO-behaviortree-cpp)
  run sudo apt-get update
  run sudo apt-get install -y "${pk[@]}"
}

step_rosdep() {
  log "rosdep: ROS dependencies of every package in src/"
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then run sudo rosdep init; fi
  run rosdep update
  run rosdep install --from-paths "$ROOT/src" --ignore-src -r -y --rosdistro "$DISTRO"
}

step_python() {
  log "python: ultralytics (YOLO11), transformers (SLM), huggingface_hub (+ STT/TTS extras)"
  # Ubuntu 24.04 marks the system Python as externally managed; ROS nodes run on it, so the
  # packages must live there. --user keeps them out of /usr.
  local pip=("$PY" -m pip install --user -c "$LOCK")   # tested versions
  # Ubuntu 22.04's pip 22 does not know --break-system-packages (and has no externally-managed marker)
  [[ $DISTRO == humble ]] || pip+=(--break-system-packages)
  # matplotlib: ROS's apt matplotlib is built for NumPy 1 but pip brings NumPy 2, and Ultralytics
  # training imports matplotlib, so install a NumPy-2-compatible build into ~/.local
  local core=(ultralytics lap transformers huggingface_hub 'matplotlib>=3.9')
  local extras=(faster-whisper vosk piper-tts kokoro-onnx soundfile)
  run "${pip[@]}" "${core[@]}"
  if [[ $MINIMAL -eq 1 ]]; then
    echo "  (--minimal: skipping ${extras[*]})"
  else
    run "${pip[@]}" "${extras[@]}"
  fi
}

llama_build_number() { "$1" --version 2>&1 | sed -n 's/^version: \([0-9]\+\).*/\1/p' | head -1; }

step_llamacpp_prebuilt() {
  log "llama.cpp: prebuilt CUDA 12.8 release into $LLAMA_DIR (NVIDIA driver only, no toolkit)"
  local tag="$LLAMA_TAG"
  if [[ $tag == latest ]]; then
    tag="$(curl -fsS 'https://api.github.com/repos/ggml-org/llama.cpp/releases?per_page=1' | sed -n 's/.*"tag_name": *"\(b[0-9]*\)".*/\1/p' | head -1)"
    [[ $DRY -eq 1 ]] && tag="${tag:-bNNNN}"
  fi
  [[ -n "$tag" ]] || die "could not read the latest llama.cpp release tag"
  local base="https://github.com/ggml-org/llama.cpp/releases/download/$tag" f
  run mkdir -p "$LLAMA_DIR" "$HOME/.local/bin"
  for f in "llama-$tag-bin-ubuntu-cuda-12.8-x64" "cudart-llama-$tag-bin-ubuntu-cuda-12.8-x64"; do
    run curl -fL -o "$LLAMA_DIR/$f.tar.gz" "$base/$f.tar.gz"
    run tar -xzf "$LLAMA_DIR/$f.tar.gz" -C "$LLAMA_DIR"
    run rm -f "$LLAMA_DIR/$f.tar.gz"
  done
  if [[ $DRY -eq 0 ]]; then
    printf '#!/bin/sh\nD="%s"\nexport LD_LIBRARY_PATH="$D/llama-%s:$D/cudart-llama-%s-bin-ubuntu-cuda-12.8-x64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"\nexec "$D/llama-%s/llama-server" "$@"\n' \
      "$LLAMA_DIR" "$tag" "$tag" "$tag" > "$HOME/.local/bin/llama-server"
    chmod +x "$HOME/.local/bin/llama-server"
    echo "  llama-server: $("$HOME/.local/bin/llama-server" --version 2>&1 | head -1)"
  else echo "  (would write the llama-server wrapper in ~/.local/bin)"; fi
}

step_llamacpp() {
  [[ $BACKEND == prebuilt ]] && { step_llamacpp_prebuilt; return; }
  log "llama.cpp: llama-server (needs build >= $MIN_LLAMA_BUILD for Gemma 4)"
  local be="$BACKEND" flags=(-DCMAKE_BUILD_TYPE=Release -DLLAMA_CURL=ON -DLLAMA_USE_PREBUILT_UI=OFF)   # we use the API only; the HF-downloaded web UI can leave ui.cpp pointing at missing bundle*.js
  if [[ $be == auto ]]; then command -v nvcc >/dev/null && be=cuda || be=cpu; fi
  case "$be" in
    cpu) ;;
    cuda) command -v nvcc >/dev/null || die "--backend cuda needs the CUDA toolkit (nvcc)"; flags+=(-DGGML_CUDA=ON) ;;
    vulkan) flags+=(-DGGML_VULKAN=ON) ;;
    *) die "unknown backend '$be'" ;;
  esac
  [[ $be == cpu ]] && warn "building CPU-only; for a GPU install the CUDA toolkit and use --backend cuda (or vulkan)"
  local ref=(--branch "$LLAMA_TAG"); [[ $LLAMA_TAG == latest ]] && ref=()
  if [[ -d "$LLAMA_DIR/.git" ]]; then run git -C "$LLAMA_DIR" fetch --depth 1 origin "${ref[@]:+tag}" "${LLAMA_TAG/latest/HEAD}" && run git -C "$LLAMA_DIR" checkout -q FETCH_HEAD
  else run git clone --depth 1 "${ref[@]}" https://github.com/ggml-org/llama.cpp "$LLAMA_DIR"; fi
  run cmake -S "$LLAMA_DIR" -B "$LLAMA_DIR/build" "${flags[@]}"
  run cmake --build "$LLAMA_DIR/build" --config Release -j "$(nproc)" --target llama-server llama-cli
  run mkdir -p "$HOME/.local/bin"
  run ln -sf "$LLAMA_DIR/build/bin/llama-server" "$HOME/.local/bin/llama-server"
  run ln -sf "$LLAMA_DIR/build/bin/llama-cli" "$HOME/.local/bin/llama-cli"
  if [[ $DRY -eq 0 ]]; then
    local n; n="$(llama_build_number "$LLAMA_DIR/build/bin/llama-server" || true)"
    echo "  llama-server build: ${n:-unknown}"
    [[ -n "$n" && "$n" -ge $MIN_LLAMA_BUILD ]] || warn "build ${n:-?} < $MIN_LLAMA_BUILD: Gemma 4 may fail to load"
    case ":$PATH:" in *":$HOME/.local/bin:"*) ;; *) warn "add ~/.local/bin to PATH (env.sh does this)" ;; esac
  fi
}

# menu TITLE DEFAULT "label|value"...  -> MENU_VAL. A value of "other" asks for free text. Unattended: the default.
menu() {
  local title="$1" def="$2"; shift 2; local opts=("$@") i=1 ans
  if [[ $YES -eq 1 || $DRY -eq 1 || ! -t 0 ]]; then MENU_VAL="${opts[def-1]#*|}"; return; fi
  echo; echo "$title"
  for o in "${opts[@]}"; do printf '  %d) %s%s\n' "$i" "${o%%|*}" "$([[ $i -eq $def ]] && echo '   [default]')"; i=$((i + 1)); done
  read -r -p "Choice [$def]: " ans; ans="${ans:-$def}"
  [[ "$ans" =~ ^[0-9]+$ && $ans -ge 1 && $ans -le ${#opts[@]} ]] || die "bad choice: $ans"
  MENU_VAL="${opts[ans-1]#*|}"
  if [[ "$MENU_VAL" == other ]]; then read -r -p "  enter it: " MENU_VAL; [[ -n "$MENU_VAL" ]] || die "empty answer"; fi
}

# Models are chosen per computer: nothing machine-specific is committed (models/local.yaml overrides the param files).
step_choose() {
  local vram=0
  command -v nvidia-smi >/dev/null && vram="$(nvidia-smi --query-gpu=memory.total --format=csv,noheader,nounits 2>/dev/null | sort -n | tail -1)"
  vram="${vram:-0}"; [[ "$vram" =~ ^[0-9]+$ ]] || vram=0
  log "choose models (GPU memory detected: ${vram} MiB; 0 = no NVIDIA GPU, everything runs on the CPU)"
  [[ -f "$CHOICE" ]] && { local a=$LLM_REPO b=$STT_MODEL c=$STT_DEVICE d=$STT_COMPUTE e=$TTS_ENGINE; . "$CHOICE"
    LLM_REPO=${a:-$LLM_REPO}; STT_MODEL=${b:-$STT_MODEL}; STT_DEVICE=${c:-$STT_DEVICE}; STT_COMPUTE=${d:-$STT_COMPUTE}; TTS_ENGINE=${e:-$TTS_ENGINE}
    echo "  earlier choice from $CHOICE kept: delete it to choose again"; }
  local big=1; (( vram >= 8000 )) || big=2
  if [[ -z "$LLM_REPO" ]]; then
    menu "LLM (planner, runs in llama-server):" "$big" \
      "Gemma 4 E4B Q4_0, 4.3 GiB, needs a GPU with >= 8 GB|ggml-org/gemma-4-E4B-it-GGUF:Q4_0" \
      "Gemma 4 E2B, smaller, for 4 GB GPUs and slow CPUs|ggml-org/gemma-4-E2B-it-GGUF" \
      "other: a Hugging Face GGUF repo, optionally with :quant|other"
    LLM_REPO="$MENU_VAL"
  fi
  if [[ -z "$STT_MODEL" ]]; then
    local sd=4; (( vram >= 6000 )) && sd=1
    menu "Speech-to-text (faster-whisper; GPU entries get CUDA 12 libraries installed):" "$sd" \
      "large-v3-turbo, best accuracy, GPU|deepdml/faster-whisper-large-v3-turbo-ct2" \
      "distil-large-v3, English only, GPU|Systran/faster-distil-whisper-large-v3" \
      "small.en, CPU|small.en" \
      "base.en, CPU, smallest|base.en" \
      "other: a faster-whisper size name or a CTranslate2 Hugging Face repo|other"
    STT_MODEL="$MENU_VAL"
  fi
  if [[ -z "$STT_DEVICE" ]]; then
    local dd=2; (( vram >= 6000 )) && dd=1
    case "$STT_MODEL" in *.en|tiny|base|small|medium) dd=2 ;; esac
    menu "STT device for $STT_MODEL (falls back to the CPU at run time if CUDA is unusable):" "$dd" "GPU (cuda, int8_float16)|cuda:int8_float16" "CPU (int8)|cpu:int8"
    STT_DEVICE="${MENU_VAL%%:*}"; STT_COMPUTE="${MENU_VAL##*:}"
  fi
  if [[ -z "$TTS_ENGINE" ]]; then
    menu "Text-to-speech:" 1 \
      "Kokoro, natural voice, runs on the CPU|kokoro" \
      "Piper lessac, lighter|piper" \
      "espeak, robotic, no download|espeak" \
      "other: an engine name (mock) you configure by hand|other"
    TTS_ENGINE="$MENU_VAL"
  fi
  if [[ $DRY -eq 0 ]]; then
    mkdir -p "$ROOT/models"
    printf 'LLM_REPO=%q\nSTT_MODEL=%q\nSTT_DEVICE=%q\nSTT_COMPUTE=%q\nTTS_ENGINE=%q\n' "$LLM_REPO" "$STT_MODEL" "$STT_DEVICE" "$STT_COMPUTE" "$TTS_ENGINE" >"$CHOICE"
  fi
  echo "  LLM $LLM_REPO | STT $STT_MODEL on $STT_DEVICE/$STT_COMPUTE | TTS $TTS_ENGINE   (saved to $CHOICE)"
}

# The later steps also run alone (--only models): take what choose saved, else the hardware default.
load_choice() {
  [[ -n "$LLM_REPO" && -n "$STT_MODEL" && -n "$STT_DEVICE" && -n "$TTS_ENGINE" ]] && return
  step_choose >/dev/null
}

step_models() {
  load_choice
  log "models: LLM $LLM_REPO (via llama-server) and SLM base $SLM_BASE"
  local srv="$HOME/.local/bin/llama-server"; command -v llama-server >/dev/null && srv="$(command -v llama-server)"
  if [[ $DRY -eq 1 ]]; then
    echo "  \$ $srv -hf $LLM_REPO --port 18080   # downloads to ~/.cache/llama.cpp, waits for /health"
    echo "  \$ $PY -c 'snapshot_download(\"$SLM_BASE\", local_dir=\"$ROOT/models/slm-base\")'"
    return
  fi
  [[ -x "$srv" ]] || die "llama-server not found; run the llamacpp step first"
  # Using llama-server's own downloader fills the cache the runtime reads, and proves the model
  # actually loads and serves with this build.
  "$srv" -hf "$LLM_REPO" --port 18080 >"$ROOT/.llm_download.log" 2>&1 &
  local pid=$! waited=0
  trap 'kill $pid 2>/dev/null || true' EXIT
  until curl -sf http://127.0.0.1:18080/health >/dev/null; do
    kill -0 $pid 2>/dev/null || { tail -5 "$ROOT/.llm_download.log"; die "llama-server exited (gated repo? old build? see $ROOT/.llm_download.log)"; }
    (( waited % 60 == 0 )) && echo "  ... waiting for $LLM_REPO ($((waited / 60)) min) $(tail -c 120 "$ROOT/.llm_download.log" | tr '\r\n' '  ')"
    sleep 5; waited=$((waited + 5)); [[ $waited -lt 7200 ]] || die "timed out after 2 h"
  done
  echo "  LLM ready: llama-server served $LLM_REPO"
  kill $pid; wait $pid 2>/dev/null || true; trap - EXIT; rm -f "$ROOT/.llm_download.log"

  mkdir -p "$ROOT/models"
  "$PY" - "$SLM_BASE" "$ROOT/models/slm-base" <<'PYEOF' || die "SLM download failed (see message above)"
import sys
from huggingface_hub import snapshot_download
from huggingface_hub.errors import GatedRepoError, HfHubHTTPError
repo, dest = sys.argv[1:3]
try:
    p = snapshot_download(repo, local_dir=dest)
except GatedRepoError:
    sys.exit(f"{repo} is gated: accept its license on huggingface.co, then `hf auth login`")
except HfHubHTTPError as e:
    sys.exit(f"download of {repo} failed: {e}")
print("  SLM base saved to", p)
PYEOF
  warn "the SLM base is NOT usable for planning yet: fine-tune it first (./setup.sh --only slm)"
}

step_voice() {
  load_choice
  log "voice models: STT $STT_MODEL, TTS $TTS_ENGINE into $ROOT/models; settings for this computer go to models/local.yaml"
  if [[ $DRY -eq 1 ]]; then echo "  (dry run: nothing downloaded or written)"; return; fi
  if [[ $STT_DEVICE != cpu ]]; then   # ctranslate2 wants CUDA 12 cuBLAS + cuDNN 9; the system may only have CUDA 13
    local pip=("$PY" -m pip install --user); [[ $DISTRO == humble ]] || pip+=(--break-system-packages)
    run "${pip[@]}" nvidia-cublas-cu12 nvidia-cudnn-cu12
  fi
  "$PY" - "$ROOT/models" "$STT_MODEL" "$STT_DEVICE" "$STT_COMPUTE" "$TTS_ENGINE" "$LLM_REPO" <<'PYEOF' || die "voice model download failed (pip install faster-whisper huggingface_hub piper-tts kokoro-onnx first: python step)"
import glob, os, sys, urllib.request
import yaml
root, stt, dev, comp, tts, llm = sys.argv[1:7]
os.makedirs(root, exist_ok=True)
if '/' in stt:
    from huggingface_hub import snapshot_download
    snapshot_download(stt, cache_dir=root + "/stt")
    sub = 'models--' + stt.replace('/', '--')
else:
    from faster_whisper import WhisperModel
    WhisperModel(stt, device="cpu", compute_type="int8", download_root=root + "/stt")
    sub = 'models--Systran--faster-whisper-' + stt
snap = sorted(glob.glob(f"{root}/stt/{sub}/snapshots/*"))
# a local folder: a bare name makes faster-whisper contact huggingface.co first, which hung start-up
stt_node = {'model': snap[-1] if snap else stt, 'device': dev, 'compute_type': comp}
tts_node = {'engine': tts}
if tts == 'piper':
    from huggingface_hub import hf_hub_download
    d = "en/en_US/lessac/medium/en_US-lessac-medium.onnx"
    for f in (d, d + ".json"):
        hf_hub_download("rhasspy/piper-voices", f, local_dir=root + "/tts")
    tts_node['piper_model'] = f"{root}/tts/{d}"
elif tts == 'kokoro':
    base = "https://github.com/thewh1teagle/kokoro-onnx/releases/download/model-files-v1.0/"
    os.makedirs(root + "/tts/kokoro", exist_ok=True)
    for f in ("kokoro-v1.0.onnx", "voices-v1.0.bin"):
        if not os.path.exists(f"{root}/tts/kokoro/{f}"):
            urllib.request.urlretrieve(base + f, f"{root}/tts/kokoro/{f}")
    tts_node.update(kokoro_model=f"{root}/tts/kokoro/kokoro-v1.0.onnx", kokoro_voices=f"{root}/tts/kokoro/voices-v1.0.bin", voice='af_heart')
local = {'stt_node': {'ros__parameters': stt_node}, 'tts_node': {'ros__parameters': tts_node}, 'llama_server': {'ros__parameters': {'hf_repo': llm}}}   # no such node: ROS ignores it, pipeline.launch.py reads it
yaml.safe_dump(local, open(root + "/local.yaml", "w"), sort_keys=False)
print("  wrote", root + "/local.yaml")
PYEOF
}

step_build() {
  log "colcon build (system Python first on PATH)"
  run bash -c "cd '$ROOT' && export PATH=/usr/bin:\$PATH && . /opt/ros/$DISTRO/setup.bash && colcon build --symlink-install"
  if [[ $DRY -eq 0 ]]; then
    cat >"$ROOT/env.sh" <<EOF
# source this: system Python first, ROS, this workspace, and ~/.local/bin (llama-server)
export PATH=/usr/bin:\$HOME/.local/bin:\$PATH
. /opt/ros/$DISTRO/setup.bash
. "$ROOT/install/setup.bash"
EOF
    # exports beyond the above only when you answer: ROS_DOMAIN_ID (0-101 is the range DDS port numbers allow)
    local dom="$ROS_DOMAIN_ID_CHOICE"
    if [[ -z "$dom" && $YES -eq 0 && -t 0 ]]; then
      read -r -p "ROS_DOMAIN_ID for this workspace (0-101; Enter = write nothing, keep the shell's value '${ROS_DOMAIN_ID:-unset}'): " dom
    fi
    if [[ -n "$dom" ]]; then
      [[ "$dom" =~ ^[0-9]+$ && $dom -le 101 ]] || die "ROS_DOMAIN_ID must be 0-101, got '$dom'"
      echo "export ROS_DOMAIN_ID=$dom" >>"$ROOT/env.sh"
    fi
    echo "  wrote $ROOT/env.sh   ->  source it in every new terminal"
  fi
}

step_robot() {
  [[ $DISTRO == jazzy ]] || { warn "robot: the Kobuki fork is Jazzy only, skipping on $DISTRO"; return; }
  log "robot: Kobuki, RPLIDAR, audio_common into $ROBOT_WS/src"
  local src="$ROBOT_WS/src"
  run mkdir -p "$src"
  [[ -d $src/turtlebot2_ros2 ]] || run git clone -b jazzy https://github.com/lkaixian/turtlebot2_ros2_jazzy.git "$src/turtlebot2_ros2"
  [[ -d $src/sllidar_ros2 ]] || run git clone https://github.com/Slamtec/sllidar_ros2.git "$src/sllidar_ros2"
  [[ -d $src/audio_common ]] || run git clone https://github.com/mgonzs13/audio_common.git "$src/audio_common"
  # its own script: submodules (kobuki_*, ecl_*), apt/rosdep, /dev/kobuki udev rule, dialout, build of kobuki_node
  run bash "$src/turtlebot2_ros2/setup.sh"
  run bash -c "cd '$ROBOT_WS' && . /opt/ros/$DISTRO/setup.bash && rosdep install --from-paths src --ignore-src -r -y --rosdistro $DISTRO"
  # whole workspace (Kobuki, sllidar, audio_common); same flags as the Kobuki script so ecl is not reconfigured
  run bash -c "cd '$ROBOT_WS' && export PATH=/usr/bin:\$PATH && . /opt/ros/$DISTRO/setup.bash && colcon build --symlink-install --executor sequential --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -Wno-dev -DCMAKE_CXX_FLAGS=-Wno-error=overloaded-virtual"
  # /dev/rplidar (CP210x), the name framework_bringup expects
  if [[ $DRY -eq 1 ]]; then echo "  \$ write /etc/udev/rules.d/rplidar.rules (sudo)"; return; fi
  echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="rplidar"' \
    | sudo tee /etc/udev/rules.d/rplidar.rules >/dev/null
  sudo udevadm control --reload-rules && sudo udevadm trigger
  echo "  source $ROBOT_WS/install/setup.bash in every robot terminal (log out/in once for the dialout group)"
}

step_slmweights() {
  log "SLM weights: download a published fine-tuned model into $ROOT/models/bt-slm (license: LFM Open License v1.0, included with the files)"
  [[ -n "${SLM_WEIGHTS_REPO:-}" ]] || die "set SLM_WEIGHTS_REPO=<user>/<repo> (a Hugging Face repo made with tools/slm/publish_hf.py)"
  if [[ $DRY -eq 1 ]]; then echo "  \$ $PY -c 'snapshot_download(\"$SLM_WEIGHTS_REPO\", local_dir=\"$ROOT/models/bt-slm\")'"; return; fi
  "$PY" - "$SLM_WEIGHTS_REPO" "$ROOT/models/bt-slm" <<'PYEOF' || die "download failed (private repo? run `hf auth login`)"
import sys
from huggingface_hub import snapshot_download
print("  saved to", snapshot_download(sys.argv[1], local_dir=sys.argv[2]))
PYEOF
  echo "  bt_slm_node finds models/bt-slm by itself (or set ROBOCUP_SLM_MODEL)"
}

step_slm() {
  log "SLM: synthetic dataset + fine-tune $SLM_BASE (GPU strongly recommended)"
  run bash -c ". '$ROOT/env.sh' && cd '$ROOT' && ros2 run bt_slm make_bt_dataset --out-dir bt_dataset --n 30000"
  run bash -c ". '$ROOT/env.sh' && cd '$ROOT' && ros2 run bt_slm finetune_bt_slm --data-dir bt_dataset --base '$ROOT/models/slm-base' --out models/bt-slm"
  echo "  then: ros2 launch framework_bringup framework.launch.py slm_model_path:=$ROOT/models/bt-slm"
}

# ---------------------------------------------------------------- main
preflight
confirm
for s in "${ALL_STEPS[@]}" slm slmweights; do
  if wanted "$s"; then "step_$s"; else echo "skip: $s"; fi
done
log "done"
cat <<EOF
Next:
  source $ROOT/env.sh
  ros2 run framework_bringup check_params          # param files vs real nodes
  ros2 launch launch/framework.launch.py start_llama_server:=true
LLM runs from llama.cpp's cache ($LLM_REPO); the SLM falls back to the rule-based generator until
you fine-tune one (./setup.sh --only slm) and pass slm_model_path:=...
EOF
