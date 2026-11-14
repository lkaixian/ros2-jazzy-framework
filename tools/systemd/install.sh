#!/bin/bash
# install.sh [--enable]: install the user units (no sudo). With --enable they start now and after every login/boot
# (run `sudo loginctl enable-linger $USER` once so they also start without a login).
set -euo pipefail
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$HOME/.config/systemd/user"
cp "$D"/robocup-llm.service "$D"/robocup-stack.service "$HOME/.config/systemd/user/"
systemctl --user daemon-reload
if [[ "${1:-}" == "--enable" ]]; then systemctl --user enable --now robocup-llm.service robocup-stack.service; fi
echo "installed; status: systemctl --user status robocup-stack"
