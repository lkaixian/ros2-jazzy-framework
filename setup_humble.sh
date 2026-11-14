#!/usr/bin/env bash
DISTRO=humble exec "$(dirname "${BASH_SOURCE[0]}")/setup.sh" "$@"
