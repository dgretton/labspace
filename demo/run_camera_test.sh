#!/bin/bash
# Handheld-camera ray test (camera_ray_test.py): your right hand is a camera; the Gauss-Newton aligner back-solves it.
# Usage:  ./run_camera_test.sh [--depth-sigma 0.5] [--pixel-sigma 2] [--fresh-noise] [--convergence-time 0.3]
# Needs the Quest arm-demo app running and plugged in. Stops a running follow_hand.py (same ports).
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$HERE/../../../nestbox"
PY="${NESTBOX_PY:-$HOME/.venvs/nestbox/bin/python}"
export PYTHONPATH="$REPO/python:$HERE"
export PYTHONDONTWRITEBYTECODE=1
[ "$(git -C "$REPO" branch --show-current)" = "armdemo" ] || echo "warning: nestbox repo is not on branch armdemo"
pkill -INT -f "follow_hand.py" 2>/dev/null && sleep 1 || true
pkill -INT -f "camera_ray_test.py" 2>/dev/null && sleep 1 || true
adb reverse tcp:12345 tcp:12345 >/dev/null && adb reverse tcp:8651 tcp:8651 >/dev/null && echo "adb reverse: 12345 (hands), 8651 (overlay)"
cd "$HERE"
"$PY" -u camera_ray_test.py "$@"
