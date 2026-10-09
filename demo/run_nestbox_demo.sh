#!/bin/bash
# One-shot launcher: Redis + a fresh nestbox daemon (armdemo branch, TCP :8650) + adb ports + follow_hand.py --nestbox.
# Usage:  ./run_nestbox_demo.sh --arm-host 100.78.66.117 [--load-pairs data/cal_pairs.json]
# Python: set NESTBOX_PY to a venv with nestbox's python/requirements-armdemo.txt installed.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$HERE/../../../nestbox"
PY="${NESTBOX_PY:-$HOME/.venvs/nestbox/bin/python}"
export NESTBOX_CONFIG="$REPO/config/armdemo_config.yaml"
export PYTHONPATH="$REPO/python:$HERE"
export PYTHONDONTWRITEBYTECODE=1

[ "$(git -C "$REPO" branch --show-current)" = "armdemo" ] || echo "warning: nestbox repo is not on branch armdemo"
redis-cli ping >/dev/null 2>&1 || redis-server --save "" --appendonly no --daemonize yes >/dev/null
pkill -INT -f "follow_hand.py --nestbox|camera_ray_test.py" 2>/dev/null && sleep 1 || true   # an earlier run holds ports 12345/8651
pkill -TERM -f "nestbox.networking.daemon_server" 2>/dev/null && sleep 1 || true   # fresh daemon: CS names are global
(cd /tmp && MPLBACKEND=Agg "$PY" -u -m nestbox.networking.daemon_server > /tmp/nestbox_daemon.log 2>&1 &)
for i in $(seq 1 30); do nc -z 127.0.0.1 8650 2>/dev/null && break; sleep 0.5; done
nc -z 127.0.0.1 8650 || { echo "daemon did not start; see /tmp/nestbox_daemon.log"; exit 1; }
echo "nestbox daemon up on :8650 (log: /tmp/nestbox_daemon.log)"
adb reverse tcp:12345 tcp:12345 >/dev/null && adb reverse tcp:8651 tcp:8651 >/dev/null && echo "adb reverse: 12345 (hands), 8651 (overlay)"
cd "$HERE"
trap 'pkill -TERM -f "nestbox.networking.daemon_server"' EXIT
"$PY" -u follow_hand.py --nestbox "$@"
