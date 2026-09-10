#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
required=(
  package.xml
  setup.py
  setup.cfg
  resource/rby1_control
  config/default.yaml
  launch/control.launch.py
  launch/control_debug.launch.py
  rby1_control/__init__.py
  rby1_control/qt_compat.py
  rby1_control/backend_contract.py
  rby1_control/topic_protocol.py
  rby1_control/topic_backend.py
  rby1_control/frontend_node.py
  rby1_control/backend_main.py
  rby1_control/ros_backend.py
  rby1_control/task_commands.py
  rby1_control/task.py
  rby1_control/task_runner.py
  rby1_control/scenario_ui.py
  rby1_control/main_window.py
  rby1_control/main.py
)

missing=0
for item in "${required[@]}"; do
  if [[ ! -f "$ROOT/$item" ]]; then
    echo "MISSING: $item"
    missing=1
  else
    echo "OK: $item"
  fi
done

for excluded in rby1_control/camera.py rby1_control/mock_backend.py; do
  if [[ -e "$ROOT/$excluded" ]]; then
    echo "UNEXPECTED: $excluded" >&2
    missing=1
  fi
done

if [[ $missing -ne 0 ]]; then
  echo "File-layout check failed." >&2
  exit 1
fi

echo "File-layout check passed. No ROS node or GUI was executed."
