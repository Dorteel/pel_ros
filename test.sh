#!/usr/bin/env bash
# Repository-level test entry point.
#   ./test.sh            offline unit tests of the research code
#   ./test.sh --all      additionally re-derive the tables from stored results
# The ROS 2 packages keep their own ament tests (colcon test); research/ is
# excluded from colcon by research/COLCON_IGNORE.
set -euo pipefail
cd "$(dirname "$0")"
PYTHON="${PYTHON:-.venv/bin/python}"
[ -x "$PYTHON" ] || PYTHON=python3
PYTHONPATH=research "$PYTHON" -m pytest research/tests -q
if [ "${1:-}" = "--all" ]; then
  "$PYTHON" research/experiments/summarise_results.py > /dev/null
  echo "summary tables regenerated from stored results"
fi
