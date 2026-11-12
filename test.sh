#!/usr/bin/env bash
# Repository-level test entry point.
#   ./test.sh            offline unit tests of the research code
#   ./test.sh --all      additionally re-derive tables and figures from stored
#                        results and execute the tutorial notebooks (offline)
# The ROS 2 packages keep their own ament tests (colcon test); research/ is
# excluded from colcon by research/COLCON_IGNORE.
set -euo pipefail
cd "$(dirname "$0")"
PYTHON="${PYTHON:-.venv/bin/python}"
[ -x "$PYTHON" ] || PYTHON=python3
PYTHONPATH=research "$PYTHON" -m pytest research/tests -q
if [ "${1:-}" = "--all" ]; then
  "$PYTHON" research/experiments/summarise_results.py > /dev/null
  "$PYTHON" research/experiments/make_reproduction_figures.py > /dev/null
  "$PYTHON" research/experiments/make_concept_diagrams.py > /dev/null
  for report in research/experiments/results/improvements/analysis_*.json; do
    name="$(basename "$report" .json)"
    "$PYTHON" research/experiments/make_improvement_figures.py --query-type "${name#analysis_}" > /dev/null
  done
  echo "tables and figures regenerated from stored results"
  "$PYTHON" research/tutorial/run_notebooks.py 2>&1 | grep -E "^(ok|FAILED)"
fi
