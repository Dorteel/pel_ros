"""Execute the tutorial notebooks in place (offline: they use recorded data).

Usage:
    python research/tutorial/run_notebooks.py            # all chapters
    python research/tutorial/run_notebooks.py 04 07      # selected chapters
Exits non-zero if any cell raises.
"""
from __future__ import annotations

import sys
from pathlib import Path

import nbformat
from nbclient import NotebookClient

TUTORIAL = Path(__file__).resolve().parent


def main() -> int:
    wanted = sys.argv[1:]
    failures = 0
    for path in sorted(TUTORIAL.glob("[0-9][0-9]_*.ipynb")):
        if wanted and path.name[:2] not in wanted:
            continue
        notebook = nbformat.read(path, as_version=4)
        try:
            NotebookClient(notebook, timeout=600, kernel_name="python3",
                           resources={"metadata": {"path": str(TUTORIAL)}}).execute()
            nbformat.write(notebook, path)
            print(f"ok      {path.name}")
        except Exception as error:      # report every failing chapter, then fail
            failures += 1
            print(f"FAILED  {path.name}: {str(error)[-600:]}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
