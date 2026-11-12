"""Turn stored per-label results into the paper's Table 1 layout (hit@1/3/5),
with Wilson 95 % intervals, and compare with the published numbers.

Usage:  python research/experiments/summarise_results.py [--results-dir DIR]
Writes <DIR>/summary.json and <DIR>/table1.md.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

RESEARCH = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RESEARCH))

from pel.evaluation import hit_at_k, wilson_interval   # noqa: E402

KGS = ("wikidata", "dbpedia", "yago")
DATASETS = ("coco", "visual_genome")
KS = (1, 3, 5)


def summarise(results_dir: Path) -> dict:
    summary: dict = {}
    for dataset in DATASETS:
        for kg in KGS:
            path = results_dir / f"{dataset}_{kg}.json"
            if not path.exists():
                continue
            run = json.loads(path.read_text())
            records = run["records"]
            for condition in records[0]["first_hit_rank"]:
                ranks = [record["first_hit_rank"][condition] for record in records]
                cell = {"n": len(ranks),
                        "empty": sum(record["returned"][condition] == 0 for record in records)}
                for k in KS:
                    hits = sum(rank is not None and rank <= k for rank in ranks)
                    cell[f"hit@{k}"] = hit_at_k(ranks, k)
                    cell[f"hit@{k}_ci95"] = wilson_interval(hits, len(ranks))
                summary.setdefault(dataset, {}).setdefault(condition, {})[kg] = cell
            summary[dataset].setdefault("_failed_queries", {})[kg] = run["failed_queries"]
    return summary


def markdown_table(summary: dict, paper: dict) -> str:
    lines = []
    for dataset in DATASETS:
        if dataset not in summary:
            continue
        first = next(v for c, v in summary[dataset].items() if not c.startswith("_"))
        sizes = ", ".join(f"{kg} n = {first[kg]['n']}" for kg in KGS if kg in first)
        lines += [f"### {dataset} ({sizes}) — reproduced / (paper)", "",
                  "| Query | Process | " + " | ".join(f"{kg} h@{k}" for kg in KGS for k in KS) + " |",
                  "|---|---|" + "---|" * 9]
        for condition, cells in summary[dataset].items():
            if condition.startswith("_"):
                continue
            query, process = condition.split("|")
            row = []
            for kg in KGS:
                for index, k in enumerate(KS):
                    ours = f"{cells[kg][f'hit@{k}']:.2f}" if kg in cells else "–"
                    theirs = paper.get(dataset, {}).get(condition, {}).get(kg)
                    row.append(f"{ours} ({theirs[index]:.2f})" if theirs else ours)
            lines.append(f"| {query} | {process} | " + " | ".join(row) + " |")
        lines.append("")
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-dir", default=str(RESEARCH / "experiments" / "results" / "reproduction"))
    args = parser.parse_args()
    results_dir = Path(args.results_dir)
    summary = summarise(results_dir)
    paper = json.loads((RESEARCH / "experiments" / "paper_table1.json").read_text())
    (results_dir / "summary.json").write_text(json.dumps(summary, indent=1))
    table = markdown_table(summary, paper)
    (results_dir / "table1.md").write_text(table)
    print(table)


if __name__ == "__main__":
    main()
