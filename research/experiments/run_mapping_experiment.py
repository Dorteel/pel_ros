"""Run the paper's mapping experiment (Sec. 5) for one dataset and one target graph.

For every label, every query type (S, S_links, L, L_links) and every process
(f_q, f_c, f_c+f_d) the ranked entity list is computed, compared with the
ground truth and stored.  All SPARQL responses are cached in
research/data/cache/, so the script is restartable; re-run it until it reports
0 transient failures.

Usage:
    python research/experiments/run_mapping_experiment.py --dataset coco --kg wikidata
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import random
import sys
import time
from pathlib import Path

RESEARCH = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RESEARCH))

from pel.evaluation import first_hit_rank                     # noqa: E402
from pel.knowledge_graphs import KNOWLEDGE_GRAPHS, QUERY_TYPES  # noqa: E402
from pel.mapping import (DEFAULT_RESULT_LIMIT, PROCESSES, KnowledgeGraphAccess,  # noqa: E402
                         map_labels)
from pel.sparql_client import CachedSparqlClient               # noqa: E402

STORED_TOP_K = 10


def load_entries(dataset: str, max_entries: int | None = None, seed: int = 0) -> list[dict]:
    """Ground-truth entries; optionally a seeded simple random sample of them
    (the same sample for every knowledge graph and every method)."""
    path = RESEARCH / "data" / "ground_truth" / f"{dataset}.json"
    entries = json.loads(path.read_text())["entries"]
    if max_entries is not None and max_entries < len(entries):
        chosen = sorted(random.Random(seed).sample(range(len(entries)), max_entries))
        entries = [entries[index] for index in chosen]
    return entries


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--dataset", required=True, choices=["coco", "visual_genome"])
    parser.add_argument("--kg", required=True, choices=sorted(KNOWLEDGE_GRAPHS))
    parser.add_argument("--result-limit", type=int, default=DEFAULT_RESULT_LIMIT)
    parser.add_argument("--max-entries", type=int, default=None,
                        help="evaluate a seeded random sample of this many entries")
    parser.add_argument("--seed", type=int, default=0, help="seed of the entry sample")
    parser.add_argument("--label-case", choices=["as_is", "capitalize"], default=None,
                        help="override the graph's label casing (sensitivity analysis)")
    parser.add_argument("--offline", action="store_true", help="use cached responses only")
    parser.add_argument("--output-dir", default=str(RESEARCH / "experiments" / "results" / "reproduction"))
    args = parser.parse_args()

    kg = KNOWLEDGE_GRAPHS[args.kg]
    if args.label_case:
        kg = dataclasses.replace(kg, label_case=args.label_case)
    client = CachedSparqlClient(RESEARCH / "data" / "cache" / f"{kg.name}.sqlite",
                                offline=args.offline)
    access = KnowledgeGraphAccess(kg, client, args.result_limit)
    entries = load_entries(args.dataset, args.max_entries, args.seed)
    labels = sorted({entry["label"] for entry in entries})
    conditions = [(query_type, process) for query_type in QUERY_TYPES for process in PROCESSES]

    started = time.time()
    rankings = map_labels(labels, access, conditions, progress=lambda done, total: print(
        f"[{args.dataset}/{kg.name}] {done}/{total} labels, {time.time() - started:.0f}s, "
        f"{len(access.failures)} failed queries", flush=True))

    records = []
    for entry in entries:
        truth = set(entry["ground_truth"][kg.name])
        per_condition = rankings[entry["label"]]
        records.append({
            "label": entry["label"], "synset": entry["synset"], "ground_truth": sorted(truth),
            "first_hit_rank": {name: first_hit_rank(ranking, truth)
                               for name, ranking in per_condition.items()},
            "returned": {name: len(ranking) for name, ranking in per_condition.items()},
            "top": {name: ranking[:STORED_TOP_K] for name, ranking in per_condition.items()},
        })
    reasons: dict[str, int] = {}
    for _, reason in access.failures:
        reasons[reason] = reasons.get(reason, 0) + 1
    output = {"dataset": args.dataset, "knowledge_graph": kg.name, "endpoint": kg.endpoint,
              "result_limit": args.result_limit, "max_entries": args.max_entries,
              "seed": args.seed, "label_case": kg.label_case,
              "entries": len(records), "unique_labels": len(labels),
              "failed_queries": reasons, "records": records}
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"{args.dataset}_{kg.name}.json"
    out_path.write_text(json.dumps(output, ensure_ascii=False))
    print(f"wrote {out_path}; failed queries: {reasons or 'none'}")


if __name__ == "__main__":
    main()
