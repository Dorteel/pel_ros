"""Record, for every label, the candidate pool and the raw evidence about each
candidate (outgoing links, class labels, description, the names that
retrieved it).  All scoring variants - the original Algorithm 2 and every
improvement - are then computed offline from this one recording
(analyse_improvements.py), which guarantees identical candidate pools,
identical data and identical ground truth for all methods compared.

Usage:
    python research/experiments/extract_candidate_features.py --dataset coco --kg yago
"""
from __future__ import annotations

import argparse
import gzip
import json
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

RESEARCH = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RESEARCH))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from pel.candidate_selection import select_candidate_names        # noqa: E402
from pel.evaluation import normalise_iri                           # noqa: E402
from pel.improved_scoring import label_synsets                     # noqa: E402
from pel.knowledge_graphs import KNOWLEDGE_GRAPHS                  # noqa: E402
from pel.mapping import DEFAULT_RESULT_LIMIT, KnowledgeGraphAccess  # noqa: E402
from pel.sparql_client import CachedSparqlClient                   # noqa: E402
from pel.wordnet_access import get_names                           # noqa: E402
from run_mapping_experiment import load_entries                    # noqa: E402

MAX_DESCRIPTION_CHARACTERS = 300
MAX_CLASS_LABELS = 15


def names_of_inventory(label: str, inventory: str) -> set[str]:
    """Names Algorithm 1 would produce if only this sense inventory were used."""
    synsets = label_synsets(label, inventory)
    return {name for synset in synsets for name in get_names(synset)} or {label}


def record_label(label: str, access: KnowledgeGraphAccess, query_type: str) -> dict:
    scored_names = select_candidate_names(label)                    # Algorithm 1, unchanged
    noun_names, physical_names = (names_of_inventory(label, i) for i in ("noun", "physical"))
    names = [{"name": name, "similarity": score, "noun": name in noun_names,
              "physical": name in physical_names} for score, name in scored_names]
    pool: dict[str, dict] = {}
    for index, (_, name) in enumerate(scored_names):
        for iri, links in access.retrieve(name, query_type):
            entry = pool.setdefault(iri, {"iri": iri, "links": links, "names": []})
            entry["names"].append(index)
            entry["links"] = max(entry["links"] or 0, links or 0)
    iris = sorted(pool)
    classes, descriptions = access.class_labels(iris), access.descriptions(iris)
    for iri, entry in pool.items():
        entry["classes"] = classes[iri][:MAX_CLASS_LABELS]
        entry["description"] = descriptions[iri][:MAX_DESCRIPTION_CHARACTERS]
    return {"names": names, "candidates": list(pool.values())}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--dataset", required=True, choices=["coco", "visual_genome"])
    parser.add_argument("--kg", required=True, choices=sorted(KNOWLEDGE_GRAPHS))
    parser.add_argument("--query-type", default="L_links", choices=["L_links", "S_links"])
    parser.add_argument("--max-entries", type=int, default=None)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--subset-size", type=int, default=None)
    parser.add_argument("--offline", action="store_true")
    args = parser.parse_args()

    kg = KNOWLEDGE_GRAPHS[args.kg]
    access = KnowledgeGraphAccess(kg, CachedSparqlClient(
        RESEARCH / "data" / "cache" / f"{kg.name}.sqlite", offline=args.offline), DEFAULT_RESULT_LIMIT)
    entries = load_entries(args.dataset, args.max_entries, args.seed, args.subset_size)
    labels = sorted({entry["label"] for entry in entries})
    with ThreadPoolExecutor(max_workers=kg.max_parallel_requests) as pool:
        recorded = dict(zip(labels, pool.map(
            lambda label: record_label(label, access, args.query_type), labels)))
    reasons: dict[str, int] = {}
    for _, reason in access.failures:
        reasons[reason] = reasons.get(reason, 0) + 1
    output = {"dataset": args.dataset, "knowledge_graph": kg.name, "query_type": args.query_type,
              "result_limit": DEFAULT_RESULT_LIMIT, "max_entries": args.max_entries,
              "seed": args.seed, "subset_size": args.subset_size, "failed_queries": reasons,
              "entries": [{"label": e["label"], "synset": e["synset"],
                           "ground_truth": sorted(normalise_iri(i) for i in e["ground_truth"][kg.name])}
                          for e in entries],
              "labels": recorded}
    out_dir = RESEARCH / "experiments" / "results" / "improvements" / "candidates"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"{args.dataset}_{kg.name}_{args.query_type}.json.gz"
    with gzip.open(out_path, "wt", encoding="utf-8") as handle:
        json.dump(output, handle, ensure_ascii=False)
    print(f"wrote {out_path} ({out_path.stat().st_size / 1e6:.1f} MB); failed queries: {reasons or 'none'}")


if __name__ == "__main__":
    main()
