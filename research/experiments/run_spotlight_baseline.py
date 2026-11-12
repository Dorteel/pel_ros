"""External baseline of paper Sec. 5.2: DBpedia Spotlight on the dataset labels.

Each label is sent on its own as the text to annotate (there is no sentence
context in the paper's setting).  A label is a hit if Spotlight returns the
ground-truth DBpedia entity as its first annotation.  The paper does not state
the confidence threshold; Spotlight's default (0.5) and 0.0 are both reported.

AIDA, the paper's YAGO baseline, is not run: the disambiguation service behind
the URL given in the paper answered HTTP 404 on 2026-11-11 (methodology.md).

Usage:  python research/experiments/run_spotlight_baseline.py --dataset coco
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import requests

RESEARCH = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RESEARCH))

from pel.evaluation import first_hit_rank, wilson_interval       # noqa: E402
from pel.sparql_client import USER_AGENT                          # noqa: E402
from run_mapping_experiment import load_entries                   # noqa: E402

SPOTLIGHT = "https://api.dbpedia-spotlight.org/en/annotate"


def annotate(text: str, confidence: float) -> list[str]:
    for attempt in range(4):
        try:
            response = requests.get(SPOTLIGHT, params={"text": text, "confidence": confidence},
                                    headers={"Accept": "application/json", "User-Agent": USER_AGENT},
                                    timeout=30)
            if response.status_code == 200:
                return [r["@URI"] for r in response.json().get("Resources", [])]
        except (requests.RequestException, ValueError):
            pass
        time.sleep(3 * (attempt + 1))
    raise RuntimeError(f"Spotlight unavailable for {text!r}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", required=True, choices=["coco", "visual_genome"])
    parser.add_argument("--max-entries", type=int, default=None)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    entries = load_entries(args.dataset, args.max_entries, args.seed)
    output = {"dataset": args.dataset, "entries": len(entries), "service": SPOTLIGHT, "runs": {}}
    for confidence in (0.5, 0.0):
        records, hits = [], 0
        for entry in entries:
            uris = annotate(entry["label"], confidence)
            hit = first_hit_rank(uris[:1], set(entry["ground_truth"]["dbpedia"])) == 1
            hits += hit
            records.append({"label": entry["label"], "returned": uris, "hit": hit})
            time.sleep(0.2)
        output["runs"][str(confidence)] = {
            "hit@1": hits / len(entries), "hit@1_ci95": wilson_interval(hits, len(entries)),
            "no_annotation": sum(not r["returned"] for r in records), "records": records}
        print(f"confidence {confidence}: hit@1 = {hits}/{len(entries)} = {hits / len(entries):.3f}")
    out = RESEARCH / "experiments" / "results" / "reproduction" / f"spotlight_{args.dataset}.json"
    out.write_text(json.dumps(output, ensure_ascii=False, indent=1))
    print("wrote", out)


if __name__ == "__main__":
    main()
