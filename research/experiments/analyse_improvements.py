"""Controlled comparison of the original mapping process with the improvements.

Everything is computed offline from the candidate recordings produced by
extract_candidate_features.py, so every method sees the same candidate pool,
the same evidence and the same ground truth.

Protocol
    development set : MS COCO (80) + one half of the Visual Genome sample
    test set        : the other half of the Visual Genome sample (seed 1 split)
    All choices (sense inventory, fusion weights, which original process is
    the "best original") are made on the development set only; the test set
    is used once, for the final tables.

Usage:  python research/experiments/analyse_improvements.py [--query-type L_links]
Writes  research/experiments/results/improvements/analysis_<query_type>.json
"""
from __future__ import annotations

import argparse
import gzip
import itertools
import json
import random
import sys
from pathlib import Path

import numpy as np
from scipy.stats import binomtest

RESEARCH = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RESEARCH))

sys.path.insert(0, str(Path(__file__).resolve().parent))

from pel.evaluation import normalise_iri, wilson_interval                 # noqa: E402
from run_mapping_experiment import load_entries                           # noqa: E402
from pel.improved_scoring import (FusionWeights, gloss_overlap, label_synsets,  # noqa: E402
                                  popularity, type_score)

CANDIDATES = RESEARCH / "experiments" / "results" / "improvements" / "candidates"
OUTPUT = RESEARCH / "experiments" / "results" / "improvements"
KGS = ("wikidata", "dbpedia", "yago")
FEATURES = ("name", "type", "gloss", "popularity")
SPLIT_SEED = 1
VISUAL_GENOME_SAMPLE = 600
BOOTSTRAP_SAMPLES = 2000


# --- data ---------------------------------------------------------------------------
def load(dataset: str, kg: str, query_type: str):
    path = CANDIDATES / f"{dataset}_{kg}_{query_type}.json.gz"
    if not path.exists():
        return None
    with gzip.open(path, "rt", encoding="utf-8") as handle:
        return json.load(handle)


def entry_key(entry: dict) -> tuple:
    return (entry["label"], entry["synset"])


def visual_genome_split() -> tuple[set, set]:
    """Seeded half/half split of the 600-entry Visual Genome sample into
    development and test keys.  Defined on the full sample, so a graph that
    was evaluated on a subset of it uses the same assignment."""
    sample = load_entries("visual_genome", VISUAL_GENOME_SAMPLE, 0)
    indices = list(range(len(sample)))
    random.Random(SPLIT_SEED).shuffle(indices)
    half = len(sample) // 2
    return ({entry_key(sample[i]) for i in indices[:half]},
            {entry_key(sample[i]) for i in indices[half:]})


def stored_original_ranks(dataset: str, kg: str) -> dict:
    """first-hit ranks of all 12 original conditions from the reproduction run."""
    path = RESEARCH / "experiments" / "results" / "reproduction" / f"{dataset}_{kg}.json"
    return {(r["label"], r["synset"]): r["first_hit_rank"]
            for r in json.loads(path.read_text())["records"]}


# --- feature tables -------------------------------------------------------------------
class LabelEvidence:
    """Feature matrix of one label's candidate pool under one sense inventory
    and one type-similarity definition.  Rows are in the order of the original
    f_c pool for a *_links query: outgoing links descending, ties by name rank."""

    def __init__(self, label: str, recording: dict, inventory: str, directed: bool,
                 word_backoff: bool = False):
        names = recording["names"]
        allowed = [i for i, name in enumerate(names)
                   if inventory == "all" or name[inventory]]
        allowed_set = set(allowed)
        rows = []
        for candidate in recording["candidates"]:
            via = [i for i in candidate["names"] if i in allowed_set]
            if via:
                rows.append((candidate, min(via)))
        rows.sort(key=lambda row: (-(row[0]["links"] or 0), row[1]))
        senses = label_synsets(label, inventory)
        class_inventory = "all" if inventory == "all" else "noun"
        max_links = max((c["links"] or 0 for c, _ in rows), default=0)
        self.iris = [normalise_iri(c["iri"]) for c, _ in rows]
        self.own_query = np.array([0 in c["names"] for c, _ in rows], dtype=bool)
        self.matrix = np.array([[names[first]["similarity"],
                                 type_score(senses, c["classes"], directed, class_inventory, word_backoff),
                                 gloss_overlap(senses, c["description"]),
                                 popularity(c["links"], max_links)] for c, first in rows],
                               dtype=float).reshape(len(rows), len(FEATURES))

    def rank_of_truth(self, truth: set[str], weights=None, own_query_only=False) -> int | None:
        """1-based rank of the first ground-truth entity.  ``weights`` = None
        keeps the pool order (the original f_q / f_c)."""
        order = np.arange(len(self.iris))
        if own_query_only:
            order = order[self.own_query]
        elif weights is not None:
            scores = self.matrix @ np.asarray(weights, dtype=float)
            order = np.argsort(-scores, kind="stable")
        for rank, index in enumerate(order, start=1):
            if self.iris[index] in truth:
                return rank
        return None


class Evidence:
    """Lazily built LabelEvidence for every (label, inventory, directed)."""

    def __init__(self, recording: dict):
        self.recording = recording
        self._cache: dict = {}

    def get(self, label: str, inventory: str, directed: bool,
            word_backoff: bool = False) -> LabelEvidence:
        key = (label, inventory, directed, word_backoff)
        if key not in self._cache:
            self._cache[key] = LabelEvidence(label, self.recording["labels"][label],
                                             inventory, directed, word_backoff)
        return self._cache[key]


# --- methods --------------------------------------------------------------------------
def method_ranks(evidence: Evidence, entries: list[dict], method: dict) -> list[int | None]:
    ranks = []
    for entry in entries:
        table = evidence.get(entry["label"], method.get("inventory", "all"),
                             method.get("directed", False), method.get("word_backoff", False))
        ranks.append(table.rank_of_truth(set(entry["ground_truth"]), method.get("weights"),
                                         method.get("own_query_only", False)))
    return ranks


def w(name=0.0, type=0.0, gloss=0.0, popularity=0.0) -> tuple:
    return (name, type, gloss, popularity)


ORIGINAL_METHODS = {
    "fq": {"own_query_only": True},
    "fc": {},
    "fc+fd": {"weights": w(type=1.0)},
}


def metrics(ranks: list[int | None]) -> dict:
    n = len(ranks)
    result = {"n": n, "mrr": float(np.mean([1.0 / r if r else 0.0 for r in ranks])) if n else 0.0}
    for k in (1, 3, 5):
        hits = sum(r is not None and r <= k for r in ranks)
        result[f"hit@{k}"] = hits / n if n else float("nan")
        result[f"hit@{k}_ci95"] = wilson_interval(hits, n)
    return result


def paired_comparison(ranks_a, ranks_b, seed: int = 0) -> dict:
    """Paired comparison of hit@1 of method B against method A on the same
    labels: exact McNemar test and a paired bootstrap CI of the difference."""
    a = np.array([r == 1 for r in ranks_a]); b = np.array([r == 1 for r in ranks_b])
    only_a, only_b = int(np.sum(a & ~b)), int(np.sum(~a & b))
    p_value = binomtest(only_b, only_a + only_b, 0.5).pvalue if only_a + only_b else 1.0
    rng = np.random.default_rng(seed)
    samples = rng.integers(0, len(a), size=(BOOTSTRAP_SAMPLES, len(a)))
    differences = b[samples].mean(axis=1) - a[samples].mean(axis=1)
    return {"difference": float(b.mean() - a.mean()),
            "difference_ci95": [float(np.percentile(differences, 2.5)),
                                float(np.percentile(differences, 97.5))],
            "b_wins": only_b, "a_wins": only_a, "mcnemar_exact_p": float(p_value)}


def simplex_grid(step: float = 0.1):
    """All weight vectors (name, type, gloss, popularity) >= 0 summing to 1."""
    units = round(1 / step)
    for combination in itertools.product(range(units + 1), repeat=len(FEATURES)):
        if sum(combination) == units:
            yield tuple(c * step for c in combination)


# --- analysis --------------------------------------------------------------------------
def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--query-type", default="L_links")
    args = parser.parse_args()

    development, test, evidences, stored = {}, {}, {}, {}
    dev_keys, test_keys = visual_genome_split()
    for kg in KGS:
        coco, genome = load("coco", kg, args.query_type), load("visual_genome", kg, args.query_type)
        if coco is None or genome is None:
            print(f"[skip] {kg}: candidate recordings missing")
            continue
        evidences[kg] = {"coco": Evidence(coco), "visual_genome": Evidence(genome)}
        stored[kg] = {d: stored_original_ranks(d, kg) for d in ("coco", "visual_genome")}
        development[kg] = [("coco", coco["entries"]),
                           ("visual_genome", [e for e in genome["entries"] if entry_key(e) in dev_keys])]
        test[kg] = [("visual_genome", [e for e in genome["entries"] if entry_key(e) in test_keys])]
    kgs = list(evidences)

    def ranks(kg, parts, method):
        return [rank for dataset, entries in parts
                for rank in method_ranks(evidences[kg][dataset], entries, method)]

    def dev_objective(method) -> tuple:
        """Macro average over graphs of (hit@1, hit@3, MRR) on the development set."""
        scores = [metrics(ranks(kg, development[kg], method)) for kg in kgs]
        return tuple(float(np.mean([s[key] for s in scores])) for key in ("hit@1", "hit@3", "mrr"))

    report = {"query_type": args.query_type, "knowledge_graphs": kgs, "split_seed": SPLIT_SEED,
              "development_n": {kg: sum(len(e) for _, e in development[kg]) for kg in kgs},
              "test_n": {kg: sum(len(e) for _, e in test[kg]) for kg in kgs}}

    # 1. single hypotheses, each applied alone to the original f_c + f_d
    single = {
        "H1 directed hypernym score": {"directed": True, "weights": w(type=1.0)},
        "H2 noun senses": {"inventory": "noun", "weights": w(type=1.0)},
        "H2 physical senses": {"inventory": "physical", "weights": w(type=1.0)},
        "H4 gloss overlap (replaces type)": {"weights": w(gloss=1.0)},
        "H5 class-label word back-off": {"word_backoff": True, "weights": w(type=1.0)},
        "H1+H5 directed score with back-off": {"directed": True, "word_backoff": True,
                                               "weights": w(type=1.0)},
    }
    # H3: fuse the ORIGINAL evidence (symmetric type score) with popularity and name
    h3_grid = [(dev_objective({"weights": w(name=a, type=b, popularity=c)}), (a, b, 0.0, c))
               for a, b, _, c in simplex_grid() if _ == 0]
    h3_weights = max(h3_grid)[1]
    single["H3 fusion of original evidence"] = {"weights": h3_weights}

    # 2. combined method: choose inventory and weights on the development set
    search = []
    for inventory in ("all", "noun", "physical"):
        for word_backoff in (False, True):
            for weights in simplex_grid():
                method = {"inventory": inventory, "directed": True, "word_backoff": word_backoff,
                          "weights": weights}
                search.append((dev_objective(method), inventory, word_backoff, weights))
    best_objective, best_inventory, best_backoff, best_weights = max(search)
    full = {"inventory": best_inventory, "directed": True, "word_backoff": best_backoff,
            "weights": best_weights}
    report["selected_on_development"] = {
        "inventory": best_inventory, "word_backoff": best_backoff,
        "weights": dict(zip(FEATURES, best_weights)),
        "objective_hit@1_hit@3_mrr": best_objective, "h3_weights": dict(zip(FEATURES, h3_weights))}
    report["weight_search"] = [{"inventory": inventory, "word_backoff": word_backoff,
                                "weights": weights, "dev_hit@1": objective[0]}
                               for objective, inventory, word_backoff, weights in search]

    # 3. ablations of the combined method (one ingredient removed, nothing re-tuned)
    def without(feature):
        weights = list(best_weights)
        weights[FEATURES.index(feature)] = 0.0
        return {**full, "weights": tuple(weights)}
    ablations = {
        "full − H1 (symmetric similarity)": {**full, "directed": False},
        "full − H2 (all senses)": {**full, "inventory": "all"},
        "full, H5 toggled": {**full, "word_backoff": not best_backoff},
        "full − type evidence": without("type"),
        "full − gloss evidence (H4)": without("gloss"),
        "full − popularity": without("popularity"),
        "full − name similarity": without("name"),
    }
    methods = {**ORIGINAL_METHODS, **single, "full (combined)": full, **ablations}

    # which original process is best on development (per graph)?
    best_original = {kg: max(ORIGINAL_METHODS, key=lambda name: metrics(
        ranks(kg, development[kg], ORIGINAL_METHODS[name]))["hit@1"]) for kg in kgs}
    report["best_original_on_development"] = best_original

    # the strongest of ALL 12 original conditions (any query type), chosen on development
    def stored_ranks(kg, parts, condition):
        return [stored[kg][dataset][entry_key(entry)][condition]
                for dataset, entries in parts for entry in entries]
    conditions = list(next(iter(stored[kgs[0]]["coco"].values())))
    best_of_twelve = {kg: max(conditions, key=lambda c: metrics(
        stored_ranks(kg, development[kg], c))["hit@1"]) for kg in kgs}
    report["best_of_12_original_on_development"] = best_of_twelve

    for split_name, split in (("development", development), ("test", test)):
        report[split_name] = {}
        for kg in kgs:
            all_ranks = {name: ranks(kg, split[kg], method) for name, method in methods.items()}
            table = {name: metrics(r) for name, r in all_ranks.items()}
            comparisons = {
                "full vs fc+fd": paired_comparison(all_ranks["fc+fd"], all_ranks["full (combined)"]),
                f"full vs best original ({best_original[kg]})": paired_comparison(
                    all_ranks[best_original[kg]], all_ranks["full (combined)"]),
                f"full vs best of 12 original conditions ({best_of_twelve[kg]})": paired_comparison(
                    stored_ranks(kg, split[kg], best_of_twelve[kg]), all_ranks["full (combined)"]),
                **{f"{name} vs fc+fd": paired_comparison(all_ranks["fc+fd"], all_ranks[name])
                   for name in single},
            }
            table.update({f"original {condition}": metrics(stored_ranks(kg, split[kg], condition))
                          for condition in conditions})
            report[split_name][kg] = {"methods": table, "comparisons": comparisons,
                                      "ranks": {name: all_ranks[name] for name in
                                                ("fq", "fc", "fc+fd", "full (combined)")}}

    OUTPUT.mkdir(parents=True, exist_ok=True)
    out = OUTPUT / f"analysis_{args.query_type}.json"
    out.write_text(json.dumps(report, indent=1))
    print("selected on development:", report["selected_on_development"])
    for split_name in ("development", "test"):
        for kg in kgs:
            print(f"\n== {split_name} / {kg} (n = {report[split_name + '_n'][kg]}) ==")
            for name, m in report[split_name][kg]["methods"].items():
                if name.startswith("original ") and name != f"original {best_of_twelve[kg]}":
                    continue
                print(f"  {name:38s} hit@1 {m['hit@1']:.3f}  hit@3 {m['hit@3']:.3f}  "
                      f"hit@5 {m['hit@5']:.3f}  MRR {m['mrr']:.3f}")
            for name, c in report[split_name][kg]["comparisons"].items():
                print(f"  {name:48s} Δhit@1 {c['difference']:+.3f} "
                      f"[{c['difference_ci95'][0]:+.3f}, {c['difference_ci95'][1]:+.3f}]  "
                      f"p = {c['mcnemar_exact_p']:.4f}")
    print("\nwrote", out)


if __name__ == "__main__":
    main()
