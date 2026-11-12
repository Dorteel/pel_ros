"""Why is the correct entity sometimes absent from the candidate pool?

For every Visual Genome entry whose ground-truth entity was not retrieved by
any name of Algorithm 1 (L_links pool), fetch the English labels of the
ground-truth entity and classify the relation between those labels and the
names that were tried.  This separates what better string matching could fix
from what no label-matching method can reach.

Usage:  python research/experiments/analyse_retrieval_misses.py
Writes  research/experiments/results/additional/retrieval_misses.json
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

RESEARCH = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RESEARCH))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import analyse_improvements as analysis                        # noqa: E402
from pel.evaluation import normalise_iri                       # noqa: E402
from pel.knowledge_graphs import KNOWLEDGE_GRAPHS, RDFS_LABEL  # noqa: E402
from pel.sparql_client import CachedSparqlClient               # noqa: E402

CATEGORIES = ("case differs only", "label has a parenthesised qualifier",
              "name is part of a longer label (or vice versa)", "different word",
              "ground-truth entity has no English label")


def english_labels(client, kg, iris: list[str]) -> dict[str, list[str]]:
    labels: dict[str, list[str]] = {iri: [] for iri in iris}
    for start in range(0, len(iris), 80):
        values = " ".join(f"<{iri}>" for iri in iris[start:start + 80])
        query = (f"SELECT ?entity ?label WHERE {{ VALUES ?entity {{ {values} }} "
                 f'?entity {RDFS_LABEL} ?label . FILTER(LANG(?label) = "en") }}')
        for row in client.select(kg.endpoint, query, kg.min_seconds_between_requests):
            labels.setdefault(normalise_iri(row["entity"]), []).append(row["label"])
    return labels


def classify(names: list[str], truth_labels: list[str]) -> str:
    lowered = [name.lower() for name in names]
    if not truth_labels:
        return CATEGORIES[4]
    if any(label.lower() in lowered for label in truth_labels):
        return CATEGORIES[0]
    if any(label.split(" (")[0].lower() in lowered for label in truth_labels if " (" in label):
        return CATEGORIES[1]
    if any(name in label.lower().split() or label.lower() in name.split()
           for label in truth_labels for name in lowered):
        return CATEGORIES[2]
    return CATEGORIES[3]


def main() -> None:
    report = {}
    for kg_name in ("wikidata", "dbpedia", "yago"):
        recording = analysis.load("visual_genome", kg_name, "L_links")
        if recording is None:
            continue
        kg = KNOWLEDGE_GRAPHS[kg_name]
        client = CachedSparqlClient(RESEARCH / "data" / "cache" / f"{kg_name}.sqlite")
        missed = []
        for entry in recording["entries"]:
            pool = {normalise_iri(c["iri"]) for c in recording["labels"][entry["label"]]["candidates"]}
            if not pool & set(entry["ground_truth"]):
                missed.append(entry)
        labels = english_labels(client, kg, sorted({i for e in missed for i in e["ground_truth"]}))
        counts = {category: 0 for category in CATEGORIES}
        examples = {category: [] for category in CATEGORIES}
        for entry in missed:
            names = [n["name"] for n in recording["labels"][entry["label"]]["names"]]
            truth_labels = [l for iri in entry["ground_truth"] for l in labels.get(iri, [])]
            category = classify(names, truth_labels)
            counts[category] += 1
            if len(examples[category]) < 12:
                examples[category].append({"label": entry["label"], "synset": entry["synset"],
                                           "ground_truth_labels": truth_labels[:3]})
        report[kg_name] = {"entries": len(recording["entries"]), "not_retrieved": len(missed),
                           "categories": counts, "examples": examples}
        print(kg_name, len(recording["entries"]), "entries;", len(missed), "not retrieved:", counts)
    out = RESEARCH / "experiments" / "results" / "additional" / "retrieval_misses.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=1, ensure_ascii=False))
    print("wrote", out)


if __name__ == "__main__":
    main()
