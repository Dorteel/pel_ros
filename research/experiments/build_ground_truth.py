"""Build the ground-truth link sets for Visual Genome and MS COCO (paper Sec. 5.1).

Visual Genome (paper procedure, reproduced):
    VG object synset (WordNet) -> WikiData entity via "WordNet 3.1 Synset ID"
    (P8814) -> DBpedia / YAGO entity via owl:sameAs.  Only synsets represented
    in all three graphs are kept; the label is the synset's first lemma.

MS COCO (paper: manual search, file not available; closest approximation):
    reproducer-chosen WordNet synset -> same chain as above, then the manual
    corrections in data/coco_manual_ground_truth.json are applied.

Usage:  python research/experiments/build_ground_truth.py
"""
from __future__ import annotations

import json
import sys
import zipfile
from pathlib import Path

RESEARCH = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RESEARCH))

from nltk.corpus import wordnet as wn, wordnet31 as wn31   # noqa: E402

from pel.knowledge_graphs import DBPEDIA, WIKIDATA, YAGO    # noqa: E402
from pel.sparql_client import CachedSparqlClient             # noqa: E402

DATA = RESEARCH / "data"
WD_ENTITY = "http://www.wikidata.org/entity/"
SAME_AS = "<http://www.w3.org/2002/07/owl#sameAs>"


def wordnet31_synset_id(synset_name: str) -> str | None:
    """WordNet 3.0 synset name -> WordNet 3.1 synset ID as used by P8814
    ("07763583-n").  Synset offsets change between versions but sense keys are
    stable, so we go through the sense keys of the synset's lemmas."""
    try:
        synset = wn.synset(synset_name)
    except Exception:
        return None
    for lemma in synset.lemmas():
        try:
            synset31 = wn31.lemma_from_key(lemma.key()).synset()
        except Exception:
            continue
        return f"{synset31.offset():08d}-{synset31.pos()}"
    return None


def wikidata_by_synset_id(client) -> dict[str, list[str]]:
    rows = client.select(WIKIDATA.endpoint,
                         "SELECT ?entity ?synset WHERE { ?entity "
                         "<http://www.wikidata.org/prop/direct/P8814> ?synset }")
    index: dict[str, list[str]] = {}
    for row in rows:
        index.setdefault(row["synset"], []).append(row["entity"])
    return {synset: sorted(entities) for synset, entities in index.items()}


def same_as(client, kg, wikidata_iris: list[str], prefix: str) -> dict[str, list[str]]:
    """Entities of ``kg`` declared owl:sameAs the given WikiData entities."""
    found: dict[str, list[str]] = {}
    for start in range(0, len(wikidata_iris), 80):
        values = " ".join(f"<{iri}>" for iri in wikidata_iris[start:start + 80])
        query = (f"SELECT ?entity ?wikidata WHERE {{ VALUES ?wikidata {{ {values} }} "
                 f"?entity {SAME_AS} ?wikidata }}")
        for row in client.select(kg.endpoint, query, kg.min_seconds_between_requests):
            if row["entity"].startswith(prefix):
                found.setdefault(row["wikidata"], []).append(row["entity"])
    return found


def link_synsets(client, synset_names: list[str]) -> dict[str, dict]:
    """synset name -> {wikidata: [...], dbpedia: [...], yago: [...]}."""
    by_synset_id = wikidata_by_synset_id(client)
    wikidata = {name: by_synset_id.get(wordnet31_synset_id(name) or "", [])
                for name in synset_names}
    all_wikidata = sorted({iri for iris in wikidata.values() for iri in iris})
    dbpedia = same_as(client, DBPEDIA, all_wikidata, "http://dbpedia.org/resource/")
    yago = same_as(client, YAGO, all_wikidata, "http://yago-knowledge.org/resource/")
    return {name: {"wikidata": wikidata[name],
                   "dbpedia": sorted({e for w in wikidata[name] for e in dbpedia.get(w, [])}),
                   "yago": sorted({e for w in wikidata[name] for e in yago.get(w, [])})}
            for name in synset_names}


def main() -> None:
    client = CachedSparqlClient(DATA / "cache" / "ground_truth.sqlite")
    out = DATA / "ground_truth"

    archive = zipfile.ZipFile(DATA / "raw" / "visual_genome" / "object_synsets.json.zip")
    object_synsets = json.load(archive.open("object_synsets.json"))
    vg_synsets = sorted(set(object_synsets.values()))
    links = link_synsets(client, vg_synsets)
    entries = [{"label": wn.synset(name).lemma_names()[0].replace("_", " "), "synset": name,
                "ground_truth": links[name]}
               for name in vg_synsets if all(links[name][kg] for kg in ("wikidata", "dbpedia", "yago"))]
    summary = {"object_names": len(object_synsets), "unique_synsets": len(vg_synsets),
               "with_wikidata": sum(bool(links[n]["wikidata"]) for n in vg_synsets),
               "with_dbpedia": sum(bool(links[n]["dbpedia"]) for n in vg_synsets),
               "with_yago": sum(bool(links[n]["yago"]) for n in vg_synsets),
               "in_all_three": len(entries)}
    (out / "visual_genome.json").write_text(json.dumps(
        {"summary": summary, "entries": entries}, indent=1, ensure_ascii=False))
    print("Visual Genome:", summary)

    coco = json.loads((DATA / "coco_labels.json").read_text())["labels"]
    manual_path = DATA / "coco_manual_ground_truth.json"
    manual = json.loads(manual_path.read_text())["corrections"] if manual_path.exists() else {}
    links = link_synsets(client, [synset for _, synset in coco if synset])
    entries = []
    for label, synset in coco:
        truth = dict(links.get(synset, {"wikidata": [], "dbpedia": [], "yago": []}))
        truth.update(manual.get(label, {}))
        entries.append({"label": label, "synset": synset, "ground_truth": truth})
    summary = {"labels": len(entries),
               **{f"with_{kg}": sum(bool(e["ground_truth"][kg]) for e in entries)
                  for kg in ("wikidata", "dbpedia", "yago")}}
    (out / "coco.json").write_text(json.dumps(
        {"summary": summary, "entries": entries}, indent=1, ensure_ascii=False))
    print("MS COCO:", summary)


if __name__ == "__main__":
    main()
