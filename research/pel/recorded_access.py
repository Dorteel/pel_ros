"""Offline stand-in for KnowledgeGraphAccess, backed by a candidate recording.

extract_candidate_features.py stores, per label, everything the live graph
returned for one *_links query type.  RecordedAccess replays it through the
same interface (``retrieve``, ``class_labels``, ``descriptions``), so the real
pipeline code (mapping.py, improved_mapping.py) runs without network access -
in the tutorial notebooks, in tests, and in the offline analysis.
"""
from __future__ import annotations

import gzip
import json
from pathlib import Path

from .knowledge_graphs import KNOWLEDGE_GRAPHS

RECORDINGS = (Path(__file__).resolve().parents[1] / "experiments" / "results"
              / "improvements" / "candidates")


class RecordedAccess:
    def __init__(self, dataset: str, kg_name: str, query_type: str = "L_links",
                 directory: Path = RECORDINGS):
        path = directory / f"{dataset}_{kg_name}_{query_type}.json.gz"
        with gzip.open(path, "rt", encoding="utf-8") as handle:
            self.recording = json.load(handle)
        self.kg = KNOWLEDGE_GRAPHS[kg_name]
        self.query_type = query_type
        self.entries = self.recording["entries"]
        self._by_name: dict[str, list[tuple[str, int]]] = {}
        self._candidates: dict[str, dict] = {}
        for label_record in self.recording["labels"].values():
            names = [name["name"] for name in label_record["names"]]
            for candidate in label_record["candidates"]:
                self._candidates[candidate["iri"]] = candidate
                for index in candidate["names"]:
                    rows = self._by_name.setdefault(names[index], [])
                    if all(iri != candidate["iri"] for iri, _ in rows):
                        rows.append((candidate["iri"], candidate["links"] or 0))
        for rows in self._by_name.values():
            rows.sort(key=lambda row: -row[1])                 # ORDER BY DESC(?links)

    def labels(self) -> list[str]:
        return sorted(self.recording["labels"])

    def retrieve(self, name: str, query_type: str) -> list[tuple[str, int | None]]:
        if query_type != self.query_type:
            raise ValueError(f"this recording holds {self.query_type} results only")
        return list(self._by_name.get(name, []))

    def class_labels(self, iris: list[str]) -> dict[str, list[str]]:
        return {iri: list(self._candidates.get(iri, {}).get("classes", [])) for iri in iris}

    def descriptions(self, iris: list[str]) -> dict[str, str]:
        return {iri: self._candidates.get(iri, {}).get("description", "") for iri in iris}
