"""Disambiguation f_d (paper Sec. 4.2, Algorithm 2).

Idea: among the candidates for "orange" are a fruit, a colour and a city.  A
robot perceives physical things, so the candidate whose *class* is close to
the perceived label in WordNet's taxonomy should win.  Formally, for label l
with synsets Syn(l) and candidate c with class labels Cls(c):

    score(c) = max_{k in Cls(c)}  max_{s in Syn(k)}  max_{t in Syn(l)}  sim_path(s, t)
    sim_path(s, t) = 1 / (1 + d(s, t)),   d = shortest-path length in WordNet

Candidates are ranked by decreasing score.  Ties keep their incoming order,
i.e. the order produced by candidate selection (stable sort).
"""
from __future__ import annotations

import re

from .wordnet_access import get_synsets, path_similarity


def class_label_from_iri(iri: str) -> str:
    """Fallback class label when the graph has no English rdfs:label:
    ".../ontology/PopulatedPlace" -> "populated place",
    ".../class/yago/Seat104161981" -> "seat"."""
    local_name = re.split(r"[/#]", iri.rstrip("/#"))[-1]
    local_name = re.sub(r"\d+$", "", local_name)
    local_name = re.sub(r"(?<=[a-z])(?=[A-Z])", " ", local_name)
    return local_name.replace("_", " ").strip().lower()


def class_similarity(label_synsets, class_labels: list[str]) -> float:
    """Inner loops of Algorithm 2 (lines 6-13) for one candidate."""
    high_score = 0.0                                               # line 6
    for class_label in class_labels:                               # line 7
        for class_synset in get_synsets(class_label):              # lines 8-9
            for label_synset in label_synsets:
                sim_score = path_similarity(class_synset, label_synset)   # line 10
                high_score = max(sim_score, high_score)            # line 11
    return high_score


def disambiguate(label: str, candidates: list, class_labels_of: dict[str, list[str]]
                 ) -> list[tuple[float, object]]:
    """Algorithm 2 (DISAMBIGUATE): rank candidates by class/label similarity.

    ``candidates`` are objects with an ``iri`` attribute, in candidate-selection
    order; ``class_labels_of`` maps a candidate IRI to the labels of its
    classes in the target graph (GET_CLASS).  Returns ``[(score, candidate)]``
    by decreasing score.
    """
    label_synsets = get_synsets(label)                             # line 2
    scores = [(class_similarity(label_synsets, class_labels_of.get(candidate.iri, [])),
               candidate) for candidate in candidates]             # lines 4-15
    scores.sort(key=lambda scored: -scored[0])                     # line 16 (stable)
    return scores                                                  # line 17
