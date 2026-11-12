"""Improved mapping process (this project's extension of Adamik et al. 2024).

Same interface as mapping.map_label, same candidate retrieval, different
ranking: the hypotheses H1-H4 of improved_scoring.py, switchable one by one so
that the original method remains runnable as a special case:

    ImprovedConfig(inventory="all", directed=False, weights=FusionWeights(type=1))
        == the original f_c + f_d on a *_links query
"""
from __future__ import annotations

from dataclasses import dataclass, field

from .candidate_selection import Candidate, collect_candidates, select_candidate_names
from .improved_scoring import (FusionWeights, gloss_overlap, label_synsets, popularity,
                               type_score)
from .wordnet_access import get_names


@dataclass(frozen=True)
class ImprovedConfig:
    inventory: str = "all"            # H2: "all" | "noun" | "physical"
    directed: bool = False            # H1: directed hypernym score instead of path similarity
    weights: FusionWeights = field(default_factory=FusionWeights)   # H3 (+ H4 via .gloss)
    word_backoff: bool = False        # H5: class-label word back-off


ORIGINAL_DISAMBIGUATION = ImprovedConfig()


def candidate_names(label: str, inventory: str) -> list[tuple[float, str]]:
    """Algorithm 1 restricted to the names of one sense inventory (H2).  The
    scoring and ordering of names is exactly Algorithm 1's."""
    scored = select_candidate_names(label)
    if inventory == "all":
        return scored
    allowed = {name for synset in label_synsets(label, inventory) for name in get_names(synset)}
    return [(score, name) for score, name in scored if name in allowed] or [(1.0, label)]


def score_candidates(label: str, candidates: list[Candidate], class_labels: dict[str, list[str]],
                     descriptions: dict[str, str], config: ImprovedConfig
                     ) -> list[tuple[float, Candidate, dict]]:
    """Rank a candidate pool; returns ``[(score, candidate, evidence)]`` by
    decreasing score (stable, so ties keep the pool order)."""
    senses = label_synsets(label, config.inventory)
    class_inventory = "all" if config.inventory == "all" else "noun"
    max_links = max((candidate.links or 0 for candidate in candidates), default=0)
    scored = []
    for candidate in candidates:
        evidence = {
            "name": candidate.name_similarity,
            "type": type_score(senses, class_labels.get(candidate.iri, []), config.directed,
                               class_inventory, config.word_backoff),
            "gloss": (gloss_overlap(senses, descriptions.get(candidate.iri, ""))
                      if config.weights.gloss else 0.0),
            "popularity": popularity(candidate.links, max_links),
        }
        scored.append((config.weights.score(evidence), candidate, evidence))
    scored.sort(key=lambda row: -row[0])
    return scored


def improved_map_label(label: str, access, query_type: str, config: ImprovedConfig,
                       explain: bool = False):
    """Ranked target entities for one label under the improved method.

    ``access`` is a KnowledgeGraphAccess or a RecordedAccess; ``query_type``
    must be a *_links type (popularity evidence needs the link counts).
    With ``explain`` the full ``[(score, candidate, evidence)]`` list is returned.
    """
    if not query_type.endswith("_links"):
        raise ValueError("the improved method needs a *_links query type")
    candidates = collect_candidates(candidate_names(label, config.inventory),
                                    access.retrieve, query_type)
    iris = [candidate.iri for candidate in candidates]
    descriptions = access.descriptions(iris) if config.weights.gloss else {}
    scored = score_candidates(label, candidates, access.class_labels(iris), descriptions, config)
    return scored if explain else [candidate.iri for _, candidate, _ in scored]
