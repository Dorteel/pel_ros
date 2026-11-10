"""Candidate selection f_c (paper Sec. 4.1, Algorithm 1).

Idea: the label produced by the recogniser ("couch") may not be the string the
knowledge graph uses ("sofa").  WordNet supplies alternate names: every lemma
of every synset of the label.  Each name is scored by its Levenshtein
similarity to the label, the names are sorted, and *all* entities the graph
returns for any name become candidates.
"""
from __future__ import annotations

from dataclasses import dataclass

from .text_similarity import levenshtein_similarity
from .wordnet_access import get_names, get_synsets


def select_candidate_names(label: str) -> list[tuple[float, str]]:
    """Algorithm 1 (SELECT_CANDIDATES): alternate names for a perceived label.

    Returns ``[(similarity, name), ...]`` sorted by decreasing similarity
    (ties keep WordNet order).  If WordNet does not know the label, the label
    itself is the only name (Algorithm 1, lines 4-6).  The label is always
    present with similarity 1, because it is a lemma of its own synsets.
    """
    synsets = get_synsets(label)                                   # line 2
    if not synsets:                                                # lines 4-6
        return [(1.0, label)]
    scored_names: dict[str, float] = {}                            # line 3
    for synset in synsets:                                         # line 7
        for name in get_names(synset):                             # lines 8-9
            if name not in scored_names:
                scored_names[name] = levenshtein_similarity(label, name)  # line 10
    candidates = [(score, name) for name, score in scored_names.items()]  # line 12
    candidates.sort(key=lambda scored: -scored[0])                 # line 14 (stable)
    return candidates                                              # line 15


@dataclass(frozen=True)
class Candidate:
    """One candidate entity e_{i,Gt} for a perceived label."""
    iri: str
    source_name: str            # the WordNet name whose query returned it
    name_similarity: float      # Levenshtein similarity of that name to the label
    links: int | None           # outgoing-link count (only for *_links queries)


def collect_candidates(scored_names: list[tuple[float, str]], retrieve,
                       query_type: str) -> list[Candidate]:
    """Query the graph with every name and pool the returned entities.

    ``retrieve(name, query_type)`` returns ``[(iri, links_or_None), ...]`` in
    endpoint order.  The paper does not define how the per-name result lists
    are merged (specification, interpretation I1).  We use the only reading
    under which the query's own ordering criterion is kept for the pooled set:

    * S / L (no ordering criterion): concatenate the lists in order of name
      similarity, so entities found for the most label-like name come first.
    * S_links / L_links ("ordered by outgoing links"): order the *pooled* set
      by outgoing links, descending.  Ties keep name-similarity order.

    An entity returned for several names is kept once, at its first position.
    """
    pooled: list[Candidate] = []
    seen: set[str] = set()
    for similarity, name in scored_names:
        for iri, links in retrieve(name, query_type):
            if iri not in seen:
                seen.add(iri)
                pooled.append(Candidate(iri, name, similarity, links))
    if query_type.endswith("_links"):
        pooled.sort(key=lambda candidate: -(candidate.links or 0))   # stable
    return pooled
