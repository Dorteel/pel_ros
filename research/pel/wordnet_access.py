"""Thin, explicit wrappers around WordNet 3.0 (NLTK) used by Algorithms 1 and 2.

Kept in one place so that the paper's helper names map one-to-one:

    GET_SYNSET(label)        -> get_synsets
    GET_NAMES(synset)        -> get_names
    GET_SIMILARITY(ss, ss')  -> path_similarity  (Algorithm 2)

NLTK's WordNet reader loads lazily and seeks in shared file handles, which is
not thread-safe; every access therefore goes through one lock.
"""
from __future__ import annotations

import threading
from functools import lru_cache

from nltk.corpus import wordnet as wn

_wordnet_lock = threading.RLock()
with _wordnet_lock:
    wn.ensure_loaded()


def get_synsets(label: str) -> tuple:
    """All WordNet synsets (any part of speech) whose lemmas match ``label``.

    Multi-word labels are looked up in WordNet's collocation form
    ("traffic light" -> "traffic_light").  Returns () if WordNet has no entry.
    """
    with _wordnet_lock:
        return _get_synsets(label.strip().replace(" ", "_"))


@lru_cache(maxsize=None)
def _get_synsets(wordnet_form: str) -> tuple:
    return tuple(wn.synsets(wordnet_form))


def get_names(synset) -> list[str]:
    """Alternate surface forms of a synset: its lemma names, with WordNet's
    underscores turned back into spaces ("orange_tree" -> "orange tree")."""
    with _wordnet_lock:
        return [name.replace("_", " ") for name in synset.lemma_names()]


def path_similarity(synset_a, synset_b) -> float:
    """WordNet path similarity 1 / (1 + d), where d is the number of edges on
    the shortest hypernym/hyponym path between the two synsets.

    NLTK returns None when no path exists (e.g. different parts of speech);
    we map that to 0 so that "unrelated" is the minimum score.
    """
    with _wordnet_lock:
        return _path_similarity(synset_a, synset_b)


@lru_cache(maxsize=None)
def _path_similarity(synset_a, synset_b) -> float:
    similarity = synset_a.path_similarity(synset_b)
    return 0.0 if similarity is None else float(similarity)
