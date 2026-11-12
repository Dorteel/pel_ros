"""Improved disambiguation for Perceived-Entity Linking (this project's extension).

NOT part of Adamik et al. (2024).  Four independent, individually testable
changes to the original mapping process (hypotheses H1-H4 in
research/improvements/hypotheses.md):

H1  directed hypernym score   replaces the symmetric path similarity of Alg. 2
H2  noun senses only          restricts the WordNet senses used by Alg. 1 and 2
H3  evidence fusion           combines type, popularity and name evidence
H4  gloss-description overlap adds the entity description as evidence
H5  class-label word back-off  lets phrase-like class labels contribute evidence

All of them are pure functions of data the original pipeline already has
(plus entity descriptions for H4), so original and improved methods can be
compared on identical candidate pools.
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass
from functools import lru_cache

from nltk.corpus import wordnet as wn

from .wordnet_access import _wordnet_lock, get_synsets, path_similarity

NOUN = "n"


# --- H2: sense inventory ---------------------------------------------------------
SENSE_INVENTORIES = ("all", "noun", "physical")


def label_synsets(label: str, inventory: str = "all") -> tuple:
    """Senses of a string under one of three inventories.

    all       every WordNet sense (the original method)
    noun      noun senses only: a detector label names a kind of thing, not
              an action ("couch" = 'to phrase', "bear" = 'to give birth')
    physical  noun senses below physical_entity.n.01: the paper's own argument
              that perceived things are physical (Sec. 4.2), made explicit

    A restricted inventory falls back to the next wider one when it is empty,
    so no label loses all its senses.
    """
    synsets = get_synsets(label)
    if inventory == "all":
        return synsets
    nouns = tuple(s for s in synsets if s.pos() == NOUN)
    if inventory == "noun":
        return nouns or synsets
    if inventory == "physical":
        physical = tuple(s for s in nouns if _is_physical(s))
        return physical or nouns or synsets
    raise ValueError(f"unknown sense inventory {inventory!r}")


@lru_cache(maxsize=None)
def _is_physical(synset) -> bool:
    with _wordnet_lock:
        root = wn.synset("physical_entity.n.01")
        return any(root in path for path in synset.hypernym_paths())


# --- H1: directed hypernym score ---------------------------------------------------
def _min_hypernym_distance(specific, general) -> int | None:
    """Fewest hypernym edges from ``specific`` up to ``general``; None if
    ``general`` is not an ancestor of ``specific``; 0 iff they are equal."""
    with _wordnet_lock:
        distances = [d for ancestor, d in specific.hypernym_distances() if ancestor == general]
    return min(distances) if distances else None


def directed_hypernym_similarity(class_synset, label_synset) -> float:
    """sim_hyp(k, t) = 1/d if the class sense k is a *proper* generalisation of
    the label sense t at upward distance d >= 1, else 0.

    Rationale: the entity that denotes the concept "dog" is typed by
    generalisations of dog (domestic animal, canine).  An entity whose class
    *is* "dog" is one particular dog or a kind of dog - the symmetric path
    similarity of Algorithm 2 gives it the maximum score 1; this score gives
    it 0, and gives 1 to the concept whose class is the direct hypernym.
    """
    distance = _min_hypernym_distance(label_synset, class_synset)
    return 0.0 if not distance else 1.0 / distance


def class_senses(class_label: str, inventory: str, word_backoff: bool) -> tuple:
    """Senses of a class label.  Many class labels are phrases WordNet does not
    list ("fruit of Maloideae", "type of vehicle"); with ``word_backoff`` (H5)
    such a label contributes the senses of its individual content words
    instead of nothing."""
    senses = label_synsets(class_label, inventory)
    if senses or not word_backoff:
        return senses
    words = [word for word in re.findall(r"[A-Za-z]+", class_label)
             if len(word) > 2 and word.lower() not in STOPWORDS]
    return tuple(sense for word in words for sense in label_synsets(word, inventory))


def type_score(label_senses, class_labels: list[str], directed: bool,
               class_inventory: str = "all", word_backoff: bool = False) -> float:
    """max over classes k, senses s of k, senses t of the label of the chosen
    similarity (symmetric path similarity = Algorithm 2; directed = H1)."""
    best = 0.0
    for class_label in class_labels:
        for class_synset in class_senses(class_label, class_inventory, word_backoff):
            for label_synset in label_senses:
                if directed:
                    value = directed_hypernym_similarity(class_synset, label_synset)
                else:
                    value = path_similarity(class_synset, label_synset)
                best = max(best, value)
    return best


# --- H4: gloss-description overlap ---------------------------------------------------
STOPWORDS = frozenset("""a an the of or and in on at to for from by with without as is are was were be
been being that this these those it its which who whom whose such other any some one two
used use uses using made make usually often typically especially etc can may also not no
into over under between than then so very more most has have having""".split())


def content_words(text: str) -> frozenset:
    """Lower-cased alphabetic tokens without stop words, crude plural folding."""
    words = set()
    for token in re.findall(r"[a-z]+", text.lower()):
        if token in STOPWORDS or len(token) < 3:
            continue
        words.add(token[:-1] if token.endswith("s") and not token.endswith("ss") else token)
    return frozenset(words)


def sense_signature(synset) -> frozenset:
    """Extended-Lesk signature of a sense: its gloss, its lemmas, and the
    lemmas and glosses of its direct hypernyms."""
    with _wordnet_lock:
        parts = [synset.definition(), " ".join(synset.lemma_names())]
        for hypernym in synset.hypernyms() + synset.instance_hypernyms():
            parts += [hypernym.definition(), " ".join(hypernym.lemma_names())]
    return content_words(" ".join(parts).replace("_", " "))


def gloss_overlap(label_senses, description: str) -> float:
    """max over senses t of |sig(t) ∩ words(description)| / sqrt(|sig(t)|·|words|)
    (cosine of the two word sets); 0 when the entity has no description."""
    description_words = content_words(description or "")
    if not description_words:
        return 0.0
    best = 0.0
    for synset in label_senses:
        signature = sense_signature(synset)
        if signature:
            shared = len(signature & description_words)
            best = max(best, shared / math.sqrt(len(signature) * len(description_words)))
    return best


# --- H3: evidence fusion --------------------------------------------------------------
def popularity(links: int | None, max_links: int) -> float:
    """log(1 + links) / log(1 + max links in the pool), in [0, 1]."""
    if not links or max_links <= 0:
        return 0.0
    return math.log1p(links) / math.log1p(max_links)


@dataclass(frozen=True)
class FusionWeights:
    """score(c) = name·sim_lev + type·type_score + gloss·gloss_overlap + popularity·pop"""
    name: float = 0.0
    type: float = 1.0
    gloss: float = 0.0
    popularity: float = 0.0

    def score(self, features: dict) -> float:
        return (self.name * features["name"] + self.type * features["type"]
                + self.gloss * features["gloss"] + self.popularity * features["popularity"])
