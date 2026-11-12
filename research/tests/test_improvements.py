"""Tests of the improved scoring (H1-H5) and of the offline comparison harness."""
import json
import sys
from pathlib import Path

import pytest
from nltk.corpus import wordnet as wn

RESEARCH = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RESEARCH / "experiments"))

from pel.candidate_selection import Candidate
from pel.improved_mapping import (ORIGINAL_DISAMBIGUATION, ImprovedConfig, candidate_names,
                                  improved_map_label, score_candidates)
from pel.improved_scoring import (FusionWeights, class_senses, content_words,
                                  directed_hypernym_similarity, gloss_overlap, label_synsets,
                                  popularity, type_score)
from pel.mapping import map_label

from test_core import _FakeAccess


# --- H1 -------------------------------------------------------------------------
def test_directed_similarity_rewards_generalisations_only():
    dog = wn.synset("dog.n.01")
    assert directed_hypernym_similarity(dog, dog) == 0.0                         # self: not a generalisation
    assert directed_hypernym_similarity(wn.synset("canine.n.02"), dog) == 1.0    # direct hypernym
    assert directed_hypernym_similarity(wn.synset("carnivore.n.01"), dog) == 0.5
    assert directed_hypernym_similarity(wn.synset("puppy.n.01"), dog) == 0.0     # more specific
    assert directed_hypernym_similarity(wn.synset("cat.n.01"), dog) == 0.0       # sibling branch


def test_instance_trap_is_removed_by_directed_score():
    senses = label_synsets("dog", "all")
    # an individual dog is typed "dog"; the concept dog is typed "domestic animal"
    assert type_score(senses, ["dog"], directed=False) == 1.0
    assert type_score(senses, ["domestic animal"], directed=False) == 0.5
    assert type_score(senses, ["dog"], directed=True) < type_score(senses, ["domestic animal"], directed=True)


# --- H2 -------------------------------------------------------------------------
def test_sense_inventories_are_nested_and_never_empty():
    for label in ("couch", "orange", "bear", "zzzunknownzzz", "run"):
        everything, nouns, physical = (set(label_synsets(label, i)) for i in ("all", "noun", "physical"))
        assert physical <= nouns <= everything
        assert bool(everything) == bool(nouns) == bool(physical)
    assert all(s.pos() == "n" for s in label_synsets("couch", "noun"))
    assert wn.synset("orange.n.02") not in label_synsets("orange", "physical")   # the colour
    with pytest.raises(ValueError):
        label_synsets("couch", "everything")


def test_noun_inventory_drops_verb_names():
    names = [name for _, name in candidate_names("couch", "noun")]
    assert "sofa" in names and "couch" in names
    assert not {"put", "redact", "cast", "frame"} & set(names)
    assert candidate_names("sports ball", "noun") == [(1.0, "sports ball")]


# --- H4 / H5 / H3 building blocks --------------------------------------------------
def test_gloss_overlap_bounds_and_preference():
    senses = label_synsets("orange", "physical")
    assert gloss_overlap(senses, "") == 0.0
    fruit = gloss_overlap(senses, "round yellow to orange citrus fruit of any of several trees")
    city = gloss_overlap(senses, "city in Orange County, California, United States")
    assert 0.0 <= city < fruit <= 1.0
    assert content_words("The fruits of the trees") == frozenset({"fruit", "tree"})


def test_word_backoff_only_applies_to_unknown_phrases():
    assert class_senses("fruit of Maloideae", "noun", word_backoff=False) == ()
    assert wn.synset("fruit.n.01") in class_senses("fruit of Maloideae", "noun", word_backoff=True)
    assert class_senses("citrus fruit", "noun", True) == class_senses("citrus fruit", "noun", False)


def test_popularity_is_monotone_and_bounded():
    values = [popularity(links, 1000) for links in (0, 1, 10, 100, 1000)]
    assert values == sorted(values) and values[0] == 0.0 and values[-1] == 1.0
    assert popularity(None, 1000) == 0.0 and popularity(5, 0) == 0.0


def test_fusion_score_is_linear():
    evidence = {"name": 1.0, "type": 0.5, "gloss": 0.2, "popularity": 0.4}
    assert FusionWeights(0.1, 0.2, 0.3, 0.4).score(evidence) == pytest.approx(0.1 + 0.1 + 0.06 + 0.16)


# --- the original method is a special case of the improved code ----------------------
def test_original_disambiguation_is_recovered():
    access = _FakeAccess()
    access.descriptions = lambda iris: {iri: "" for iri in iris}
    for label in ("orange", "zzzunknownzzz"):
        assert (improved_map_label(label, access, "L_links", ORIGINAL_DISAMBIGUATION)
                == map_label(label, access, "L_links", "fc+fd"))
    with pytest.raises(ValueError):
        improved_map_label("orange", access, "L", ORIGINAL_DISAMBIGUATION)


def test_score_candidates_is_stable_for_ties():
    candidates = [Candidate(str(i), "x", 1.0, 5) for i in range(3)]
    ranked = score_candidates("orange", candidates, {}, {}, ImprovedConfig(directed=True))
    assert [c.iri for _, c, _ in ranked] == ["0", "1", "2"]


# --- offline analysis reproduces the stored experiment, label by label ------------------
@pytest.mark.parametrize("dataset,kg", [(d, k) for d in ("coco", "visual_genome")
                                        for k in ("wikidata", "dbpedia", "yago")])
def test_offline_analysis_matches_stored_reproduction(dataset, kg):
    import analyse_improvements as analysis
    recording = analysis.load(dataset, kg, "L_links")
    stored_path = RESEARCH / "experiments" / "results" / "reproduction" / f"{dataset}_{kg}.json"
    if recording is None or not stored_path.exists():
        pytest.skip("recording or stored result not available")
    stored = {(r["label"], r["synset"]): r["first_hit_rank"]
              for r in json.loads(stored_path.read_text())["records"]}
    evidence = analysis.Evidence(recording)
    for process, method in analysis.ORIGINAL_METHODS.items():
        ranks = analysis.method_ranks(evidence, recording["entries"], method)
        mismatches = [entry["label"] for entry, rank in zip(recording["entries"], ranks)
                      if (rank == 1) != (stored[entry["label"], entry["synset"]][f"L_links|{process}"] == 1)]
        # ties between equal scores may be ordered differently; anything beyond
        # a handful of labels would mean the two code paths disagree
        assert len(mismatches) <= max(2, len(ranks) // 100), (process, mismatches[:10])
