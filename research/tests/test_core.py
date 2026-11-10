"""Offline unit tests for the mathematical and algorithmic components."""
import pytest

from pel.candidate_selection import Candidate, collect_candidates, select_candidate_names
from pel.disambiguation import class_label_from_iri, class_similarity, disambiguate
from pel.evaluation import (first_hit_rank, hit_at_k, required_sample_size,
                            wilson_interval)
from pel.knowledge_graphs import DBPEDIA, WIKIDATA, build_class_query, build_retrieval_query
from pel.observation_graph import HAS_LABEL, from_detections
from pel.text_similarity import levenshtein_distance, levenshtein_similarity
from pel.wordnet_access import get_synsets, path_similarity


# --- Levenshtein (Sec. 4.1) -------------------------------------------------
@pytest.mark.parametrize("a,b,d", [("kitten", "sitting", 3), ("", "abc", 3), ("abc", "abc", 0),
                                   ("flaw", "lawn", 2), ("couch", "sofa", 4)])
def test_levenshtein_distance_known_values(a, b, d):
    assert levenshtein_distance(a, b) == d
    assert levenshtein_distance(b, a) == d          # symmetry


def test_levenshtein_similarity_bounds_and_identity():
    assert levenshtein_similarity("orange", "orange") == 1.0
    assert levenshtein_similarity("abc", "xyz") == 0.0
    assert levenshtein_similarity("", "") == 1.0
    assert levenshtein_similarity("orange", "orangeness") == pytest.approx(0.6)


def test_levenshtein_triangle_inequality():
    words = ["orange", "orangeness", "orange tree", "sofa", "couch", "lounge"]
    for a in words:
        for b in words:
            for c in words:
                assert levenshtein_distance(a, c) <= levenshtein_distance(a, b) + levenshtein_distance(b, c)


# --- WordNet -----------------------------------------------------------------
def test_path_similarity_definition():
    orange, fruit = get_synsets("orange")[0], get_synsets("fruit")[0]
    assert path_similarity(orange, orange) == 1.0
    distance = orange.shortest_path_distance(fruit)
    assert path_similarity(orange, fruit) == pytest.approx(1 / (1 + distance))


def test_multiword_and_unknown_labels():
    assert get_synsets("traffic light")
    assert get_synsets("zzzunknownzzz") == ()


# --- Algorithm 1 ---------------------------------------------------------------
def test_candidate_names_orange():
    names = select_candidate_names("orange")
    assert names[0] == (1.0, "orange")
    assert [name for _, name in names] == ["orange", "Orange", "orangish", "orangeness",
                                           "orange tree", "Orange River"]
    scores = [score for score, _ in names]
    assert scores == sorted(scores, reverse=True)


def test_candidate_names_unknown_label_returns_label():
    assert select_candidate_names("sports ball") == [(1.0, "sports ball")]


def _fake_retrieve(table):
    return lambda name, query_type: table.get(name, [])


def test_pooling_plain_query_concatenates_in_name_order():
    table = {"a": [("x", None), ("y", None)], "b": [("y", None), ("z", None)]}
    pooled = collect_candidates([(1.0, "a"), (0.5, "b")], _fake_retrieve(table), "L")
    assert [c.iri for c in pooled] == ["x", "y", "z"]
    assert pooled[2].source_name == "b"


def test_pooling_links_query_sorts_pool_by_links():
    table = {"a": [("x", 5), ("y", 2)], "b": [("z", 90), ("w", 2)]}
    pooled = collect_candidates([(1.0, "a"), (0.5, "b")], _fake_retrieve(table), "L_links")
    assert [c.iri for c in pooled] == ["z", "x", "y", "w"]   # ties keep name order


# --- Algorithm 2 ---------------------------------------------------------------
def test_disambiguation_prefers_physical_class_for_orange():
    candidates = [Candidate("city", "orange", 1.0, None), Candidate("fruit", "orange", 1.0, None),
                  Candidate("noclass", "orange", 1.0, None)]
    classes = {"city": ["city in the United States"], "fruit": ["citrus fruit"], "noclass": []}
    ranked = disambiguate("orange", candidates, classes)
    assert [c.iri for _, c in ranked] == ["fruit", "city", "noclass"]
    assert ranked[0][0] == pytest.approx(0.5)       # orange.n.01 is a direct hyponym of citrus
    assert ranked[1][0] == 0.0                      # class label unknown to WordNet


def test_disambiguation_is_stable_and_handles_unknown_label():
    candidates = [Candidate(str(i), "x", 1.0, None) for i in range(4)]
    ranked = disambiguate("sports ball", candidates, {str(i): ["ball"] for i in range(4)})
    assert [c.iri for _, c in ranked] == ["0", "1", "2", "3"]
    assert all(score == 0.0 for score, _ in ranked)


def test_class_similarity_is_max_over_classes_and_senses():
    synsets = get_synsets("orange")
    assert class_similarity(synsets, ["city", "color"]) >= class_similarity(synsets, ["city"])
    assert class_similarity(synsets, []) == 0.0


def test_class_label_from_iri():
    assert class_label_from_iri("http://dbpedia.org/ontology/PopulatedPlace") == "populated place"
    assert class_label_from_iri("http://dbpedia.org/class/yago/Seat104161981") == "seat"


# --- Queries (Sec. 5.1) --------------------------------------------------------
def test_query_shapes():
    s = build_retrieval_query("orange", "S", 50)
    l = build_retrieval_query("orange", "L", 50)
    ll = build_retrieval_query("orange", "L_links", 50)
    assert '?anyProperty "orange"@en' in s and "ORDER BY" not in s
    assert 'rdf-schema#label> "orange"@en' in l
    assert "ORDER BY DESC(?links)" in ll and "COUNT(?target)" in ll
    sl = build_retrieval_query("orange", "S_links", 50)
    assert "COUNT(DISTINCT ?anyProperty)" in sl and "ORDER BY DESC(?links)" in sl
    assert all(q.endswith("LIMIT 50") for q in (s, l, ll, sl))
    with pytest.raises(ValueError):
        build_retrieval_query("orange", "X", 5)


def test_literal_escaping_and_surface_forms():
    assert '"a \\"b\\""@en' in build_retrieval_query('a "b"', "L", 5)
    assert DBPEDIA.surface_form("traffic light") == "Traffic light"
    assert WIKIDATA.surface_form("traffic light") == "traffic light"
    assert "P279" in build_class_query(WIKIDATA, ["http://www.wikidata.org/entity/Q1"])


# --- Observation graph (Sec. 3.2) ----------------------------------------------
def test_observation_graph_grounding():
    graph = from_detections([("orange", (50, 50, 40, 30), 0.95)], time=17.0)
    assert graph.labels() == ["orange"]
    triples = graph.triples()
    assert ("entity_1", HAS_LABEL, "orange") in triples
    assert len(triples) == 4          # label, location, time, certainty


# --- Metrics ---------------------------------------------------------------------
def test_hit_at_k():
    truth = {"http://www.wikidata.org/entity/Q13191"}
    assert first_hit_rank(["a", "https://www.wikidata.org/entity/Q13191"], truth) == 2
    assert first_hit_rank(["a"], truth) is None and first_hit_rank([], truth) is None
    ranks = [1, 2, None, 5, 7]
    assert hit_at_k(ranks, 1) == 0.2 and hit_at_k(ranks, 3) == 0.4 and hit_at_k(ranks, 5) == 0.6


def test_wilson_interval_and_sample_size():
    low, high = wilson_interval(55, 80)
    assert low < 55 / 80 < high and 0.57 < low < 0.59 and 0.77 < high < 0.79
    assert required_sample_size(76340, 0.02) == 2328      # the paper's figure (Sec. 5.1)


# --- Mapping process (Sec. 4, Fig. 6) with a fake graph ----------------------------
class _FakeAccess:
    """Stands in for KnowledgeGraphAccess: 'orange' names a city (many links)
    and a fruit (fewer links); the synonym 'orange tree' names a tree."""
    def __init__(self):
        from pel.knowledge_graphs import WIKIDATA
        self.kg = WIKIDATA

    def retrieve(self, name, query_type):
        table = {"orange": [("city", 900), ("fruit", 500)], "orange tree": [("tree", 950)]}
        rows = table.get(name, [])
        if query_type.endswith("_links"):
            return sorted(rows, key=lambda row: -row[1])
        return [(iri, None) for iri, _ in rows]

    def class_labels(self, iris):
        table = {"city": ["city"], "fruit": ["citrus fruit"], "tree": ["plant"]}
        return {iri: table.get(iri, []) for iri in iris}


def test_three_processes_on_fake_graph():
    from pel.mapping import map_label, map_label_all_conditions
    access = _FakeAccess()
    assert map_label("orange", access, "L", "fq") == ["city", "fruit"]
    assert map_label("orange", access, "L", "fc") == ["city", "fruit", "tree"]
    # pooled set ordered by links: the synonym's entity overtakes the label's own
    assert map_label("orange", access, "L_links", "fc") == ["tree", "city", "fruit"]
    # disambiguation: fruit (0.5) > tree > city
    assert map_label("orange", access, "L_links", "fc+fd")[0] == "fruit"
    everything = map_label_all_conditions("orange", access, [("L", "fq"), ("L", "fc+fd")])
    assert everything["L|fq"] == ["city", "fruit"] and everything["L|fc+fd"][0] == "fruit"
    assert map_label("zzzunknownzzz", access, "L", "fc+fd") == []     # NIL link
