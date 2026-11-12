"""Target knowledge graphs G_t and the four retrieval queries (paper Sec. 5.1).

G_t := (N_t, L_t) with L_t ⊆ N_t × P_t × N_t (paper Sec. 3.2).  We access the
three graphs of the paper through their public SPARQL endpoints.

Query types (paper wording -> SPARQL written here; the paper gives no SPARQL):

    S        any entity with any property whose object is "label"@en
    S_links  S, ordered by the number of outgoing links of the entity (desc.)
    L        only rdfs:label "label"@en
    L_links  L, ordered by the number of outgoing links (desc.)
"""
from __future__ import annotations

from dataclasses import dataclass

QUERY_TYPES = ("S", "S_links", "L", "L_links")
RDFS_LABEL = "<http://www.w3.org/2000/01/rdf-schema#label>"
RDF_TYPE = "<http://www.w3.org/1999/02/22-rdf-syntax-ns#type>"
RDFS_SUBCLASS = "<http://www.w3.org/2000/01/rdf-schema#subClassOf>"


@dataclass(frozen=True)
class KnowledgeGraph:
    """Static description of one target knowledge graph."""
    name: str
    endpoint: str
    # Predicates whose objects are the "classes" of a candidate (GET_CLASS in
    # Algorithm 2).
    class_predicates: tuple[str, ...]
    # How dataset labels are cased before exact string matching.  DBpedia and
    # YAGO 4.5 labels are Wikipedia page titles ("Chair"); WikiData labels of
    # common nouns are lower case ("chair").  See specification I3.
    label_case: str            # "as_is" | "capitalize"
    # Predicate holding a short English description of an entity (used only
    # by the improved method, hypothesis H4).
    description_predicate: str = "<http://www.w3.org/2000/01/rdf-schema#comment>"
    max_parallel_requests: int = 2
    min_seconds_between_requests: float = 0.2

    def surface_form(self, name: str) -> str:
        """The exact literal that is matched against the graph."""
        if self.label_case == "capitalize" and name:
            return name[0].upper() + name[1:]
        return name


WIKIDATA = KnowledgeGraph(
    name="wikidata", endpoint="https://query.wikidata.org/sparql",
    class_predicates=("<http://www.wikidata.org/prop/direct/P31>",
                      "<http://www.wikidata.org/prop/direct/P279>"),
    description_predicate="<http://schema.org/description>",
    label_case="as_is", max_parallel_requests=4, min_seconds_between_requests=0.12)
DBPEDIA = KnowledgeGraph(
    name="dbpedia", endpoint="https://dbpedia.org/sparql",
    class_predicates=(RDF_TYPE, RDFS_SUBCLASS),
    label_case="capitalize", max_parallel_requests=1, min_seconds_between_requests=0.5)
YAGO = KnowledgeGraph(
    name="yago", endpoint="https://yago-knowledge.org/sparql/qlever",
    class_predicates=(RDF_TYPE, RDFS_SUBCLASS),
    label_case="capitalize", max_parallel_requests=3, min_seconds_between_requests=0.1)

KNOWLEDGE_GRAPHS = {kg.name: kg for kg in (WIKIDATA, DBPEDIA, YAGO)}


def sparql_literal(text: str) -> str:
    """An English-tagged SPARQL literal with the necessary escaping."""
    escaped = text.replace("\\", "\\\\").replace('"', '\\"')
    return f'"{escaped}"@en'


def build_retrieval_query(surface_form: str, query_type: str, limit: int) -> str:
    """SPARQL for one of the paper's four query types.

    The *_links variants count every outgoing triple of the matched entity and
    sort by that count, "to encourage more common elements" (paper Sec. 5.1).
    For S an entity can match through several predicates (label, alias, ...),
    which would repeat its outgoing triples once per matching predicate; the
    division by the number of distinct matching predicates undoes exactly
    that.  (A flat join is used because WikiData's engine times out on the
    equivalent sub-query formulation.)
    """
    if query_type not in QUERY_TYPES:
        raise ValueError(f"unknown query type {query_type!r}")
    literal = sparql_literal(surface_form)
    if query_type == "L_links":
        return ("SELECT ?entity (COUNT(?target) AS ?links) WHERE { "
                f"?entity {RDFS_LABEL} {literal} . ?entity ?outgoing ?target }} "
                f"GROUP BY ?entity ORDER BY DESC(?links) LIMIT {limit}")
    if query_type == "S_links":
        return ("SELECT ?entity (COUNT(?target) / COUNT(DISTINCT ?anyProperty) AS ?links) "
                f"WHERE {{ ?entity ?anyProperty {literal} . ?entity ?outgoing ?target }} "
                f"GROUP BY ?entity ORDER BY DESC(?links) LIMIT {limit}")
    predicate = RDFS_LABEL if query_type == "L" else "?anyProperty"
    return f"SELECT DISTINCT ?entity WHERE {{ ?entity {predicate} {literal} }} LIMIT {limit}"


def build_class_query(kg: KnowledgeGraph, entity_iris: list[str]) -> str:
    """SPARQL returning, for each entity, its classes and their English labels
    (GET_CLASS of Algorithm 2).  Several entities are batched through VALUES
    purely to reduce endpoint load; results are identical to one query each."""
    values = " ".join(f"<{iri}>" for iri in entity_iris)
    predicates = " ".join(kg.class_predicates)
    return ("SELECT ?entity ?classPredicate ?class ?classLabel WHERE { "
            f"VALUES ?entity {{ {values} }} VALUES ?classPredicate {{ {predicates} }} "
            "?entity ?classPredicate ?class . "
            f"OPTIONAL {{ ?class {RDFS_LABEL} ?classLabel . "
            'FILTER(LANG(?classLabel) = "en") } }')


def build_description_query(kg: KnowledgeGraph, entity_iris: list[str]) -> str:
    """SPARQL returning the English description of each entity (improved
    method, H4)."""
    values = " ".join(f"<{iri}>" for iri in entity_iris)
    return ("SELECT ?entity ?description WHERE { "
            f"VALUES ?entity {{ {values} }} ?entity {kg.description_predicate} ?description . "
            'FILTER(LANG(?description) = "en") }')
