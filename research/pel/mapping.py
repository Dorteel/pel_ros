"""The mapping process f_m : G_o × G_t -> G_l (paper Sec. 3.2, Sec. 4, Fig. 6).

Three processes are compared in the paper's Table 1:

    f_q         run the retrieval query for the label only
    f_c         candidate selection (Algorithm 1)
    f_c + f_d   candidate selection followed by disambiguation (Algorithm 2)

Each returns a ranked list of target-graph entities; rank 1 is the link.
An empty list is the NIL link l_∅.
"""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor

from .candidate_selection import Candidate, collect_candidates, select_candidate_names
from .disambiguation import class_label_from_iri, disambiguate
from .knowledge_graphs import KnowledgeGraph, build_class_query, build_retrieval_query
from .sparql_client import CachedSparqlClient, SparqlError

PROCESSES = ("fq", "fc", "fc+fd")
DEFAULT_RESULT_LIMIT = 50      # LIMIT of every retrieval query (specification I4)
CLASS_QUERY_BATCH_SIZE = 100


class KnowledgeGraphAccess:
    """All access the mapping process needs to one target graph."""

    def __init__(self, kg: KnowledgeGraph, client: CachedSparqlClient,
                 result_limit: int = DEFAULT_RESULT_LIMIT):
        self.kg = kg
        self.client = client
        self.result_limit = result_limit
        self.failures: list[tuple[str, str]] = []     # (query, reason)

    def _select(self, query: str) -> list[dict]:
        try:
            return self.client.select(self.kg.endpoint, query,
                                      self.kg.min_seconds_between_requests)
        except SparqlError as error:
            self.failures.append((query, str(error)))
            return []

    def retrieve(self, name: str, query_type: str) -> list[tuple[str, int | None]]:
        """Entities matching ``name`` under one of the four query types, in
        endpoint order, with their outgoing-link count for *_links queries."""
        query = build_retrieval_query(self.kg.surface_form(name), query_type,
                                      self.result_limit)
        return [(row["entity"], int(float(row["links"])) if "links" in row else None)
                for row in self._select(query) if "entity" in row]

    def class_labels(self, iris: list[str]) -> dict[str, list[str]]:
        """GET_CLASS: labels of the classes of each entity."""
        labels: dict[str, list[str]] = {iri: [] for iri in iris}
        fetchable = [iri for iri in dict.fromkeys(iris)
                     if not any(char in iri for char in '<>"{}|^` \\')]
        for start in range(0, len(fetchable), CLASS_QUERY_BATCH_SIZE):
            batch = fetchable[start:start + CLASS_QUERY_BATCH_SIZE]
            for row in self._select(build_class_query(self.kg, batch)):
                if "class" not in row:
                    continue
                label = row.get("classLabel") or class_label_from_iri(row["class"])
                if label and label not in labels[row["entity"]]:
                    labels[row["entity"]].append(label)
        return labels


def query_only(label: str, access: KnowledgeGraphAccess, query_type: str) -> list[Candidate]:
    """f_q: the label's own query result, in the order the query defines."""
    return [Candidate(iri, label, 1.0, links)
            for iri, links in access.retrieve(label, query_type)]


def candidate_selection(label: str, access: KnowledgeGraphAccess,
                        query_type: str) -> list[Candidate]:
    """f_c: Algorithm 1 followed by pooling of all per-name query results."""
    return collect_candidates(select_candidate_names(label), access.retrieve, query_type)


def map_label_all_conditions(label: str, access: KnowledgeGraphAccess,
                             conditions) -> dict[str, list[str]]:
    """Ranked target entities for one label under every (query_type, process)
    condition.  Candidate classes are fetched once per label for the union of
    all candidates; this only reduces endpoint load, each condition's ranking
    is exactly what the process defines."""
    candidates: dict[tuple[str, str], list[Candidate]] = {}
    for query_type, process in conditions:
        if process == "fq":
            candidates[query_type, process] = query_only(label, access, query_type)
        elif process in ("fc", "fc+fd"):
            if (query_type, "fc") not in candidates:
                candidates[query_type, "fc"] = candidate_selection(label, access, query_type)
            candidates[query_type, process] = candidates[query_type, "fc"]
        else:
            raise ValueError(f"unknown process {process!r}")
    to_classify = sorted({candidate.iri for (_, process), pool in candidates.items()
                          if process == "fc+fd" for candidate in pool})
    class_labels = access.class_labels(to_classify) if to_classify else {}
    rankings: dict[str, list[str]] = {}
    for query_type, process in conditions:
        pool = candidates[query_type, process]
        if process == "fc+fd":
            pool = [candidate for _, candidate in disambiguate(label, pool, class_labels)]
        rankings[f"{query_type}|{process}"] = [candidate.iri for candidate in pool]
    return rankings


def map_label(label: str, access: KnowledgeGraphAccess, query_type: str,
              process: str) -> list[str]:
    """Ranked target entities for one perceived label under one condition;
    rank 1 is the link, an empty list is the NIL link."""
    return map_label_all_conditions(label, access, [(query_type, process)])[
        f"{query_type}|{process}"]


def map_labels(labels: list[str], access: KnowledgeGraphAccess, conditions,
               progress=None) -> dict[str, dict[str, list[str]]]:
    """Run every condition for every label.  Labels are processed in parallel
    only to overlap network latency; results do not depend on parallelism."""
    def run(label: str):
        return label, map_label_all_conditions(label, access, conditions)

    results: dict[str, dict[str, list[str]]] = {}
    with ThreadPoolExecutor(max_workers=access.kg.max_parallel_requests) as pool:
        for done, (label, rankings) in enumerate(pool.map(run, labels), start=1):
            results[label] = rankings
            if progress and (done % 25 == 0 or done == len(labels)):
                progress(done, len(labels))
    return results
