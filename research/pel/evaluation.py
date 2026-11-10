"""Evaluation metrics of the paper (Sec. 5.1-5.2) and their uncertainty.

hit@k for a set of labels D with ground-truth entity sets GT(l) and ranked
predictions r_1(l), r_2(l), ...:

    hit@k = (1/|D|) * |{ l in D : {r_1(l), ..., r_k(l)} ∩ GT(l) ≠ ∅ }|

hit@1 is the paper's linking accuracy.  The denominator is *all* labels (the
paper reports 0.69 as "55 out of the 80 labels"), so a label without any
returned entity, or without a ground-truth entity, counts as a miss.
"""
from __future__ import annotations

import math
from urllib.parse import unquote


def normalise_iri(iri: str) -> str:
    """Canonical form for comparing IRIs from different sources."""
    return unquote(iri.strip()).replace("https://", "http://")


def first_hit_rank(ranking: list[str], ground_truth: set[str]) -> int | None:
    """1-based rank of the first correct entity, or None if absent."""
    truth = {normalise_iri(iri) for iri in ground_truth}
    for rank, iri in enumerate(ranking, start=1):
        if normalise_iri(iri) in truth:
            return rank
    return None


def hit_at_k(first_hit_ranks: list[int | None], k: int) -> float:
    """Fraction of labels whose first correct entity is within the top k."""
    if not first_hit_ranks:
        return float("nan")
    return sum(rank is not None and rank <= k for rank in first_hit_ranks) / len(first_hit_ranks)


def wilson_interval(successes: int, n: int, z: float = 1.959963984540054) -> tuple[float, float]:
    """Wilson score interval for a binomial proportion (default 95 %)."""
    if n == 0:
        return (float("nan"), float("nan"))
    p = successes / n
    centre = (p + z * z / (2 * n)) / (1 + z * z / n)
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return (max(0.0, centre - half), min(1.0, centre + half))


def required_sample_size(population: int, margin: float, z: float = 1.959963984540054,
                         p: float = 0.5) -> int:
    """Cochran's sample size with finite-population correction.

    n0 = z² p (1-p) / e² ;  n = n0 / (1 + (n0 - 1) / N).  The paper uses this
    to argue that 2,399 classes are representative of 76,340 (needs 2,328 at
    a 2 % margin, 95 % confidence)."""
    n0 = z * z * p * (1 - p) / (margin * margin)
    return math.ceil(n0 / (1 + (n0 - 1) / population))
