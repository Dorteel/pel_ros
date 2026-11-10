"""String similarity used by candidate selection (paper Sec. 4.1).

The paper states that "the Levenshtein edit-distance [is used] to calculate the
similarity between two strings" but does not say how a distance is turned into
a similarity.  We use the standard length-normalised form (interpretation I2 in
research/reproduction/specification.md):

    sim(a, b) = 1 - lev(a, b) / max(|a|, |b|)      in [0, 1]
"""
from __future__ import annotations


def levenshtein_distance(a: str, b: str) -> int:
    """Minimum number of single-character insertions, deletions and
    substitutions turning ``a`` into ``b`` (Wagner-Fischer dynamic programme,
    O(|a||b|) time, O(min(|a|,|b|)) memory)."""
    if len(a) < len(b):
        a, b = b, a
    previous = list(range(len(b) + 1))
    for i, char_a in enumerate(a, start=1):
        current = [i]
        for j, char_b in enumerate(b, start=1):
            current.append(min(
                previous[j] + 1,                        # delete char_a
                current[j - 1] + 1,                     # insert char_b
                previous[j - 1] + (char_a != char_b),   # substitute / match
            ))
        previous = current
    return previous[-1]


def levenshtein_similarity(a: str, b: str) -> float:
    """Length-normalised Levenshtein similarity in [0, 1]; 1 iff a == b."""
    longest = max(len(a), len(b))
    if longest == 0:
        return 1.0
    return 1.0 - levenshtein_distance(a, b) / longest
