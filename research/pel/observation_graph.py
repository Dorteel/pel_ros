"""Observation graph G_o and the recognition process f_r (paper Sec. 3.2).

Definition (paper): G_o is a set of triples  G_o ⊆ E_o × P × (E_o ∪ A).
Every entity e ∈ E_o carries a label literal l_e ∈ L and is grounded in the
observation by a location attribute a_e^loc ∈ R^n and a time attribute
a_e^time ∈ R.  The location is deliberately not restricted to bounding boxes.

The paper's baseline uses YOLO as f_r.  Its *evaluation* (Sec. 5) only feeds
the label literals of MS COCO / Visual Genome to the mapping process, so no
detector is required to reproduce the reported numbers.  ``from_detections``
is the adapter that turns any detector output (YOLO included) into G_o.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, Sequence

HAS_LABEL = "hasLabel"
HAS_LOCATION = "hasLocation"       # a_e^loc, e.g. a bounding box (x, y, w, h)
HAS_TIME = "hasTime"               # a_e^time, seconds
HAS_CERTAINTY = "hasCertainty"     # detector confidence (paper Fig. 4)


@dataclass(frozen=True)
class PerceivedEntity:
    """One entity e ∈ E_o with its grounding attributes."""
    identifier: str
    label: str
    location: tuple[float, ...]
    time: float
    certainty: float | None = None


@dataclass
class ObservationGraph:
    """G_o as an explicit triple set plus typed access to its entities."""
    entities: list[PerceivedEntity] = field(default_factory=list)
    relations: list[tuple[str, str, str]] = field(default_factory=list)

    def triples(self) -> list[tuple[str, str, object]]:
        """Materialise G_o ⊆ E_o × P × (E_o ∪ A)."""
        result: list[tuple[str, str, object]] = []
        for entity in self.entities:
            result.append((entity.identifier, HAS_LABEL, entity.label))
            result.append((entity.identifier, HAS_LOCATION, entity.location))
            result.append((entity.identifier, HAS_TIME, entity.time))
            if entity.certainty is not None:
                result.append((entity.identifier, HAS_CERTAINTY, entity.certainty))
        result.extend(self.relations)
        return result

    def labels(self) -> list[str]:
        return [entity.label for entity in self.entities]


def from_detections(detections: Iterable[Sequence], time: float) -> ObservationGraph:
    """Recognition process f_r : O -> G for detector output.

    ``detections`` is an iterable of ``(label, bounding_box, confidence)`` as
    produced by YOLO-style detectors; ``time`` is the observation timestamp.
    """
    graph = ObservationGraph()
    for index, (label, bounding_box, confidence) in enumerate(detections, start=1):
        graph.entities.append(PerceivedEntity(
            identifier=f"entity_{index}", label=str(label),
            location=tuple(float(v) for v in bounding_box), time=float(time),
            certainty=None if confidence is None else float(confidence)))
    return graph
