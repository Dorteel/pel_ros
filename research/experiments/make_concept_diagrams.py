"""Conceptual diagrams (no experimental data): the PEL pipeline, the original
and improved disambiguation side by side, and the research journey.

Usage:  python research/experiments/make_concept_diagrams.py
Writes  research/experiments/figures/concepts/*.png|svg
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch   # noqa: E402

from plot_style import INK, INK_SOFT, KG_COLOURS, plt, save      # noqa: E402

FIGURES = Path(__file__).resolve().parents[1] / "experiments" / "figures" / "concepts"
BLUE, ORANGE, AQUA = KG_COLOURS["wikidata"], KG_COLOURS["dbpedia"], KG_COLOURS["yago"]
PALE = {"blue": "#e3eefc", "orange": "#fde9e0", "aqua": "#dff5ec", "grey": "#f0efec"}


def box(axis, x, y, width, height, title, body="", fill=PALE["grey"], edge=INK_SOFT, title_size=9):
    axis.add_patch(FancyBboxPatch((x, y), width, height, boxstyle="round,pad=0.02,rounding_size=0.08",
                                  facecolor=fill, edgecolor=edge, linewidth=1.1))
    axis.text(x + width / 2, y + height - 0.16, title, ha="center", va="top", fontsize=title_size,
              fontweight="bold", color=INK)
    if body:
        axis.text(x + width / 2, y + height - 0.52, body, ha="center", va="top", fontsize=7.5,
                  color=INK_SOFT, linespacing=1.35)


def arrow(axis, start, end, label="", colour=INK_SOFT, label_offset=(0, 0.13)):
    axis.add_patch(FancyArrowPatch(start, end, arrowstyle="-|>", mutation_scale=12, linewidth=1.3,
                                   color=colour, shrinkA=2, shrinkB=2))
    if label:
        axis.text((start[0] + end[0]) / 2 + label_offset[0], (start[1] + end[1]) / 2 + label_offset[1],
                  label, ha="center", va="bottom", fontsize=9, color=INK, style="italic")


def canvas(width, height):
    figure, axis = plt.subplots(figsize=(width, height))
    axis.set_xlim(0, width), axis.set_ylim(0, height)
    axis.axis("off")
    return figure, axis


def pipeline():
    figure, axis = canvas(13, 3.6)
    stages = [("Environment", "world state x ∈ X\n(an orange on a table)", PALE["grey"]),
              ("Observation o", "sensor tensor\n(RGB image)", PALE["grey"]),
              ("Observation graph $G_o$", "entity e\nlabel “orange”\nlocation (50,50,40,30)\ntime 17:28:23", PALE["blue"]),
              ("Candidates ⊂ $G_t$", "orange (fruit)\nOrange (city)\norange (colour)\nOrange River …", PALE["orange"]),
              ("Link in $G_l$", "e ↦ wd:Q13191\n(orange, the fruit)\nor NIL ($l_∅$)", PALE["aqua"])]
    x, width, gap = 0.2, 2.15, 0.42
    for index, (title, body, fill) in enumerate(stages):
        box(axis, x + index * (width + gap), 1.0, width, 1.9, title, body, fill)
    for index, label in enumerate(["$f_o$", "$f_r$ (YOLO)", "$f_c$ (Alg. 1)", "$f_d$ (Alg. 2)"]):
        start = x + index * (width + gap) + width
        arrow(axis, (start, 1.95), (start + gap, 1.95), label, label_offset=(0, 1.05))
    left = x + 2 * (width + gap) + width / 2
    right = x + 4 * (width + gap) + width / 2
    axis.annotate("", (right, 0.75), (left, 0.75), arrowprops={"arrowstyle": "-", "color": INK_SOFT})
    axis.text((left + right) / 2, 0.55, "mapping process $f_m = f_d ∘ f_c$ — what the paper evaluates",
              ha="center", va="top", fontsize=9, color=INK)
    axis.text(x + 1.5 * (width + gap) + width / 2 - 1.3, 0.55, "recognition (not evaluated)", ha="center",
              va="top", fontsize=8, color=INK_SOFT)
    axis.set_title("Perceived-Entity Linking pipeline (after Adamik et al. 2024, Fig. 6)")
    save(figure, FIGURES, "pel_pipeline")


def original_vs_improved():
    figure, axis = canvas(13, 5.4)
    axis.text(0.2, 5.15, "Original disambiguation (Algorithm 2)", fontsize=10, fontweight="bold")
    axis.text(6.8, 5.15, "Improved disambiguation (this project)", fontsize=10, fontweight="bold")
    # original
    box(axis, 0.2, 3.4, 2.6, 1.35, "label senses", "ALL WordNet senses\n(nouns, verbs, adjectives)")
    box(axis, 3.3, 3.4, 2.7, 1.35, "candidate classes", "labels of instance-of /\nsubclass-of classes")
    box(axis, 1.4, 1.5, 3.6, 1.35, "symmetric path similarity",
        "max 1/(1+d(class sense, label sense))\n= 1 when class IS the label", PALE["orange"])
    box(axis, 1.4, 0.1, 3.6, 0.95, "ranking", "sort by this one score", PALE["grey"])
    arrow(axis, (1.5, 3.4), (2.6, 2.85)); arrow(axis, (4.65, 3.4), (3.8, 2.85)); arrow(axis, (3.2, 1.5), (3.2, 1.05))
    # improved
    box(axis, 6.8, 3.4, 1.8, 1.35, "label senses", "H2: noun /\nphysical senses", PALE["blue"])
    box(axis, 8.8, 3.4, 1.8, 1.35, "candidate\nclasses", "", PALE["grey"])
    box(axis, 10.8, 3.4, 2.0, 1.35, "more evidence", "outgoing links\nname similarity\nH4: description", PALE["blue"])
    box(axis, 6.8, 1.5, 3.8, 1.35, "H1: directed hypernym score",
        "1/d if the class is a proper\ngeneralisation of the label, else 0", PALE["blue"])
    box(axis, 7.9, 0.1, 4.9, 0.95, "H3: evidence fusion",
        "score = Σ weight · evidence   (weights fixed on development data)", PALE["aqua"])
    arrow(axis, (7.7, 3.4), (8.3, 2.85)); arrow(axis, (9.7, 3.4), (9.1, 2.85))
    arrow(axis, (8.7, 1.5), (9.6, 1.05)); arrow(axis, (11.8, 3.4), (11.5, 1.05))
    axis.plot([6.4, 6.4], [0.1, 5.2], color="#e3e2dc", linewidth=1.2)
    save(figure, FIGURES, "original_vs_improved_disambiguation")


def research_journey():
    figure, axis = canvas(13, 4.4)
    steps = [("Research problem", "sensor data ↦\nKG identifier?", PALE["grey"]),
             ("Assumptions", "labels suffice;\nWordNet bridges;\nphysical prior", PALE["grey"]),
             ("Original method", "queries S/L(+links)\nAlg. 1, Alg. 2", PALE["orange"]),
             ("Implementation", "research/pel/\noffline unit tests", PALE["orange"]),
             ("Reproduction", "Table 1 re-run on\n2026 graphs", PALE["orange"]),
             ("Weaknesses", "instance trap;\nunused evidence;\nrecall ceiling;\nnoisy benchmark", PALE["blue"]),
             ("Improvements", "H1 directed score\nH2 senses, H3 fusion\nH4 glosses\nH5 back-off", PALE["blue"]),
             ("Validation", "dev/test split,\npaired tests,\nablations", PALE["aqua"]),
             ("Improved method", "H1 + H3 (+ H4):\nwhat survived\nthe evidence", PALE["aqua"])]
    width, height, gap = 2.25, 1.55, 0.35
    for index, (title, body, fill) in enumerate(steps):
        row, column = divmod(index, 5)
        x = 0.25 + column * (width + gap) + (row * (width + gap) / 2)
        y = 2.55 - row * 2.2
        box(axis, x, y, width, height, title, body, fill, title_size=8.5)
        if column < 4 and index < len(steps) - 1:
            arrow(axis, (x + width, y + height / 2), (x + width + gap, y + height / 2))
    arrow(axis, (0.25 + 4 * (width + gap) + width / 2, 2.55), (0.25 + (width + gap) / 2 + width / 2 + 0.2, 1.95),
          colour="#b9b8b2")
    axis.text(0.25, 4.25, "orange = the original paper     blue = critique and hypotheses     "
              "green = evidence", fontsize=8, color=INK_SOFT)
    save(figure, FIGURES, "research_journey")


if __name__ == "__main__":
    pipeline()
    original_vs_improved()
    research_journey()
