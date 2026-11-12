"""Shared figure style: one palette, recessive axes, PNG + SVG export."""
from __future__ import annotations

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt   # noqa: E402

# Categorical slots 1-3 (validated as a colour-blind-safe triple); colour
# follows the knowledge graph everywhere.
KG_COLOURS = {"wikidata": "#2a78d6", "dbpedia": "#eb6834", "yago": "#1baf7a"}
KG_NAMES = {"wikidata": "WikiData", "dbpedia": "DBpedia", "yago": "YAGO"}
ORDINAL_BLUES = ["#104281", "#256abf", "#5598e7", "#9ec5f4"]   # dark = best outcome
NEUTRAL = "#b9b8b2"
INK, INK_SOFT, GRID = "#0b0b0b", "#52514e", "#e3e2dc"
METHOD_COLOURS = {"original": "#52514e", "improved": "#2a78d6"}

plt.rcParams.update({
    "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb", "savefig.facecolor": "#fcfcfb",
    "font.size": 9, "axes.titlesize": 10, "axes.titleweight": "bold", "axes.titlelocation": "left",
    "axes.edgecolor": INK_SOFT, "axes.labelcolor": INK_SOFT, "text.color": INK,
    "xtick.color": INK_SOFT, "ytick.color": INK_SOFT, "axes.spines.top": False,
    "axes.spines.right": False, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
    "axes.axisbelow": True, "legend.frameon": False, "svg.fonttype": "none",
    "svg.hashsalt": "pel-research",      # deterministic SVG output
})


def save(figure, directory: Path, name: str) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    for extension in ("png", "svg"):
        figure.savefig(directory / f"{name}.{extension}", dpi=200, bbox_inches="tight",
                       metadata={"Date": None} if extension == "svg" else None)
    plt.close(figure)
    print("wrote", directory / f"{name}.png")
