"""Comparative figures: original versus improved mapping process.

Usage:  python research/experiments/make_improvement_figures.py [--query-type L_links]
Reads   research/experiments/results/improvements/analysis_<query_type>.json
Writes  research/experiments/figures/improvements/*.png|svg
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from plot_style import (INK, INK_SOFT, KG_COLOURS, KG_NAMES, NEUTRAL, ORDINAL_BLUES,  # noqa: E402
                        plt, save)

RESEARCH = Path(__file__).resolve().parents[1]
FIGURES = RESEARCH / "experiments" / "figures" / "improvements"
ORIGINAL = ("fq", "fc", "fc+fd")
NICE = {"fq": "original $f_q$", "fc": "original $f_c$", "fc+fd": "original $f_c{+}f_d$",
        "full (combined)": "improved (combined)"}
SUFFIX = ""
ORIGINAL_GREY, SINGLE_BLUE, FULL_BLUE = "#8a8983", "#9ec5f4", "#256abf"


def colour_of(name: str) -> str:
    return ORIGINAL_GREY if name in ORIGINAL else FULL_BLUE if name.startswith("full (") else SINGLE_BLUE


def figure_methods(report, split):
    """hit@1 of every method on one split, one panel per graph."""
    kgs = report["knowledge_graphs"]
    names = [n for n in next(iter(report[split].values()))["methods"] if "−" not in n and "toggled" not in n and not n.startswith("original ")]
    figure, axes = plt.subplots(1, len(kgs), figsize=(4.6 * len(kgs), 0.36 * len(names) + 1.3),
                                sharey=True, squeeze=False)
    y = np.arange(len(names))[::-1]
    for axis, kg in zip(axes[0], kgs):
        cells = [report[split][kg]["methods"][n] for n in names]
        values = np.array([c["hit@1"] for c in cells])
        errors = [values - [c["hit@1_ci95"][0] for c in cells], [c["hit@1_ci95"][1] for c in cells] - values]
        axis.barh(y, values, height=0.72, color=[colour_of(n) for n in names])
        axis.errorbar(values, y, xerr=errors, fmt="none", ecolor=INK, elinewidth=0.8, capsize=2)
        for yi, value, cell in zip(y, values, cells):
            axis.text(min(cell["hit@1_ci95"][1] + 0.02, 0.97), yi, f"{value:.2f}", va="center", fontsize=7.5)
        axis.set_yticks(y, [NICE.get(n, n) for n in names], fontsize=8)
        axis.set_xlim(0, 1.05), axis.grid(axis="y", visible=False)
        axis.set_xlabel("hit@1 (95 % Wilson interval)")
        axis.set_title(f"{KG_NAMES[kg]} — {split} set, n = {cells[0]['n']}")
    figure.tight_layout()
    save(figure, FIGURES, f"methods_hit1_{split}" + SUFFIX)


def figure_paired(report):
    """Paired hit@1 differences on the test set with bootstrap intervals."""
    kgs = report["knowledge_graphs"]
    figure, axis = plt.subplots(figsize=(10.5, 0.55 * 3 * len(kgs) + 1.2))
    rows = []
    for kg in kgs:
        for name, comparison in report["test"][kg]["comparisons"].items():
            if name.startswith("full vs"):
                rows.append((kg, name.replace("full vs ", "improved − "), comparison))
    y = np.arange(len(rows))[::-1]
    for yi, (kg, name, comparison) in zip(y, rows):
        low, high = comparison["difference_ci95"]
        axis.plot([low, high], [yi, yi], color=KG_COLOURS[kg], linewidth=2.2, solid_capstyle="round")
        axis.scatter([comparison["difference"]], [yi], s=46, color=KG_COLOURS[kg], edgecolor="#fcfcfb",
                     linewidth=1.5, zorder=3)
        axis.text(max(high, 0) + 0.015, yi, f"{comparison['difference']:+.3f}   p = {comparison['mcnemar_exact_p']:.3g}",
                  va="center", fontsize=8)
    axis.axvline(0, color=INK_SOFT, linewidth=1)
    axis.set_yticks(y, [f"{KG_NAMES[kg]}: {name}" for kg, name, _ in rows], fontsize=8)
    axis.set_xlabel("difference in test hit@1  (bar: paired bootstrap 95 % interval;  p: exact McNemar)")
    axis.set_xlim(min(-0.1, min(r[2]["difference_ci95"][0] for r in rows) - 0.05),
                  max(r[2]["difference_ci95"][1] for r in rows) + 0.3)
    axis.grid(axis="y", visible=False)
    axis.set_title("Is the improvement real?  Paired comparisons on identical labels")
    figure.tight_layout()
    save(figure, FIGURES, "paired_differences_test" + SUFFIX)


def figure_ablation(report):
    """Change in test hit@1 when one ingredient is removed from the combined method."""
    kgs = report["knowledge_graphs"]
    names = [n for n in report["test"][kgs[0]]["methods"] if "−" in n or "toggled" in n]
    figure, axis = plt.subplots(figsize=(7.5, 0.5 * len(names) + 1.4))
    y = np.arange(len(names))[::-1]
    offsets = np.linspace(-0.22, 0.22, len(kgs)) if len(kgs) > 1 else [0.0]
    for kg, offset in zip(kgs, offsets):
        full = report["test"][kg]["methods"]["full (combined)"]["hit@1"]
        deltas = [report["test"][kg]["methods"][n]["hit@1"] - full for n in names]
        axis.barh(y + offset, deltas, height=0.4 / max(len(kgs) - 1, 1) if len(kgs) > 1 else 0.5,
                  color=KG_COLOURS[kg], label=KG_NAMES[kg])
    axis.axvline(0, color=INK_SOFT, linewidth=1)
    axis.set_yticks(y, [n.replace("full − ", "without ").replace("full, ", "") for n in names], fontsize=8)
    axis.set_xlabel("change in test hit@1 relative to the combined method")
    axis.grid(axis="y", visible=False)
    axis.legend(loc="lower left", fontsize=8)
    axis.set_title("Ablation: what each ingredient contributes")
    figure.tight_layout()
    save(figure, FIGURES, "ablation_test" + SUFFIX)


def figure_rank_shift(report):
    """Where the correct entity ends up, before and after (test set)."""
    kgs = report["knowledge_graphs"]
    categories = ["rank 1", "rank 2–5", "rank > 5", "not in pool"]
    colours = ORDINAL_BLUES[:3] + [NEUTRAL]
    methods = ["fq", "fc", "fc+fd", "full (combined)"]
    figure, axes = plt.subplots(1, len(kgs), figsize=(4.4 * len(kgs), 2.6), sharey=True, squeeze=False)
    for axis, kg in zip(axes[0], kgs):
        left = np.zeros(len(methods))
        y = np.arange(len(methods))[::-1]
        shares = []
        for method in methods:
            ranks = report["test"][kg]["ranks"][method]
            counts = [sum(r == 1 for r in ranks), sum(r is not None and 2 <= r <= 5 for r in ranks),
                      sum(r is not None and r > 5 for r in ranks), sum(r is None for r in ranks)]
            shares.append(np.array(counts) / len(ranks))
        shares = np.array(shares)
        for index, category in enumerate(categories):
            axis.barh(y, shares[:, index], left=left, height=0.72, color=colours[index],
                      edgecolor="#fcfcfb", linewidth=1.5, label=category)
            left += shares[:, index]
        axis.set_yticks(y, [NICE[m] for m in methods], fontsize=8)
        axis.set_xlim(0, 1), axis.grid(axis="y", visible=False)
        axis.set_xlabel("share of test labels")
        axis.set_title(KG_NAMES[kg])
    handles, names = axes[0][0].get_legend_handles_labels()
    figure.tight_layout(rect=(0, 0.1, 1, 1))
    figure.legend(handles, names, loc="lower center", ncol=4, fontsize=8)
    save(figure, FIGURES, "rank_of_correct_entity_test" + SUFFIX)


def figure_weight_sensitivity(report):
    """Best development hit@1 reachable when one weight is fixed."""
    features = ["name", "type", "gloss", "popularity"]
    colours = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]
    selected = report["selected_on_development"]
    grid = [g for g in report["weight_search"] if g["inventory"] == selected["inventory"]
            and g["word_backoff"] == selected["word_backoff"]]
    figure, axis = plt.subplots(figsize=(6.4, 3.4))
    steps = np.round(np.arange(0, 1.01, 0.1), 1)
    for index, (feature, colour) in enumerate(zip(features, colours)):
        best = [max((g["dev_hit@1"] for g in grid if abs(g["weights"][index] - step) < 1e-6), default=np.nan)
                for step in steps]
        axis.plot(steps, best, color=colour, linewidth=2, marker="o", markersize=4, label=feature)
        axis.scatter([selected["weights"][feature]], [selected["objective_hit@1_hit@3_mrr"][0]],
                     s=70, facecolor="none", edgecolor=colour, linewidth=1.6, zorder=3)
    axis.set_xlabel("weight fixed for one evidence (others re-optimised)")
    axis.set_ylabel("best development hit@1\n(mean over graphs)")
    axis.legend(title="evidence", fontsize=8, title_fontsize=8, loc="lower left")
    axis.set_title("Sensitivity of the fusion weights (circles: selected)")
    figure.tight_layout()
    save(figure, FIGURES, "weight_sensitivity_development" + SUFFIX)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--query-type", default="L_links")
    args = parser.parse_args()
    global SUFFIX
    SUFFIX = "" if args.query_type == "L_links" else f"_{args.query_type}"
    report = json.loads((RESEARCH / "experiments" / "results" / "improvements"
                         / f"analysis_{args.query_type}.json").read_text())
    for split in ("development", "test"):
        figure_methods(report, split)
    figure_paired(report)
    figure_ablation(report)
    figure_rank_shift(report)
    figure_weight_sensitivity(report)


if __name__ == "__main__":
    main()
