"""Figures of the reproduction, generated only from stored results.

Usage:  python research/experiments/make_reproduction_figures.py
Reads   research/experiments/results/reproduction/*.json  (+ paper_table1.json)
Writes  research/experiments/figures/reproduction/*.png|svg
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

RESEARCH = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RESEARCH))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np                                             # noqa: E402
from scipy.stats import spearmanr                              # noqa: E402

from plot_style import (INK, INK_SOFT, KG_COLOURS, KG_NAMES, NEUTRAL, ORDINAL_BLUES,  # noqa: E402
                        plt, save)
from pel.candidate_selection import select_candidate_names    # noqa: E402
from pel.wordnet_access import get_synsets                     # noqa: E402

RESULTS = RESEARCH / "experiments" / "results" / "reproduction"
FIGURES = RESEARCH / "experiments" / "figures" / "reproduction"
KGS = ("wikidata", "dbpedia", "yago")
DATASETS = {"coco": "MS COCO", "visual_genome": "Visual Genome"}
QUERIES = ("S", "S_links", "L", "L_links")
PROCESSES = ("fq", "fc", "fc+fd")
PROCESS_NAMES = {"fq": r"$f_q$", "fc": r"$f_c$", "fc+fd": r"$f_c{+}f_d$"}
QUERY_NAMES = {"S": "S", "S_links": r"S$_{links}$", "L": "L", "L_links": r"L$_{links}$"}


def figure_hit1_by_condition(summary, paper):
    """Reproduced hit@1 (bars, Wilson 95 % CI) against the published value (diamond)."""
    datasets = [d for d in DATASETS if d in summary]
    figure, axes = plt.subplots(len(datasets), 3, figsize=(12, 3.1 * len(datasets)),
                                sharey=True, squeeze=False)
    conditions = [f"{q}|{p}" for q in QUERIES for p in PROCESSES]
    x = np.arange(len(conditions)) + np.repeat(np.arange(len(QUERIES)), len(PROCESSES)) * 0.6
    for row, dataset in enumerate(datasets):
        for column, kg in enumerate(KGS):
            axis = axes[row][column]
            cells = [summary[dataset][c].get(kg) for c in conditions]
            if all(cells):
                values = np.array([cell["hit@1"] for cell in cells])
                low = values - np.array([cell["hit@1_ci95"][0] for cell in cells])
                high = np.array([cell["hit@1_ci95"][1] for cell in cells]) - values
                axis.bar(x, values, width=0.8, color=KG_COLOURS[kg], label="reproduced (95 % CI)")
                axis.errorbar(x, values, yerr=[low, high], fmt="none", ecolor=INK, elinewidth=0.8,
                              capsize=2)
                n = cells[0]["n"]
            else:
                n = "–"
            published = [paper[dataset][c][kg][0] for c in conditions]
            axis.scatter(x, published, marker="D", s=22, color="white", edgecolor=INK,
                         linewidth=1.0, zorder=3, label="published (Table 1)")
            axis.set_xticks(x, [PROCESS_NAMES[c.split("|")[1]] for c in conditions], fontsize=7)
            for group, query in enumerate(QUERIES):
                axis.text(x[group * 3 + 1], -0.2, QUERY_NAMES[query], ha="center", va="top",
                          color=INK_SOFT, transform=axis.get_xaxis_transform())
            axis.set_ylim(0, 1)
            axis.grid(axis="x", visible=False)
            axis.set_title(f"{DATASETS[dataset]} → {KG_NAMES[kg]}  (n = {n})")
            if column == 0:
                axis.set_ylabel("hit@1 (linking accuracy)")
    handles, names = axes[0][0].get_legend_handles_labels()
    if len(handles) < 2:
        handles, names = axes[0][-1].get_legend_handles_labels()
    figure.tight_layout(h_pad=2.5)
    figure.legend(handles, names, loc="lower center", ncol=2, fontsize=8,
                  bbox_to_anchor=(0.5, -0.04))
    save(figure, FIGURES, "hit1_reproduced_vs_published")


def figure_scatter(summary, paper):
    """Every Table-1 cell: published value against reproduced value."""
    figure, axes = plt.subplots(1, 2, figsize=(9, 4.2), sharex=True, sharey=True)
    statistics = {}
    for axis, dataset in zip(axes, DATASETS):
        axis.plot([0, 1], [0, 1], color=NEUTRAL, linewidth=1, zorder=1)
        axis.set_title(DATASETS[dataset])
        axis.set_xlabel("published hit@k")
        if dataset not in summary:
            continue
        for kg in KGS:
            xs, ys = [], []
            for condition, cells in summary[dataset].items():
                if condition.startswith("_") or kg not in cells:
                    continue
                for index, k in enumerate((1, 3, 5)):
                    xs.append(paper[dataset][condition][kg][index])
                    ys.append(cells[kg][f"hit@{k}"])
            if xs:
                rho = spearmanr(xs, ys).statistic
                statistics[f"{dataset}/{kg}"] = {
                    "spearman_rho": rho, "cells": len(xs),
                    "mean_abs_difference": float(np.mean(np.abs(np.array(xs) - np.array(ys))))}
                axis.scatter(xs, ys, s=22, color=KG_COLOURS[kg], edgecolor="#fcfcfb", linewidth=0.6,
                             label=f"{KG_NAMES[kg]} (ρ = {rho:.2f})", zorder=2)
        axis.legend(loc="upper left", fontsize=8)
        axis.set_xlim(0, 1), axis.set_ylim(0, 1), axis.set_aspect("equal")
    axes[0].set_ylabel("reproduced hit@k")
    figure.tight_layout()
    save(figure, FIGURES, "published_vs_reproduced_scatter")
    (RESULTS / "agreement_with_paper.json").write_text(json.dumps(statistics, indent=1))


def figure_failure_decomposition():
    """Where does each process lose the correct entity?  (L_links query)"""
    categories = ["rank 1", "rank 2–5", "rank > 5", "not retrieved", "nothing returned"]
    colours = ORDINAL_BLUES[:3] + [NEUTRAL, "#e3e2dc"]
    datasets = [d for d in DATASETS if any((RESULTS / f"{d}_{kg}.json").exists() for kg in KGS)]
    figure, axes = plt.subplots(1, len(datasets), figsize=(5.6 * len(datasets), 3.6), squeeze=False)
    table = {}
    for axis, dataset in zip(axes[0], datasets):
        labels, rows = [], []
        for kg in KGS:
            path = RESULTS / f"{dataset}_{kg}.json"
            if not path.exists():
                continue
            records = json.loads(path.read_text())["records"]
            for process in PROCESSES:
                condition = f"L_links|{process}"
                counts = np.zeros(5)
                for record in records:
                    rank, returned = record["first_hit_rank"][condition], record["returned"][condition]
                    index = (4 if returned == 0 else 3 if rank is None else
                             0 if rank == 1 else 1 if rank <= 5 else 2)
                    counts[index] += 1
                rows.append(counts / counts.sum())
                labels.append(f"{KG_NAMES[kg]}  {PROCESS_NAMES[process]}")
                table[f"{dataset}/{kg}/{process}"] = dict(zip(categories, (counts / counts.sum()).tolist()))
        rows = np.array(rows)
        y = np.arange(len(labels))[::-1] + np.repeat(np.arange(len(labels) // 3), 3)[::-1] * 0.5
        left = np.zeros(len(labels))
        for index, category in enumerate(categories):
            axis.barh(y, rows[:, index], left=left, height=0.8, color=colours[index],
                      edgecolor="#fcfcfb", linewidth=1.5, label=category)
            left += rows[:, index]
        axis.set_yticks(y, labels, fontsize=8)
        axis.set_xlim(0, 1)
        axis.grid(axis="y", visible=False)
        axis.set_xlabel("share of labels")
        axis.set_title(f"{DATASETS[dataset]}: fate of the correct entity (L$_{{links}}$)")
    axes[0][-1].legend(loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=5, fontsize=8)
    figure.tight_layout()
    save(figure, FIGURES, "failure_decomposition")
    (RESULTS / "failure_decomposition.json").write_text(json.dumps(table, indent=1))


def figure_mechanisms():
    """The two similarity functions the method is built on."""
    figure, axes = plt.subplots(1, 3, figsize=(12, 3.3), gridspec_kw={"width_ratios": [1, 1, 1.1]})
    for axis, label in zip(axes[:2], ("orange", "couch")):
        names = select_candidate_names(label)
        y = np.arange(len(names))[::-1]
        axis.barh(y, [score for score, _ in names], height=0.7, color=KG_COLOURS["wikidata"])
        axis.set_yticks(y, [name for _, name in names])
        axis.set_xlim(0, 1)
        axis.grid(axis="y", visible=False)
        axis.set_xlabel("Levenshtein similarity to the label")
        axis.set_title(f"Algorithm 1: names tried for “{label}”")
    axis = axes[2]
    distances = np.arange(0, 15)
    axis.plot(distances, 1 / (1 + distances), color=INK_SOFT, linewidth=2, marker="o", markersize=4)
    orange = get_synsets("orange")[0]
    for class_label, offset in (("citrus", (12, 8)), ("fruit", (12, 8)), ("food", (12, -14)),
                                ("city", (-6, 14)), ("color", (-30, 14))):
        distance = min(orange.shortest_path_distance(s) for s in get_synsets(class_label)
                       if orange.shortest_path_distance(s) is not None)
        axis.annotate(class_label, (distance, 1 / (1 + distance)), xytext=offset,
                      textcoords="offset points", fontsize=8,
                      arrowprops={"arrowstyle": "-", "color": INK_SOFT, "linewidth": 0.6})
        axis.scatter([distance], [1 / (1 + distance)], s=40, color=KG_COLOURS["dbpedia"], zorder=3)
    axis.set_xlabel("WordNet path length d to orange.n.01 (fruit)")
    axis.set_ylabel("path similarity 1/(1+d)")
    axis.set_title("Algorithm 2: class-to-label similarity")
    figure.tight_layout()
    save(figure, FIGURES, "mechanism_similarity_functions")


def main() -> None:
    summary = json.loads((RESULTS / "summary.json").read_text())
    paper = json.loads((RESEARCH / "experiments" / "paper_table1.json").read_text())
    figure_hit1_by_condition(summary, paper)
    figure_scatter(summary, paper)
    figure_failure_decomposition()
    figure_mechanisms()


if __name__ == "__main__":
    main()
