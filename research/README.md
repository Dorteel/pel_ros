# Perceived-Entity Linking — reproduction, critique, improvement, tutorial

Research project built around

> M. Adamik, R. Pernisch, I. Tiddi, S. Schlobach. *Advancing Robotic Perception
> with Perceived-Entity Linking.* ISWC 2024, LNCS 15232, pp. 192–209.
> doi:10.1007/978-3-031-77850-6_11

This directory is a plain Python project. It is independent of the ROS 2
packages of the repository (`COLCON_IGNORE` keeps colcon out of it).

## Layout

```
research/
├── pel/                    the implementation (one paper concept per module)
│   ├── observation_graph.py, text_similarity.py, wordnet_access.py
│   ├── knowledge_graphs.py, sparql_client.py
│   ├── candidate_selection.py   Algorithm 1          ┐
│   ├── disambiguation.py        Algorithm 2          ├ original method
│   ├── mapping.py               f_q, f_c, f_c+f_d    ┘
│   ├── improved_scoring.py      H1–H5                ┐ this project's extension
│   ├── improved_mapping.py                           ┘ (improvement branch only)
│   ├── recorded_access.py       offline replay of recorded graph answers
│   └── evaluation.py            hit@k, Wilson interval
├── tests/                  offline unit tests (./test.sh)
├── data/                   COCO labels, ground truth, VG synsets, (ignored) SPARQL caches
├── experiments/            scripts, results/ (stored measurements), figures/
├── reproduction/           specification.md, methodology.md, results.md
├── improvements/           critique.md, hypotheses.md, results.md
├── tutorial/               01 … 10 notebooks (executed), run_notebooks.py
└── manuscript/             improved_paper.md, figures/
```

## Branches

| Branch | Content |
|---|---|
| `research/paper-reproduction` | faithful baseline, frozen; the experimental control |
| `research/paper-improvements` | branches from it; critique, hypotheses, improved method, tutorial, manuscript |

## Quick start

```bash
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -r research/requirements.txt
.venv/bin/python -c "import nltk; [nltk.download(c) for c in ('wordnet','wordnet31','omw-1.4')]"
./test.sh                # unit tests, offline
./test.sh --all          # + regenerate result tables from stored measurements
```

Everything below `experiments/results/` was measured against the live
knowledge graphs in November 2026. Tables and figures are regenerated from
those stored files without network access; re-measuring requires the public
SPARQL endpoints and several hours (see `reproduction/methodology.md`).

## Reading order

1. `tutorial/01_problem.ipynb` … `10_conclusions.ipynb` — the guided tour.
2. `reproduction/specification.md` — what the paper says, what had to be
   interpreted, what deviates.
3. `reproduction/results.md`, `improvements/critique.md`,
   `improvements/hypotheses.md`, `improvements/results.md` — the evidence.
4. `manuscript/improved_paper.md` — the synthesis.
