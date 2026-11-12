# Reproduction results

Measured 10–12 November 2026 against the live SPARQL endpoints of WikiData,
DBpedia and YAGO (4.5). Stored measurements:
`research/experiments/results/reproduction/`. Everything here is regenerated
from those files by `summarise_results.py` and `make_reproduction_figures.py`.

Kinds of numbers: **published** = the paper's Table 1; **reproduced** = this
project, original method; **additional** = analyses not in the paper.

## 1. Sample sizes

| Dataset | WikiData | DBpedia | YAGO | Paper |
|---|---|---|---|---|
| MS COCO | 80 | 80 | 80 | 80 |
| Visual Genome | 600 | 150 | 600 | 2,399 |

Visual Genome: 1,965 synsets are linkable in all three graphs today (paper:
2,399). A seeded simple random sample of 600 of them is evaluated; for
DBpedia, whose endpoint throttled this project to about 30 requests per
minute, a seeded random subset of 150 of those 600. All queries of all runs
completed (no failed query in the final runs).

## 2. Table 1, reproduced

Full table with published values in parentheses:
[`table1.md`](../experiments/results/reproduction/table1.md); with Wilson 95 %
intervals: `summary.json`. hit@1 only:

**MS COCO (n = 80; 95 % half-width ≈ ±0.10)**

| Query | Process | WikiData | DBpedia | YAGO |
|---|---|---|---|---|
| S | f_q | 0.04 (0.08) | 0.26 (0.19) | 0.31 (0.49) |
| S | f_c | 0.04 (0.07) | 0.29 (0.19) | 0.33 (0.51) |
| S | f_c+f_d | 0.31 (0.07) | 0.31 (0.19) | 0.11 (0.51) |
| S_links | f_q | **0.88** (0.59) | 0.21 (0.24) | 0.51 (0.71) |
| S_links | f_c | 0.64 (0.59) | 0.15 (0.24) | 0.50 (0.62) |
| S_links | f_c+f_d | 0.35 (0.59) | 0.26 (0.24) | 0.11 (0.71) |
| L | f_q | 0.39 (0.50) | 0.78 (0.68) | 0.51 (0.40) |
| L | f_c | 0.44 (0.69) | **0.82** (0.69) | 0.54 (0.53) |
| L | f_c+f_d | 0.50 (0.69) | 0.80 (0.69) | 0.11 (0.55) |
| L_links | f_q | 0.79 (0.51) | 0.79 (0.84) | 0.51 (0.64) |
| L_links | f_c | 0.75 (0.14) | 0.66 (0.45) | **0.55** (0.28) |
| L_links | f_c+f_d | 0.51 (0.51) | 0.64 (0.84) | 0.11 (0.64) |

**Visual Genome (WikiData, YAGO n = 600, ±0.04; DBpedia n = 150, ±0.08)**

| Query | Process | WikiData | DBpedia | YAGO |
|---|---|---|---|---|
| S | f_q | 0.04 (0.01) | 0.25 (0.09) | 0.35 (0.37) |
| S | f_c | 0.04 (0.04) | 0.27 (0.13) | 0.38 (0.45) |
| S | f_c+f_d | 0.28 (0.22) | 0.31 (0.14) | 0.16 (0.38) |
| S_links | f_q | **0.60** (0.09) | 0.19 (0.08) | 0.53 (0.26) |
| S_links | f_c | 0.54 (0.10) | 0.14 (0.11) | 0.56 (0.27) |
| S_links | f_c+f_d | 0.32 (0.31) | 0.25 (0.22) | 0.18 (0.40) |
| L | f_q | 0.42 (0.41) | 0.63 (0.37) | 0.52 (0.36) |
| L | f_c | 0.46 (0.45) | 0.66 (0.40) | 0.56 (0.43) |
| L | f_c+f_d | 0.41 (0.29) | **0.70** (0.51) | 0.18 (0.42) |
| L_links | f_q | 0.56 (0.08) | 0.63 (0.34) | 0.52 (0.25) |
| L_links | f_c | 0.58 (0.05) | 0.65 (0.24) | **0.61** (0.19) |
| L_links | f_c+f_d | 0.42 (0.24) | 0.65 (0.44) | 0.19 (0.39) |

Figures: `experiments/figures/reproduction/hit1_reproduced_vs_published.{png,svg}`,
`published_vs_reproduced_scatter`, `failure_decomposition`,
`mechanism_similarity_functions`.

## 3. Agreement with the publication

Over the 36 cells (12 conditions × hit@1/3/5) of each dataset–graph pair
(`agreement_with_paper.json`):

| Pair | mean abs. difference | Spearman ρ |
|---|---|---|
| COCO / WikiData | 0.14 | 0.71 |
| COCO / DBpedia | 0.10 | 0.89 |
| COCO / YAGO | 0.24 | 0.12 |
| Visual Genome / WikiData | 0.17 | 0.56 |
| Visual Genome / DBpedia | 0.17 | 0.86 |
| Visual Genome / YAGO | 0.13 | 0.20 |

**This is a partial reproduction.** No pair matches numerically. The ranking
of conditions reproduces well on DBpedia, moderately on WikiData, and not on
YAGO.

### What reproduces

* Unordered "any property" queries (`S`) are nearly useless on WikiData
  (hit@1 0.04; published 0.01–0.08).
* On DBpedia, label queries beat simple queries by a wide margin, and `f_c`
  with `L_links` loses hit@1 against `f_q` on COCO (0.79 → 0.66; published
  0.84 → 0.45) while gaining hit@3/5 — the "popular entities push the correct
  one down" effect of Sec. 5.2.
* `f_c` raises hit@3 and hit@5 for label queries on every graph.
* `f_d` is not a reliable gain: it helps the unordered `S` query on WikiData
  (0.04 → 0.28–0.31), is neutral to mildly positive on DBpedia, and lowers
  hit@1 for the link-ordered queries on WikiData.
* DBpedia Visual Genome best condition: `L`, `f_c+f_d`, 0.70 [0.62, 0.77]
  (published best 0.51 for the same condition).

### What does not reproduce, and the likely reasons

| Discrepancy | Assessment |
|---|---|
| YAGO: `f_d` collapses hit@1 to 0.11–0.19 (published 0.38–0.71) | **Explained.** The endpoint now serves YAGO 4.5, which contains `X_generic_instance` entities typed by class `X`; Algorithm 2 gives them the maximum score (see `improvements/critique.md`, O1). YAGO 4 had no such entities. |
| WikiData/YAGO, Visual Genome, `_links`: published hit@1 0.05–0.27 with hit@5 up to 0.68; reproduced hit@1 0.52–0.61 | **Unexplained.** The published pattern (right entity retrieved but rarely first) suggests a different link count or ordering. Sec. 5.3 of the paper mentions "incoming links" while Sec. 5.1 describes outgoing ones. Without the original queries this cannot be settled. |
| WikiData COCO `L`: reproduced below published (0.39–0.50 vs 0.50–0.69) | Unordered query: the order is the triple store's; also a different ground truth. |
| Higher absolute numbers on DBpedia Visual Genome | Different entry set (150 of today's 1,965 vs 2,399 of 2024) and our capitalisation rule (I3). |

Deviations that apply throughout: graphs two years newer (D1), rebuilt COCO
ground truth (D3), ten inferred details (I1–I10), sampled Visual Genome (D2).

### Inconsistencies in the published table

Two cells have hit@k decreasing in k, impossible by definition:
COCO/WikiData/`L`/`f_q` (hit@3 0.70, hit@5 0.65) and
COCO/DBpedia/`S_links`/`f_c` (hit@1 0.24, hit@3 0.19).

## 4. External baselines (Sec. 5.2 of the paper)

Not reproduced. The DBpedia Spotlight web service
(`api.dbpedia-spotlight.org`) refused connections on every attempt on 10–12
November 2026; the AIDA service returned HTTP 404. The published values
(Spotlight 0.525 on COCO; AIDA no hits) are neither confirmed nor
contradicted. `experiments/run_spotlight_baseline.py` is ready to run.

## 5. Additional analyses

### 5.1 Where the correct entity is lost (`L_links`; `failure_decomposition.json`)

Share of labels, Visual Genome:

| Graph | Process | rank 1 | rank 2–5 | rank > 5 | not retrieved | nothing returned |
|---|---|---|---|---|---|---|
| WikiData | f_q | 0.56 | 0.04 | 0.01 | 0.27 | 0.12 |
| WikiData | f_c | 0.58 | 0.17 | 0.04 | 0.17 | 0.04 |
| WikiData | f_c+f_d | 0.42 | 0.23 | 0.14 | 0.17 | 0.04 |
| DBpedia | f_q | 0.63 | 0.01 | 0.00 | 0.30 | 0.07 |
| DBpedia | f_c | 0.65 | 0.13 | 0.01 | 0.19 | 0.03 |
| DBpedia | f_c+f_d | 0.65 | 0.13 | 0.01 | 0.19 | 0.03 |
| YAGO | f_q | 0.52 | 0.00 | 0.00 | 0.35 | 0.13 |
| YAGO | f_c | 0.61 | 0.09 | 0.00 | 0.25 | 0.04 |
| YAGO | f_c+f_d | 0.19 | 0.37 | 0.14 | 0.25 | 0.04 |

Candidate selection does what it is for: it cuts "not retrieved + nothing
returned" from 0.37–0.48 to 0.21–0.29. What remains is a **recall ceiling** of
roughly 0.71–0.79 that no re-ranking can lift, and — under `f_c+f_d` on
WikiData and YAGO — a large block of labels where the right entity is in the
pool but not first.

### 5.2 Sensitivity to label casing (`results/additional/label_case/`)

COCO → YAGO with lower-case labels instead of capitalised ones: hit@1 of
`L`/`f_q` 0.03 instead of 0.51; `L_links`/`f_c` 0.05 instead of 0.55. The
casing convention, which the paper does not mention, decides whether the
method works at all on two of the three graphs.

## 6. Environment

Python 3.12.3; versions pinned in `research/requirements.txt` (NLTK 3.10.4
with WordNet 3.0 and 3.1, requests 2.31.0, numpy 1.26.4, scipy 1.11.4,
matplotlib 3.6.3). Linux, no GPU used. Result limit 50; Visual Genome sample
seed 0 (subset seed 1000). Endpoints: `query.wikidata.org/sparql`,
`dbpedia.org/sparql`, `yago-knowledge.org/sparql/qlever`.

## 7. Limitations of the reproduction

1. Not the authors' code, queries or COCO ground truth (unavailable).
2. Live graphs: numbers describe November 2026; `S`/`L` result order is
   engine-dependent and frozen only in the local cache (not in git).
3. Visual Genome evaluated on samples (600 / 150), not on all entries.
4. External baselines not re-run.
5. Recognition (YOLO) not executed; the paper's evaluation uses labels only.
