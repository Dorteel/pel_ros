# Critical analysis of the PEL baseline

Subject: Adamik et al., *Advancing Robotic Perception with Perceived-Entity
Linking* (ISWC 2024). Evidence comes from the reproduction
(`research/reproduction/results.md`) and from analyses on the **development
data only** (MS COCO + development half of the Visual Genome sample), so that
the test half stays unseen for the improvement study. Executable versions of
every demonstration: `research/tutorial/07_weaknesses.ipynb`.

The paper's lasting contributions — the task definition, its decomposition
into observation, recognition and mapping, and the benchmark protocol — are
not in question here. The criticism concerns the baseline method and the
evaluation.

## A. Limitations acknowledged by the authors

| # | Limitation | Where |
|---|---|---|
| A1 | Labels are linked one at a time; context of co-perceived entities unused | Sec. 5.3, 6 |
| A2 | Attributes from recognition (colour, size, shape) unused | Sec. 5.3 |
| A3 | Candidates come from WordNet alternatives only | Sec. 5.3 |
| A4 | Link counts "might not be a good indicator of the candidates' correctness" | Sec. 5.3 |
| A5 | Instance-level linking (Perceived-Entity Recognition) deferred | Sec. 3.3 |
| A6 | The disambiguation is called "naive"; more sophisticated EL to be investigated | Sec. 4.2, 5.3 |

## B. Limitations observed in the reproduction

### O1 — The instance trap (algorithmic design) — *high significance*

Algorithm 2 ranks by symmetric path similarity between a candidate's class
and the label; the maximum, 1.0, is reached when the class *is* a sense of
the label. Such a candidate is an instance or a kind of the perceived concept,
not the concept. The concept itself is typed by generalisations and scores at
most 0.5.

* "dog" → WikiData, `L_links`, `f_c+f_d`: ranks 1–2 are museum objects of class
  *andiron* (a WordNet synonym of "dog"), score 1.0; the entity for the
  animal is at rank 29 with score 0.
* YAGO 4.5: `Apple_generic_instance`, `Truck_generic_instance` (typed by the
  class) outrank `Apple`, `Truck`.
* Development data, labels where `f_c+f_d` ranks a wrong entity first although
  the right one is in the pool: the winner has a class equal to a label sense
  in **74 of 144 cases on WikiData (51 %)**, **164 of 198 on YAGO (83 %)**,
  **0 of 31 on DBpedia**.
* Consequence in the reproduction: with `L_links` on Visual Genome, `f_d`
  lowers hit@1 from 0.58 to 0.42 (WikiData) and from 0.61 to 0.19 (YAGO),
  and leaves DBpedia unchanged (0.65).

This also explains why the paper could report `f_d` as roughly neutral: on
DBpedia the trap does not exist (resources are typed by ontology classes, not
by other resources), and YAGO 4 had no generic instances.

### O2 — Senses that cannot be perceived (theoretical assumption) — *frequent, low measured impact*

Algorithms 1–2 use every WordNet sense, of any part of speech. "couch" yields
the names put, redact, cast, frame; "bear" yields 34 names, 33 of them only
through verb senses. Share of labels with at least one name from a non-noun sense:
34 % (COCO), 14 % (Visual Genome); with a name from a non-physical sense:
41 % / 21 %. The "physical prior" that motivates Algorithm 2 (Sec. 4.2) is
not encoded anywhere. For "couch" on WikiData the top result of `f_c+f_d` is
a picture frame (class *frame* = verb synonym of "to couch").

### O3 — Evidence is used one piece at a time (algorithmic design) — *high significance*

Each ranking uses exactly one signal: link count (`f_q`, `f_c` on `_links`)
or class similarity (`f_d`). The processes fail on different labels:
development hit@1 of the best single process vs. "at least one of the three
is right": WikiData 0.62 vs 0.76, DBpedia 0.75 vs 0.82, YAGO 0.59 vs 0.65.
Class similarity is exactly 0 for 42–56 % of COCO candidates, so much of the
final order is decided by ties.

### O4 — Recall ceiling (algorithmic design + benchmark) — *high significance*

Even after candidate selection the correct entity is absent from the pool for
21 % (WikiData), 21 % (DBpedia) and 30 % (YAGO) of Visual Genome entries.
Comparing the English label of each missed ground-truth entity with the names
tried (`results/additional/retrieval_misses.json`):

| Reason | WikiData (124 of 600) | DBpedia (32 of 150) | YAGO (178 of 600) |
|---|---|---|---|
| case differs only | 10 | 1 | 8 |
| parenthesised qualifier ("Cameo (carving)") | 1 | 4 | 34 |
| name is part of a longer label ("base" / "military base") | 32 | 8 | 37 |
| different word ("ape" / "Hominoidea") | 77 | 17 | 99 |
| ground-truth entity has no English label | 4 | 2 | 0 |

Only the first two rows (2–7 % of entries) are fixable by better string
matching. The rest reflects the benchmark: labels that under-specify their
target, and ground-truth links that are debatable.

### O5 — The benchmark's ground truth and metric (evaluation methodology) — *high significance*

* **Label under-specification.** The label is "the synset's lemma"; the first
  lemma of *military base* is "base". 21 labels of the full Visual Genome set
  occur with two different targets.
* **Noisy automatic links.** P8814 links *accelerator* to a valve-train item,
  *alfalfa* to *hay*, taxa instead of common-name items (*ape* → Hominoidea).
* **Selection effect.** Only synsets present in all three graphs are
  evaluated; the ±2 % margin of Sec. 5.1 describes sampling noise under
  random sampling, which this is not.
* **No uncertainty.** Table 1 has no intervals or tests; with 80 COCO labels
  the 95 % half-width is about ±0.10.
* **Internally inconsistent cells.** Two cells have hit@k decreasing in k.
* **Metric definition** (correct links / generated links) differs from the
  one used (correct links / all labels).
* **NIL is never evaluated**, although `l_∅` is part of the task definition
  and the baseline has no mechanism to choose it.
* **Named individuals in an "object class" benchmark.** Visual Genome synsets
  include people and places (Emerson, Chandler, Brasilia), which is entity
  linking proper rather than perceived-entity typing.

### O6 — Fragility to surface conventions (robustness) — *high significance, easy to fix*

Exact, case-sensitive matching: COCO → YAGO with lower-case labels gives hit@1
0.03 (`L`, `f_q`) instead of 0.51 with capitalised labels. The paper does not
mention casing. For the unordered `S` and `L` queries the result order is
chosen by the triple store, so their hit@k are partly artefacts of the engine.

### O7 — Dependence on graph version and modelling (generalisation)

The same method, same labels: `f_d` on YAGO changes from neutral (published,
YAGO 4) to destructive (YAGO 4.5). Results of this method are a property of
(method × graph snapshot), which the paper's single snapshot cannot show.

### O8 — Cost (computational complexity)

Per label: 4 query types × (1 + number of WordNet names) retrieval queries
plus class look-ups for up to 50 candidates per name — about 30 requests per
label per graph in our runs; hours for a few hundred labels on public
endpoints. Not usable on-line without a local graph or cache.

## C. Hypothesised limitations (not tested here)

* H-a: accuracy on labels from open-vocabulary detectors (free phrases) will
  be lower than on WordNet-covered vocabulary.
* H-b: with real detections, label noise from the recogniser will dominate
  linking errors; the paper's evaluation assumes perfect labels.
* H-c: context (co-occurring objects) would resolve much of the label
  under-specification in O5.
* H-d: non-English labels/graphs break both WordNet steps.
* H-e: the public-endpoint dependency makes results irreproducible over time
  unless snapshots are archived.

## D. Prioritisation

| Weakness | Significance | Feasible here | Expected measurable gain | Action |
|---|---|---|---|---|
| O1 instance trap | high | yes, no new data | large for `f_d` on WikiData/YAGO | **H1** |
| O3 single evidence | high | yes | moderate | **H3**, **H4**, **H5** |
| O2 unperceivable senses | medium | yes | small | **H2** |
| O4 string-matching part | medium | needs new queries on throttled endpoints | ≤ 2–7 points | future work |
| O4/O5 benchmark | high | needs a new benchmark | — | documented |
| O6 casing | high | done as part of the reproduction (I3) | — | documented |
| A1, A2, NIL | high | needs images / new ground truth | — | future work |
