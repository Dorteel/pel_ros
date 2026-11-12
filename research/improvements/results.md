# Improvement study — results

Branch `research/paper-improvements`; control: `research/paper-reproduction`
(commit `374601a`). Hypotheses: `hypotheses.md`. Stored measurements:
`experiments/results/improvements/` (`candidates/*.json.gz` = recorded
candidate pools and evidence; `analysis_L_links.json`, `analysis_S_links.json`
= everything reported here). Regenerate with

```bash
.venv/bin/python research/experiments/analyse_improvements.py                      # ~15 min, offline
.venv/bin/python research/experiments/analyse_improvements.py --query-type S_links
.venv/bin/python research/experiments/make_improvement_figures.py [--query-type S_links]
```

## 1. Design of the comparison

| Controlled (identical for all methods) | |
|---|---|
| labels and ground truth | the reproduction's entries |
| candidate pool | recorded once per label from the original retrieval (`L_links`; in a second analysis `S_links`), result limit 50 |
| evidence | recorded link counts, class labels, descriptions |
| development / test split | COCO + half of the Visual Genome sample / other half (seed 1) |

| Manipulated | sense inventory (H2), similarity function (H1), class-label back-off (H5), fusion weights (H3), use of descriptions (H4) |
|---|---|
| **Measured** | hit@1 (primary), hit@3, hit@5, MRR; Wilson 95 % intervals; paired bootstrap 95 % interval and exact McNemar test of the hit@1 difference |

Sizes: development 380 (WikiData, YAGO), 147 (DBpedia); test 300 (WikiData,
YAGO), 83 (DBpedia; S_links not recorded for DBpedia). The offline harness
reproduces the stored reproduction results label by label
(`tests/test_improvements.py::test_offline_analysis_matches_stored_reproduction`).

Selected on the development set (one configuration for all graphs):
`L_links` — physical senses, no back-off, weights name 0.2, type 0.1, gloss
0.3, popularity 0.4. `S_links` — physical senses, no back-off, weights 0.1,
0.1, 0.2, 0.6.

## 2. Test-set results, `L_links` pools (hit@1 [95 % CI])

| Method | WikiData (n=300) | DBpedia (n=83) | YAGO (n=300) |
|---|---|---|---|
| original f_q | 0.557 [0.50, 0.61] | 0.578 [0.47, 0.68] | 0.500 [0.44, 0.56] |
| original f_c | 0.583 [0.53, 0.64] | 0.651 [0.54, 0.74] | 0.623 [0.57, 0.68] |
| original f_c+f_d | 0.423 [0.37, 0.48] | 0.651 [0.54, 0.74] | 0.203 [0.16, 0.25] |
| H1 directed hypernym score | 0.563 | 0.651 | 0.587 |
| H2 noun senses | 0.433 | 0.651 | 0.210 |
| H2 physical senses | 0.430 | 0.651 | 0.200 |
| H4 gloss overlap instead of type | 0.517 | 0.651 | 0.557 |
| H5 class-label back-off | 0.467 | 0.639 | 0.237 |
| H3 fusion of original evidence | 0.630 | 0.687 | 0.650 |
| **combined (H1+H2+H3+H4)** | **0.647 [0.59, 0.70]** | **0.663 [0.56, 0.76]** | **0.650 [0.59, 0.70]** |
| best of the 12 original conditions (chosen on development) | 0.610 (S_links, f_q) | 0.639 (L, f_c) | 0.623 (L_links, f_c) |

Rows H1–H5 apply one change to the original `f_c+f_d`.

### Paired comparisons on the test set (Δ hit@1, bootstrap 95 % CI, exact McNemar p)

| Comparison | WikiData | DBpedia | YAGO |
|---|---|---|---|
| combined − original f_c+f_d | **+0.223** [+0.170, +0.277], p < 0.0001 | +0.012 [−0.072, +0.096], p = 1.0 | **+0.447** [+0.387, +0.507], p < 0.0001 |
| combined − best original process on the same pool | +0.090 [+0.050, +0.133], p < 0.0001 (f_q) | +0.084 [+0.036, +0.145], p = 0.016 (f_q) | +0.027 [−0.003, +0.060], p = 0.13 (f_c) |
| combined − best of 12 original conditions | +0.037 [−0.020, +0.093], p = 0.25 | +0.024 [−0.024, +0.072], p = 0.63 | +0.027 [−0.003, +0.060], p = 0.13 |
| H1 − f_c+f_d | **+0.140** [+0.093, +0.187], p < 0.0001 | 0.000 (no label changes) | **+0.383** [+0.327, +0.443], p < 0.0001 |
| H2 noun − f_c+f_d | +0.010, p = 0.51 | 0.000 | +0.007, p = 0.50 |
| H2 physical − f_c+f_d | +0.007, p = 0.75 | 0.000 | −0.003, p = 1.0 |
| H3 − f_c+f_d | **+0.207** [+0.153, +0.257], p < 0.0001 | +0.036 [−0.048, +0.120], p = 0.55 | **+0.447** [+0.387, +0.503], p < 0.0001 |
| H4 − f_c+f_d | +0.093 [+0.043, +0.147], p = 0.001 | 0.000, p = 1.0 | **+0.353** [+0.293, +0.413], p < 0.0001 |
| H5 − f_c+f_d | +0.043 [+0.007, +0.077], p = 0.024 | −0.012, p = 1.0 | +0.033 [0.000, +0.067], p = 0.076 |

("best original process on the same pool" was chosen on development data;
on WikiData and DBpedia the development choice, `f_q`, turned out not to be
the best on test — against test-best `f_c` the differences are +0.063 and
+0.012.)

## 3. Test-set results, `S_links` pools (WikiData, YAGO)

| Method | WikiData hit@1 / hit@5 | YAGO hit@1 / hit@5 |
|---|---|---|
| original f_q | 0.610 / 0.780 | 0.513 / 0.543 |
| original f_c | 0.560 / 0.807 | 0.567 / 0.717 |
| original f_c+f_d | 0.350 / 0.623 | 0.200 / 0.533 |
| H1 alone | 0.587 / 0.840 | 0.540 / 0.700 |
| H3 alone | 0.627 / 0.857 | 0.607 / 0.713 |
| **combined** | **0.673 [0.62, 0.72] / 0.873** | **0.627 [0.57, 0.68] / 0.723** |

| Comparison | WikiData | YAGO |
|---|---|---|
| combined − f_c+f_d | +0.323 [+0.257, +0.383], p < 0.0001 | +0.427 [+0.363, +0.493], p < 0.0001 |
| combined − best of 12 original conditions | **+0.063 [+0.017, +0.113], p = 0.013** (S_links, f_q) | +0.003 [−0.030, +0.043], p = 1.0 (L_links, f_c) |

## 4. Ablation of the combined method (test hit@1, change when one ingredient is removed)

| Removed | WikiData L | DBpedia L | YAGO L | WikiData S | YAGO S |
|---|---|---|---|---|---|
| H1 (back to symmetric similarity) | −0.010 | −0.024 | −0.003 | −0.040 | −0.010 |
| H2 (back to all senses) | **+0.007** | **+0.012** | **+0.007** | **+0.020** | **+0.003** |
| type evidence | 0.000 | −0.024 | −0.007 | −0.027 | +0.003 |
| gloss evidence (H4) | −0.017 | 0.000 | −0.027 | −0.023 | −0.010 |
| popularity | −0.110 | −0.024 | −0.143 | −0.180 | −0.170 |
| name similarity | −0.027 | 0.000 | −0.023 | −0.023 | −0.010 |

(Individual ablation differences below about ±0.03 are within noise at these
sample sizes.)

## 5. Verdict per hypothesis

| Hypothesis | Verdict | Evidence |
|---|---|---|
| **H1** directed hypernym score | **Supported** as a repair of Algorithm 2 on graphs with the instance trap: +14 points (WikiData), +38 (YAGO), both p < 0.0001; no effect on DBpedia, where the trap does not occur. H1 alone does **not** beat the original `f_c` (0.563 vs 0.583; 0.587 vs 0.623): it removes the damage done by `f_d` rather than adding accuracy. Inside the combined method its contribution is 0–4 points. | §2, §4 |
| **H2** restricted senses | **Rejected.** No measurable effect alone (within ±1 point everywhere), and the combined method is 0.3–2.0 points *better* on the test set without it in all five settings. It was selected on development data by a margin that did not transfer. | §2, §4 |
| **H3** evidence fusion | **Supported.** Fusing the original evidence alone already gives the largest part of the gain: 0.630 / 0.687 / 0.650 against the best single original process on the pool 0.583 / 0.651 / 0.623. Popularity is by far the most important evidence (−11 to −18 points when removed on WikiData and YAGO). | §2, §4 |
| **H4** gloss–description overlap | **Weakly supported.** As a replacement for the type score it is much better than Algorithm 2 on WikiData and YAGO (+9, +35) but no better than doing nothing (`f_c`). Inside the fusion it adds 1.7–2.7 points on WikiData and YAGO, 0 on DBpedia; not individually significant. | §2, §4 |
| **H5** class-label back-off | **Not supported.** Small gain over `f_c+f_d` on WikiData (+4.3, p = 0.024 uncorrected), none elsewhere; not selected for the combined method; toggling it changes the combined method by ≤ 1.4 points. | §2 |

## 6. What can and cannot be concluded

**Supported**

1. The original disambiguation step (Algorithm 2) is harmful on WikiData and
   YAGO 4.5, and the cause is the symmetric similarity (O1). Replacing it by
   the directed score removes the harm.
2. Combining popularity, name similarity, type and description evidence in
   one score is at least as good as every original process on every
   graph–pool combination tested, and clearly better than `f_c+f_d` on
   WikiData and YAGO.

**Not supported / modest**

3. Against the *best* of the twelve original conditions the combined method
   is ahead by 0.3–6.3 points in all five settings, but only one difference
   (WikiData, S_links: +6.3, p = 0.013) is individually significant, and
   after Holm correction over the five primary comparisons (smallest p × 5 =
   0.065) **none is significant at 0.05**. The honest summary is "consistently
   not worse, possibly a few points better".
4. On DBpedia nothing is demonstrated beyond "no harm": n = 83, intervals of
   ±0.10.

**Not established**

* Transfer to other detector vocabularies, to full Visual Genome, to other
  snapshots of the graphs.
* Any gain in end-to-end robot perception: no images or detections are
  involved, exactly as in the paper.
* The fusion weights are tuned on 380 development entries and are not
  claimed to be optimal; the sensitivity plot
  (`figures/improvements/weight_sensitivity_development.png`) shows a broad
  plateau as long as popularity carries substantial weight.

## 7. Where the original method is better

On the WikiData test half (`L_links` pool), 8 labels are linked correctly by
the original `f_c+f_d` and wrongly by the combined method (75 the other way
round); against the best of the twelve original conditions (`S_links`, `f_q`)
it is 33 lost and 44 gained. Typical losses are **named individuals**, where the fused score
prefers a popular homonym or a generic sense:

| Label (target) | Original f_c (correct) | Combined (wrong) |
|---|---|---|
| Chandler (Raymond Chandler) | American novelist and screenwriter | city in Maricopa County, Arizona |
| Emerson (R. W. Emerson) | American philosopher | family name |
| Jesus | central figure of Christianity | male given name |
| comb | toothed device used for styling hair | crest on the head of gallinaceous birds |
| meter (YAGO) | SI unit of length | a 2005 studio album |

The physical-sense restriction and the description evidence both work
against persons (WordNet files them under *person*, but their descriptions
share few words with a WordNet gloss). The benchmark mixes object classes
with named individuals; a deployed system would treat them separately.

## 8. Typical repairs

| Label | Original f_c+f_d (wrong) | Combined (correct) |
|---|---|---|
| apple (YAGO) | `Apple_generic_instance` | fruit of the apple tree |
| abbey (YAGO) | `Abbey_generic_instance` | monastery or convent under an abbot or abbess |
| barn (WikiData) | a rural building in Boswil, canton of Aargau | agricultural building used for storage |
| bongo (WikiData) | drum from Suriname | type of drum from Cuba |
| air force | the air service branch of the US Armed Forces | military branch concerned with aerial warfare |

## 9. Figures (`experiments/figures/improvements/`)

| File | Shows | Legitimate conclusion |
|---|---|---|
| `methods_hit1_test.png` (`…_S_links`) | hit@1 of every method with Wilson intervals | Algorithm 2 is the outlier; fused methods lead; intervals of the leaders overlap with the best original |
| `paired_differences_test.png` | paired Δ hit@1 with bootstrap CIs | large, certain gain over `f_c+f_d` (WikiData, YAGO); small, uncertain gain over the best original condition |
| `ablation_test.png` | effect of removing each ingredient | popularity dominates; H2 contributes nothing |
| `rank_of_correct_entity_test.png` | rank 1 / 2–5 / >5 / not in pool, before and after | re-ranking moves entities from "2–5" to "1"; the "not in pool" block is untouched |
| `weight_sensitivity_development.png` | best development hit@1 with one weight fixed | flat optimum; popularity weight 0 or 1 is clearly worse |
| `methods_hit1_development.png` | same as the first, development set | development and test agree in ordering |

## 10. Cost

The improved ranking needs one additional description query per ≤100
candidates (H4) and no other new request; scoring is WordNet look-ups,
milliseconds per label. Retrieval cost is unchanged — the improvement is not
bought with a larger computational budget.

## 11. Disclosure and threats to validity

* YAGO test results were printed during development before H5 was added (see
  `hypotheses.md`); WikiData and DBpedia test results were not.
* p-values are uncorrected unless stated; about 50 tests were run.
* The benchmark's ground truth is noisy (critique O5); absolute accuracies
  under-state usefulness, paired differences remain valid.
* All measurements are against November 2026 snapshots of live graphs.
