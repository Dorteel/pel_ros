# Reproduction methodology

Companion to `specification.md` (what is implemented and why) — this file says
how the experiments are run, what is recorded, and how each component is
tested.

## 1. Theory → code map

| Concept (paper) | Component | Input → output | Relies on | Tested by (`research/tests/test_core.py`) |
|---|---|---|---|---|
| Observation graph `G_o`, `f_r` (3.2) | `pel/observation_graph.py::from_detections` | detections `(label, box, confidence)`, time → triples | detector supplies an English label | `test_observation_graph_grounding` |
| Levenshtein similarity (4.1) | `pel/text_similarity.py` | two strings → `[0,1]` | normalisation I2 | known distances, symmetry, triangle inequality, bounds |
| WordNet helpers | `pel/wordnet_access.py` | label → synsets; synset → names; two synsets → `1/(1+d)` | WordNet 3.0 | `test_path_similarity_definition`, `test_multiword_and_unknown_labels` |
| Algorithm 1 (4.1) | `pel/candidate_selection.py::select_candidate_names` | label → `[(similarity, name)]` sorted | I9 | `test_candidate_names_*` |
| Pooling of per-name results | `…::collect_candidates` | names + retrieval function → ordered candidates | I1 | `test_pooling_*` |
| Queries S, S_links, L, L_links (5.1) | `pel/knowledge_graphs.py::build_retrieval_query` | surface form, type, limit → SPARQL | I3, I4, I5 | `test_query_shapes`, `test_literal_escaping_and_surface_forms` |
| `GET_CLASS` | `pel/mapping.py::KnowledgeGraphAccess.class_labels` | IRIs → class labels | I6 | `test_class_label_from_iri`; live behaviour via the experiment |
| Algorithm 2 (4.2) | `pel/disambiguation.py::disambiguate` | label, candidates, class labels → `[(score, candidate)]` | I7 | `test_disambiguation_*`, `test_class_similarity_*` |
| `f_q`, `f_c`, `f_c+f_d` (Table 1) | `pel/mapping.py::map_label_all_conditions` | label → ranking per condition | — | `test_three_processes_on_fake_graph` |
| hit@k, intervals (5.1–5.2) | `pel/evaluation.py` | rankings, ground truth → hit@k, Wilson CI | I8 | `test_hit_at_k`, `test_wilson_interval_and_sample_size` |

All unit tests are offline (WordNet only). Behaviour against the live graphs is
exercised by the experiment itself; every response is recorded.

## 2. Procedure

```bash
python3 -m venv --system-site-packages .venv && .venv/bin/pip install -r research/requirements.txt
.venv/bin/python -c "import nltk; [nltk.download(c) for c in ('wordnet','wordnet31','omw-1.4')]"

./test.sh                                                       # unit tests
.venv/bin/python research/experiments/build_ground_truth.py     # ground truth (network)
for kg in wikidata dbpedia yago; do                             # the experiment (network, hours)
  .venv/bin/python research/experiments/run_mapping_experiment.py --dataset coco --kg $kg
  .venv/bin/python research/experiments/run_mapping_experiment.py --dataset visual_genome --kg $kg --max-entries 600 --seed 0
done   # DBpedia (throttled endpoint): add --subset-size 150 to the Visual Genome line
.venv/bin/python research/experiments/summarise_results.py      # Table 1 layout + Wilson CIs
.venv/bin/python research/experiments/make_reproduction_figures.py
```

* **Caching.** `pel/sparql_client.py` stores every response in
  `research/data/cache/<kg>.sqlite` (git-ignored, regenerable). Runs are
  restartable; re-running a finished experiment issues no request.
* **Failures.** A query that times out twice is recorded as such and treated
  as an empty result (that *is* what a robot would get). Network errors and
  throttling are never recorded; the affected run reports them and must be
  repeated until it reports none.
* **Politeness.** Requests are throttled per endpoint (1–4 parallel, 0.1–0.5 s
  spacing) and back off for minutes when a connection is refused. DBpedia's
  public endpoint refused connections twice during this work after bursts of
  ≈1,000 requests; it is therefore queried single-threaded.
* **Determinism.** The method has no random component. The Visual Genome
  sample is drawn with `random.Random(0).sample`. The order of `S`/`L` results
  is decided by the endpoint and may change between graph updates; the cache
  freezes the order observed in this run.

## 3. What is stored

`research/experiments/results/reproduction/<dataset>_<kg>.json` holds, per
ground-truth entry and per condition: rank of the first correct entity over
the *complete* ranking (or null), number of entities returned, and the top-10
IRIs. `summary.json` / `table1.md` are derived from these files only;
`experiments/paper_table1.json` holds the published values for comparison.

## 4. Statistics

* hit@k with Wilson 95 % score intervals (binomial, per cell).
* Agreement with the publication: mean absolute difference and Spearman rank
  correlation over the 36 cells of each (dataset, graph) pair
  (`agreement_with_paper.json`).
* No significance tests are run in the reproduction: nothing is being
  compared with anything of ours yet. Paired tests appear with the
  improvements.

## 5. External baselines of the paper (Sec. 5.2)

* **DBpedia Spotlight** (`experiments/run_spotlight_baseline.py`): each label
  would be submitted alone to the public Spotlight web service; hit = first
  annotation is a ground-truth entity. The paper reports 0.525 on COCO but not
  the confidence threshold; the script reports the default 0.5 and 0.0. The
  service refused connections on every attempt (10–12 November 2026), so no
  result exists.
* **AIDA**: the paper reports no hits. The service URL given in the paper
  still serves a project page, but the disambiguation service it points to
  answered HTTP 404 on 2026-11-11, so this baseline could not be re-run.

## 6. Additional (non-paper) analyses run on the reproduction

* `results/additional/label_case/` — effect of the label-casing convention
  (I3): YAGO queried with lower-case labels.
* `failure_decomposition.json` — for each process, whether the correct entity
  was ranked first, within five, lower, not retrieved at all, or nothing was
  returned.
