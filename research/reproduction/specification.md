# Reproduction specification — Perceived-Entity Linking

**Paper.** M. Adamik, R. Pernisch, I. Tiddi, S. Schlobach: *Advancing Robotic
Perception with Perceived-Entity Linking.* ISWC 2024, LNCS 15232, pp. 192–209.
doi:10.1007/978-3-031-77850-6_11.

This document is the scientific reference for the code in `research/pel/`.
Three kinds of statement are kept apart throughout:

* **[P]** stated explicitly in the paper (section given);
* **[I]** our interpretation of something the paper leaves open (numbered I1…);
* **[D]** a deviation forced by circumstances (numbered D1…).

The official implementation cited in the paper (`github.com/Dorteel/pel`)
returned HTTP 404 on 2026-11-10, so neither the authors' SPARQL queries nor
their ground-truth files could be consulted. Everything marked [I] is
therefore inferred from the text and from Table 1.

---

## 1. Scientific reasoning, reconstructed

**1. Problem.** A robot's sensors produce numbers; the commonsense knowledge it
could use lives in knowledge graphs (KGs) as *identified* entities
(`wd:Q13191` = orange, the fruit). The robot needs the arrow between the two:
*given sensory data, which KG identifier does this region of the data denote?*
[P, Sec. 1]. The paper names this **Perceived-Entity Linking (PEL)**.

**2. Why it is hard.** (a) Recognisers emit *strings* from a small closed
vocabulary ("orange"), and a string names many KG entities (fruit, colour,
city, telecom company). (b) Unlike text entity linking there is no sentence
around the mention to disambiguate it. (c) Labels are class names, often
coarser than the KG's classes ("chair" vs. armchair/office chair) or absent
from it ("sports ball"). (d) KGs differ in naming conventions, coverage and
structure.

**3. Assumptions introduced.** A recognition process exists that turns
observations into *symbolic descriptions* (labels + grounding) [P, 3.2]; the
world model against which recognition happens can be played by a public KG
[P, 3.2, after Besl & Jain]; an English label string is enough to retrieve the
right entity among a small candidate set [implicit]; perceived things are
physical, so physical senses deserve a higher prior [P, 4.2]; WordNet's
taxonomy is a usable bridge between detector vocabulary and KG classes
[implicit in Alg. 1–2].

**4. Built upon.** Entity linking as *candidate generation → disambiguation*
(Balog 2018; Shen et al. 2015); object recognition as description +
understanding against a world model (Besl & Jain 1985); scene graphs (Zhu et
al. 2022); WordNet (Miller 1995); YOLO as recogniser.

**5. Minimum concept set.** Observation, observation graph `G_o`, target KG
`G_t`, mapping `f_m`, candidate selection `f_c`, disambiguation `f_d`, WordNet
synset/lemma/path, Levenshtein distance, SPARQL label query, hit@k.

**6. How they relate.**

```
world X --f_o--> observation o --f_r--> G_o (labels l_e) --f_c--> candidates ⊂ G_t --f_d--> ranked entities
                                              \__________________ f_m __________________/
```

**7. Mechanism.** *Recall* comes from string expansion: WordNet lemmas of all
senses of the label are each matched exactly against KG labels (Alg. 1).
*Precision* comes from two orderings: a popularity prior (number of outgoing
links, the `_links` queries) and a type prior (WordNet path similarity between
the candidate's class label and the perceived label, Alg. 2).

**8. Why it should work.** KG labels of common objects are usually the common
noun itself, so exact matching finds the right entity among few candidates;
prominent entities have more statements, so sorting by out-degree approximates
"commonness"; the right candidate for "orange" is typed *fruit*, and
`fruit`↔`orange.n.01` are close in WordNet whereas `city`↔`orange` are not.

**9. When it should fail.** Label not a KG label (casing, plural "skis",
compound "sports ball", synonym not in WordNet); class label not a WordNet
entry ("citrus fruit" is, "piece of furniture"… is not always) → score 0;
maximum over *all* senses of the label lets a wrong sense win (colour
"orange" typed *color* scores as high as the fruit typed *fruit*);
popularity favours famous named entities over plain classes; synonym
expansion injects entities of other senses ("Orange River").

**10. Evidence in the paper.** Table 1 shows large variation by KG and query
type, improvements from `f_c`/`f_d` in some cells and regressions in others
(e.g. Visual Genome/WikiData/`L`: 0.45 → 0.29 with `f_d`). The claim actually
supported is modest: *a conventional EL pipeline gives a usable but far from
solved baseline*; no confidence intervals or significance tests are reported.

---

## 2. Mathematical objects

| Symbol | Meaning [P, Sec. 3] | Code |
|---|---|---|
| `r`, `S^r` | robot and its set of sensors | — |
| `X` | set of world states | — |
| `E_X`, `A`, `P` | entities in the world, attribute types, relation types | — |
| `f_o^s : X → O` | observation process of sensor `s` (noisy, partial) | hardware; out of scope [P] |
| `o_i ∈ R^{d_1×…×d_n}` | one observation, a real tensor whose shape depends on the sensor | — |
| `G_o ⊆ E_o × P × (E_o ∪ A)` | observation graph: triples over perceived entities and attributes | `observation_graph.ObservationGraph.triples()` |
| `l_e ∈ L` | label literal of entity `e` | `PerceivedEntity.label` |
| `a_e^loc ∈ R^n`, `a_e^time ∈ R` | grounding of `e` in the observation | `PerceivedEntity.location`, `.time` |
| `f_r : O → G` | recognition process | `observation_graph.from_detections` |
| `G_t = (N_t, L_t)`, `L_t ⊆ N_t × P_t × N_t` | target knowledge graph | `knowledge_graphs.KnowledgeGraph` + endpoint |
| `f_m : G_o × G_t → G_l` | mapping process; unmatched entities map to `l_∅` | `mapping.map_label` (empty list = `l_∅`) |
| `f_c`, `f_d` | candidate selection, disambiguation; `f_m = f_d ∘ f_c` [P, Sec. 4] | `candidate_selection`, `disambiguation` |

Notes on the definitions as printed (not corrected in code, recorded here):

* The observation tensor is printed as `R^{d_1×d_1×…×d_n}`; the repeated `d_1`
  is read as a typo for `d_1×d_2×…×d_n`.
* `G_l = (E_{G_o}, P, E_{G_t})` is printed as a tuple of sets; consistent with
  `G_o` it is read as a set of link triples `G_l ⊆ E_{G_o} × P × (E_{G_t} ∪ {l_∅})`.
* The paper's baseline has no mechanism that *decides* for `l_∅`; NIL arises
  only when retrieval returns nothing.

### 2.1 Levenshtein similarity (Sec. 4.1)

`lev(a,b)` = minimum number of single-character insertions, deletions,
substitutions turning `a` into `b`, computed by the recurrence

```
D[i][0] = i,  D[0][j] = j
D[i][j] = min( D[i-1][j] + 1,  D[i][j-1] + 1,  D[i-1][j-1] + [a_i ≠ b_j] )
```

**[I2]** The paper says the edit distance is used "to calculate the
similarity" without giving the conversion. We use
`sim(a,b) = 1 − lev(a,b)/max(|a|,|b|) ∈ [0,1]`.
Example: `lev("orange","orangeness") = 4`, `sim = 1 − 4/10 = 0.6`.
Code: `text_similarity.levenshtein_similarity`.

### 2.2 WordNet path similarity (Sec. 4.2)

For synsets `s,t`, `d(s,t)` = number of edges of the shortest path in the
hypernym/hyponym graph; `sim_path(s,t) = 1/(1+d(s,t))`. **[I7]** The paper
says "the class synset with the shortest path to `e_{G_o}` in the WordNet
taxonomy is considered as a similarity measure"; we take this to be NLTK's
`path_similarity` (the standard function with exactly this definition), with
"no path" (different parts of speech) mapped to 0.
Example: `orange.n.01 → citrus.n.01` is one edge, so `sim = 1/2`.

### 2.3 Disambiguation score (Algorithm 2)

```
score(c) = max_{k ∈ Cls(c)} max_{s ∈ Syn(k)} max_{t ∈ Syn(l)} sim_path(s, t)
```

`Cls(c)` = labels of the classes of candidate `c` in `G_t`; `Syn(·)` = WordNet
synsets of a string (all parts of speech). Candidates are sorted by
decreasing score. Printed line 16 sorts `candidates`; we read it as sorting
by the `scores` built in line 14. Ties: stable sort, so candidate-selection
order is kept (**[I7]**).

### 2.4 Evaluation metric (Sec. 5.1–5.2)

`hit@k = |{ l ∈ D : top_k(l) ∩ GT(l) ≠ ∅ }| / |D|`, `k ∈ {1,3,5}`; hit@1 is the
reported "accuracy". **[I8]** The text defines accuracy as correct links over
"all the links generated by the method", but reports 0.69 as "55 out of the 80
labels"; we use the latter (denominator = all labels, also those with no
result or no ground truth). `GT(l)` is a *set*: several WikiData items can
carry the same WordNet synset ID.

---

## 3. Algorithms

### 3.1 Queries (Sec. 5.1) — **[I5]** (the paper gives prose only)

| Type | Paper wording | SPARQL used (`knowledge_graphs.build_retrieval_query`) |
|---|---|---|
| `S` | any entity with any property whose object is the label `@en` | `SELECT DISTINCT ?entity WHERE { ?entity ?anyProperty "l"@en } LIMIT n` |
| `S_links` | S, count outgoing links, order descending | `SELECT ?entity (COUNT(?target)/COUNT(DISTINCT ?anyProperty) AS ?links) WHERE { ?entity ?anyProperty "l"@en . ?entity ?outgoing ?target } GROUP BY ?entity ORDER BY DESC(?links) LIMIT n` |
| `L` | only `rdfs:label` | `SELECT DISTINCT ?entity WHERE { ?entity rdfs:label "l"@en } LIMIT n` |
| `L_links` | L, ordered by outgoing links | `SELECT ?entity (COUNT(?target) AS ?links) WHERE { ?entity rdfs:label "l"@en . ?entity ?outgoing ?target } GROUP BY ?entity ORDER BY DESC(?links) LIMIT n` |

* **[I4]** `n = 50` (`mapping.DEFAULT_RESULT_LIMIT`). The paper states no limit.
* For `S` and `L` the order is whatever the endpoint returns: hit@k of these
  two query types is therefore partly an artefact of the triple store.
* **[I3]** *Label casing.* Dataset labels are lower case. Exact matching of
  `"chair"@en` finds nothing useful in DBpedia or YAGO 4.5, whose labels are
  Wikipedia titles (`"Chair"@en`), yet the paper reports 0.68–0.84 hit@1 for
  DBpedia `L` queries. Some casing step must have existed. We capitalise the
  first character for DBpedia and YAGO and leave WikiData labels as they are
  (`KnowledgeGraph.label_case`). The alternative casing is run as a
  sensitivity experiment.

### 3.2 Algorithm 1 — candidate selection (`candidate_selection.py`)

```
SELECT_CANDIDATES(label):
    synsets ← WordNet synsets of label           # any part of speech
    if none: return [label]
    for each synset, each lemma name:  score ← sim_lev(label, name)
    return names sorted by score, descending
```

* **[I9]** "the most similar entities to the label are selected" could mean a
  cut-off; the pseudocode returns the whole sorted list, so no cut-off.
* **[I1]** *Pooling.* "all entities returned from `G_t` are added as potential
  candidates" does not say in which order. We use: for `S`/`L`, concatenate
  per-name results in name-similarity order; for `S_links`/`L_links`, sort
  the pooled set by outgoing links. This is the only reading under which the
  `_links` criterion still applies to the pooled set, and it matches the
  paper's remark that with `f_c` alone "entities … with many outgoing links …
  push the correct entity further down the list" (Sec. 5.2).

### 3.3 Algorithm 2 — disambiguation (`disambiguation.py`)

As in §2.3. **[I6]** `GET_CLASS(candidate)`: WikiData `wdt:P31` and
`wdt:P279`; DBpedia and YAGO `rdf:type` and `rdfs:subClassOf`. The class label
is the class's English `rdfs:label`, else derived from the IRI local name
(`…/PopulatedPlace` → "populated place", `…/Seat104161981` → "seat").

### 3.4 Processes compared (Table 1)

`f_q` = label query only; `f_c` = Alg. 1 + pooling; `f_c + f_d` = then Alg. 2.

---

## 4. Data

| Item | Paper | Here |
|---|---|---|
| MS COCO labels | 80 category names | same 80 (`data/coco_labels.json`) |
| COCO ground truth | manual search, "viable alternatives" allowed, empty if none | **[D3]** re-curated: WordNet synset → WikiData P8814 → `owl:sameAs`, plus 23 manual corrections checked against live labels/descriptions (`data/coco_manual_ground_truth.json`) |
| Visual Genome ground truth | VG object synsets → WikiData via P8814 → DBpedia/YAGO via `owl:sameAs`; synsets present in all three: 2,399 | same procedure (`experiments/build_ground_truth.py`): **1,965** entries today **[D2]** |
| VG labels | "the synset's lemma" | first lemma of the synset, `_`→space |
| WordNet | "WordNet 3.1" IDs for P8814 | **[I10]** VG synset names resolve in WordNet 3.0 (NLTK default); converted to 3.1 IDs through sense keys; Algorithms 1–2 use WordNet 3.0 |

## 5. Experimental configuration

* Target KGs: WikiData (`query.wikidata.org/sparql`), DBpedia
  (`dbpedia.org/sparql`), YAGO (`yago-knowledge.org/sparql/qlever`).
* Conditions: 4 query types × 3 processes × 3 KGs × 2 datasets, hit@1/3/5.
* No stochastic component in the method. The only seed is that of the Visual
  Genome entry sample (seed 0).
* Expected output: the 72-row × 3-metric layout of Table 1
  (`experiments/paper_table1.json` holds the published values).

## 6. Deviations

* **[D1] Graph versions.** The paper queried the live graphs in 2024 (YAGO 4
  is cited). We query the live graphs of November 2026; the YAGO endpoint now
  serves YAGO 4.5, which drops most classes without instances. Numbers are
  therefore not expected to match digit for digit.
* **[D2] Visual Genome size.** 1,965 instead of 2,399 entries survive the
  "in all three graphs" filter (YAGO 4.5 is the bottleneck: 2,037). Because
  one full pass needs ≈70k SPARQL requests per graph, the reproduction
  evaluates a seeded simple random sample of 600 entries (seed 0), identical
  across methods; WikiData and YAGO use all 600, DBpedia (whose endpoint
  throttled us to about 30 requests per minute) a seeded subset of 150 of them.
* **[D3] COCO ground truth** is ours, not the authors'.
* **[D4] No reference code**; see [I1]–[I10].
* **[D5] Recognition.** YOLO is not executed: the paper's evaluation feeds
  dataset *labels* to `f_m`, no image is involved. `from_detections` is the
  tested adapter from detector output to `G_o`.
* **[D6] External baselines.** DBpedia Spotlight and AIDA (Sec. 5.2) could not
  be re-run (services unreachable); see `methodology.md` and `results.md`.

## 7. Inconsistencies found in the paper (not "fixed", only recorded)

1. Table 1 contains cells where hit@k decreases with k, which the definition
   forbids: COCO/WikiData/`L`/`f_q` (hit@3 0.70 > hit@5 0.65) and
   COCO/DBpedia/`S_links`/`f_c` (hit@1 0.24 > hit@3 0.19).
2. Sec. 5.3 attributes the drop under `f_c` to "incoming links", while the
   queries count outgoing links.
3. Accuracy is defined over generated links but reported over all labels (I8).
4. "76,340 object classes" in Visual Genome: the public `object_synsets.json`
   has 40,154 object names mapping to 5,996 synsets; the source of 76,340 is
   not stated.
5. The sample-size argument (2,328 needed for ±2 % at 95 %) presumes a random
   sample; the 2,399 entries are selected by *being linkable in all three
   graphs*, which is not random with respect to linking difficulty.
6. Algorithm 2, line 16 sorts `candidates` although the scores are in `scores`.
