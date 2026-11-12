# Hypotheses for improving the PEL baseline

Written before the test split was evaluated for WikiData and DBpedia (see
"Disclosure" at the end). Every hypothesis changes one mechanism of the
original mapping process and leaves the rest untouched, so it can be tested
alone. Code: `research/pel/improved_scoring.py`, `improved_mapping.py`.

Notation: label `l`, its WordNet senses `Syn(l)`, candidate `c`, its class
labels `Cls(c)`, outgoing links `links(c)`.

Common experimental frame (details in `results.md`):

* candidate pools: the original `L_links` retrieval, recorded once per label
  (`experiments/extract_candidate_features.py`); every method ranks the same
  pool with the same evidence;
* development set: MS COCO (80) + half of the Visual Genome sample; test set:
  the other half (seeded split); all choices are made on development data;
* primary metric hit@1; paired exact McNemar test and paired bootstrap 95 %
  interval of the hit@1 difference against (a) the original `f_c+f_d` and
  (b) the best original process on the development set.

---

## H1 — Directed hypernym score

| | |
|---|---|
| **Original mechanism** | Algorithm 2 scores a candidate by the symmetric WordNet path similarity `1/(1+d)` between a class label of the candidate and the perceived label. |
| **Weakness** | The score is maximal (1.0) when the class *is* the label. Entities whose class is "dog" are individual dogs or kinds of dog; the entity for the concept *dog* is typed by generalisations (domestic animal, canine) and scores at most 0.5. The algorithm therefore prefers instances and subclasses of the target over the target. |
| **Suspected cause** | A symmetric relatedness measure is used where a directed relation is meant: "the candidate's class should be a generalisation of what was perceived". |
| **Modification** | `sim_hyp(k, t) = 1/d↑(t, k)` if class sense `k` is a proper hypernym-ancestor of label sense `t` at upward distance `d↑ ≥ 1`, else 0. Self-matches, hyponyms and siblings score 0. |
| **Justification** | In a taxonomy, the node for concept `t` is linked to its parents; a graph entity that correctly denotes `t` should be typed by ancestors of `t`. The maximum (1.0) is now reached exactly when the class is the direct hypernym. |
| **Expected effect** | hit@1 of `f_c+f_d` rises substantially wherever the graph contains instances of the labelled class (WikiData individuals, YAGO 4.5 `…_generic_instance`). |
| **Experiment** | Replace the similarity inside Algorithm 2, nothing else. Compare with `f_c+f_d` on identical pools. |
| **Possible harm** | Candidates whose only WordNet-known class is a sibling or the concept itself lose all type evidence; fewer candidates receive a non-zero score, so more ties fall back to pool order. |

## H2 — Restrict the sense inventory

| | |
|---|---|
| **Original mechanism** | Algorithms 1 and 2 use every WordNet sense of the label, of any part of speech. |
| **Weakness** | Verb and adjective senses add spurious names ("couch" → put, redact, cast, frame; "bear" → birth, deliver) and spurious type matches. |
| **Suspected cause** | The label is treated as an arbitrary word, although a detector label always names a kind of thing. |
| **Modification** | Two nested variants: **noun** — noun senses only; **physical** — noun senses below `physical_entity.n.01` (the paper's own "physical prior", Sec. 4.2). Empty inventories fall back to the next wider one. Applied to the names of Algorithm 1 and the label/class senses of Algorithm 2. |
| **Justification** | Removes candidates that cannot be right without touching those that can. |
| **Expected effect** | Small gain in hit@1 for `f_c` and `f_c+f_d`; larger for labels that are also common verbs. |
| **Experiment** | Recompute pools and scores under each inventory; compare with `f_c+f_d`. |
| **Possible harm** | "physical" may remove the correct sense for labels WordNet files as abstractions or locations (e.g. a "park", a "sign"). |

## H3 — Evidence fusion

| | |
|---|---|
| **Original mechanism** | A ranking is produced by exactly one signal: outgoing links (`_links` queries, `f_q`/`f_c`) *or* class similarity (`f_d`). Name similarity only orders the queries. |
| **Weakness** | Each signal is unreliable alone: popularity favours famous entities reached through a synonym; type similarity is zero for most candidates and ties are broken arbitrarily. |
| **Suspected cause** | The pipeline is sequential — each stage discards the evidence of the previous one. |
| **Modification** | `score(c) = w_n·sim_lev(name, l) + w_t·type(c) + w_g·gloss(c) + w_p·pop(c)`, with `pop(c) = log(1+links(c)) / log(1+max links in pool)`, weights ≥ 0 summing to 1, chosen by grid search (step 0.1) on the development set, one weight vector for all graphs. |
| **Justification** | Standard practice in entity linking (commonness prior + context features; Shen et al. 2015). A linear combination is the smallest model that can express "prefer the popular entity found under the label's own name unless type evidence says otherwise". |
| **Expected effect** | hit@1 at least that of the best single original process on each graph. |
| **Experiment** | (a) fusion of the *original* evidence only (`w_g = 0`, symmetric type score) — isolates the effect of combining; (b) the combined method; (c) leave-one-evidence-out ablations. |
| **Possible harm** | Weights tuned on one mixture of graphs may not transfer; popularity can dominate and re-introduce the "famous entity" error. |

## H4 — Gloss–description overlap

| | |
|---|---|
| **Original mechanism** | The only semantic evidence about a candidate is the label of its classes. |
| **Weakness** | Many class labels are not WordNet entries, so most candidates get score 0; meanwhile graphs carry a short textual description of nearly every entity, which is ignored. |
| **Suspected cause** | The method was designed around taxonomy only. |
| **Modification** | Extended-Lesk evidence: `gloss(c) = max_t |sig(t) ∩ W(desc(c))| / sqrt(|sig(t)|·|W(desc(c))|)`, where `sig(t)` is the set of content words in the gloss and lemmas of sense `t` and of its direct hypernyms, and `W(·)` the content words of the entity description. |
| **Justification** | Lesk (1986) / Banerjee & Pedersen (2002): the right sense of a word shares vocabulary with the definition of the thing it denotes. |
| **Expected effect** | Gain where descriptions are informative (WikiData); little where they are sparse. |
| **Experiment** | (a) gloss overlap alone replacing the type score; (b) inside the fusion; (c) ablation. |
| **Possible harm** | Descriptions of non-physical senses overlap too ("orange: colour between red and yellow" matches the colour sense); needs one extra query per ~100 candidates. |

## H5 — Class-label word back-off

| | |
|---|---|
| **Original mechanism** | A class label contributes evidence only if the whole label is a WordNet entry. |
| **Weakness** | Phrase-like class labels ("fruit of Maloideae", "type of vehicle", "individual means of transport") contribute nothing. |
| **Modification** | If the class label has no WordNet sense, use the senses of its content words. |
| **Expected effect** | More candidates receive type evidence; modest gain for H1 in particular. |
| **Possible harm** | Words in a phrase need not be its head ("city *in the United States*" → "states"); spurious matches. |

---

## Not pursued, and why

* **Retrieval recall** (redirects, disambiguation pages, parenthesised titles
  such as "Orange (fruit)", both casings): the reproduction shows this is the
  largest remaining loss (see `critique.md`), but each fix needs new queries
  against endpoints that throttled this project; left as the first item of
  future work rather than half-tested.
* **NIL detection** (`l_∅`): needs ground truth with labels that have no
  counterpart; the benchmark as defined in the paper has almost none.
* **Context/coherence between co-perceived entities** and **attributes**
  (colour, size): the paper's evaluation has no images, hence no context.

## Disclosure

The scoring code was developed on MS COCO and on the YAGO recordings. During
that development the YAGO *test* half was printed twice by the analysis
script (before H5 was added). No design decision was taken from those
numbers — H5 was motivated by a development-set observation (the correct YAGO
entity for "apple" had type score 0) — but YAGO test results are, strictly
speaking, not blind. WikiData and DBpedia test results were first computed
after this file was written.
