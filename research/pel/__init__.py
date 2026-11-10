"""Perceived-Entity Linking (PEL) research package.

Reproduction of: Adamik, Pernisch, Tiddi, Schlobach, "Advancing Robotic
Perception with Perceived-Entity Linking", ISWC 2024 (LNCS 15232, pp. 192-209).

Module map (paper concept -> module):

    observation graph G_o, recognition f_r (Sec. 3.2) -> observation_graph.py
    target knowledge graph G_t, queries S/L/S_links/L_links (Sec. 5.1)
                                                      -> knowledge_graphs.py
    candidate selection f_c (Sec. 4.1, Algorithm 1)   -> candidate_selection.py
    disambiguation f_d (Sec. 4.2, Algorithm 2)        -> disambiguation.py
    mapping process f_m = f_d o f_c (Sec. 4, Fig. 6)  -> mapping.py
    accuracy / hit@k (Sec. 5.1-5.2)                   -> evaluation.py
"""
