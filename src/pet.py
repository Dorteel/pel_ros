# Perceived-Entity Typing

import pickle
import numpy as np
from sklearn.metrics.pairwise import cosine_similarity
import pandas as pd
import os
from SPARQLWrapper import SPARQLWrapper, JSON


# Load the model and extract necessary components

# model_path = f"/home/mark/kai_ws/src/locobot_development/wikidata5m/pre-trained-models/{modelName.lower()}_wikidata5m.pkl"



def generate_embedding(obs_graph):
    """
    Create an embedding based on the entities contained in the observation graph
    """
    # Fruit examples
    entity_embeddings = []
    for entity in obs_graph:
        emb = get_embedding(entity)
        if emb is not None:
            entity_embeddings.append(emb)

    return sum(entity_embeddings)


def generate_refinement_candidates(service_point, entity_id, k):
    sparql = SPARQLWrapper(service_point)
    
    query = f"""
    SELECT ?subclass ?subclassLabel (COUNT(?link) AS ?count) WHERE {{
      ?subclass wdt:P279* wd:{entity_id}.
      ?link ?anylink ?subclass.
      SERVICE wikibase:label {{ bd:serviceParam wikibase:language "en". }}
    }} GROUP BY ?subclass ?subclassLabel ORDER BY DESC(?count) LIMIT {k}
    """
    
    sparql.setQuery(query)
    sparql.setReturnFormat(JSON)
    results = sparql.query().convert()
    
    subclasses = []
    for result in results["results"]["bindings"]:
        subclasses.append({
            "subclass": result["subclass"]["value"],
            "subclassLabel": result["subclassLabel"]["value"],
            "count": int(result["count"]["value"])
        })
    
    return subclasses

def generate_correction_candidates(target_kb, entity):
    pass

# Function to get the embedding of an entity by its id
def get_embedding(id):
    if id[0] == 'Q':
        entity_id = entity2id.get(id)
    else:
        entity_id = relation2id.get(id)
    if entity_id is not None:
        return entity_embeddings[entity_id]
    else:
        return None
        # raise ValueError(f"Entity '{id}' not found in the graph.")
    

# Function to calculate cosine similarity between two embeddings
def calculate_cosine_similarity(embedding1, embedding2):
    embedding1 = embedding1.reshape(1, -1)  # Reshape to 2D array
    embedding2 = embedding2.reshape(1, -1)  # Reshape to 2D array
    return cosine_similarity(embedding1, embedding2)[0][0]


# Run the experiment
if __name__ == "__main__":
    models = ['transe', 'simple', 'rotate', 'quate', 'distmult', 'complex']
    query_service = "https://query.wikidata.org/sparql"
    data_path =  "experiment_2.csv"
    candidate_number = 15
    file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), data_path)
    first_run = True
    # Read the CSV file into a DataFrame
    df = pd.read_csv(file_path)
    properties = ['P279', 'P462', 'P1419', 'P186', 'P527']
    cols = ['Concept_name', 'WikiDataID'] + models
    results = pd.DataFrame(columns=cols)
    for modelname in models:
        print(f"Running model {modelname}...")
        model_path = f"/home/user/pel_ws/kge_models/{modelname.lower()}_wikidata5m.pkl"
        with open(model_path, "rb") as fin:
            model = pickle.load(fin)
        entity2id = model.graph.entity2id
        relation2id = model.graph.relation2id
        entity_embeddings = model.solver.entity_embeddings
        relation_embeddings = model.solver.relation_embeddings
        for index, row in df.iterrows():
            # Load general data
            target_label = row['targetLabel']
            target_id = row['targetID']
            task_type = row['taskType']
            label_id = row['P279']
            image_id = row['id'][:-4]
            # open results:
            if os.path.exists(f'results/{image_id}.csv'):
                results = pd.read_csv(f'results/{image_id}.csv')
                first_run = False
            else: results = pd.DataFrame(columns=cols)
            # Create synthetic observation graph
            obs_graph = []
            for p in properties:
                if not pd.isna(row[p]):
                    obs_graph.append(p)
                    obs_graph.append(row[p])

            # Generate embedding from the observation_graph
            entity_embedding = generate_embedding(obs_graph)

            # Create candidates
            if task_type == 'refinement':
                candidates = generate_refinement_candidates(query_service, label_id, candidate_number)
                
                if target_id not in [cand['subclass'] for cand in candidates]:
                    print(f"WARNING: {target_label} not among candidates")

                # Go through each candidate
                for candidate in candidates:
                    candidate_id = candidate['subclass'].split('/')[-1]
                    candidate_label = candidate['subclassLabel']
                    
                    
                    # Get embedding of the candidate
                    candidate_embedding = get_embedding(candidate_id)
                    
                    # If the candidate is not found skip it
                    if candidate_embedding is None:
                        print(f"Entity '{candidate_id}' not found in the graph, skipping.")
                        continue
                    else:
                        print(f"Subclass: {candidate_label} (ID: {candidate_id})")
                    # Calculate similarity scores
                    similarity = calculate_cosine_similarity(entity_embedding, candidate_embedding)
                    if first_run:
                        results = pd.concat([results, pd.DataFrame([{'Concept_name' : candidate_label,
                                                'WikiDataID' : candidate_id,
                                                modelname : similarity}])], ignore_index=True)
                    else:
                        results.loc[results['WikiDataID'] == candidate_id, modelname] = similarity
                results.to_csv(f'results/{image_id}.csv', index=False)
            else:
                continue
            # Loop through candidates, generate scores
            pass
        del model