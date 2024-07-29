import pickle
import numpy as np
from sklearn.metrics.pairwise import cosine_similarity

# Load the model and extract necessary components
with open("/home/mark/kai_ws/src/locobot_development/wikidata5m/pre-trained-models/transe_wikidata5m.pkl", "rb") as fin:
    model = pickle.load(fin)

entity2id = model.graph.entity2id
relation2id = model.graph.relation2id
entity_embeddings = model.solver.entity_embeddings
relation_embeddings = model.solver.relation_embeddings

# Function to get the embedding of an entity by its name
def get_entity_embedding(entity_name):
    entity_id = entity2id.get(entity_name)
    if entity_id is not None:
        return entity_embeddings[entity_id]
    else:
        raise ValueError(f"Entity '{entity_name}' not found in the graph.")

# Function to calculate cosine similarity between two embeddings
def calculate_cosine_similarity(embedding1, embedding2):
    embedding1 = embedding1.reshape(1, -1)  # Reshape to 2D array
    embedding2 = embedding2.reshape(1, -1)  # Reshape to 2D array
    return cosine_similarity(embedding1, embedding2)[0][0]


def create_embedding(entities):
    """
    Create an embedding based on the entities contained in the observation graph
    """
    # Fruit examples
    entity_embeddings = []
    for entity in entities:
        entity_embeddings.append(get_entity_embedding(entity))
    return sum(entity_embeddings)

def compare_embeddings(obs_graph, candidates):
    # Create the embedding from the obs_graph
    colors = {"orange" : "Q39338",
                "brown" : "Q47071",
                "yellow" : "Q2720565",
                "red" : "Q3142",
                "green" : "Q3133",
                "purple" : "Q3257809"}

    fruits = {"orange" : "Q13191",
                "lemon" : "Q500",
                "grapefruit" : "Q41350",
                "apple" : "Q89",
                "grape" : "Q10978"}

    obs_graph_embedding = create_embedding(obs_graph)
    
    for candidate in candidates:
        # Get the embedding of the candidate
        embedding = get_entity_embedding(candidate)
        # Compare the similarity
        similarity_1 = calculate_cosine_similarity(obs_graph_embedding, embedding)
        name_to_compare = list(colors.keys())[list(colors.values()).index(obs_graph[0])]
        name_compared = list(fruits.keys())[list(fruits.values()).index(candidate)]
        # Print the results
        print(f"The similarity between {name_to_compare} and {name_compared} is : {similarity_1}")


def compare_embeddings_example(obs_graph, candidates):

    colors = {"orange" : "Q39338",
            "brown" : "Q47071",
            "yellow" : "Q2720565"}

    fruits = {"orange" : "Q13191",
            "lemon" : "Q500",
            "grapefruit" : "Q41350"}
    
    obs_graph_ids = [fruits[obs_graph[0]], colors[obs_graph[1]]]
    # Create the embedding from the obs_graph
    obs_graph_embedding = create_embedding(obs_graph_ids)
    
    for candidate in candidates:
        # Get the embedding of the candidate
        embedding = get_entity_embedding(fruits[candidate])
        # Compare the similarity
        similarity_1 = calculate_cosine_similarity(obs_graph_embedding, embedding)
        # Print the results
        print(f"The similarity between {obs_graph} and {candidate} is : {similarity_1}")
    print("----")

or_color_wd_id = "Q39338"
brown_color_wd_id = "Q47071"
yellow_color_wd_id = "Q2720565"



lemon_fruit_wd_id = "Q500"
orange_fruit_wd_id = "Q13191"
grapefruit_fruit_wd_id = "Q41350"

# dog examples
dog_wd_id = "Q144"
Chihuahua_wd_id = "Q653"
great_dane_wd_id = "Q5414"

size = "Q322481"

colors = {"orange" : "Q39338",
            "brown" : "Q47071",
            "yellow" : "Q2720565",
            "red" : "Q3142",
            "green" : "Q3133",
            "purple" : "Q3257809"}

fruits = {"orange" : "Q13191",
            "lemon" : "Q500",
            "grapefruit" : "Q41350",
            "apple" : "Q89",
            "grape" : "Q10978"}


try:
    # compare_embeddings_example(["orange", "orange"], ["orange", "lemon", "grapefruit"])
    # compare_embeddings_example(["orange", "yellow"], ["orange", "lemon", "grapefruit"])
    # compare_embeddings_example(["orange", "brown"], ["orange", "lemon", "grapefruit"])
    compare_embeddings([colors["yellow"]], [fruits["lemon"], fruits["orange"], fruits["grapefruit"], fruits["apple"], fruits["grape"]])
    compare_embeddings([colors["brown"]], [fruits["lemon"], fruits["orange"], fruits["grapefruit"], fruits["apple"], fruits["grape"]])
    compare_embeddings([colors["orange"]], [fruits["lemon"], fruits["orange"], fruits["grapefruit"], fruits["apple"], fruits["grape"]])
    compare_embeddings([colors["red"]], [fruits["lemon"], fruits["orange"], fruits["grapefruit"], fruits["apple"], fruits["grape"]]) 
    compare_embeddings([colors["green"]], [fruits["lemon"], fruits["orange"], fruits["grapefruit"], fruits["apple"], fruits["grape"]])
    compare_embeddings([colors["purple"]], [fruits["lemon"], fruits["orange"], fruits["grapefruit"], fruits["apple"], fruits["grape"]])      
except ValueError as e:
    print(e)