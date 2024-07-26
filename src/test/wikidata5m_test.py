import csv
import requests
import time

# Function to read groundtruth.csv and construct yolo_entities dictionary
def read_groundtruth(filepath):
    yolo_entities = {}
    print("Reading groundtruth.csv...")
    with open(filepath, mode='r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            name = row['name']
            wikidata_id = row['wikidata']
            last_part_of_id = wikidata_id.split('/')[-1]
            yolo_entities[name] = last_part_of_id
    print("Finished reading groundtruth.csv.")
    return yolo_entities

# Function to read wikidata5m_entity.txt to get a set of entities
def read_wikidata5m(filepath):
    wikidata5m_entities = set()
    print("Reading wikidata5m_entity.txt...")
    with open(filepath, mode='r') as file:
        for line in file:
            id = str(line.strip()).split('\t')[0]
            wikidata5m_entities.add(id)
    print("Finished reading wikidata5m_entity.txt.")
    return wikidata5m_entities

# Function to query Wikidata for subclass of or instance of relationships
def query_wikidata_subclasses(entity_id):
    print(f"Querying Wikidata for subclasses/instances of entity {entity_id}...")
    query = f"""
    SELECT ?item ?itemLabel WHERE {{
      {{
        ?item wdt:P279 wd:{entity_id} . # subclass of
      }} UNION {{
        ?item wdt:P31 wd:{entity_id} . # instance of
      }}
      SERVICE wikibase:label {{ bd:serviceParam wikibase:language "[AUTO_LANGUAGE],en". }}
    }} LIMIT 10
    """
    url = 'https://query.wikidata.org/sparql'
    headers = {
        'Accept': 'application/sparql-results+json',
        'User-Agent': 'MyWikidataScript/1.0 (your_email@example.com)'
    }
    response = requests.get(url, headers=headers, params={'query': query})
    response.raise_for_status()
    data = response.json()
    results = [(result['item']['value'].split('/')[-1], result['itemLabel']['value']) for result in data['results']['bindings']]
    return results

# Function to query Wikidata for superclass of or class relationships
def query_wikidata_superclasses(entity_id):
    print(f"Querying Wikidata for superclasses/classes of entity {entity_id}...")
    query = f"""
    SELECT ?item ?itemLabel WHERE {{
      {{
        wd:{entity_id} wdt:P279 ?item . # subclass of
      }} UNION {{
        wd:{entity_id} wdt:P31 ?item . # instance of
      }}
      SERVICE wikibase:label {{ bd:serviceParam wikibase:language "[AUTO_LANGUAGE],en". }}
    }} LIMIT 10
    """
    url = 'https://query.wikidata.org/sparql'
    headers = {
        'Accept': 'application/sparql-results+json',
        'User-Agent': 'MyWikidataScript/1.0 (your_email@example.com)'
    }
    response = requests.get(url, headers=headers, params={'query': query})
    response.raise_for_status()
    data = response.json()
    results = [(result['item']['value'].split('/')[-1], result['itemLabel']['value']) for result in data['results']['bindings']]
    return results

# Function to save results to a CSV file
def save_to_csv(filepath, data, header):
    with open(filepath, mode='w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(header)
        for row in data:
            writer.writerow(row)

# Main script logic
yolo_entities = read_groundtruth('groundtruth.csv')
wikidata5m_entities = read_wikidata5m('wikidata5m_entity.txt')

print("Checking entities against wikidata5m_entities...")
missing_entities = []
valid_entities = []
for name, entity_id in yolo_entities.items():
    if entity_id not in wikidata5m_entities:
        missing_entities.append((name, entity_id))
        print(f"Entity '{name}' with ID {entity_id} not found in wikidata5m_entities.")
    else:
        valid_entities.append((name, entity_id))

print("Querying Wikidata for subclasses and instances of valid entities...")
refinement_results = []
correction_results = []
header = ["name", "id"]
max_results = 10

for name, entity_id in valid_entities:
    try:
        # Query for subclasses and instances
        subclasses = query_wikidata_subclasses(entity_id)
        refinement_row = [name, entity_id]
        for subclass_id, subclass_label in subclasses:
            refinement_row.append(subclass_label)
            refinement_row.append(subclass_id)
            if len(refinement_row) >= max_results * 2 + 2:
                break
        refinement_results.append(refinement_row)
        time.sleep(1)  # Add delay to avoid rate limiting

        # Query for superclasses and classes
        superclasses = query_wikidata_superclasses(entity_id)
        correction_row = [name, entity_id]
        for superclass_id, superclass_label in superclasses:
            correction_row.append(superclass_label)
            correction_row.append(superclass_id)
            if len(correction_row) >= max_results * 2 + 2:
                break
        correction_results.append(correction_row)
        time.sleep(1)  # Add delay to avoid rate limiting

    except requests.exceptions.HTTPError as e:
        print(f"Error querying entity {entity_id}: {e}")

# Extend headers for CSV files based on max_results
for i in range(1, max_results + 1):
    header.append(f"result{i}_name")
    header.append(f"result{i}_id")

# Save results to CSV files
save_to_csv('yolo_refinement.csv', refinement_results, header)
save_to_csv('yolo_correction.csv', correction_results, header)

print("Final Results:")
print("Refinement results saved to yolo_refinement.csv")
print("Correction results saved to yolo_correction.csv")
print("Script execution completed.")
