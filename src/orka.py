import os
from owlready2 import get_ontology, sync_reasoner_pellet

# Path to the OWL file
owl_file_path = '/home/user/pel_ws/src/pel_ros/orka/owl/orka-core.owl'

# Load the ontology
onto = get_ontology(f"file://{os.path.abspath(owl_file_path)}").load(only_local=True)

# Optionally, run a reasoner to infer new knowledge
sync_reasoner_pellet()

# Now, you can access classes, properties, and instances in the ontology
print("Classes in the ontology:")
for cls in onto.classes():
    print(cls)

print("\nObject properties in the ontology:")
for prop in onto.object_properties():
    print(prop)

print("\nIndividuals in the ontology:")
for indiv in onto.individuals():
    print(indiv)