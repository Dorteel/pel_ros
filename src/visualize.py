import os
from pyvis.network import Network
from owlready2 import get_ontology, Thing

#owl_file_path = '/home/user/pel_ws/src/pel_ros/orka/owl/orka-full-inferred.rdf'
owl_file_path = '/home/user/pel_ws/src/pel_ros/src/obs_graphs/knowledge_graph__TIAGo_LITE_head_1_joint_sensor_20240516_142433.rdf'

def load_ontology(owl_file_path):
    # Load the ontology
    onto = get_ontology(f"file://{os.path.abspath(owl_file_path)}").load()
    return onto

def create_interactive_graph(ontology):
    net = Network(height="1000px", width="100%", bgcolor="#222222", font_color="white", directed=True)
    
    # Add classes to the graph
    for cls in ontology.classes():
        net.add_node(cls.name, label=cls.name, title=cls.iri, color='gray')
    
    # Add instances to the graph
    for indiv in ontology.individuals():
        net.add_node(indiv.name, label=indiv.name, title=indiv.iri, color='blue')
        # Link individuals to their classes
        for cls in indiv.is_a:
            net.add_edge(cls.name, indiv.name, title='instance of', label='instance of', color='purple', arrows='to')
    
    # Add subclass relationships to the graph
    for cls in ontology.classes():
        for sub_cls in cls.subclasses():
            net.add_edge(cls.name, sub_cls.name, title='subclass of', label='subclass of', color='green', arrows='to')

    # Add object properties to the graph with edge labels and arrows
    for prop in ontology.object_properties():
        for domain in prop.domain:
            for range in prop.range:
                net.add_edge(domain.name, range.name, title=prop.name, label=prop.name, color='gray', arrows='to')

    # Add object property assertions (relationships between individuals)
    for prop in ontology.object_properties():
        for assertion in prop.get_relations():
            domain = assertion[0].name
            range = assertion[1].name
            net.add_edge(domain, range, title=prop.name, label=prop.name, color='orange', arrows='to')

    # Add data property assertions (attribute values for individuals)
    for prop in ontology.data_properties():
        for assertion in prop.get_relations():
            individual = assertion[0].name
            value = str(assertion[1])
            net.add_node(f"{individual}_{prop.name}", label=value, title=prop.name, color='red')
            net.add_edge(individual, f"{individual}_{prop.name}", title=prop.name, label=prop.name, color='red', arrows='to')

    # Configure physics to space out the nodes
    net.set_options("""
    var options = {
      "physics": {
        "barnesHut": {
          "gravitationalConstant": -30000,
          "centralGravity": 0.1,
          "springLength": 250,
          "springConstant": 0.05,
          "damping": 0.09
        },
        "minVelocity": 0.75
      },
      "nodes": {
        "borderWidth": 2,
        "borderWidthSelected": 4,
        "font": {
          "size": 16,
          "strokeWidth": 2
        },
        "shapeProperties": {
          "useBorderWithImage": true
        }
      }
    }
    """)
    
    return net

def visualize_graph(net):
    # Ensure the template is correctly loaded and written to HTML
    try:
        net.show("knowledge_graph.html", notebook=False)
        print("Knowledge graph visualization generated: knowledge_graph.html")
    except AttributeError as e:
        print(f"Error generating visualization: {e}")
        print("Ensure pyvis is correctly installed and accessible in your environment.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

if __name__ == '__main__':
    ontology = load_ontology(owl_file_path)
    net = create_interactive_graph(ontology)
    visualize_graph(net)
