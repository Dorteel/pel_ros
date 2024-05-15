import os
from pyvis.network import Network
from owlready2 import get_ontology

def load_ontology(owl_file_path):
    # Load the ontology
    onto = get_ontology(f"file://{os.path.abspath(owl_file_path)}").load()
    return onto

def create_interactive_graph(ontology):
    net = Network(height="750px", width="100%", bgcolor="#222222", font_color="white", directed=True)
    
    # Add classes to the graph
    for cls in ontology.classes():
        net.add_node(cls.name, label=cls.name, title=cls.iri, color='gray')
    
    # Add instances to the graph
    for indiv in ontology.individuals():
        net.add_node(indiv.name, label=indiv.name, title=indiv.iri, color='blue')
    
    # Add object properties to the graph with edge labels and arrows
    for prop in ontology.object_properties():
        for domain in prop.domain:
            for range in prop.range:
                net.add_edge(domain.name, range.name, title=prop.name, label=prop.name, color='gray', arrows='to')

    # Configure physics to space out the nodes
    net.set_options("""
    var options = {
      "physics": {
        "barnesHut": {
          "gravitationalConstant": -2000,
          "centralGravity": 0.3,
          "springLength": 200,
          "springConstant": 0.05,
          "damping": 0.09
        },
        "minVelocity": 0.75
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
    owl_file_path = '/home/user/pel_ws/src/pel_ros/orka/owl/orka-core.owl'
    ontology = load_ontology(owl_file_path)
    net = create_interactive_graph(ontology)
    visualize_graph(net)
