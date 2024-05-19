import os
import rdflib
from pyvis.network import Network

# Directory containing the RDF files
base_directory = 'obs_graphs'

# Define colors for different node and edge types
color_instance = '#FFA500'  # Orange
color_data_property = '#00FF00'  # Green
color_object_property = '#FF0000'  # Red
color_class = '#0000FF'  # Blue

# Define additional colors for different classes
class_colors = {
    'ssn:Observation': '#FFD700',  # Gold
    'ssn:Sensor': '#8A2BE2',  # BlueViolet
    # Add more class colors as needed
}

def is_literal(node):
    """
    Check if the node is a literal.

    Args:
    node (rdflib.term.Node): The RDF node to check.

    Returns:
    bool: True if the node is a literal, False otherwise.
    """
    return isinstance(node, rdflib.Literal)

def is_class(node):
    """
    Check if the node is a class.

    Args:
    node (rdflib.term.Node): The RDF node to check.

    Returns:
    bool: True if the node is a class, False otherwise.
    """
    return isinstance(node, rdflib.URIRef) and str(node).startswith('http://www.w3.org/2002/07/owl#Class')

def get_prefixed_name(graph, node):
    """
    Get the prefixed name of a node using the graph's namespace manager.

    Args:
    graph (rdflib.Graph): The RDF graph containing the namespaces.
    node (rdflib.term.Node): The RDF node to get the prefixed name for.

    Returns:
    str: The prefixed name of the node.
    """
    return graph.namespace_manager.normalizeUri(node)

def clean_node_name(name):
    """
    Clean the node name by removing specific prefixes.

    Args:
    name (str): The original name of the node.

    Returns:
    str: The cleaned name of the node.
    """
    if name.startswith('http://example.org/sensors//'):
        return name.replace('http://example.org/sensors//', '')
    return name

def visualize_graph(graph, filename):
    """
    Visualize an RDF graph using pyvis.

    Args:
    graph (rdflib.Graph): The RDF graph to visualize.
    filename (str): The filename of the RDF file.
    """
    net = Network(directed=True)

    # Iterate through RDF triples and add nodes and edges to the network
    for s, p, o in graph:
        s_str = clean_node_name(get_prefixed_name(graph, s))
        p_str = get_prefixed_name(graph, p)

        # Determine node types and apply colors
        if is_class(s):
            net.add_node(s_str, label=s_str, color=color_class)
        else:
            class_label = graph.value(s, rdflib.RDF.type)
            class_color = class_colors.get(get_prefixed_name(graph, class_label), color_instance)
            net.add_node(s_str, label=s_str, color=class_color)

        if is_literal(o):
            # For literals, create a node with the last 10 characters of the value
            o_str = str(o)[-100:]
            net.add_node(o_str, label=o_str, color=color_data_property)
            net.add_edge(s_str, o_str, label=p_str, color=color_data_property)
        else:
            o_str = clean_node_name(get_prefixed_name(graph, o))
            if is_class(o):
                net.add_node(o_str, label=o_str, color=color_class)
            else:
                class_label = graph.value(o, rdflib.RDF.type)
                class_color = class_colors.get(get_prefixed_name(graph, class_label), color_instance)
                net.add_node(o_str, label=o_str, color=class_color)
            net.add_edge(s_str, o_str, label=p_str, color=color_object_property)

    # Save the visualized graph to an HTML file
    output_filename = filename.replace(".rdf", ".html")
    net.show(output_filename, notebook=False)
    print(f"Graph visualized and saved to {output_filename}")

def get_latest_folder(directory):
    """
    Get the most recently created folder in the specified directory.

    Args:
    directory (str): The path to the base directory.

    Returns:
    str: The path to the most recently created folder.
    """
    folders = [os.path.join(directory, d) for d in os.listdir(directory) if os.path.isdir(os.path.join(directory, d))]
    latest_folder = max(folders, key=os.path.getmtime)
    return latest_folder

def main():
    """
    Main function to load RDF files and visualize them.
    """
    if not os.path.exists(base_directory):
        print(f"Directory {base_directory} does not exist.")
        return

    latest_folder = get_latest_folder(base_directory)

    rdf_files = [f for f in os.listdir(latest_folder) if f.endswith('.rdf')]

    if not rdf_files:
        print(f"No RDF files found in directory {latest_folder}.")
        return

    for rdf_file in rdf_files:
        rdf_path = os.path.join(latest_folder, rdf_file)
        graph = rdflib.Graph()
        graph.parse(rdf_path, format='xml')
        visualize_graph(graph, rdf_path)

if __name__ == '__main__':
    main()
