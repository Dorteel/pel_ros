# ORKA ROS2 Wrapper Guide

This package provides a ROS2 wrapper around the ORKA (Ontology-based Robot Knowledge Abstraction) library, exposing its key functionalities as ROS2 services.

## Services Provided

The `graph_manager_node` exposes the following ROS2 services:

### 1. `load_graph` (LoadGraph)
Loads an ontology graph from a local OWL file.

**Request:**
- `string graph_path` - Path to the OWL file

**Response:**
- `bool success` - Whether the operation succeeded
- `string message` - Status message

**Example:**
```bash
ros2 service call /load_graph pel_ros2/srv/LoadGraph "{graph_path: '/path/to/ontology.owl'}"
```

---

### 2. `query_graph` (QueryGraph)
Run a SPARQL query against the loaded ontology graph.

**Request:**
- `string sparql_query` - SPARQL query string

**Response:**
- `bool success` - Whether the operation succeeded
- `string[] results` - Query results as strings
- `string message` - Status message

**Example:**
```bash
ros2 service call /query_graph pel_ros2/srv/QueryGraph "{sparql_query: 'SELECT ?robot WHERE { ?robot a Robot. }'}"
```

---

### 3. `save_graph` (SaveGraph)
Save the currently loaded ontology graph to disk.

**Request:**
- `string output_path` - Path where to save the file
- `string format` - Save format (e.g., "rdfxml", "ntriples", default: "rdfxml")

**Response:**
- `bool success` - Whether the operation succeeded
- `string saved_path` - Full path to the saved file
- `string message` - Status message

**Example:**
```bash
ros2 service call /save_graph pel_ros2/srv/SaveGraph "{output_path: '/tmp/ontology_out.owl', format: 'rdfxml'}"
```

---

### 4. `update_graph` (UpdateGraph)
Insert one triple into the loaded ontology graph.

**Request:**
- `string subject` - Triple subject URI
- `string predicate` - Triple predicate URI
- `string object_value` - Triple object value
- `bool object_is_literal` - Whether the object is a literal or a URI

**Response:**
- `bool success` - Whether the operation succeeded
- `string message` - Status message

**Example:**
```bash
ros2 service call /update_graph pel_ros2/srv/UpdateGraph "{subject: 'https://w3id.org/def/orka#obs1', predicate: 'https://w3id.org/def/orka#hasRawObservation', object_value: 'raw payload', object_is_literal: true}"
```

---

### 5. `reason_graph` (ReasonGraph)
Run automated reasoning over the ontology and optionally save the materialized graph.

**Request:**
- `string reasoner` - Reasoning engine to use: "hermit" (default) or "pellet"
- `bool infer_property_values` - Materialize inferred object property assertions (default: true)
- `bool infer_data_property_values` - Materialize inferred data assertions, Pellet only (default: true)
- `string save_path` - Optional path to save the materialized ontology
- `string format` - Save format for materialized ontology (default: "rdfxml")

**Response:**
- `bool success` - Whether the operation succeeded
- `bool consistent` - Whether the ontology is logically consistent
- `string used_reasoner` - The reasoner that was used
- `string saved_path` - Path to saved materialized ontology (if applicable)
- `string message` - Status message

**Example:**
```bash
ros2 service call /reason_graph pel_ros2/srv/ReasonGraph "{reasoner: 'hermit', infer_property_values: true, infer_data_property_values: true, save_path: '/tmp/materialized.owl'}"
```

---

## Workflow Example

Here's a typical workflow:

1. **Start the node:**
   ```bash
   ros2 run pel_ros2 graph_manager_node
   ```

2. **Load an ontology:**
   ```bash
   ros2 service call /load_graph pel_ros2/srv/LoadGraph "{graph_path: '/path/to/ontology.owl'}"
   ```

3. **Insert or update graph triples:**
   ```bash
   ros2 service call /update_graph pel_ros2/srv/UpdateGraph "{subject: 'https://w3id.org/def/orka#obs1', predicate: 'https://w3id.org/def/orka#hasRawObservation', object_value: 'raw payload', object_is_literal: true}"
   ```

4. **Perform reasoning:**
   ```bash
   ros2 service call /reason_graph pel_ros2/srv/ReasonGraph "{reasoner: 'hermit', save_path: '/tmp/materialized.owl'}"
   ```

5. **Query the reasoned ontology:**
   ```bash
   ros2 service call /query_graph pel_ros2/srv/QueryGraph "{sparql_query: 'SELECT ?sensor WHERE { ?sensor a Sensor. }'}"
   ```

6. **Save the modified ontology:**
   ```bash
   ros2 service call /save_graph pel_ros2/srv/SaveGraph "{output_path: '/path/to/output.owl'}"
   ```

---

## Important Notes

- A graph must be loaded via `load_graph` before calling any other service (query, save, update, or reason).
- The node maintains a single loaded ontology in memory. Calling `load_graph` will replace any previously loaded graph.
- All file paths should be absolute paths or properly resolvable relative paths.
- Reasoning can be computationally expensive; allow appropriate time for large ontologies.

---

## Dependencies

- `rclpy` - ROS2 Python client library
- `owlready2` - Python OWL ontology library
- `orka` (submodule) - ORKA ontology management library
