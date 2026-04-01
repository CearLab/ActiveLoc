import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from shapely.geometry import Point
from shapely.ops import unary_union
import plotly.graph_objs as go
import concurrent.futures

def create_graph():
    """Create and return an empty undirected graph."""
    return nx.Graph()

def generate_graph(pos,max_dist,weighted=False):
    """Build a geometric communication graph from agent positions.

    Nodes are indexed by their position order. An edge is created when the
    Euclidean distance between two agents is below max_dist.
    """
    G = nx.Graph()
    # Persist positions on nodes so downstream metrics can be computed from G only.
    for i in range(len(pos)):
        G.add_node(i,pos=pos[i])
    # Connect pairs within communication range.
    for i in range(len(pos)):
        for j in range(i+1,len(pos)):
            dij = np.linalg.norm(np.array(pos[i]) - np.array(pos[j]))
            if dij < max_dist:
                if weighted == False:
                    G.add_edge(i,j)
                else:
                    # Weighted mode stores geometric distance as edge weight.
                    weight_val = get_weight(G,i,j)
                    G.add_edge(i,j,weight=weight_val)
    return G

def get_weight(graph, node1, node2):
    """Return Euclidean distance between two nodes using stored node positions."""
    p1 = np.array(graph.nodes[node1]['pos'])
    p2 = np.array(graph.nodes[node2]['pos'])
    weight = np.linalg.norm(p1 - p2)
    return weight

def get_positions(graph):
    """Return node positions as a {node_id: position} dictionary."""
    return nx.get_node_attributes(graph, 'pos')

def add_node(graph, node, **attrs):
    """Add a node and optional attributes to the graph."""
    graph.add_node(node, **attrs)

def remove_node(graph, node):
    """Remove a node from the graph."""
    graph.remove_node(node)
    
def add_edge(graph, node1, node2, **attrs):
    """Add an edge and optional attributes between two nodes."""
    graph.add_edge(node1, node2, **attrs)

def remove_edge(graph, node1, node2):
    """Remove an edge between two nodes."""
    graph.remove_edge(node1, node2)

def get_nodes(graph):
    """Return graph nodes with attributes."""
    return graph.nodes(data=True)

def get_edges(graph):
    """Return graph edges with attributes."""
    return graph.edges(data=True)

def find_shortest_path(graph, source, target):
    """Find shortest path between two nodes, or None if disconnected."""
    try:
        return nx.shortest_path(graph, source=source, target=target)
    except nx.NetworkXNoPath:
        return None

def is_connected(graph):
    """Check whether the full graph is connected."""
    return nx.is_connected(graph)

def get_algebraic_connectivity(graph):            
    """Compute normalized algebraic connectivity (Fiedler value)."""
    lambda2 = nx.algebraic_connectivity(graph, normalized=True, method='lanczos')
    return lambda2

def get_neighbors_distance(graph, node=None):
    """Collect distances between nodes and their neighbors.

    Distances are returned as a flat list and include both directions for each
    undirected edge when iterating by node neighborhood.
    """
    if node is None:
        node_list = graph.nodes()
        
    distances = []
    for node in node_list:
        neighbors = list(graph.neighbors(node))    
        for neighbor in neighbors:
            distances.append(np.linalg.norm(np.array(graph.nodes[node]['pos']) - np.array(graph.nodes[neighbor]['pos'])))
    return distances

def get_edge_connectivity(graph):
    """Return edge connectivity scaled by graph size.

    The scaling term normalizes magnitude across different node counts.
    """
    return 2 * nx.edge_connectivity(graph) * (1 - np.cos(np.pi / graph.number_of_nodes()))

def get_edge_relation(graph):        
    """Return a lightweight connectivity score derived from algebraic connectivity."""
    """The 0.5 factor is because the normalized laplacian eigenvalues are in [0, 2], so this maps lambda_2 to [0, 1]. """
    lambda_2 = 0.5 * get_algebraic_connectivity(graph)
    return lambda_2

def get_neighbors(graph, node=None):
    """Return neighbors of a node, defaulting to the last node index."""
    if node is None:
        node = graph.number_of_nodes()-1
    return list(graph.neighbors(node))

def get_coverage(graph, max_radius=1):
    """Compute a normalized overlap-based coverage score in [0, 1].

    The metric penalizes pairwise overlap (distance < 2*max_radius). A score
    near 1 means well spread agents; near 0 means strong overlap/clustering.
    """
    # Extract positions from the graph.
    positions = np.array([graph.nodes[node]['pos'] for node in graph.nodes])
    # Accumulate overlap penalty between all pairs.
    total_repulsion = 0
    for i in range(len(positions)):
        for j in range(i + 1, len(positions)):
            dist = np.linalg.norm(positions[i] - positions[j])
            # If agents are closer than 2*radius, disks overlap and incur penalty.
            if dist < 2 * max_radius:
                total_repulsion += (2 * max_radius - dist)**2

    # Convert penalty to a normalized score by comparing against worst case
    # (all agents collocated).
    n = len(positions)
    num_pairs = (n * (n - 1)) / 2
    max_rep = num_pairs * (2 * max_radius)**2 

    if max_rep > 0:
        norm_cov = 1 - (total_repulsion / max_rep)
    else:
        norm_cov = 1.0
    return norm_cov
    
