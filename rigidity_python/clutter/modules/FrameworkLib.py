import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from shapely.geometry import Point
from shapely.ops import unary_union
import plotly.graph_objs as go
import concurrent.futures

def create_graph():
    return nx.Graph()

def generate_graph(pos,max_dist,weighted=False):
    G = nx.Graph()
    for i in range(len(pos)):
        G.add_node(i,pos=pos[i])
    for i in range(len(pos)):
        for j in range(i+1,len(pos)):
            dij = np.linalg.norm(np.array(pos[i]) - np.array(pos[j]))
            if dij < max_dist:
                if weighted == False:
                    G.add_edge(i,j)
                else:
                    weight_val = get_weight(G,i,j)
                    G.add_edge(i,j,weight=weight_val)
    return G

def get_weight(graph, node1, node2):
    p1 = np.array(graph.nodes[node1]['pos'])
    p2 = np.array(graph.nodes[node2]['pos'])
    weight = np.linalg.norm(p1 - p2)
    return weight

def get_positions(graph):
    return nx.get_node_attributes(graph, 'pos')

def add_node(graph, node, **attrs):
    graph.add_node(node, **attrs)

def remove_node(graph, node):
    graph.remove_node(node)
    
def add_edge(graph, node1, node2, **attrs):
    graph.add_edge(node1, node2, **attrs)

def remove_edge(graph, node1, node2):
    graph.remove_edge(node1, node2)

def get_nodes(graph):
    return graph.nodes(data=True)

def get_edges(graph):
    return graph.edges(data=True)

def find_shortest_path(graph, source, target):
    try:
        return nx.shortest_path(graph, source=source, target=target)
    except nx.NetworkXNoPath:
        return None

def is_connected(graph):
    return nx.is_connected(graph)

def get_algebraic_connectivity(graph):            
    lambda2 = nx.algebraic_connectivity(graph, normalized=True, method='lanczos')
    return lambda2

def get_neighbors_distance(graph, node=None):
    if node is None:
        node_list = graph.nodes()
        
    distances = []
    for node in node_list:
        neighbors = list(graph.neighbors(node))    
        for neighbor in neighbors:
            distances.append(np.linalg.norm(np.array(graph.nodes[node]['pos']) - np.array(graph.nodes[neighbor]['pos'])))
    return distances

def get_edge_connectivity(graph):
    return 2 * nx.edge_connectivity(graph) * (1 - np.cos(np.pi / graph.number_of_nodes()))

def get_edge_relation(graph):        
    lambda_2 = 0.5 * get_algebraic_connectivity(graph)
    return lambda_2

def get_neighbors(graph, node=None):
    if node is None:
        node = graph.number_of_nodes()-1
    return list(graph.neighbors(node))

def get_coverage(graph, max_radius=1):
    # Extract positions from the graph
    positions = np.array([graph.nodes[node]['pos'] for node in graph.nodes])
    # This measures "how much we are NOT overlapping"
    total_repulsion = 0
    for i in range(len(positions)):
        for j in range(i + 1, len(positions)):
            dist = np.linalg.norm(positions[i] - positions[j])
            # If they are closer than 2*radius, they overlap.
            # We create a penalty that is high when they are on top of each other
            if dist < 2 * max_radius:
                total_repulsion += (2 * max_radius - dist)**2
    # We invert it: High repulsion = Low Coverage
    # Normalize by the max possible repulsion (all robots at same point)
    n = len(positions)
    num_pairs = (n * (n - 1)) / 2
    # The most repulsion possible is when distance is 0 for every pair
    max_rep = num_pairs * (2 * max_radius)**2 

    if max_rep > 0:
        norm_cov = 1 - (total_repulsion / max_rep)
    else:
        norm_cov = 1.0
    return norm_cov
    
