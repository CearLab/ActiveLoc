import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from shapely.geometry import Point
from shapely.ops import unary_union
import plotly.graph_objs as go

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
    laplacian = nx.laplacian_matrix(graph).todense()
    eigvals, eigvect = np.linalg.eig(laplacian)
    # Sort eigenvalues and eigenvectors
    idx = eigvals.argsort()
    eigvals = eigvals[idx]
    eigvect = eigvect[:, idx]                    
    lambda2 = eigvals[1]            
    N_agents = eigvect.shape[0]
    v_fiedler = eigvect[:,1].reshape(N_agents, 1)
    return lambda2, v_fiedler    

def get_neighbors_distance(graph, node=None):
    if node is None:
        node_list = graph.nodes()
        
    distances = []
    for node in node_list:
        neighbors = list(graph.neighbors(node))    
        for neighbor in neighbors:
            distances.append(np.linalg.norm(np.array(graph.nodes[node]['pos']) - np.array(graph.nodes[neighbor]['pos'])))
    return distances

def get_edge_relation(graph):    
    # distances = get_neighbors_distance(graph)
    # return 2 * nx.edge_connectivity(graph) * (1 - np.cos(np.pi / graph.number_of_nodes()))
    lambda_2 = get_algebraic_connectivity(graph)[0]
    return lambda_2

def get_neighbors(graph, node=None):
    if node is None:
        node = graph.number_of_nodes()-1
    return list(graph.neighbors(node))

def get_dispersion(graph):
    E_measure = 0
    for edge in graph.edges:
        node1, node2 = edge
        pos1 = np.array(graph.nodes[node1]['pos'])
        pos2 = np.array(graph.nodes[node2]['pos'])
        distance = np.linalg.norm(pos1 - pos2)               
        E_measure += (distance** 3)
    return E_measure   

def get_coverage(graph,max_radius=1):
    circles = [Point(graph.nodes[node]['pos']).buffer(max_radius) for node in graph.nodes]
    union = unary_union(circles)
    return union.area
    
