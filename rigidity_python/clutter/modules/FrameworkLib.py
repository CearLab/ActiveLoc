import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from shapely.geometry import Point
from shapely.ops import unary_union
import plotly.graph_objs as go

def create_graph():
    return nx.Graph()

def generate_graph(pos,max_dist):
    G = nx.Graph()
    for i in range(len(pos)):
        G.add_node(i,pos=pos[i])
    for i in range(len(pos)):
        for j in range(i+1,len(pos)):
            if np.linalg.norm(np.array(pos[i]) - np.array(pos[j])) < max_dist:
                G.add_edge(i,j)
    return G

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

def get_degree(graph, node):
    return graph.degree(node)

def get_adjacency_matrix(graph):
    return nx.adjacency_matrix(graph).todense()

def get_incidence_matrix(graph):
    return nx.incidence_matrix(graph).todense()

def get_laplacian_matrix(graph):
    return nx.laplacian_matrix(graph).todense()

def get_algebraic_connectivity(graph):
    distances = get_neighbors_distance(graph, node=None)
    edge_core = nx.algebraic_connectivity(graph,normalized=False)
    return np.sum(distances) ** 1 * edge_core    
    return nx.algebraic_connectivity(graph,normalized=False)    

def get_vertex_connectivity(graph):
    return nx.node_connectivity(graph)

def get_edge_connectivity(graph):
    return nx.edge_connectivity(graph)

def get_density(graph):
    return nx.density(graph)

def get_neighbors_distance(graph, node=None):
    if node is None:
        node = graph.number_of_nodes()-1
    neighbors = list(graph.neighbors(node))
    distances = []
    for neighbor in neighbors:
        distances.append(np.linalg.norm(np.array(graph.nodes[node]['pos']) - np.array(graph.nodes[neighbor]['pos'])))
    return distances

def get_edge_relation_core(graph):    
    return 2 * get_edge_connectivity(graph) * (1 - np.cos(np.pi / graph.number_of_nodes()))

def get_edge_relation(graph, node=None):
    distances = get_neighbors_distance(graph, node)
    edge_core = get_edge_relation_core(graph)
    return np.sum(distances) ** 1 * edge_core    
    # return edge_core    

def get_rigidity_matrix(graph):
    n = graph.number_of_nodes()
    m = graph.number_of_edges()
    rigidity_matrix = np.zeros((m, n * 2))
    i = 0
    for edge in graph.edges:
        a = edge[0]
        b = edge[1]
        pos_a = graph.nodes[a]['pos']
        pos_b = graph.nodes[b]['pos']
        rigidity_matrix[i, a * 2] = pos_b[0] - pos_a[0]
        rigidity_matrix[i, a * 2 + 1] = pos_b[1] - pos_a[1]
        rigidity_matrix[i, b * 2] = pos_a[0] - pos_b[0]
        rigidity_matrix[i, b * 2 + 1] = pos_a[1] - pos_b[1]
        i += 1   
    rr_matrix = np.dot(rigidity_matrix.T, rigidity_matrix)
    eigenvalues = np.sort(np.linalg.eigvals(rr_matrix))
    eigenvectors = np.linalg.eig(rr_matrix)
    return rigidity_matrix, rr_matrix, eigenvalues, eigenvectors

def is_rigid(graph, threshold=1e-10):
    R, RR, eigenvalues, eigenvectors = get_rigidity_matrix(graph)
    if len(eigenvalues) < 4:
        return False, 0.0
    return np.real(eigenvalues[3]) > threshold, np.real(eigenvalues[3])

def get_singular_values(graph):
    R, RR, eigenvalues, eigenvectors = get_rigidity_matrix(graph)
    U, sing_vals, V = np.linalg.svd(RR)
    sing_vals = sing_vals[sing_vals > 1e-5]
    condition_number = sing_vals[0] / sing_vals[-1]
    return sing_vals, condition_number

def measure_energy(graph):
    E_measure = 0
    for edge in graph.edges:
        node1, node2 = edge
        pos1 = np.array(graph.nodes[node1]['pos'])
        pos2 = np.array(graph.nodes[node2]['pos'])
        distance = np.linalg.norm(pos1 - pos2)
        # Assuming 'dij' is stored in edge attributes, default to 0 if not present
        dij = graph[node1][node2].get('dij', 1)  
        # wij is 1/dij^2 if dij present, otherwise 0
        wij = 1 / (distance ** 2) if dij != 0 else 0        
        E_measure += wij*(distance** 2)
    return E_measure

def get_dispersion(graph):
    L = get_laplacian_matrix(graph)
    x = np.array([graph.nodes[node]['pos'][0] for node in graph.nodes])
    y = np.array([graph.nodes[node]['pos'][1] for node in graph.nodes])
    spread_x = np.sqrt(np.dot(x, np.dot(L, x.transpose())))
    spread_y = np.sqrt(np.dot(y, np.dot(L, y.transpose())))
    return np.sqrt(spread_x**2 + spread_y**2)    

def get_coverage(graph,max_radius=1):
    circles = [Point(graph.nodes[node]['pos']).buffer(max_radius) for node in graph.nodes]
    union = unary_union(circles)
    
    return union.area
    
