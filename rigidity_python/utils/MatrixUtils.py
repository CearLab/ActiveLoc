import numpy as np
import networkx as nx



def CalcAdjacencyMatrix(graph: nx.Graph):
    """
    Calculate the adjacency matrix of a given graph
    :param graph: networkx graph
    :return: numpy matrix
    """
    return nx.adjacency_matrix(graph).toarray()

def calcDistanceMatrix(graph: nx.Graph):
    """
    Calculate the distance matrix of a given graph
    :param graph: networkx graph
    :return: numpy matrix
    """
    # get the number of nodes (witch is the number of agents)
    n = graph.number_of_nodes()
    # initialize the matrix
    distance_matrix = np.zeros((n, n))
    # go throw all the edges and assign the distance between the two agents
    for edge in graph.edges:
        a = graph.nodes[edge[0]]['agent']
        b = graph.nodes[edge[1]]['agent']
        d = np.linalg.norm(a.gt_pos.to_array() - b.gt_pos.to_array())
        distance_matrix[a.id, b.id] = d
        distance_matrix[b.id, a.id] = d
    
    return distance_matrix

def calcIncidenceMatrix(graph: nx.Graph):
    """
    Calculate the incidence matrix of a given graph
    :param graph: networkx graph
    :return: numpy matrix
    """
    return nx.incidence_matrix(graph).toarray()

def calcRigitdyMatrix(graph: nx.Graph):
    """
    Calculate the rigidity matrix of a given graph
    :param graph: networkx graph
    :return: numpy matrix
    """
    # get the number of nodes (witch is the number of agents)
    n = graph.number_of_nodes()
    # get the number of edges
    m = graph.number_of_edges()
    # initialize the matrix
    rigidity_matrix = np.zeros((m, n*2))
    # go throw all the edges and assign the rigidity matrix
    i = 0
    for edge in graph.edges:
        a = graph.nodes[edge[0]]['agent']
        b = graph.nodes[edge[1]]['agent']
        rigidity_matrix[i, a.id*2] = b.gt_pos.x - a.gt_pos.x
        rigidity_matrix[i, a.id*2+1] = b.gt_pos.y - a.gt_pos.y
        rigidity_matrix[i, b.id*2] = a.gt_pos.x - b.gt_pos.x
        rigidity_matrix[i, b.id*2+1] = a.gt_pos.y - b.gt_pos.y
        i += 1
    return rigidity_matrix

def checkConnectivity(graph: nx.Graph):
    """
    Check if the graph is connected
    :param graph: networkx graph
    :return: bool
    """
    A = nx.adjacency_matrix(graph).toarray()
    n = A.shape[0]
    Apow = np.linalg.matrix_power(A, n-1)
    # if A^(n-1) has no zero elements than is connected
    return not np.any(Apow == 0)

def checkKconnectivity(graph: nx.Graph, k):
    """
    Check if the graph is k-connected
    :param graph: networkx graph
    :param k: int
    :return: bool
    """
    return nx.is_k_edge_connected(graph, k)

def isRidged(graph: nx.Graph):
    """
    Check if the graph is ridged
    :param graph: networkx graph
    :return: bool
    """
    _TOL = 1e-10
    ridgidty_matrix = calcRigitdyMatrix(graph)
    
    eigvals = np.linalg.eigvals(ridgidty_matrix.T @ ridgidty_matrix)
    # ! i think there is some mistake here, the graph from the test should not be ridged
    #check if only the first 3 eigenvalues are zero with tolerance
    return np.sum(np.abs(eigvals) < _TOL) == 3
    

