import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import plotly.graph_objs as go

def remove_node(graph, node):
    graph.remove_node(node)

def remove_edge(graph, node1, node2):
    graph.remove_edge(node1, node2)

def add_node(graph, node, **attrs):
    graph.add_node(node, **attrs)

def add_edge(graph, node1, node2, **attrs):
    graph.add_edge(node1, node2, **attrs)

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

def calculate_rigidity_matrix(graph):
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
    eigenvalues = np.linalg.eigvals(rr_matrix)
    eigenvectors = np.linalg.eig(rr_matrix)
    return rigidity_matrix, rr_matrix, eigenvalues, eigenvectors

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
        wij = 1 / (dij ** 2) if dij != 0 else 0        
        E_measure += wij*(np.abs(distance - dij)) ** 2
    return E_measure

def measure_connectivity(graph):
    return len(graph.edges) / (len(graph.nodes) * (len(graph.nodes) - 1) / 2)

def plot_histograms(E):
    real_parts = np.real(E)
    imag_parts = np.imag(E)

    fig, axs = plt.subplots(1, 2, figsize=(10, 4))

    # Plot the histogram
    sns.histplot(real_parts, kde=True, bins=10, ax=axs[0])
    axs[0].set_title('Histogram of Fourth Eigenvalues')
    axs[0].set_xlabel('Fourth Eigenvalue')
    axs[0].set_ylabel('Frequency')
    axs[0].text(0.25, 0.5, f'Number of Data Points: {len(E)}', transform=axs[0].transAxes)

    # Plot the real and imaginary parts
    axs[1].scatter(real_parts, imag_parts)
    axs[1].set_title('Real vs. Imaginary Parts of Fourth Eigenvalues')
    axs[1].set_xlabel('Real Part')
    axs[1].set_ylabel('Imaginary Part')

    plt.tight_layout()
    plt.show()
    
# Create a 2D heatmap plot with marker size based on the fourth eigenvalue
def plot_coverage_energy(E,C,S,multiplier=50):
    real_parts = np.real(E)    
    plt.figure(figsize=(8, 6))
    marker_size = np.array(real_parts)/max(np.array(real_parts)) * multiplier
    sc = plt.scatter(C, S, c=real_parts, cmap='plasma', s=marker_size, alpha=0.5)
    # plt.yscale('log')
    plt.colorbar(sc, label='Real Part of Fourth Eigenvalue')
    plt.xlabel('Coverage')
    plt.ylabel('Energy')
    plt.title('2D Plot of Energy vs Coverage with Real Part of Fourth Eigenvalue')
    plt.grid(True)
    plt.show()
    
def plot_coverage_connectivity(E, C, CO, multiplier=50):
    real_parts = np.real(E)
    plt.figure(figsize=(8, 6))
    marker_size = np.array(real_parts) / max(np.array(real_parts)) * multiplier
    sc = plt.scatter(C, CO, c=real_parts, cmap='plasma', s=marker_size, alpha=0.5)
    plt.colorbar(sc, label='Real Part of Fourth Eigenvalue')
    plt.xlabel('Coverage')
    plt.ylabel('Connectivity')
    plt.title('2D Plot of Connectivity vs Coverage with Real Part of Fourth Eigenvalue')
    plt.grid(True)
    plt.show()
    
def plot_energy_connectivity(E, S, CO, multiplier=50):
    real_parts = np.real(E)
    plt.figure(figsize=(8, 6))
    marker_size = np.array(real_parts) / max(np.array(real_parts)) * multiplier
    sc = plt.scatter(S, CO, c=real_parts, cmap='plasma', s=marker_size, alpha=0.5)
    plt.colorbar(sc, label='Real Part of Fourth Eigenvalue')
    plt.xlabel('Energy')
    plt.ylabel('Connectivity')
    plt.title('2D Plot of Connectivity vs Energy with Real Part of Fourth Eigenvalue')
    plt.grid(True)
    plt.show()
    
def plot_3D_vis(E, C, S, CO, multiplier = 50):
    real_parts = np.real(E)
    marker_size = np.array(real_parts) / max(np.array(real_parts)) * multiplier

    sc = go.Scatter3d(
        x=C,
        y=S,#np.log10(S),  
        z=CO,
        mode='markers',
        marker=dict(
            size=marker_size,
            color=real_parts,
            colorscale='plasma',
            opacity=0.7,
            line=dict(
                color=real_parts,
                width=0.5
            ),
            colorbar=dict(title='Real Part of Fourth Eigenvalue')
        )
    )

    layout = go.Layout(
        title='3D Scatter plot',
        scene=dict(
            xaxis_title='Coverage',
            yaxis_title='Energy',
            zaxis=dict(title='Connectivity', range=[0, 1])  # Set the scale of connectivity to [0, 1]
        ),
        width=1000,  # Increase the width of the figure
        height=800  # Increase the height of the figure
    )

    fig = go.Figure(data=[sc], layout=layout)
    fig.show()
    
