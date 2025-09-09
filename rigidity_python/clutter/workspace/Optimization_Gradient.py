import modules.FrameworkLib as FL
import numpy as np
from scipy.sparse.linalg import cg
    
# class gradient optimization
class Optimization_gradient:
    def __init__(self, graph, indices_var, max_dist, weighted, box_margin, mu, beta):
        
        self._G = graph
        
        self._max_dist = max_dist                
        self._map_radius = 1   
        self._weighted = weighted     
        
        self._N_agents = len(graph.nodes)
        self._N_edges = len(graph.edges)
        self._indices_var = False*np.ones(self._N_agents)
        self._indices_var[indices_var] = True
        self._indices_var = self._indices_var.astype(bool)
        
        # step parameterz
        self._mu_init = mu
        self._mu = mu
        self._beta = beta          
        self._eps = []       
        self._safety_connectivity = []
        self._gamma_grad_update = []
        self._grad_filter = np.ones((self._N_agents, 2))
                
        if np.isscalar(box_margin) == 1:       
            self._box_margin = np.ones((4*self._N_agents,1))
            self._box_margin[::2,0] = 0
            self._box_margin[1::2,0] = box_margin
        else:
            self._box_margin = box_margin
                            
        self._pos = np.array(list(FL.get_positions(graph).values()))
        self._pos_var = self._pos[self._indices_var,:]
        self._pos_fix = self._pos[np.logical_not(self._indices_var),:]
        
        # matrices
        self._matrix_adjacency = self.update_adjacency(self._pos, True * np.ones([self._N_agents, 1]), np.zeros((self._N_agents, self._N_agents)))
        self._matrix_degree = self.adjacency2degree(self._matrix_adjacency)
        self._matrix_incidence = self.adjacency2incidence(self._matrix_adjacency)
        self._matrix_laplacian, dL = self.laplacian(self._matrix_degree, self._matrix_adjacency, init=True)                
        
        # connectivity
        self._eigvals = []
        self._eigvect = []
        self._algebraic_connectivity, self._v_fiedler = self.get_connectivity(self._matrix_laplacian, dL, init=True)        
        
        # cost function
        self._objective, self._cost, self._barrier, self._constraints = self.objective()
                
    
    def gradient_step(self, x) -> np.ndarray:                
        
        # gradient
        grad = self.gradient()
        
        # gamma scaling        
        self._grad_filter = self._gamma_grad_update * self._grad_filter + (1-self._gamma_grad_update) * grad ** 2
        self._mu = self._mu_init / np.sqrt(self._grad_filter + 1e-6)
        
        # step
        step = self._mu * grad
        clip_step = 1e0
        step[:,0] = np.clip(step[:,0], -clip_step, clip_step)
        step[:,1] = np.clip(step[:,1], -clip_step, clip_step)
        x = x + step
        
        # box constraints
        for i in range(len(x)//2):
            x[i,0] = np.clip(x[i,0], self._box_margin[2*i,0], self._box_margin[2*i+1,0])
            x[i,1] = np.clip(x[i,1], self._box_margin[2*(i+1),0], self._box_margin[2*(i+1)+1,0])        
        
        # update laplacian
        self.update_graph(x)
        
        return x, step
    
    def objective(self) -> float:
                                        
        barrier_connectivity =  self._beta[0] * np.log(np.clip(1 + self._algebraic_connectivity - self._safety_connectivity, self._eps, 1))
        
        cost = 0
        J = cost + barrier_connectivity
        
        return J, cost, [barrier_connectivity], [self._algebraic_connectivity]
    
    def gradient(self) -> np.ndarray:                
        
        # cycle over variables
        grad = np.zeros([self._N_agents, 2])
        grad_L = self.gradient_laplacian(self._pos, self._matrix_adjacency)
        
        for index in range(len(self._indices_var)):
            
            if self._indices_var[index] == True:                
            
                #### GRADIENT COST ####                                                
                grad_cost = np.array((0, 0))
                
                #### GRADIENT BARRIER CONNECTIVITY ####      
                grad_algebraic_connectivity = self.gradient_algebraic_connectivity(grad_L, index)
                grad_barrier_connectivity =   self._beta[0] * grad_algebraic_connectivity/ \
                    (np.clip(1 + self._algebraic_connectivity - self._safety_connectivity, self._eps, 1))
                                 
                # gradient                
                grad[index,:] = grad_cost + grad_barrier_connectivity
                                
        return grad
    
    def gradient_algebraic_connectivity(self, grad_L, index) -> np.ndarray:
        
        grad_L_index = 0 * grad_L
        grad_L_index[index,:,:] = grad_L[index,:,:]
        
        grad_algebraic_connectivity = np.zeros(2)
                
        eigv = self._v_fiedler.reshape(self._N_agents, 1)
        grad_algebraic_connectivity[0] = eigv.T @ grad_L_index[:,:,0] @ eigv
        grad_algebraic_connectivity[1] = eigv.T @ grad_L_index[:,:,1] @ eigv
        
        return grad_algebraic_connectivity
    
    def gradient_laplacian(self, pos, adjacency) -> np.ndarray:
        
        # init gradient
        grad_L = np.zeros((self._N_agents, self._N_agents, 2))
        
        # cycle over agents
        for index in range(len(self._indices_var)):
            
            if self._indices_var[index] == True:
            
                # distances
                Dd = adjacency[index, :]
                
                # selector
                e_index = np.zeros(self._N_agents)
                e_index[index] = 1
                
                # cycle over agents
                for j in range(self._N_agents):
                    if index != j and Dd[j] > 0:
                        
                        # selector
                        e_j = np.zeros(self._N_agents)
                        e_j[j] = 1
                        
                        # gradient
                        grad_L[index, j,:] += (pos[index,:] - pos[j,:])/Dd[j] * ( (e_index @ e_index.T) - (e_index @ e_j.T) )                                    
        return grad_L
    
    def update_graph(self, x) -> float:
        
        # manage positions
        self._pos = x
        self._pos_var = x[self._indices_var,:]
        self._pos_fix = x[np.logical_not(self._indices_var),:]
        
        self._matrix_adjacency = self.update_adjacency(self._pos, self._indices_var, self._matrix_adjacency)
        self._matrix_degree = self.adjacency2degree(self._matrix_adjacency)
        self._matrix_incidence = self.adjacency2incidence(self._matrix_adjacency)
        self._matrix_laplacian, dL = self.laplacian(self._matrix_degree, self._matrix_adjacency, init=False)                
        
        self._algebraic_connectivity, self._v_fiedler = self.get_connectivity(self._matrix_laplacian, dL, init=False)
        
        self._objective, self._cost, self._barrier, self._constraints = self.objective()
        
    def get_connectivity(self, laplacian, dL=[], init=False) -> float:
        
        #### here you can compute the eigenvalues and eigenvectors o(N^2) ####
        if init == True:
            eigvals, eigvect = np.linalg.eig(laplacian)
            # Sort eigenvalues and eigenvectors
            idx = eigvals.argsort()
            eigvals = eigvals[idx]
            eigvect = eigvect[:, idx]                    
            algebraic_connectivity = eigvals[1]            
            v_fiedler = eigvect[:,1].reshape(self._N_agents, 1)
        else:        
            # here you can update the algebraic connectivity and the fiedler vector                                    
            
            # power iteration method
            v_fiedler = self._v_fiedler.copy()
            
            A = laplacian - self._algebraic_connectivity * np.eye(self._N_agents)
            b = dL @ v_fiedler
            correction, info = cg(A, b, rtol = 1e-6, maxiter=100)            
            v_fiedler = v_fiedler + correction.reshape(self._N_agents, 1)
            v_fiedler = v_fiedler / np.linalg.norm(v_fiedler)                                
            algebraic_connectivity = float(v_fiedler.T @ laplacian @ v_fiedler / (v_fiedler.T @ v_fiedler))
        
        return algebraic_connectivity, v_fiedler        
    
    def update_adjacency(self, pos, indices_var, adjacency) -> np.ndarray:
        
        # compute the new distances and adjacency matrix
        for index in range(len(indices_var)):
            if indices_var[index] == True:
                # get new distances
                dist = np.linalg.norm(pos - pos[index,:], axis=1)
                
                # cut to max_dist
                dist[dist > self._max_dist] = 0
                
                # update adjacency
                adjacency[index,:] = dist
                adjacency[:,index] = dist
                
        return adjacency
    
    def adjacency2incidence(self, adj) -> np.ndarray:                
        
        # incidence matrix
        N_edges_max = self._N_agents*(self._N_agents-1)//2
        B = np.zeros([N_edges_max, self._N_agents])
        row = 0
        for i in range(self._N_agents):
            for j in range(i+1, self._N_agents):
                if (i != j) and adj[i, j] > 0:                    
                    B[row, i] = 1
                    B[row, j] = -1
                    row = row + 1
                    
        # remove empty rows
        B = B[~np.all(B == 0, axis=1)]
        
        return B
    
    def adjacency2degree(self, adj) -> np.ndarray:
        
        # degree matrix
        D = np.zeros((self._N_agents, self._N_agents))
        for i in range(self._N_agents):
            D[i, i] = np.sum(adj[i, :])
        
        return D
    
    def laplacian(self, degree, adjacency, init=False) -> np.ndarray:
        laplacian = degree - adjacency
        if init == True:
            delta_laplacian = 0 *laplacian
        else:
            delta_laplacian = laplacian - self._matrix_laplacian
        return laplacian, delta_laplacian        