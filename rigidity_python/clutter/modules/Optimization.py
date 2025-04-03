import modules.FrameworkLib as FL
import numpy as np

class Objective:
    def __init__(self, N_agents, max_dist, threshold, box_margin, alpha):
        
        self.N_agents = N_agents
        self.max_dist = max_dist
        self.threshold = threshold
        self.alpha = alpha 
        self.leader_ID = N_agents-1 # the last agent is the leader (see objectuve_function)
        
        self.map_radius = 1
        self.init = True               
                
        if np.isscalar(box_margin) == 1:       
            self.box_margin = np.ones((4*N_agents,1))
            self.box_margin[::2,0] = 0
            self.box_margin[1::2,0] = box_margin
        else:
            self.box_margin = box_margin
            
        self.G = []
        self.pos_fix = []                
        self.J_edge = []
        self.J_coverage = []
        self.J_algebraic_connectivity = []
        self.update_normalizers()

    def __call__(self, trial):
        return self.objective_function(trial)                

    def constraints(trial):
        return trial.user_attrs["constraint"]
    
    def update_normalizers(self):
        # get edge_relation and coverage max
        area_box = (self.box_margin[1]-self.box_margin[0])*(self.box_margin[3]-self.box_margin[2])
        self.coverage_max = self.N_agents*np.pi*self.map_radius**2#/area_box
        
        # if edge_relation_core
        self.edge_relation_max = 2*(self.N_agents-1)*(1-np.cos(np.pi/self.N_agents))
        self.algebraic_connectivity_max = self.N_agents-1
        
        # if edge_relation_length        
        self.edge_relation_max = self.edge_relation_max * self.max_dist * (self.N_agents-1)#/np.sqrt(area_box)
        self.algebraic_connectivity_max = self.algebraic_connectivity_max * self.max_dist * (self.N_agents-1)#/np.sqrt(area_box)
        
        magnitude = self.edge_relation_max + self.coverage_max
        self.edge_relation_normalizer = 1   * magnitude/self.edge_relation_max
        self.coverage_normalizer = 1        * magnitude/self.coverage_max
        self.algebraic_connectivity_normalizer = 1 * magnitude/self.algebraic_connectivity_max
    
    def objective_function(self, trial):
        
        # manage positions
        pos = self.pos_fix
        nparams = 2*self.N_agents-len(self.pos_fix)
        pos_move = np.zeros(nparams)
        for i in range(nparams):
            pos_move[i] = trial.suggest_uniform('pos'+str(i), self.box_margin[2*i], self.box_margin[2*i+1])
        pos = np.hstack((pos, pos_move))
        pos_M = pos.reshape(self.N_agents, 2)
        
        # Store the constraints as user attributes so that they can be restored after optimization.
        C_RIG, C_CONN = self.constraint_function(pos_move)
        trial.set_user_attr("constraint", (C_RIG,C_CONN))    
        
        # pass to graph
        self.G = FL.generate_graph(pos_M,self.max_dist)
        
        # connectivity        
        edge_relation = FL.get_edge_relation(self.G)
        algebraic_connectivity = FL.get_algebraic_connectivity(self.G)
        
        # coverage
        coverage = FL.get_coverage(self.G, self.map_radius)                 
        
        # normalize and cost
        edge_relation = self.edge_relation_normalizer*edge_relation
        coverage = self.coverage_normalizer*coverage     
        algebraic_connectivity = self.algebraic_connectivity_normalizer*algebraic_connectivity                            
        
        cost = self.alpha*edge_relation + (1 - self.alpha)*coverage        
        cost  = self.alpha*algebraic_connectivity + (1 - self.alpha)*coverage
        
        self.J_edge.append(edge_relation)
        self.J_algebraic_connectivity.append(algebraic_connectivity)
        self.J_coverage.append(coverage)
        return cost
    
    def constraint_function(self, x):
        
        # manage positions
        pos = np.hstack((self.pos_fix, x))
        pos_M = pos.reshape(self.N_agents, 2)
        
        # pass to graph
        G_con = FL.generate_graph(pos_M,self.max_dist)
        
        # Constraints which are considered feasible if less than or equal to zero.
        # eigenvalues[3] is the rigidity value and should be greater than threshold
        eps = 0.0*self.threshold
        # isRigid, egvl = FL.is_rigid(G_con, self.threshold)                
        
        # dispersion
        dispersion = FL.get_dispersion(G_con)
        
        C_RIG = (self.threshold - eps) - dispersion 
            
        isConnected = FL.is_connected(G_con)
        if isConnected:
            C_CONN = -100
        else:
            C_CONN = 100
                
        return C_RIG, C_CONN
    
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
        self._mu = mu
        self._beta = beta          
        self._eps = 1e-12
        self._safety_dispersion = 1e-1
        self._safety_connectivity = 1e-1
                
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
        self._matrix_laplacian = self.laplacian(self._matrix_degree, self._matrix_adjacency)                
        
        # connectivity
        self._eigvals = []
        self._eigvect = []
        self._algebraic_connectivity, self._v_fiedler = self.get_connectivity(self._matrix_laplacian, init=True)
        
        # cost function
        self._objective, self._cost, self._barrier, self._constraints = self.objective()
                
    
    def gradient_step(self, x) -> np.ndarray:                
        
        # gradient
        grad = self.gradient()
        
        # step
        x = x + self._mu*grad
        
        # box constraints
        for i in range(len(x)//2):
            x[i,0] = np.clip(x[i,0], self._box_margin[2*i,0], self._box_margin[2*i+1,0])
            x[i,1] = np.clip(x[i,1], self._box_margin[2*(i+1),0], self._box_margin[2*(i+1)+1,0])        
        
        # update laplacian
        self.update_graph(x)
        
        return x
    
    def objective(self) -> float:
                
        P = self._pos                
        
        dispersion = np.linalg.trace(P.T @ self._matrix_laplacian @ P)        
        barrier_connectivity =  self._beta[1] * np.log(np.maximum(self._algebraic_connectivity - self._safety_connectivity, self._eps))
        
        cost = 1
        J = cost + barrier_connectivity
        
        return J, cost, [barrier_connectivity], [self._algebraic_connectivity]
    
    def gradient(self) -> np.ndarray:                
        
        # cycle over variables
        grad = np.zeros([self._N_agents, 2])
        grad_L = self.gradient_laplacian(self._pos, self._matrix_adjacency)
        P = self._pos
        v = self._v_fiedler                
        
        for index in range(len(self._indices_var)):
            
            if self._indices_var[index] == True:                
            
                #### GRADIENT LAMBDA2 ####
                # selector
                e = np.zeros([self._N_agents, 1])
                e[index] = 1
                
                # incidence matrix
                B = self._matrix_incidence[self._matrix_incidence[:,index] != 0, :]                
                N_edges = B.shape[0]
                
                # position difference
                DP = B @ P
                
                # distances
                Dd = self._matrix_adjacency[index, :].reshape(self._N_agents, 1)                
                Dd = Dd[Dd != 0] # Remove the elements that are zero
                Dd = Dd.reshape([len(Dd), 1])
                
                # weights            
                W = np.eye(N_edges)
                
                # fiedler differences
                vb = B @ v                
                grad_lambda2 = e.T @ B.T @ (W @ (DP / Dd) * vb**2)
                grad_distance = B.T @ self.compute_sign(B @ P)
                # grad_J = grad_distance[index,:] * self._algebraic_connectivity + self.l1_norm(B @ P) * 
                grad_cost = grad_lambda2
                
                #### GRADIENT BARRIER DISPERSION ####                
                grad_barrier_dispersion =  self._beta[0] * 2 * (self._matrix_laplacian @ P) / \
                    (np.linalg.trace(P.T @ self._matrix_laplacian @ P) - self._safety_dispersion)
                
                #### GRADIENT BARRIER CONNECTIVITY ####      
                grad_algebraic_connectivity = self.gradient_algebraic_connectivity(grad_L, index)
                grad_barrier_connectivity =   self._beta[1] * grad_algebraic_connectivity/ \
                    (np.maximum(self._algebraic_connectivity - self._safety_connectivity, self._eps))
                                 
                # gradient                
                grad[index,:] = grad_cost + grad_barrier_dispersion[index,:] + grad_barrier_connectivity
                                
        return grad
    
    def gradient_algebraic_connectivity(self, grad_L, index) -> np.ndarray:
        
        grad_L_index = 0 * grad_L
        grad_L_index[index,:,:] = grad_L[index,:,:]
        
        grad_algebraic_connectivity = np.zeros(2)
                
        eigv = self._v_fiedler.reshape(self._N_agents, 1)
        grad_algebraic_connectivity[0] = eigv.T @ grad_L[:,:,0] @ eigv
        grad_algebraic_connectivity[1] = eigv.T @ grad_L[:,:,1] @ eigv
        
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
        self._matrix_laplacian = self.laplacian(self._matrix_degree, self._matrix_adjacency)                
        
        self._algebraic_connectivity, self._v_fiedler = self.get_connectivity(self._matrix_laplacian, init=False)
        
        self._objective, self._cost, self._barrier, self._constraints = self.objective()
    
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
    
    def laplacian(self, degree, adjacency) -> np.ndarray:
        laplacian = degree - adjacency
        return laplacian
    
    def get_connectivity(self, laplacian, init=False) -> float:
        
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
            v_fiedler = laplacian @ self._v_fiedler # power iteration method
            # sigma = 1 * self._algebraic_connectivity
            # v_fiedler = np.linalg.solve(laplacian - sigma * np.eye(self._N_agents), self._v_fiedler) # inverse approximation
            v_fiedler = self._v_fiedler / np.linalg.norm(v_fiedler)
            algebraic_connectivity = float(v_fiedler.T @ laplacian @ v_fiedler / (v_fiedler.T @ v_fiedler))
        
        return algebraic_connectivity, v_fiedler
    
    def l1_norm(self, matrix):    
        return np.max(np.sum(np.abs(matrix), axis=0))
    
    def compute_sign(self, matrix):    

        # Normalize each row of BP
        norms = np.linalg.norm(matrix, axis=1, keepdims=True)  # Compute the norm of each row (m x 1)
        norms[norms == 0] = 1  # Avoid division by zero for zero-length edges

        sign = matrix / norms  # Normalize each row

        return sign
            