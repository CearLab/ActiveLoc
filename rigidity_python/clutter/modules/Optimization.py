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
    