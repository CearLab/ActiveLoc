import FrameworkLib as FL
import numpy as np

class Objective:
    def __init__(self, N_agents, max_dist, threshold, box_margin,alpha):
        self.N_agents = N_agents
        self.max_dist = max_dist
        self.threshold = threshold        
        if len(box_margin) == 1:       
            self.box_margin = np.ones((N_agents,2))
            self.box_margin[:,0] = 0
            self.box_margin[:,1] = box_margin
        else:
            self.box_margin = box_margin
        self.G = []
        self.pos_fix = []
        self.map_radius = 1
        self.init = True
        self.alpha = alpha

    def __call__(self, trial):
    
        pos = []
        for i in range(2*self.N_agents):
            pos.append(trial.suggest_uniform('pos'+str(i), 0, self.box_margin))
        pos_M = np.array(pos).reshape(self.N_agents, 2)        
        
        # Store the constraints as user attributes so that they can be restored after optimization.
        EGVL_rig = self.constraint_function(pos)
        trial.set_user_attr("constraint", (EGVL_rig,EGVL_rig))    
        
        # pass to graph
        self.G = FL.generate_graph(pos_M,self.max_dist)                    
        
        # get algebraic connectivity
        # algebraic_connectivity = FL.get_algebraic_connectivity(self.G)
        edge_relation = FL.get_edge_relation(self.G)        
        coverage = FL.get_coverage(self.G, self.map_radius)                                  
        cost = self.alpha*edge_relation + (1 - self.alpha)*coverage        
        return cost

    def constraints(trial):
        return trial.user_attrs["constraint"]
    
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
        EGVL_rig = self.constraint_function(pos_move)
        trial.set_user_attr("constraint", (EGVL_rig,EGVL_rig))    
        
        # pass to graph
        self.G = FL.generate_graph(pos_M,self.max_dist)
        
        edge_relation = FL.get_edge_relation(self.G)        
        coverage = FL.get_coverage(self.G, self.map_radius)                                  
        cost = self.alpha*edge_relation + (1 - self.alpha)*coverage        
        return cost
    
    def constraint_function(self, x):
        
        # manage positions
        pos = np.hstack((self.pos_fix, x))
        pos_M = pos.reshape(self.N_agents, 2)
        
        # pass to graph
        self.G = FL.generate_graph(pos_M,self.max_dist)
        
        # Constraints which are considered feasible if less than or equal to zero.
        # eigenvalues[3] is the rigidity value and should be greater than threshold
        R, RR, EGVL, EGVT = FL.get_rigidity_matrix(self.G)
        EGVL_rig = np.real(EGVL[3])
        C_EGVL = self.threshold - EGVL_rig
        if np.isnan(C_EGVL):
            C_EGVL = 100  # Assign a high value to indicate infeasibility         
                
        return C_EGVL
    