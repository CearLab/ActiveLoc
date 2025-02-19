import FrameworkLib as FL
import numpy as np

class Objective:
    def __init__(self, N_agents, max_dist, threshold, box_margin,alpha):
        self.N_agents = N_agents
        self.max_dist = max_dist
        self.threshold = threshold
        self.box_margin = box_margin
        self.G = []
        self.p0 = []
        self.p = []
        self.map_radius = 1
        self.init = True
        self.alpha = alpha

    def __call__(self, trial):
    
        pos = []
        for i in range(2*self.N_agents):
            pos.append(trial.suggest_uniform('pos'+str(i), 0, self.box_margin))
        pos_M = np.array(pos).reshape(self.N_agents, 2)        
        
        # Store the constraints as user attributes so that they can be restored after optimization.
        C_box, EGVL_rig = self.constraint_function(pos)
        trial.set_user_attr("constraint", (C_box, EGVL_rig))    
        
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
    
    def objective_function(self, x):
        
        # manage positions
        pos = np.hstack((x, self.p[len(x):]))
        pos_M = pos.reshape(self.N_agents, 2)
        
        # pass to graph
        self.G = FL.generate_graph(pos_M,self.max_dist)
        
        # get algebraic connectivity
        # algebraic_connectivity = FL.get_algebraic_connectivity(self.G)
        edge_relation = FL.get_edge_relation(self.G)
        
        spreadiness = FL.get_spreadiness(self.G, self.map_radius)
        
        # energy = FL.measure_energy(self.G)
        
        cost_S = spreadiness
        cost_E = edge_relation
        
        alpha = 0
        cost = alpha/cost_S + (1 - alpha)/cost_E
        cost = -edge_relation
        return cost
    
    def constraint_function(self, x):
        
        # manage positions
        pos = np.hstack((x, self.p[len(x):]))
        pos_M = pos.reshape(self.N_agents, 2)
        
        # pass to graph
        self.G = FL.generate_graph(pos_M,self.max_dist)
        
        # Constraints which are considered feasible if less than or equal to zero.
        # eigenvalues[3] is the rigidity value and should be greater than threshold
        R, RR, EGVL, EGVT = FL.get_rigidity_matrix(self.G)
        EGVL_rig = np.real(EGVL[3])
        C_EGVL = self.threshold - EGVL_rig
        
        # constrain inside box_margin
        C_box = -100
        for i in range(self.N_agents):
            if pos_M[i][0] < 0 or pos_M[i][0] > self.box_margin or pos_M[i][1] < 0 or pos_M[i][1] > self.box_margin:
                C_box = 100
                break            
                
                
        return C_box,C_EGVL
    