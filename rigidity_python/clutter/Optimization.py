import FrameworkLib as FL
import numpy as np

class Objective:
    def __init__(self, N_agents, max_dist, threshold, box_margin):
        self.N_agents = N_agents
        self.max_dist = max_dist
        self.threshold = threshold
        self.box_margin = box_margin
        self.G = []
        self.p0 = []
        self.p = []

    def __call__(self, trial):
    
        pos = []
        for i in range(2*self.N_agents):
            pos.append(trial.suggest_uniform('pos'+str(i), 0, self.box_margin))
        pos = np.array(pos).reshape(self.N_agents, 2)
        self.p = pos
        
        # pass to graph
        G = FL.generate_graph(pos,self.max_dist)
            
        # get edge_relation
        edge_relation = 2 * FL.get_edge_connectivity(G) * (1 - np.cos(np.pi / self.N_agents))
        
        # get algebraic connectivity
        algebraic_connectivity = FL.get_algebraic_connectivity(G)
        
        # Compute the Laplacian matrix
        L = FL.get_laplacian_matrix(G)
        # get spreadiness
        x = np.asarray([pos[i][0] for i in range(len(pos))])
        y = np.asarray([pos[i][1] for i in range(len(pos))])
        spread_x = np.sqrt(np.dot(x, np.dot(L, x.transpose())))
        spread_y = np.sqrt(np.dot(y, np.dot(L, y.transpose())))
        spreadiness_norm = np.sqrt(spread_x**2 + spread_y**2)
        
        # Constraints which are considered feasible if less than or equal to zero.
        # eigenvalues[3] is the rigidity value and should be greater than threshold
        R, RR, EGVL, EGVT = FL.get_rigidity_matrix(G)
        EGVL_rig = np.real(EGVL[3])
        C_EGVL = self.threshold - EGVL_rig
        
        # check each node is in the box
        for i in range(self.N_agents):
            if pos[i][0] < 0 or pos[i][0] > self.box_margin or pos[i][1] < 0 or pos[i][1] > self.box_margin:
                C_box = 1
            else:
                C_box = -1
                
        # Store the constraints as user attributes so that they can be restored after optimization.
        trial.set_user_attr("constraint", (C_EGVL, C_box))
        
        cost_S = 0  *   spreadiness_norm
        cost_E = 10 *   algebraic_connectivity
        
        alpha = 0
        cost = alpha * cost_S + (1 - alpha) * cost_E
        
        # return the cost function
        # return cost_E,cost_E
        return cost

    def constraints(trial):
        return trial.user_attrs["constraint"]
    
    def objective_function(self, x):
        
        # manage positions
        pos = np.hstack((x, self.p[len(x):]))
        pos_M = pos.reshape(self.N_agents, 2)
        
        # pass to graph
        self.G = FL.generate_graph(pos_M,self.max_dist)
            
        # get edge_relation
        edge_relation = 2 * FL.get_edge_connectivity(self.G) * (1 - np.cos(np.pi / self.N_agents))
        
        # get algebraic connectivity
        algebraic_connectivity = FL.get_algebraic_connectivity(self.G)
        
        # Compute the Laplacian matrix
        L = FL.get_laplacian_matrix(self.G)
        # get spreadiness
        x = pos[0::2]
        y = pos[1::2]
        spread_x = np.sqrt(np.dot(x, np.dot(L, x.transpose())))
        spread_y = np.sqrt(np.dot(y, np.dot(L, y.transpose())))
        spreadiness_norm = np.sqrt(spread_x**2 + spread_y**2)
        
        cost_S = 0  *   spreadiness_norm
        cost_E = 10 *   algebraic_connectivity
        
        alpha = 0
        cost = alpha/cost_S + (1 - alpha)/cost_E
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
            if pos[i][0] < 0 or pos[i][0] > self.box_margin or pos[i][1] < 0 or pos[i][1] > self.box_margin:
                C_box = 100
                break            
                
                
        return max(C_EGVL,C_box)
    