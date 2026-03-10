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
        self.J = []
        self.C_CONNECTIVITY = []
        self.C_COVERAGE = []
        self.C_DISPERSION = []        
        self.update_normalizers()

    def __call__(self, trial):
        return self.objective_function(trial)                

    def constraints(trial):
        return trial.user_attrs["constraint"]
    
    def update_normalizers(self):
        # get edge_relation and coverage max
        self.coverage_max = self.N_agents*np.pi*self.map_radius**2
        
        # if edge_relation_core
        # self.edge_relation_max = self.N_agents * (self.N_agents - 1) * self.max_dist * 2*(self.N_agents-1)*(1-np.cos(np.pi/self.N_agents))
        # self.edge_relation_max = 2*(self.N_agents-1)*(1-np.cos(np.pi/self.N_agents))                        
        self.edge_relation_max = self.N_agents
                
        self.edge_relation_normalizer = 1   / self.edge_relation_max
        self.coverage_normalizer = 1        / self.coverage_max        
    
    def objective_function(self, trial):
        
        # manage positions
        pos = self.pos_fix
        nparams = 2*self.N_agents-len(self.pos_fix)
        pos_move = np.zeros(nparams)
        for i in range(nparams):
            pos_move[i] = trial.suggest_uniform('pos'+str(i), self.box_margin[2*i], self.box_margin[2*i+1])
        pos = np.hstack((pos, pos_move))
        pos_M = pos.reshape(self.N_agents, 2)
        
        # pass to graph
        G_con = FL.generate_graph(pos_M,self.max_dist)
        
        # connectivity        
        edge_relation = FL.get_edge_relation(G_con)
        edge_relation = self.edge_relation_normalizer*edge_relation
        self.C_CONNECTIVITY.append(edge_relation)               
        
        # coverage
        coverage = FL.get_coverage(G_con, self.map_radius)         
        coverage = coverage*self.coverage_normalizer         
        self.C_COVERAGE.append(coverage)                                
        
        # 1. Calculate your alpha-driven dispersion (the "mission style")
        dispersion = self.alpha * coverage + (1 - self.alpha) * edge_relation
        self.C_DISPERSION.append(dispersion)

        # 2. Calculate the "Violation" (how far outside the bounds we are)
        low_violation = -np.log(self.threshold[0] - dispersion) if dispersion <= self.threshold[0] else 0
        up_violation = -np.log(dispersion - self.threshold[1]) if dispersion >= self.threshold[1] else 0
        soft_cost = dispersion
        barrier_cost = low_violation + up_violation
        
        # Cost: placeholder
        # center_val = float(self.box_margin[1]) / 2
        # center = np.array([center_val, center_val])
        # cost_particular = (np.sum(np.linalg.norm(pos_M - center, axis=1)))/(self.N_agents*center_val*np.sqrt(2))
        
        # Cost: agent 1 close to the origing and agent 2 close to the edge of the box
        cost_A0 = np.linalg.norm(pos_M[0,:] - [0, 0])
        cost_A1 = np.linalg.norm(pos_M[1,:] - [self.box_margin[1], self.box_margin[1]])
        cost_particular = (cost_A0 + cost_A1)/(float(2*np.sqrt(2)*self.box_margin[1]))
                
        cost = cost_particular - soft_cost + barrier_cost
        self.J.append([cost_particular, soft_cost, barrier_cost])
        
        # Store the constraints as user attributes so that they can be restored after optimization.
        C_CONN = self.constraint_function(pos_move)
        trial.set_user_attr("constraint", (C_CONN))            
        
        return cost
    
    def constraint_function(self, x):
        
        # manage positions
        pos = np.hstack((self.pos_fix, x))
        pos_M = pos.reshape(self.N_agents, 2)
        
        # pass to graph
        G_con = FL.generate_graph(pos_M,self.max_dist)
                        
        # Constraints which are considered feasible if less than or equal to zero.        
        eps = 1e-3                                     
        isConnected = FL.is_connected(G_con)
        if isConnected:
            C_CONN = -100
        else:
            C_CONN = 100
                
        return [C_CONN]
    