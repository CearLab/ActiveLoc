import modules.FrameworkLib as FL
import numpy as np

class Objective:
    def __init__(
        self,
        N_agents,
        max_dist,
        box_margin,
        alpha,
        cost_weights=[1, 1],
        local_goal_agent_ids=None,
        local_goal_positions=None,
    ):
        
        self.N_agents = N_agents
        self.max_dist = max_dist        
        self.alpha = alpha 
        self.leader_ID = N_agents-1 # the last agent is the leader (see objectuve_function)
        self.cost_weights = cost_weights
        # Global IDs for each local row in pos_M. By default local IDs are used.
        self.agent_ids = np.arange(N_agents, dtype=int)
        # If set, only this moving agent contributes to corner-anchor local cost.
        self.moving_agent_id = None
        self.moving_local_index = None
        self.map_radius = 1
        self.init = True
        self.local_goal_agent_ids = np.array([], dtype=int)
        self.local_goal_positions = np.zeros((0, 2), dtype=float)
                
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
        self.set_local_goals(local_goal_agent_ids, local_goal_positions)
        self.update_normalizers()

    def _default_local_goals(self):
        # Default behavior: no local goals for no agents.
        ids = np.array([], dtype=int)
        pos = np.zeros((0, 2), dtype=float)
        return ids, pos

    def set_local_goals(self, local_goal_agent_ids=None, local_goal_positions=None):
        if local_goal_agent_ids is None or local_goal_positions is None:
            ids, pos = self._default_local_goals()
        else:
            ids = np.asarray(local_goal_agent_ids, dtype=int).reshape(-1)
            pos = np.asarray(local_goal_positions, dtype=float)
            if pos.ndim != 2 or pos.shape[1] != 2:
                raise ValueError("local_goal_positions must have shape (K, 2)")
            if len(ids) != len(pos):
                raise ValueError("local_goal_agent_ids and local_goal_positions must have the same length")

        self.local_goal_agent_ids = ids
        self.local_goal_positions = pos

    def get_local_goal_agent_ids(self):
        return self.local_goal_agent_ids.copy()

    def get_local_goal_positions(self):
        return self.local_goal_positions.copy()

    def set_agent_ids(self, agent_ids):
        agent_ids = np.asarray(agent_ids, dtype=int).reshape(-1)
        if len(agent_ids) != self.N_agents:
            raise ValueError(f"agent_ids length ({len(agent_ids)}) must match N_agents ({self.N_agents})")
        self.agent_ids = agent_ids

    def set_moving_agent(self, moving_agent_id, moving_local_index=None):
        self.moving_agent_id = int(moving_agent_id)
        if moving_local_index is None:
            # In decentralized local optimization, the moving node is appended last.
            self.moving_local_index = self.N_agents - 1
        else:
            self.moving_local_index = int(moving_local_index)

    def clear_moving_agent(self):
        self.moving_agent_id = None
        self.moving_local_index = None

    def __call__(self, trial):
        return self.objective_function(trial)                

    def constraints(trial):
        return trial.user_attrs["constraint"]
    
    def update_normalizers(self):
        # get edge_relation and coverage max                
        self.coverage_max = 1                
        self.edge_relation_max = 1
                
        self.edge_relation_normalizer = 1   / self.edge_relation_max
        self.coverage_normalizer = 1        / self.coverage_max

    def get_anchors(self):
        """Backward-compatible alias for local goal positions."""
        return self.get_local_goal_positions()

    def get_anchor_weights(self):
        """Backward-compatible binary weights vector over global agent IDs."""
        weights = np.zeros(self.N_agents, dtype=float)
        valid_ids = self.local_goal_agent_ids[(self.local_goal_agent_ids >= 0) & (self.local_goal_agent_ids < self.N_agents)]
        weights[valid_ids] = 1.0
        return weights

    def _corner_anchor_cost(self, pos_M):
        bm = float(self.box_margin[1]) if self.box_margin.ndim == 1 else float(self.box_margin[1, 0])
        shift = 2
        goal_pos_by_id = {int(gid): self.local_goal_positions[i] for i, gid in enumerate(self.local_goal_agent_ids)}
        if bm > 2 * shift:
            weighted_cost = 0.0
            active_weight = 0.0

            # Strict local mode: only the moving agent can contribute.
            if self.moving_agent_id is None:
                return 0.0

            local_idx = self.moving_local_index if self.moving_local_index is not None else (self.N_agents - 1)
            if 0 <= local_idx < len(pos_M):
                gid = int(self.moving_agent_id)

                if gid in goal_pos_by_id:
                    target_pos = goal_pos_by_id[gid]
                    dist = np.linalg.norm(pos_M[local_idx, :] - target_pos)
                    weighted_cost += dist
                    active_weight += 1.0

            normalizer = max(active_weight, 1.0) * np.sqrt(2) * (bm - 2 * shift)
            return - (weighted_cost / normalizer)
        return 0.0

    def compute_terms_from_state(self, pos_M, G_con=None, alpha=None):
        if G_con is None:
            G_con = FL.generate_graph(pos_M, self.max_dist)

        edge_relation = self.edge_relation_normalizer * FL.get_edge_relation(G_con)
        coverage = self.coverage_normalizer * FL.get_coverage(G_con, self.max_dist)
        alpha_eff = self.alpha if alpha is None else alpha
        soft_cost = alpha_eff * coverage + (1 - alpha_eff) * edge_relation
        # If the particular term is disabled by weight, skip it entirely.
        if len(self.cost_weights) > 0 and self.cost_weights[0] == 0:
            cost_particular = 0.0
        else:
            cost_particular = self._corner_anchor_cost(pos_M)
        return cost_particular, soft_cost, edge_relation, coverage
    
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

        cost_particular, soft_cost, edge_relation, coverage = self.compute_terms_from_state(pos_M, G_con=G_con)
        self.C_CONNECTIVITY.append(edge_relation)
        self.C_COVERAGE.append(coverage)
        self.C_DISPERSION.append(soft_cost)
                
        cost = self.cost_weights[0] * cost_particular + self.cost_weights[1] * soft_cost
        self.J.append([cost_particular, soft_cost])
        
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
    