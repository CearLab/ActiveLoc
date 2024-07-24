import networkx as nx
from utils.MapUtils import Position
from utils.AgentsUtils import Agent, AgentRole, SensorType
from utils.SensorUtils import check_measurement


class MapContiner():
    graph = None
    agents = []
    teams = []
    
    def __init__(self):
        self.graph = nx.Graph()
        self.agents = []
        self.teams = []
        self.current_id = 0
        
    def add_agent(self, gt_position: Position, role: AgentRole, sensors_type, team, est_position:Position = None):

            # default value for est_position is gt_position
            if est_position is None:
                est_position = gt_position
            if team not in self.teams:
                self.teams.append(team)
            #initialize agent
            agent = Agent(id = self.current_id,
                        gt_pos = gt_position,
                        est_pos= est_position,
                        team = team,
                        role = role,
                        neigh= None,
                        localized = False,
                        moved= False,
                        sensors = sensors_type)
            self.current_id += 1
            #set agent to the graph and to the list of agents
            self.agents.append(agent)
            self.graph.add_node(agent.id, agent = agent)
        
    def update_graph(self):
        # check if the measeurement is valid and build the vetexes between the agents
        # clean all the edges:
        self.graph.remove_edges_from(list(self.graph.edges))
        for a in self.agents:
            for b in self.agents:
                if a.id != b.id:
                    if check_measurement(SensorType.CAM, a, b):
                        self.graph.add_edge(a.id, b.id)
                    if check_measurement(SensorType.RANGE, a, b):
                        #check if there is already an edge between the two agents
                        if not self.graph.has_edge(a.id, b.id):
                            self.graph.add_edge(a.id, b.id)
                            
    def move_agent(self, id, new_position):
        self.graph.nodes[id]['agent'].gt_pos = new_position
        self.graph.nodes[id]['agent'].moved = True
    
    def update_est_position(self, id, new_position):
        self.graph.nodes[id]['agent'].est_pos = new_position
        self.graph.nodes[id]['agent'].localized = True
    
    def get_all_agents_in_team(self, team):
        return [agent for agent in self.agents if agent.team == team]

    def get_agent(self, id):
        return self.graph.nodes[id]['agent']
        