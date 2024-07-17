import sys
sys.path.append('.')
sys.path.append('./utils')
sys.path.append('./classes')
import unittest
import numpy as np
import networkx as nx
from dataclasses import dataclass

from utils.MatrixUtils import (
    CalcAdjacencyMatrix,
    calcDistanceMatrix,
    calcIncidenceMatrix,
    calcRigitdyMatrix,
    checkConnectivity,
    checkKconnectivity,
    isRidged
)
from utils.MapUtils import Position
from utils.AgentsUtils import AgentRole, SensorType

@dataclass
class Agent:
    id: int
    gt_pos: Position
    est_pos: Position
    team: int
    role: AgentRole
    neigh: list
    localized: bool
    moved: bool
    sensors: list

def check_measurement(sensor_type, agent_a, agent_b):
    # Dummy function to always return True for simplicity in tests
    return True

class MapContiner:
    def __init__(self):
        self.graph = nx.Graph()
        self.agents = []
        self.teams = []
        self.current_id = 0

    def add_agent(self, gt_position: Position, role: AgentRole, sensors_type, team, est_position: Position = None):
        if est_position is None:
            est_position = gt_position
        agent = Agent(id=self.current_id, gt_pos=gt_position, est_pos=est_position, team=team, role=role,
                      neigh=None, localized=False, moved=False, sensors=sensors_type)
        self.current_id += 1
        self.agents.append(agent)
        self.graph.add_node(agent.id, agent=agent)

    def update_graph(self):
        self.graph.remove_edges_from(list(self.graph.edges))
        for a in self.agents:
            for b in self.agents:
                if a.id != b.id:
                    if check_measurement(SensorType.CAM, a, b):
                        self.graph.add_edge(a.id, b.id)
                    if check_measurement(SensorType.RANGE, a, b):
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

def create_map_continer():
    map_continer = MapContiner()
    
    # Add agents
    map_continer.add_agent(Position(0, 0), AgentRole.LEADER, [SensorType.CAM], 1)
    map_continer.add_agent(Position(1, 0), AgentRole.Follower, [SensorType.CAM], 1)
    map_continer.add_agent(Position(0, 1), AgentRole.Follower, [SensorType.CAM], 1)
    
    # Update the graph with edges based on dummy check_measurement
    map_continer.update_graph()
    
    return map_continer

class TestGraphFunctions(unittest.TestCase):
    def setUp(self):
        self.map_continer = create_map_continer()
        self.graph = self.map_continer.graph

    def test_CalcAdjacencyMatrix(self):
        result = CalcAdjacencyMatrix(self.graph)
        expected = np.array([
            [0, 1, 1],
            [1, 0, 1],
            [1, 1, 0]
        ])
        np.testing.assert_array_equal(result, expected)

    def test_calcDistanceMatrix(self):
        result = calcDistanceMatrix(self.graph)
        expected = np.array([
            [0, 1, 1],
            [1, 0, 1.41421356],  # sqrt(2)
            [1, 1.41421356, 0]
        ])
        np.testing.assert_almost_equal(result, expected, decimal=5)

    def test_calcIncidenceMatrix(self):
        result = calcIncidenceMatrix(self.graph)
        expected = np.array([
            [1, 1, 0],
            [1, 0, 1],
            [0, 1, 1]
        ])
        np.testing.assert_array_equal(result, expected)

    def test_calcRigitdyMatrix(self):
        result = calcRigitdyMatrix(self.graph)
        expected = np.array([
            [1, 0, -1, 0, 0, 0],  # Edge (0, 1)
            [0, 1, 0, 0, 0, -1],  # Edge (0, 2)
            [0, 0, -1, 1, 1, -1],  # Edge (1, 2)
        ])
        np.testing.assert_array_equal(result, expected)

    def test_checkConnectivity(self):
        result = checkConnectivity(self.graph)
        self.assertTrue(result)

    def test_checkKconnectivity(self):
        result = checkKconnectivity(self.graph, 1)
        self.assertTrue(result)

        result = checkKconnectivity(self.graph, 2)
        self.assertFalse(result)

    def test_isRidged(self):
        result = isRidged(self.graph)
        self.assertFalse(result)

if __name__ == '__main__':
    unittest.main()
