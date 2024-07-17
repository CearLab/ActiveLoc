#add utils folder to path
import sys
sys.path.append('.')
sys.path.append('./utils')
sys.path.append('./classes')
import unittest
from GraphManger import MapContiner
from utils.AgentsUtils import AgentRole
from utils.SensorUtils import SensorType
import unittest
from unittest.mock import patch
from utils.MapUtils import Position
from utils.AgentsUtils import Agent, AgentRole, SensorType
from utils.SensorUtils import check_measurement
import networkx as nx

class TestMapContiner(unittest.TestCase):

    def setUp(self):
        self.map_container = MapContiner()
        self.position1 = Position(x=0, y=0)
        self.position2 = Position(x=1, y=1)
        self.position3 = Position(x=3, y=3)
        self.position4 = Position(x=1, y=10)
        self.position5 = Position(x=60, y=60)
        self.role = AgentRole.Follower
        self.sensors = [SensorType.CAM, SensorType.RANGE]
        self.team = 'Alpha'
    
    def test_initialization(self):
        self.assertIsInstance(self.map_container.graph, nx.Graph)
        self.assertEqual(self.map_container.agents, [])
        self.assertEqual(self.map_container.teams, [])
        self.assertEqual(self.map_container.current_id, 0)
    
    def test_add_agent(self):
        self.map_container.add_agent(self.position1, self.role, self.sensors, self.team)
        self.assertEqual(len(self.map_container.agents), 1)
        self.assertEqual(self.map_container.graph.number_of_nodes(), 1)
        agent = self.map_container.agents[0]
        self.assertEqual(agent.gt_pos, self.position1)
        self.assertEqual(agent.est_pos, self.position1)
        self.assertEqual(agent.team, self.team)
    
    @patch('utils.SensorUtils.check_measurement', return_value=True)
    def test_update_graph(self, mock_check_measurement):
        self.map_container.add_agent(self.position1, self.role, self.sensors, self.team)
        self.map_container.add_agent(self.position2, self.role, self.sensors, self.team)
        self.map_container.add_agent(self.position3, self.role, self.sensors, self.team)
        self.map_container.add_agent(self.position4, self.role, self.sensors, self.team)
        self.map_container.add_agent(self.position5, self.role, self.sensors, self.team)
        self.map_container.update_graph()
        self.assertEqual(self.map_container.graph.number_of_edges(), 5)
    def test_move_agent(self):
        self.map_container.add_agent(self.position1, self.role, self.sensors, self.team)
        new_position = Position(x=2, y=2)
        agent_id = self.map_container.agents[0].id
        self.map_container.move_agent(agent_id, new_position)
        self.assertEqual(self.map_container.get_agent(agent_id).gt_pos, new_position)
        self.assertTrue(self.map_container.get_agent(agent_id).moved)
    
    def test_update_est_position(self):
        self.map_container.add_agent(self.position1, self.role, self.sensors, self.team)
        new_position = Position(x=2, y=2)
        agent_id = self.map_container.agents[0].id
        self.map_container.update_est_position(agent_id, new_position)
        self.assertEqual(self.map_container.get_agent(agent_id).est_pos, new_position)
        self.assertTrue(self.map_container.get_agent(agent_id).localized)
    
    def test_get_all_agents_in_team(self):
        self.map_container.add_agent(self.position1, self.role, self.sensors, self.team)
        self.map_container.add_agent(self.position2, self.role, self.sensors, 'Bravo')
        agents_in_alpha = self.map_container.get_all_agents_in_team('Alpha')
        self.assertEqual(len(agents_in_alpha), 1)
        self.assertEqual(agents_in_alpha[0].team, 'Alpha')
    
    def test_get_agent(self):
        self.map_container.add_agent(self.position1, self.role, self.sensors, self.team)
        agent_id = self.map_container.agents[0].id
        agent = self.map_container.get_agent(agent_id)
        self.assertEqual(agent.gt_pos, self.position1)
        self.assertEqual(agent.team, self.team)

if __name__ == '__main__':
    unittest.main()
