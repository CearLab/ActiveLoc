from MapUtils import Position
from SensorUtils import SensorType
from dataclasses import dataclass

__all__ = ['AgentRole', 'SensorType', 'Agent']

class AgentRole(enumerate):
    LEADER = 0
    Follower = 1

@dataclass
class Agent():
    id: int
    gt_pos: Position
    est_pos: Position
    team: int
    role: AgentRole
    neigh: list
    localized: bool
    moved: bool
    sensors: list[SensorType]
