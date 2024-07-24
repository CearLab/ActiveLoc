from MapUtils import Position
import numpy as np

CAM_MAX_RANGE = 5
CAM_FOV = 360

RANER_MAX_RANGE = 10
RANGE_FOV = 360

__all__ = ['SensorType' , 'check_measurement']

class SensorType(enumerate):
    CAM = 0
    RANGE = 1

def check_measurement(sensor_type, agent, target):
    if sensor_type == SensorType.CAM:
        return check_cam(agent, target)
    elif sensor_type == SensorType.RANGE:
        return check_range(agent, target)
    else:
        return False

def check_cam(agent, target):
    #check if tha agent has a camera sensor
    if SensorType.CAM not in agent.sensors:
        return False
    if np.linalg.norm(agent.gt_pos.to_array() - target.gt_pos.to_array()) <= CAM_MAX_RANGE:
        return True
    else:
        return False

def check_range(agent, target):
    # check if both agents and target as range sensors
    if SensorType.RANGE not in agent.sensors or SensorType.RANGE not in target.sensors:
        return False
    if np.linalg.norm(agent.gt_pos.to_array() - target.gt_pos.to_array()) <= RANER_MAX_RANGE:
        return True
    else:
        return False