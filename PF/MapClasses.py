import numpy as np
import matplotlib.pyplot as plt

class MapObject:
    pos = None
    trans_model = None
    def __init__(self, pos0: np.array, trans_model):
        self.pos = pos0
        self.trans_model = trans_model
        
    def transition(self, command):
        self.pos = self.trans_model(self.pos, command)

    def get_pos(self):
        return self.pos
    
    def set_pos(self, pos):
        self.pos = pos

    def __str__(self):
        return f"{self.__class__.__name__} at ({self.pos[0]}, {self.pos[1]})"

class Beacon(MapObject):
    def __init__(self, pos0: np.array, trans_model):
        super().__init__(pos0, trans_model)

class Robot(MapObject):
    def __init__(self, pos0: np.array, trans_model):
        super().__init__(pos0, trans_model)
    
    def measure_distance_to(self, beacon: MapObject):
        robot_pos = self.get_pos()
        beacon_pos = beacon.get_pos()
        return np.linalg.norm(robot_pos - beacon_pos)
        
class Map:
    def __init__(self):
        self.objects = dict()
    
    def add_object(self, obj: MapObject, name: str):
        self.objects[name] = obj
    
    def get_object(self, name: str):
        return self.objects[name]
    
    def measure_distances(self, robot_name: str):
        robot = self.get_object(robot_name)
        distances = {}
        for name, obj in self.objects.items():
            if isinstance(obj, Beacon):
                distances[name] = robot.measure_distance_to(obj)
        return distances
    def transition(self, command: dict):
        for name in command:
            self.objects[name].transition(command[name])
    
    def plot(self):
        for obj in self.objects.values():
            plt.plot(obj.pos[0], obj.pos[1], 'ro')
        plt.show()

    def __str__(self):
        return f"Map with {len(self.objects)} objects"