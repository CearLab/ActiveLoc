__all__ = ["Position"]
import numpy as np

class Position:
    def __init__(self, x, y):
        self._coords = np.array([x, y])

    @property
    def x(self):
        return self._coords[0]

    @x.setter
    def x(self, value):
        self._coords[0] = value

    @property
    def y(self):
        return self._coords[1]

    @y.setter
    def y(self, value):
        self._coords[1] = value

    def __getitem__(self, index):
        return self._coords[index]

    def __setitem__(self, index, value):
        self._coords[index] = value
    
    def __sub__(self, other):
        return Position(self._coords[0] - other._coords[0], self._coords[1] - other._coords[1])
    
    def to_array(self):
        return self._coords

    def __repr__(self):
        return f"Position(x={self.x}, y={self.y})"