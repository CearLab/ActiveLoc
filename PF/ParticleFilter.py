import numpy as np

class Particle:
    state = None
    weight = None
    def __init__(self, state: np.array, weight0: float):
        self.state = state
        self.weight = weight0
    
class ParticleFilter:
    particles = None
    def __init__(self, particles: list):
        self.particles = particles
    
    def propgate(self, command: dict):
        pass
    
    def 