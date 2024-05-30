import numpy as np


def calculate_mean_and_cov(particles):
    mean = np.mean(particles, axis = 0)
    cov = np.cov(particles.T).flatten()
    return mean, cov