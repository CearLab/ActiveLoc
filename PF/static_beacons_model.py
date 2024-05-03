# model template
from ParticleFilter import sample_normal_model, normal_model_p, particle_filter, single_step_particle_filter
import numpy as np

__all__ = ['agent_transition_model_template', 'becon_transition_model_template', 'agent_measurement_model_template', 'measurement_likelihood_template', 'generate_agent_data']
def agent_transition_model_template(x, u, cov):
    return x + sample_normal_model(u, cov)

def becon_transition_model_template(beacons, u, cov):
    n_becons = beacons.shape[0]
    new_becons = np.zeros((n_becons, 2))
    for i in range(n_becons):
        new_becons[i] = beacons[i] + sample_normal_model(u, cov)
    return new_becons

def agent_measurement_model_template(x, beacons, sigma):
    n_becons = beacons.shape[0]
    n = x.shape[0]
    z = np.zeros(n_becons)
    for i in range(n_becons):
        z[i] = np.linalg.norm(x - beacons[i]) + sample_normal_model(0, sigma*sigma)
    return z

def measurement_likelihood_template(z, x, beacons, cov):
    return normal_model_p(z, np.linalg.norm(x - beacons, axis=1), cov)

def generate_agent_data(n_steps, stepsize, x0, agent_measurement_model, agent_transition_model):
    u_agent = (np.random.rand(n_steps, 2) - 0.5)*stepsize
    x = np.zeros((n_steps, 2))
    xi = x0
    z = np.zeros((n_steps, 4))
    for i in range(0, n_steps):
        x[i] = xi
        z[i] = agent_measurement_model(xi)
        xi = agent_transition_model(xi, u_agent[i])
    return x, z, u_agent