import numpy as np
import scipy.stats as stats

__all__ = ['prop', 'calc_weights', 'resample', 'normal_model_pdf', 'sample_normal_model']

def prop(particles, command, transition_model):
    new_particles = np.zeros(particles.shape)
    for idx,p in enumerate(particles):
        new_particles[idx] = transition_model(p, command)
    return new_particles

def normalize(weights):
    return weights/np.sum(weights)

def calc_weights(particles, measurement, measurement_model):
    weights = np.zeros(len(particles))
    for idx, p in enumerate(particles):
        weights[idx] = measurement_model(measurement,p)
    weights = normalize(weights)
    return weights
    
def systematic_resample(particles, weights):
    N = len(particles)
    weights_cumsum = np.cumsum(weights)
    new_particles = np.zeros(particles.shape)
    n = 0
    m = 0
    u_0 = np.random.uniform(0, 1/N)
    while n < N:
        u = u_0 + n/N
        while weights_cumsum[m] < u:
            m += 1
        new_particles[n] = particles[m]
        n += 1
    return new_particles

def stratified_resample(particles, weights):
    N = len(particles)
    weights_cumsum = np.cumsum(weights)
    new_particles = np.zeros(particles.shape)
    m = 0
    n = 0
    while n < N:
        u_0 = np.random.uniform(0, 1/N)
        u = u_0 + n/N
        while weights_cumsum[m] < u:
            m += 1
        new_particles[n] = particles[m]
        n += 1
    return new_particles

def multinomial_resample(particles, weights):
    N = len(particles)
    new_particles = np.zeros(particles.shape)
    weights_cumsum = np.cumsum(weights)
    m = 0
    n = 0
    while n < N:
        u = np.random.uniform(0, 1)
        m = 0
        while u > weights_cumsum[m]:
            m += 1
        new_particles[n] = particles[m]
        n += 1
    return new_particles

def calc_N_eff(weights):
    return 1/np.sum(weights**2)

def resample(particles, weights, method, N_eff_threshold=0.5):
    N = len(particles)
    if calc_N_eff(weights) > N_eff_threshold*N:
        return particles
    if method == 'systematic':
        return systematic_resample(particles, weights)
    elif method == 'stratified':
        return stratified_resample(particles, weights)
    elif method == 'multinomial':
        return multinomial_resample(particles, weights)
    else:
        raise ValueError('Invalid resample method')
    

def normal_model_pdf(x, mu, cov):
    # Check that x and mu have the same dimensions
    if x.shape != mu.shape:
        raise ValueError(f"x and mu must have the same shape, but got x.shape={x.shape} and mu.shape={mu.shape}")

    # Check that covariance matrix is square and its dimensions match those of mu
    if cov.shape[0] != cov.shape[1] or cov.shape[0] != mu.shape[0]:
        raise ValueError(f"Covariance matrix should be square and its dimensions should match those of mu. Got cov.shape={cov.shape} and mu.shape={mu.shape}")

    # Check that covariance matrix is symmetric
    if not np.allclose(cov, cov.T):
        raise ValueError("Covariance matrix is not symmetric")

    # Create the multivariate normal distribution object
    rv = stats.multivariate_normal(mean=mu, cov=cov)

    # Calculate and return the PDF value
    return rv.pdf(x)

def sample_normal_model(mu, cov):
    # Create the multivariate normal distribution object
    rv = stats.multivariate_normal(mean=mu, cov=cov)

    # Sample from the distribution
    return rv.rvs()

def single_step_particle_filter(particles, command, measurement, transition_model, measurement_model, resample_method='systematic'):
    particles = prop(particles, command, transition_model)
    weights = calc_weights(particles, measurement, measurement_model)
    particles = resample(particles, weights, resample_method)
    return particles

def particle_filter(particles, command, measurement, transition_model, measurement_model, resample_method='systematic'):
    #this function i a genrator that yields the particals at each step
    while True:
        yield single_step_particle_filter(particles, command, measurement, transition_model, measurement_model, resample_method)
        
