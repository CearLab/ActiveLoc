import numpy as np
import scipy.stats as stats

__all__ = ['prop', 'calc_weights', 'resample', 'normal_model_pdf', 'sample_normal_model','single_step_particle_filter_measurement_window']

def prop(particles, command, transition_model):
    '''
    Propagates the particles using the given command and transition model.

    Parameters:
    particles (numpy.ndarray): The array of particles to be propagated.
    command: The command used for propagation.
    transition_model: The transition model used for propagation.

    Returns:
    numpy.ndarray: The new array of propagated particles.
    '''
    new_particles = np.zeros(particles.shape)
    for idx, p in enumerate(particles):
        new_particles[idx] = transition_model(p, command)
    return new_particles

def normalize(weights):
    """
    Normalize the given weights by dividing each weight by the sum of all weights.
    
    Parameters:
    weights (numpy.ndarray): An array of weights to be normalized.
    
    Returns:
    numpy.ndarray: The normalized weights.
    """
    return weights/np.sum(weights)

def calc_weights(particles, measurement, measurement_model, normalize_weights=True):
    """
    Calculate the weights of particles based on a measurement and a measurement model.

    Parameters:
    particles (numpy.ndarray): An array of particles.
    measurement (any): The measurement used to calculate the weights.
    measurement_model (function): The measurement model function that maps a particle and a measurement to a weight.
    normalize_weights (bool, optional): Flag indicating whether to normalize the weights. Default is True.

    Returns:
    numpy.ndarray: An array of weights corresponding to the particles.
    """
    weights = np.zeros(len(particles))
    for idx, p in enumerate(particles):
        weights[idx] = measurement_model(measurement, p)
    if normalize_weights:
        weights = normalize(weights)
    return weights
    
def systematic_resample(particles, weights):
    """
    Performs systematic resampling of particles based on their weights.

    Parameters:
    particles (numpy.ndarray): Array of particles.
    weights (numpy.ndarray): Array of weights corresponding to each particle.

    Returns:
    numpy.ndarray: Array of resampled particles.

    """
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
    """
    Performs stratified resampling of particles based on their weights.

    Args:
        particles (numpy.ndarray): Array of particles.
        weights (numpy.ndarray): Array of weights corresponding to each particle.

    Returns:
        numpy.ndarray: Array of resampled particles.

    """
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
    """
    Resamples particles based on their weights using the multinomial resampling algorithm.

    Parameters:
    particles (numpy.ndarray): Array of particles.
    weights (numpy.ndarray): Array of weights corresponding to each particle.

    Returns:
    numpy.ndarray: Array of resampled particles.

    """
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
    """
    Calculate the effective number of particles based on the given weights.
    
    Parameters:
    weights (numpy.ndarray): Array of particle weights.
    
    Returns:
    float: The effective number of particles.
    """
    return 1/np.sum(weights**2)

def resample(particles, weights, method, N_eff_threshold=0.5):
    """
    Resamples particles based on their weights using the specified method.

    Parameters:
    particles (list): List of particles.
    weights (list): List of weights corresponding to the particles.
    method (str): Resampling method to use. Options: 'systematic', 'stratified', 'multinomial', 'None'.
    N_eff_threshold (float): Threshold for effective particle count. Default is 0.5.

    Returns:
    list: Resampled particles.

    Raises:
    ValueError: If an invalid resample method is provided.
    """
    method = method.lower()
    N = len(particles)
    if calc_N_eff(weights) > N_eff_threshold*N:
        return particles
    if method == 'systematic':
        return systematic_resample(particles, weights)
    elif method == 'stratified':
        return stratified_resample(particles, weights)
    elif method == 'multinomial':
        return multinomial_resample(particles, weights)
    elif method == 'none':
        return particles
    else:
        raise ValueError('Invalid resample method')
    

def normal_model_pdf(x, mu, cov):
    """
    Calculate the probability density function (PDF) of a multivariate normal distribution.

    Parameters:
        x (ndarray): The input value(s) at which to evaluate the PDF.
        mu (ndarray): The mean vector of the multivariate normal distribution.
        cov (ndarray): The covariance matrix of the multivariate normal distribution.

    Returns:
        float: The PDF value at the given input value(s).
    """
    
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
    """
    Samples from a multivariate normal distribution.

    Parameters:
        mu (array_like): Mean of the distribution.
        cov (array_like): Covariance matrix of the distribution.

    Returns:
        array_like: Sampled values from the distribution.
    """
    
    # Create the multivariate normal distribution object
    rv = stats.multivariate_normal(mean=mu, cov=cov, allow_singular=True)

    # Sample from the distribution
    return rv.rvs()

def single_step_particle_filter(particles, command, measurement, transition_model, measurement_model, resample_method='systematic'):
    """
    Performs a single step of the particle filter algorithm.

    Args:
        particles (list): List of particles representing the current state estimate.
        command (object): The command or control input for the system.
        measurement (object): The measurement received from the system.
        transition_model (function): Function that models the system's transition dynamics.
        measurement_model (function): Function that models the measurement likelihood.
        resample_method (str, optional): The resampling method to use. Defaults to 'systematic'.

    Returns:
        list: Updated list of particles after performing the particle filter step.
    """
    
    particles = prop(particles, command, transition_model)
    weights = calc_weights(particles, measurement, measurement_model)
    particles = resample(particles, weights, resample_method)
    return particles

def single_step_particle_filter_measurement_window(particles, command, measurement, transition_model, measurement_model, current_step, weights, resample_method='systematic', window_size=3):
    if current_step % window_size == 0 and current_step != 0:
        particles = resample(particles, weights, resample_method)
        weights = calc_weights(particles, measurement, measurement_model)
    else:
        weights = weights*calc_weights(particles, measurement, measurement_model, normalize_weights=False)
    # print(particles.shape, weights.shape)
    particles = prop(particles, command, transition_model)
    return particles,weights

def particle_filter(particles, command, measurement, transition_model, measurement_model, resample_method='systematic'):
    #this function i a genrator that yields the particals at each step
    while True:
        yield single_step_particle_filter(particles, command, measurement, transition_model, measurement_model, resample_method)
        
