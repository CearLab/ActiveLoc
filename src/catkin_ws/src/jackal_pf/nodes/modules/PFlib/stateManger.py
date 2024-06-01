import numpy as np
from PFlib import ParticleFilter as PF

NUM_OF_BEACONS = 4
NUM_OF_AGENTS = 1
STATE_SIZE_2D = 2
SINGLE_RANGE_MEASUREMENT_SIZE = 1
RANGE_MEASUREMENT_SIZE = NUM_OF_BEACONS * SINGLE_RANGE_MEASUREMENT_SIZE * NUM_OF_AGENTS
TOTAL_STATE_SIZE = NUM_OF_AGENTS * STATE_SIZE_2D + NUM_OF_BEACONS * STATE_SIZE_2D
SIGMA_TRANSITION_AGENT = 0.5
SIGMA_TRANSITION_BEACON = 0
SIGMA_MEASUREMENT = 0.1

get_agent_index = lambda i: slice(i*STATE_SIZE_2D, i*STATE_SIZE_2D + STATE_SIZE_2D)
get_beacon_index = lambda i: slice(STATE_SIZE_2D*NUM_OF_AGENTS + i*STATE_SIZE_2D, STATE_SIZE_2D*NUM_OF_AGENTS + i*STATE_SIZE_2D + STATE_SIZE_2D)
get_agent_position = lambda x, i: x[get_agent_index(i)]
get_beacon_postion = lambda x, j: x[get_beacon_index(j)]

def state_to_agent_and_beacons_pos(x):
    """
    Converts the state vector 'x' into arrays of agent positions and beacon positions.

    Args:
        x (numpy.ndarray): The state vector.

    Returns:
        tuple: A tuple containing two numpy arrays - agents_pos and beacons_pos.
               agents_pos: A 2D numpy array of shape (NUM_OF_AGENTS, STATE_SIZE_2D) containing agent positions.
               beacons_pos: A 2D numpy array of shape (NUM_OF_BEACONS, STATE_SIZE_2D) containing beacon positions.
    """
    agents_pos = np.zeros((NUM_OF_AGENTS, STATE_SIZE_2D))
    beacons_pos = np.zeros((NUM_OF_BEACONS, STATE_SIZE_2D))
    for i in range(NUM_OF_AGENTS):
        agents_pos[i] = get_agent_position(x, i)
    for j in range(NUM_OF_BEACONS):
        beacons_pos[j] = get_beacon_postion(x, j)
    return agents_pos, beacons_pos

def agent_and_beacons_pos_to_state(agents_pos: np.ndarray, beacons_pos: np.ndarray):
    """
    Converts the positions of agents and beacons into a state vector.

    Args:
        agents_pos (np.ndarray): List of positions of agents.
        beacons_pos (np.ndarray): List of positions of beacons.

    Returns:
        numpy.ndarray: State vector representing the positions of agents and beacons.
    """
    x = np.zeros(TOTAL_STATE_SIZE)
    for i in range(NUM_OF_AGENTS):
        x[get_agent_index(i)] = agents_pos[i]
    for j in range(NUM_OF_BEACONS):
        x[get_beacon_index(j)] = beacons_pos[j]
    return x

def calculate_true_range_meas(x):
    """
    Function to calculate the true range measurements.
    It calculates the Euclidean distance between each agent and each beacon.
    """
    z = np.zeros(RANGE_MEASUREMENT_SIZE)
    for i in range(NUM_OF_AGENTS):
        current_agent_position = x[get_agent_index(i)]
        for j in range(NUM_OF_BEACONS):
            current_beacon_position = x[get_beacon_index(j)]
            z[i*NUM_OF_BEACONS + j] = np.linalg.norm(current_agent_position - current_beacon_position)
    return z

def measurements_model(x, cov):
    '''
    Function to generate the range measurements model.
    It adds a normally distributed noise to the true range measurements.
    '''
    return calculate_true_range_meas(x) + PF.sample_normal_model(np.zeros(RANGE_MEASUREMENT_SIZE), cov)
