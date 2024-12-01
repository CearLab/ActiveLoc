from stateManger import *
import numpy as np


class GradientResamplingUtiles():
    @staticmethod
    def single_measerment_gradient_calculation(agent_positon, beacon_position, measurement, sigma_measurement = 0.1, step_size = 0.1, number_of_step = 100): 
        _MEAS_SIZE_2D = 1
        _STATE_SIZE_2D = 2
        a = agent_positon
        b = beacon_position
        z = measurement
        s = sigma_measurement
        
        dx = a - b
        z_est = np.linalg.norm(dx)
        dzest_dx = np.zeros((1, _STATE_SIZE_2D))
        dzest_dx[0:_MEAS_SIZE_2D] = dx/z_est
        dLdz = np.zeros((1, 1))
        inv_cov = np.array([1/s])
        dz = np.array([z - z_est])
        dLdz = -2 * dz * inv_cov
        L_of_x = normal_model_pdf(z, z_est, s**2)
        dLdx = -0.5 * L_of_x * dLdz @ dzest_dx
        normlized_dLdX = dLdx / np.linalg.norm(dLdx)
        return dLdx, normlized_dLdX
    
    @staticmethod
    def start_travel_along_gradient(x, z, agent_id, beacon_id, sigma_measurement = 0.1, step_size = 0.1, number_of_step = 100, do_debug = False, measurements_likelihood = measurements_likelihood):
        agent_positon = x[get_agent_index(agent_id)]
        # agent_positon = np.array([-4, 0])
        beacon_position = x[get_beacon_index(beacon_id)]
        agent_beacon_measerment = z[agent_id*NUM_OF_BEACONS + beacon_id]
        current_position = agent_positon
        prev_move_direction = None
        agent_position_log = np.zeros((number_of_step, 2))
        measurements_likelihood_log = np.zeros(number_of_step)
        x_current = x.copy()
        current_likelihood = GradientResamplingUtiles.get_likelihood(z, x_current, measurements_likelihood)
        if do_debug:
            move_direction_log = np.zeros((number_of_step, 2))
        agent_position_log[0] = current_position
        measurements_likelihood_log[0] = current_likelihood
        if do_debug:
            move_direction_log[0] = np.array([0, 0])
        for i in range(1,number_of_step):
            _, grad_normlized = GradientResamplingUtiles.single_measerment_gradient_calculation(current_position,
                                                                                                beacon_position,
                                                                                                agent_beacon_measerment,
                                                                                                sigma_measurement,
                                                                                                step_size,
                                                                                                number_of_step)
            move_direction = GradientResamplingUtiles.get_prepndicular_vector(grad_normlized)
            move_direction = GradientResamplingUtiles.set_consistent_direction(prev_move_direction, move_direction)
            prev_move_direction = move_direction
            current_position = GradientResamplingUtiles.propgate_position_to_move_direction(current_position, move_direction, step_size)
            x_current[get_agent_index(agent_id)] = current_position
            current_likelihood = GradientResamplingUtiles.get_likelihood(z, x_current, measurements_likelihood)
            
            #logging the data
            agent_position_log[i] = current_position
            measurements_likelihood_log[i] = current_likelihood
            if do_debug:
                move_direction_log[i] = move_direction

        maxima_indices = GradientResamplingUtiles.get_local_maxima(measurements_likelihood_log)
        maxima_positions = agent_position_log[maxima_indices] if maxima_indices.size > 0 else agent_position_log[0]
        maxima_likelihoods = measurements_likelihood_log[maxima_indices] if maxima_indices.size > 0 else measurements_likelihood_log[0]
        
        debug_data = None
        if do_debug:
            debug_data = {
                'agent_position_log': agent_position_log,
                'measurements_likelihood_log': measurements_likelihood_log,
                'maxima_indices': maxima_indices,
                'maxima_positions': maxima_positions,
                'maxima_likelihoods': maxima_likelihoods
            }
        
        return maxima_indices, maxima_positions, maxima_likelihoods, debug_data
        
    
    @staticmethod
    def get_prepndicular_vector(v):
        return np.array([v[1], -v[0]])
    
    @staticmethod
    def set_consistent_direction(v1, v2):
        if v1 is None:
            return v2
        if np.dot(v1, v2) < 0:
            return -v2
        return v2
    
    @staticmethod
    def propgate_position_to_move_direction(current_position, move_direction, step_size):
        return current_position + step_size * move_direction
    
    @staticmethod
    def get_likelihood(z, x, measurements_likelihood_function):
        return measurements_likelihood_function(z, x)
    
    @staticmethod
    def get_local_maxima(likelihood_log):
        maxima = []
        for i in range(1, len(likelihood_log)-1):
            if likelihood_log[i] > likelihood_log[i-1] and likelihood_log[i] > likelihood_log[i+1]:
                maxima.append(i)
        return np.array(maxima)
    
    @staticmethod
    def plot_gradient_travel(x, debug_data):
        agent_pos_0  = x[get_agent_index(0)]
        beacon0_pos_0 = x[get_beacon_index(0)]
        beacon1_pos_0 = x[get_beacon_index(1)]
        #plot positions
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 10)
        ax.set_aspect('equal')

        ax.plot(agent_pos_0[0], agent_pos_0[1], 'ro')
        ax.plot(beacon0_pos_0[0], beacon0_pos_0[1], 'bo')
        ax.plot(beacon1_pos_0[0], beacon1_pos_0[1], 'bo')
        #plot the direction of the gradient
        agent_position_log = debug_data['agent_position_log']
        for i in range(len(agent_position_log)-1):
            ax.quiver(agent_position_log[i][0], agent_position_log[i][1], agent_position_log[i+1][0] - agent_position_log[i][0], agent_position_log[i+1][1] - agent_position_log[i][1], color='r')
        fig.show()
        
    @staticmethod
    def executive_decision():
        return True
    
    @staticmethod
    def resampling_in_local_maxima(positions, particle, original_position, state_idx):
        #split the partical of num_of_positions
        num_of_positions = positions.shape[0]
        grouped_particles = np.array_split(particle, num_of_positions)
        new_particles = []
        for i in range(num_of_positions):
            translation = positions[i] - original_position
            new_particle = grouped_particles[i].copy()
            new_particle[:,state_idx] = new_particle[:,state_idx] + translation
            new_particles.append(new_particle)
        return np.concatenate(new_particles, axis=0)
    
    @staticmethod
    def main(x, z, agent_id, beacon_id, particales, sigma_measurement = 0.1, step_size = 0.1, number_of_step = 100, do_debug = False):
        debug_data = None
        if GradientResamplingUtiles.executive_decision():
            _, maxima_positions,_, debug_data = GradientResamplingUtiles.start_travel_along_gradient(x, z, agent_id, beacon_id, sigma_measurement, step_size, number_of_step, do_debug)
            new_particles = GradientResamplingUtiles.resampling_in_local_maxima(maxima_positions, particales, x[get_agent_index(agent_id)], state_idx = get_agent_index(agent_id))
        return new_particles, debug_data