import numpy as np
from PFlib.stateManger import *
from PFlib.ParticleFilter import sample_normal_model, single_step_particle_filter, normal_model_pdf, single_step_particle_filter_measurement_window
import rospy
import os
class GradientResamplingUtiles():
    @staticmethod
    def single_measurement_gradient_calculation(agent_position, beacon_position, measurement, sigma_measurement = 0.1): 
        _MEAS_SIZE_2D = 1
        _STATE_SIZE_2D = 2
        a = agent_position
        b = beacon_position
        z = measurement
        s = sigma_measurement
        #make sure that s is np array
        
        dx = a - b
        z_est = np.linalg.norm(dx)
        dzest_dx = np.zeros((1, _STATE_SIZE_2D))
        dzest_dx[0:_MEAS_SIZE_2D] = dx/z_est
        dLdz = np.zeros((1, 1))
        inv_cov = np.array([1/s])
        dz = np.array([z - z_est])
        dLdz = -2 * dz * inv_cov
        # rospy.logwarn(f"agent_position: {a} \n beacon_position: {b} \n measurement: {z} \n sigma_measurement: {s} \n dx: {dx} \n z_est: {z_est} \n  dzest_dx: {dzest_dx} \n dLdz: {dLdz}")
        L_of_x = normal_model_pdf(z, z_est, s**2)
        dLdx = -0.5 * L_of_x * dLdz @ dzest_dx
        normalized_dLdX = dLdx / np.linalg.norm(dLdx)
        return dLdx, normalized_dLdX
    
    @staticmethod
    def start_travel_along_gradient(x, z, agent_id, beacon_id, measurements_likelihood_function, sigma_measurement = 0.1, step_size = 0.1, number_of_step = 100, do_debug = False):
        agent_positon = x[get_agent_index(agent_id)]
        beacon_position = x[get_beacon_index(beacon_id)]
        agent_beacon_measerment = z[agent_id*NUM_OF_BEACONS + beacon_id]
        current_position = agent_positon
        prev_move_direction = None
        if do_debug:
            agent_position_log = np.zeros((number_of_step, 2))
            measurements_likelihood_log = np.zeros(number_of_step)
            move_direction_log = np.zeros((number_of_step, 2))
        local_maxima_indices = []
        maxima_positions = []
        maxima_likelihoods = []
        for i in range(number_of_step):
            _, grad_normlized = GradientResamplingUtiles.single_measurement_gradient_calculation(current_position,
                                                                                                beacon_position,
                                                                                                agent_beacon_measerment,
                                                                                                sigma_measurement
                                                                                                )
            move_direction = GradientResamplingUtiles.get_perpendicular_vector(grad_normlized)
            move_direction = GradientResamplingUtiles.set_consistent_direction(prev_move_direction, move_direction)
            prev_move_direction = move_direction
            last_position = current_position
            current_position = GradientResamplingUtiles.propagate_position_to_move_direction(current_position, move_direction, step_size)
            #update x with the new position
            x[get_agent_index(agent_id)] = current_position
            current_likelihood = GradientResamplingUtiles.get_likelihood(z, x, measurements_likelihood_function)
            #cheack if the last position was local maxima
            rospy.logwarn(f"current_position: {current_position} \n current_likelihood: {current_likelihood} \n last_position: {last_position}")
            last_likelihood = measurements_likelihood_log[i-1] if i > 0 else -np.inf
            two_last_likelihood = measurements_likelihood_log[i-2] if i > 1 else -np.inf
            if current_likelihood < last_likelihood and last_likelihood > two_last_likelihood:
                local_maxima_indices.append(i-1)
                maxima_positions.append(last_position)
                maxima_likelihoods.append(last_likelihood)
            if do_debug:
                agent_position_log[i] = current_position
                measurements_likelihood_log[i] = current_likelihood
                move_direction_log[i] = move_direction
            
        debug_data = None
        if do_debug:
            debug_data = {
                'agent_position_log': agent_position_log,
                'measurements_likelihood_log': measurements_likelihood_log,
                'move_direction_log': move_direction_log,
                'maxima_indices': local_maxima_indices,
                'maxima_positions': maxima_positions,
                'maxima_likelihoods': maxima_likelihoods
            }
        rospy.logwarn(f"maxima_positions: {maxima_positions} maxima_likelihoods: {maxima_likelihoods}")
        return maxima_positions, maxima_likelihoods, debug_data
        
    
    @staticmethod
    def get_perpendicular_vector(v):
        return np.array([v[1], -v[0]])
    
    @staticmethod
    def set_consistent_direction(v1, v2):
        if v1 is None:
            return v2
        if np.dot(v1, v2) < 0:
            return -v2
        return v2
    
    @staticmethod
    def propagate_position_to_move_direction(current_position, move_direction, step_size):
        return current_position + step_size * move_direction
    
    @staticmethod
    def get_likelihood(z, x, measurements_likelihood_function):
        return measurements_likelihood_function(z, x)
    
    @staticmethod
    def get_local_maxima(likelihood_log):
        raise NotImplementedError()
        maxima = []
        for i in range(1, len(likelihood_log)-1):
            if likelihood_log[i] > likelihood_log[i-1] and likelihood_log[i] > likelihood_log[i+1]:
                maxima.append(i)
        return np.array(maxima)
    
    @staticmethod
    def plot_gradient_travel(x, debug_data):
        raise NotImplementedError()
        agent_pos_0  = x[get_agent_index(0)]
        beacon0_pos_0 = x[get_beacon_index(0)]
        beacon1_pos_0 = x[get_beacon_index(1)]
        #plot positions
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_xlim(-2, 2)
        ax.set_ylim(-2, 2)
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
    def resampling_in_local_maxima(positions, particle, original_position):
        #split the partical of num_of_positions
        num_of_positions = len(positions)
        grouped_particles = np.array_split(particle, num_of_positions)
        new_particles = []
        for i in range(num_of_positions):
            translation = np.zeros(particle.shape[1])
            
            translation[0:2] = positions[i] - original_position
            #make sure that translation is padding with zeros according to the state size
            
            rospy.logwarn(f"translation: {translation}")
            new_particle = grouped_particles[i] + translation
            new_particles.append(new_particle)
        return np.concatenate(new_particles, axis=0)
    
    @staticmethod
    def calc_step_size_using_circle_radius(radius, step_size, inflate_factor = 1.2):
        circle_circumference = 2 * np.pi * radius
        num_of_steps = circle_circumference / step_size
        num_of_steps = int(num_of_steps * inflate_factor)
        return num_of_steps

    @staticmethod
    def main(x, z, agent_id, beacon_id, particles, measurements_likelihood_function, sigma_measurement = 0.1, step_size = 0.1, number_of_step = None, do_debug = False):
        # rospy.logwarn(f"x: {x} \n z: {z} \n agent_id: {agent_id} \n beacon_id: {beacon_id} \n sigma_measurement: {sigma_measurement} \n step_size: {step_size} \n number_of_step: {number_of_step} \n do_debug: {do_debug}")
        range = z[agent_id*NUM_OF_BEACONS + beacon_id]
        if number_of_step is None:
            number_of_step = GradientResamplingUtiles.calc_step_size_using_circle_radius(range, step_size)
        # rospy.logwarn(f"number_of_step: {number_of_step}")
        maxima_positions, maxima_indeces, debug_data = GradientResamplingUtiles.start_travel_along_gradient(x, z, agent_id, beacon_id, measurements_likelihood_function, sigma_measurement, step_size, number_of_step, do_debug)
        # rospy.logwarn(f"maxima_positions: {maxima_positions}")
        # rospy.logwarn(f"maxima_indeces: {maxima_indeces}")
        GradientResamplingUtiles.print_debugdata_to_files(debug_data)
        new_particles = GradientResamplingUtiles.resampling_in_local_maxima(maxima_positions, particles, x[get_agent_index(agent_id)])
        return new_particles
    
    @staticmethod
    def print_debugdata_to_files(debug_data):
        #log the pwd
        
        ros_log_dir = os.getenv("ROS_LOG_DIR", os.path.expanduser("~/.ros/log"))
        folder_to_save = os.path.join(ros_log_dir, "gradient_resampling")
        #create the folder if it dosent exist
        os.makedirs(folder_to_save, exist_ok=True)
        np.savetxt(f'{folder_to_save}/agent_position_log.csv', debug_data['agent_position_log'], delimiter=',')
        np.savetxt(f'{folder_to_save}/measurements_likelihood_log.csv', debug_data['measurements_likelihood_log'], delimiter=',')
        np.savetxt(f'{folder_to_save}/move_direction_log.csv', debug_data['move_direction_log'], delimiter=',')
        np.savetxt(f'{folder_to_save}/maxima_indices.csv', debug_data['maxima_indices'], delimiter=',')
        np.savetxt(f'{folder_to_save}/maxima_positions.csv', debug_data['maxima_positions'], delimiter=',')
        np.savetxt(f'{folder_to_save}/maxima_likelihoods.csv', debug_data['maxima_likelihoods'], delimiter=',')