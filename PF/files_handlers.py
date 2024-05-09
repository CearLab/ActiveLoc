import numpy as np
import os
import sys
import time

__all__ = ['load_params', 'load_ground_truth', 'save_experiment_setup']

def load_params(filename):
    params = {}
    with open(filename, 'r') as f:
        for line in f:
            key, value = line.strip().split(": ", 1)
            # Handle special cases
            if key == "beacons_pos":
                # Convert list-like string to a Python list
                value = eval(value)
            elif key in {"n_particles", "n_steps"}:
                value = int(value)
            elif key in {"sigma_transition", "sigma_measurement", "stepsize"}:
                value = float(value)
            params[key] = value
    return params

def load_ground_truth(folde_path):
    x_path = os.path.join(folde_path, 'ground_truth.csv')
    z_path = os.path.join(folde_path, 'measurements.csv')
    x = np.genfromtxt(x_path, delimiter=',', skip_header=1, invalid_raise=False)
    z = np.genfromtxt(z_path, delimiter=',', skip_header=1, invalid_raise=False)
    return x, z

def save_experiment_setup(n_particles, n_steps, resample_method, note, beacons_pos, sigma_transition, sigma_measurement, stepsize, x, z):
    if not os.path.exists('results'):
        os.makedirs('results')

    date = time.strftime("%Y%m%d")
    if not os.path.exists(f'results/{date}'):
        os.makedirs(f'results/{date}')
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    exp_folder_name = f'results/{date}/exp_{timestamp}_p_{n_particles}_s_{n_steps}_rsm_{resample_method}'
    if note != '':
        exp_folder_name += f'_{note}'
    os.mkdir(exp_folder_name)
    #create a file to store all parameters:
    with open(f'{exp_folder_name}/params.txt', 'w') as f:
        f.write(f"n_particles: {n_particles}\n")
        f.write(f"n_steps: {n_steps}\n")
        f.write(f"resample_method: {resample_method}\n")
        f.write(f"note: {note}\n")
        f.write(f"beacons_pos: {beacons_pos}\n")
        f.write(f"sigma_transition: {sigma_transition}\n")
        f.write(f"sigma_measurement: {sigma_measurement}\n")
        f.write(f"stepsize: {stepsize}\n")
        f.write(f"timestamp: {timestamp}\n")    
    #save grunds truth
    np.savetxt(f'{exp_folder_name}/ground_truth.csv', x, delimiter=',',header='x,y')
    #save measurements
    np.savetxt(f'{exp_folder_name}/measurements.csv', z, delimiter=',',header='b0,b1,b2,b3')
    return exp_folder_name