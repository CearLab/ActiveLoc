import os
import time
import pandas as pd

def get_exp_folder(note=''):
    if not os.path.exists('results'):
        os.makedirs('results')

    date = time.strftime("%Y%m%d")
    if not os.path.exists(os.path.join('results', date)):
        os.makedirs(f'results/{date}')
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    exp_folder_name = path = os.path.join('results', date, f'exp_{timestamp}')
    
    if note != '':
        exp_folder_name += f'_{note}'
    os.mkdir(exp_folder_name)
    frames_folder = os.path.join(exp_folder_name, 'frames')
    os.mkdir(frames_folder)
    return exp_folder_name, frames_folder

def load_data(exp_folder):
    mean_log = pd.read_csv(f'{exp_folder}/mean_log.csv', header = None)
    cov_log = pd.read_csv(f'{exp_folder}/cov_log.csv', header = None)
    errors = pd.read_csv(f'{exp_folder}/errors.csv', header = None)
    ground_truth_state = pd.read_csv(f'{exp_folder}/ground_truth_state.csv', header = None)
    ground_truth_measurement = pd.read_csv(f'{exp_folder}/ground_truth_measurement.csv', header = None)
    return mean_log, cov_log, errors, ground_truth_state, ground_truth_measurement