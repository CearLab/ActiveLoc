import os
import time


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