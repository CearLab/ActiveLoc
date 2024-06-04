import numpy as np
from os.path import join, isdir, isfile
from os import makedirs
import pickle
import os


class pfmetry():
    pkl_file = None
    output_path = None
    
    def __init__(self, is_load_mode = False):
        #get the home dir from the environment variable
        home_dir = os.environ['HOME']
        output_path = join(home_dir, '.ros/log/latest')
        dir_name = 'pfmetry'
        self.output_path = join(output_path, dir_name)
        pkl_file_name = join(self.output_path, 'pfmetry.pkl')
        if not is_load_mode:
            if not isdir(self.output_path):
                makedirs(self.output_path)
            if isfile(pkl_file_name):
                os.remove(pkl_file_name)
            self.pkl_file = open(pkl_file_name, 'wb')
        else:
            self.pkl_file = open(pkl_file_name, 'rb')
        
            

    
    def __del__(self):
        self.pkl_file.close()
        
    def write(self, data_type, data):
        return
        dic_to_dump = {'type': data_type, 'data': data}
        pickle.dump(dic_to_dump, self.pkl_file)
        
    def read(self):
        #keep reading until EOFError and arrange the data in a dict of lists accurding to the data type
        data_dict = {}
        while True:
            try:
                data = pickle.load(self.pkl_file)
                data_type = data['type']
                if data_type not in data_dict:
                    data_dict[data_type] = []
                data_dict[data_type].append(data['data'])
            except EOFError:
                break
        
