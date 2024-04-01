import os
import numpy as np

calib_file_extension = '.txt'

def Handler(calib_path: str):
    if os.path.exists(calib_path) == False: return None
    
    calib = {}
    
    with open(calib_path) as f:
        for line in f.readlines():
            line = line.strip()
            if len(line) == 0: continue
            k, v = line.split(':')
            calib[k] = np.array([float(x) for x in v.split()], dtype=np.float32)

    calib['P2'] = calib['P2'].reshape(3, 4) # 3x4
    
    calib['R0_rect'] = calib['R0_rect'].reshape(3, 3) # 3x3
    calib['R0_rect'] = np.pad(calib['R0_rect'], ((0, 1), (0, 1)), mode='constant', constant_values=0) # 4x4
    calib['R0_rect'][3, 3] = 1
    
    calib['Tr_velo_to_cam'] = calib['Tr_velo_to_cam'].reshape(3, 4) # 3x4
    calib['Tr_velo_to_cam'] = np.vstack((calib['Tr_velo_to_cam'], np.array([0,0,0,1], dtype=np.float32))) # 4x4

    return calib