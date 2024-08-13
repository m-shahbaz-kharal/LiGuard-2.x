import os
import numpy as np

calib_file_extension = '.txt'

def Handler(calib_path: str):
    """
    Loads calibration data from the specified file path and returns it as a dictionary.

    Args:
        calib_path (str): The path to the calibration file.

    Returns:
        dict: A dictionary containing the calibration data.
    """
    # Check if the calibration file exists
    if os.path.exists(calib_path) == False: return None
    
    calib = {}
    ordered_mat_names = [
        'IPU1_cam1.Intrinsics.RadialDistortion',
        'IPU1_cam1.Intrinsics.TangentialDistortion',
        'IPU1_cam1.IntrinsicMatrix',
        'IPU1_cam2.Intrinsics.RadialDistortion',
        'IPU1_cam2.Intrinsics.TangentialDistortion',
        'IPU1_cam2.IntrinsicMatrix',
        'IPU1_stereo.RotationOfCamera2',
        'IPU1_stereo.TranslationOfCamera2',
        'IPU1_stereo.FundamentalMatrix',
        'IPU1_stereo.EssentialMatrix',
        'IPU1_Lidar-->IPU1_cam1',

        'IPU2_cam1.Intrinsics.RadialDistortion',
        'IPU2_cam1.Intrinsics.TangentialDistortion',
        'IPU2_cam1.IntrinsicMatrix',
        'IPU2_cam2.Intrinsics.RadialDistortion',
        'IPU2_cam2.Intrinsics.TangentialDistortion',
        'IPU2_cam2.IntrinsicMatrix',
        'IPU2_stereo.RotationOfCamera2',
        'IPU2_stereo.TranslationOfCamera2',
        'IPU2_stereo.FundamentalMatrix',
        'IPU2_stereo.EssentialMatrix',
        'IPU2: IPU2_Lidar-->IPU2_cam1',

        'IPU2_Lidar-->IPU1_Lidar'
    ]
    # Read the calibration file and populate the calib dictionary
    with open(calib_path) as f:
        mat_index = 0
        for line in f.readlines():
            line = line.strip()
            if len(line) == 0 or line.startswith('#'): continue
            
            mat = []
            mat_rows = line[1:-1].split(';')
            if len(mat_rows) == 1:
                mat = [float(v) for v in mat_rows[0].split(',')]
            else:
                for row in mat_rows: mat.append([float(v) for v in row.split(',')])
            calib[ordered_mat_names[mat_index]] = np.array(mat, dtype=np.float32)

            mat_index += 1

    # make sure the shapes are correct
    calib['P2'] = calib['IPU1_cam1.IntrinsicMatrix'].reshape(3, 3).T # 3x3
    calib['P2'] = np.c_[calib['P2'], np.zeros((3,1), dtype=np.float32)] # 3x4
    calib['Tr_velo_to_cam'] = calib['IPU1_Lidar-->IPU1_cam1'].reshape(4, 4) # 4x4

    # K_CAM2 = calib['IPU1_cam2.IntrinsicMatrix'].reshape(3, 3) # 3x3
    # K_CAM2 = np.c_[K_CAM2, np.zeros((3,1), dtype=np.float32)] # 3x4
    # P_Lidar_CAM1 = calib['IPU1_Lidar-->IPU1_cam1'].reshape(4, 4) # 4x4
    
    # T_CAM1_CAM2 = calib['IPU1_stereo.TranslationOfCamera2'].reshape(3, ) # 3,
    # R_CAM1_CAM2 = calib['IPU1_stereo.RotationOfCamera2'].reshape(3, 3) # 3x3
    # T_CAM1_CAM2_4x4 = np.eye(4)
    # T_CAM1_CAM2_4x4[:3, :3] = R_CAM1_CAM2
    # T_CAM1_CAM2_4x4[:3, 3] = T_CAM1_CAM2
    # P2 = np.dot(np.dot(K_CAM2, T_CAM1_CAM2_4x4), P_Lidar_CAM1)

    # calib['P2'] = K_CAM2
    # calib['Tr_velo_to_cam'] = T_CAM1_CAM2_4x4 @ P_Lidar_CAM1

    return calib

if __name__ == '__main__':
    for i in range(10):
        path = f'E:\\work\\urbanity\\hands_on\\datasets\\IPS300+\\IPS300_dataset_ICRA22\\calib\\{str(i).zfill(6)}.txt'
        calib = Handler(path)
        print(calib)