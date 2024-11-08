import os
import numpy as np

calib_file_extension = '.txt' # this tells the extension of the calibration files in directory given by config['data']['calib_subdir']


def Handler(calib_path: str): # don't change the function signature
    """
    Process a calibration file and generate a dictionary containing the calibration data in KITTI format.

    Args:
        calib_path (str): The path to the calibration file.

    Returns:
        dict: A dictionary containing the calibration data.
    """
    calib = {}

    #################################################################################
    # basic code snippet to read the calibration file, uncomment and modify as needed
    #################################################################################
    # if not os.path.exists(calib_path): return calib
    # with open(calib_path, 'r') as f: clbs = f.readlines()
    # for line in clbs:
    #     line = line.strip()
    #     if len(line) == 0 or line.startswith('#'): continue
    #     # parse your calibs
    #     calib['P2'] = ? # 3x4 projection matrix
    #     calib['R0_rect'] = ? # 4x4 rectification matrix
    #     calib['Tr_velo_to_cam'] = ? # 4x4 transformation matrix from lidar to camera

    return calib # make sure to return the calibration data dictionary, even if it is empty