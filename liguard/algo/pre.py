import inspect
from liguard.gui.config_gui import get_abs_path
from liguard.gui.logger_gui import Logger
from liguard.algo.utils import AlgoType, make_key, get_algo_params
algo_type = AlgoType.pre

def remove_nan_inf_allzero_from_pcd(data_dict: dict, cfg_dict: dict, logger: Logger):
    """
    Removes NaN values from the point cloud.

    Args:
        data_dict (dict): A dictionary containing the required data.
        cfg_dict (dict): A dictionary containing configuration parameters.
        logger (Logger): A logger object for logging messages.
    """
    # get name and params
    algo_name = inspect.stack()[0].function
    params = get_algo_params(cfg_dict, algo_type, algo_name, logger)

    # Check if required data is present in data_dict
    if 'current_point_cloud_numpy' not in data_dict:
        logger.log('current_point_cloud_numpy not found in data_dict', Logger.ERROR)
        return
    
    # imports
    import numpy as np

    # Get required data from data_dict
    current_point_cloud_numpy = data_dict['current_point_cloud_numpy']

    # Remove NaN, inf, and zero values from the point cloud
    current_point_cloud_numpy = current_point_cloud_numpy[~np.isnan(current_point_cloud_numpy).any(axis=1), ...]
    current_point_cloud_numpy = current_point_cloud_numpy[~np.isinf(current_point_cloud_numpy).any(axis=1), ...]
    current_point_cloud_numpy = current_point_cloud_numpy[~np.all(current_point_cloud_numpy[:, :3] == 0, axis=1), ...]

    # update data_dict
    data_dict['current_point_cloud_numpy'] = current_point_cloud_numpy

def manual_calibration(data_dict: dict, cfg_dict: dict, logger: Logger):
    """
    Manually calibrates the point cloud data.

    Args:
        data_dict (dict): A dictionary containing the required data.
        cfg_dict (dict): A dictionary containing configuration parameters.
        logger (Logger): A logger object for logging messages
    """
    # get name and params
    algo_name = inspect.stack()[0].function
    params = get_algo_params(cfg_dict, algo_type, algo_name, logger)

    # imports
    import numpy as np
    
    # parse params and add to data_dict
    try:
        Tr_velo_to_cam = np.array([float(x) for x in params['T_lidar_to_cam_4x4'].split(' ')], dtype=np.float32).reshape(4, 4)
        R0_rect = np.array([float(x) for x in params['rotation_matrix_4x4'].split(' ')], dtype=np.float32).reshape(4, 4)
        P2 = np.array([float(x) for x in params['cam_intrinsic_3x4'].split(' ')], dtype=np.float32).reshape(3, 4)
        calib = {'Tr_velo_to_cam': Tr_velo_to_cam, 'R0_rect': R0_rect, 'P2': P2}
        data_dict['current_calib_path'] = None
        data_dict['current_calib_data'] = calib
    except Exception as e:
        logger.log(f'Error: {e}', Logger.ERROR)