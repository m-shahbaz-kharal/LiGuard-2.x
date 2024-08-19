from gui.logger_gui import Logger

def remove_nan_inf_allzero_from_pcd(data_dict: dict, cfg_dict: dict):
    """
    Remove NaN values from the point cloud.

    Args:
        data_dict (dict): A dictionary containing the required data.
        cfg_dict (dict): A dictionary containing configuration parameters.

    Returns:
        None
    """
    # Get logger object from data_dict
    if 'logger' in data_dict: logger:Logger = data_dict['logger']
    else: print('[algo->post.py->remove_nan_from_pcd]: No logger object in data_dict. It is abnormal behavior as logger object is created by default. Please check if some script is removing the logger key in data_dict.'); return

    # Check if required data is present in data_dict
    if 'current_point_cloud_numpy' not in data_dict:
        logger.log('[algo->post.py->remove_nan_from_pcd]: current_point_cloud_numpy not found in data_dict', Logger.ERROR)
        return
    
    # imports
    import numpy as np

    # Get required data from data_dict
    current_point_cloud_numpy = data_dict['current_point_cloud_numpy']

    # Remove NaN values from the point cloud
    current_point_cloud_numpy = current_point_cloud_numpy[~np.isnan(current_point_cloud_numpy).any(axis=1), ...]

    # Remove inf values from the point cloud
    current_point_cloud_numpy = current_point_cloud_numpy[~np.isinf(current_point_cloud_numpy).any(axis=1), ...]

    # Remove points with x,y,z all zero
    current_point_cloud_numpy = current_point_cloud_numpy[~np.all(current_point_cloud_numpy[:, :3] == 0, axis=1), ...]

    data_dict['current_point_cloud_numpy'] = current_point_cloud_numpy

def manual_calibration(data_dict: dict, cfg_dict: dict):
    # get logger object from data_dict
    if 'logger' in data_dict: logger:Logger = data_dict['logger']
    else: print('[algo->calib.py->manual_calibration]: No logger object in data_dict. It is abnormal behavior as logger object is created by default. Please check if some script is removing the logger key in data_dict.'); return

    # imports
    import numpy as np

    # get params
    params = cfg_dict['proc']['pre']['manual_calibration']
    
    # parse params and add to data_dict
    try:
        Tr_velo_to_cam = np.array([float(x) for x in params['T_lidar_to_cam_4x4'].split(' ')], dtype=np.float32).reshape(4, 4)
        R0_rect = np.array([float(x) for x in params['rotation_matrix_4x4'].split(' ')], dtype=np.float32).reshape(4, 4)
        P2 = np.array([float(x) for x in params['cam_intrinsic_3x4'].split(' ')], dtype=np.float32).reshape(3, 4)
        calib = {'Tr_velo_to_cam': Tr_velo_to_cam, 'R0_rect': R0_rect, 'P2': P2}
        data_dict['current_calib_path'] = None
        data_dict['current_calib_data'] = calib
    except Exception as e:
        logger.log('[algo->pre.py->manual_calibration]: Error in parsing calibration parameters.', Logger.ERROR)