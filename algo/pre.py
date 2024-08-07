from gui.logger_gui import Logger

def remove_nan_from_pcd(data_dict: dict, cfg_dict: dict):
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
    else: print('[algo->post.py->remove_nan_from_pcd][CRITICAL]: No logger object in data_dict. It is abnormal behavior as logger object is created by default. Please check if some script is removing the logger key in data_dict.'); return

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
    data_dict['current_point_cloud_numpy'] = current_point_cloud_numpy