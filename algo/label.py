import inspect
from gui.config_gui import get_abs_path
from gui.logger_gui import Logger
from algo.utils import AlgoType, make_key, get_algo_params
algo_type = AlgoType.label

import numpy as np
import open3d as o3d

def remove_out_of_bound_labels(data_dict: dict, cfg_dict: dict, logger: Logger):
    """
    Remove labels that are out of the specified bounding box.

    Args:
        data_dict (Dict[str, any]): A dictionary containing data and logger.
        cfg_dict (Dict[str, any]): A dictionary containing configuration parameters.
        logger (gui.logger_gui.Logger): A logger object for logging messages and errors in GUI.
    """
    # get name and params
    algo_name = inspect.stack()[0].function
    params = get_algo_params(cfg_dict, algo_type, algo_name, logger)

    # Check if required data is present in data_dict
    if "current_label_list" not in data_dict:
        logger.log('current_label_list not found in data_dict', Logger.ERROR)
        return
    
    # get param values
    use_lidar_range = params['use_lidar_range']
    use_image_size = params['use_image_size']

    if not (use_lidar_range or use_image_size):
        logger.log('Both use_lidar_range and use_image_size are False. No operation to perform.', Logger.WARNING)
        return

    if use_lidar_range:
        if cfg_dict['proc']['lidar']['crop']['enabled']:
            pcd_min_xyz = cfg_dict['proc']['lidar']['crop']['min_xyz']
            pcd_max_xyz = cfg_dict['proc']['lidar']['crop']['max_xyz']
        elif 'current_point_cloud_numpy' in data_dict:
            pcd_min_xyz = np.min(data_dict['current_point_cloud_numpy'][:, 0:3], axis=0)
            pcd_max_xyz = np.max(data_dict['current_point_cloud_numpy'][:, 0:3], axis=0)
        else:
            logger.log('use_lidar_range is True but current_point_cloud_numpy not found in data_dict.', Logger.ERROR)
            return
    
    if use_image_size:
        if 'current_image_numpy' in data_dict:
            img_min_xy = np.array([0, 0], dtype=np.float32)
            img_max_xy = data_dict['current_image_numpy'].shape[:2][::-1]
        else:
            logger.log('use_image_size is True but current_image_numpy not found in data_dict.', Logger.ERROR)
            return

    # Get label list and bounding box limits
    lbl_list = data_dict['current_label_list']
    
    output = []
    for lbl_dict in lbl_list:
        add_bbox_label = True
        
        if use_lidar_range and 'bbox_3d' in lbl_dict:
            bbox_3d_center = lbl_dict['bbox_3d']['xyz_center']
            # Check if the bounding box center is within the specified limits
            x_condition = np.logical_and(pcd_min_xyz[0] <= bbox_3d_center[0], bbox_3d_center[0] <= pcd_max_xyz[0])
            y_condition = np.logical_and(pcd_min_xyz[1] <= bbox_3d_center[1], bbox_3d_center[1] <= pcd_max_xyz[1])
            z_condition = np.logical_and(pcd_min_xyz[2] <= bbox_3d_center[2], bbox_3d_center[2] <= pcd_max_xyz[2])
            # If not within the limits, skip the label
            if not (x_condition and y_condition and z_condition): add_bbox_label = False
        
        if use_image_size and 'bbox_2d' in lbl_dict:
            bbox_2d_center = lbl_dict['bbox_2d']['xy_center']
            bbox_2d_extent = lbl_dict['bbox_2d']['xy_extent']
            bbox_min_xy = bbox_2d_center - bbox_2d_extent / 2.0
            bbox_max_xy = bbox_2d_center + bbox_2d_extent / 2.0
            # Check if the bounding box is within the specified limits
            x_condition = np.logical_and(img_min_xy[0] <= bbox_min_xy[0], bbox_max_xy[0] <= img_max_xy[0])
            y_condition = np.logical_and(img_min_xy[1] <= bbox_min_xy[1], bbox_max_xy[1] <= img_max_xy[1])
            # If not within the limits, skip the label
            if not (x_condition and y_condition): add_bbox_label = False
        
        # Add the label to the output list if it is within the limits
        if add_bbox_label: output.append(lbl_dict)

    # Update the label list in data_dict
    data_dict['current_label_list'] = output

def remove_less_point_labels(data_dict: dict, cfg_dict: dict, logger: Logger):
    """
    Remove labels with fewer points than the specified threshold.

    Args:
        data_dict (dict): A dictionary containing data related to labels and point cloud.
        cfg_dict (dict): A dictionary containing configuration parameters.
        logger (gui.logger_gui.Logger): A logger object for logging messages and errors in GUI.
    """
    # get name and params
    algo_name = inspect.stack()[0].function
    params = get_algo_params(cfg_dict, algo_type, algo_name, logger)

    # Check if required data is present in data_dict
    if "current_label_list" not in data_dict:
        logger.log('current_label_list not found in data_dict', Logger.ERROR)
        return
    if 'current_point_cloud_numpy' not in data_dict:
        logger.log('current_point_cloud_numpy not found in data_dict', Logger.ERROR)
        return
    
    # Get label list and point cloud
    lbl_list = data_dict['current_label_list']
    point_cloud = data_dict['current_point_cloud_numpy']
    
    # define the output list
    output = []

    # iterate over the labels and check if the number of points in the bounding box is greater than the threshold
    # if it is, add the label to the output list, otherwise skip it
    for lbl_dict in lbl_list:
        if 'bbox_3d' not in lbl_dict: continue
        bbox_center = lbl_dict['bbox_3d']['xyz_center']
        bbox_extent = lbl_dict['bbox_3d']['xyz_extent']
        bbox_euler_angles = lbl_dict['bbox_3d']['xyz_euler_angles']
        R = o3d.geometry.OrientedBoundingBox.get_rotation_matrix_from_xyz(bbox_euler_angles)
        # create an OrientedBoundingBox object
        try: rotated_bbox = o3d.geometry.OrientedBoundingBox(bbox_center, R, bbox_extent)
        except:
            logger.log('Failed to create OrientedBoundingBox object', Logger.ERROR)
            continue
        inside_points = rotated_bbox.get_point_indices_within_bounding_box(o3d.utility.Vector3dVector(point_cloud[:, 0:3]))
        # check if the number of points inside the bounding box is greater than the threshold give in the configuration
        if len(inside_points) >= params['min_points']: output.append(lbl_dict)

    # update the label list in data_dict
    data_dict['current_label_list'] = output