import numpy as np
import open3d as o3d

from gui.logger_gui import Logger
def remove_out_of_bound_labels(data_dict: dict, cfg_dict: dict):
    logger:Logger = data_dict['logger']

    if "current_label_list" not in data_dict:
        logger.log('[algo->label.py->remove_out_of_bound_labels]: current_label_list not found in data_dict', Logger.ERROR)
        return
    lbl_list = data_dict['current_label_list']
    min_xyz = cfg_dict['proc']['lidar']['crop']['min_xyz']
    max_xyz = cfg_dict['proc']['lidar']['crop']['max_xyz']
    output = []
    for lbl_dict in lbl_list:
        if 'lidar_bbox' not in lbl_dict: continue
        bbox_center = lbl_dict['lidar_bbox']['lidar_xyz_center']
        x_condition = np.logical_and(min_xyz[0] <= bbox_center[0], bbox_center[0] <= max_xyz[0])
        y_condition = np.logical_and(min_xyz[1] <= bbox_center[1], bbox_center[1] <= max_xyz[1])
        z_condition = np.logical_and(min_xyz[2] <= bbox_center[2], bbox_center[2] <= max_xyz[2])
        if x_condition and y_condition and z_condition: output.append(lbl_dict)
    data_dict['current_label_list'] = output

def remove_less_point_labels(data_dict: dict, cfg_dict: dict):
    logger:Logger = data_dict['logger']

    if "current_label_list" not in data_dict:
        logger.log('[algo->label.py->remove_less_point_labels]: current_label_list not found in data_dict', Logger.ERROR)
        return
    if 'current_point_cloud_numpy' not in data_dict:
        logger.log('[algo->label.py->remove_less_point_labels]: current_point_cloud_numpy not found in data_dict', Logger.ERROR)
        return

    lbl_list = data_dict['current_label_list']
    point_cloud = data_dict['current_point_cloud_numpy']
    
    output = []

    for lbl_dict in lbl_list:
        if 'lidar_bbox' not in lbl_dict: continue
        bbox_center = lbl_dict['lidar_bbox']['lidar_xyz_center']
        bbox_extent = lbl_dict['lidar_bbox']['lidar_xyz_extent']
        bbox_euler_angles = lbl_dict['lidar_bbox']['lidar_xyz_euler_angles']
        R = o3d.geometry.OrientedBoundingBox.get_rotation_matrix_from_xyz(bbox_euler_angles)
        try: rotated_bbox = o3d.geometry.OrientedBoundingBox(bbox_center, R, bbox_extent)
        except:
            logger.log(f'[algo->label.py->remove_less_point_labels]: failed to create an OrientedBoundingBox, skipping ...', Logger.WARNING)
            continue
        inside_points = rotated_bbox.get_point_indices_within_bounding_box(o3d.utility.Vector3dVector(point_cloud[:, 0:3]))
        if len(inside_points) >= cfg_dict['proc']['label']['remove_less_point_labels']['min_points']: output.append(lbl_dict)

    data_dict['current_label_list'] = output