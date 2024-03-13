import numpy as np
import open3d as o3d

def remove_out_of_bound_labels(data_dict: dict, cfg_dict: dict):
    if "current_label_list" not in data_dict: return
    lbl_list = data_dict['current_label_list']
    min_xyz = cfg_dict['proc']['lidar']['crop']['min_xyz']
    max_xyz = cfg_dict['proc']['lidar']['crop']['max_xyz']
    output = []
    for lbl_dict in lbl_list:
        bbox_center = lbl_dict['lidar_bbox']['lidar_xyz_center']
        x_condition = np.logical_and(min_xyz[0] <= bbox_center[0], bbox_center[0] <= max_xyz[0])
        y_condition = np.logical_and(min_xyz[1] <= bbox_center[1], bbox_center[1] <= max_xyz[1])
        z_condition = np.logical_and(min_xyz[2] <= bbox_center[2], bbox_center[2] <= max_xyz[2])
        if x_condition and y_condition and z_condition: output.append(lbl_dict)
    data_dict['current_label_list'] = output

def remove_empty_labels(data_dict: dict, cfg_dict: dict):
    if "current_label_list" not in data_dict: return
    if 'current_point_cloud_numpy' not in data_dict: return

    lbl_list = data_dict['current_label_list']
    point_cloud = data_dict['current_point_cloud_numpy']
    
    output = []

    for lbl_dict in lbl_list:
        bbox_center = lbl_dict['lidar_bbox']['lidar_xyz_center']
        bbox_extent = lbl_dict['lidar_bbox']['lidar_xyz_extent']
        bbox_euler_angles = lbl_dict['lidar_bbox']['lidar_xyz_euler_angles']
        R = o3d.geometry.OrientedBoundingBox.get_rotation_matrix_from_xyz(bbox_euler_angles)
        rotated_bbox = o3d.geometry.OrientedBoundingBox(bbox_center, R, bbox_extent)
        inside_points = rotated_bbox.get_point_indices_within_bounding_box(o3d.utility.Vector3dVector(point_cloud[:, 0:3]))
        if len(inside_points) > 0: output.append(lbl_dict)

    data_dict['current_label_list'] = output