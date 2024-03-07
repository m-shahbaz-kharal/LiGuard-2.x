import numpy as np

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