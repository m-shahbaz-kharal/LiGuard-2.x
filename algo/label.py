import numpy as np

def crop_out_of_bound_bboxes(data, cfg) -> dict:
    lbl_list = data.current_label_list
    min_xyz = cfg.proc.lidar.crop.min_xyz
    max_xyz = cfg.proc.lidar.crop.max_xyz
    output = []
    for lbl_dict in lbl_list:
        bbox_center = lbl_dict['lidar_bbox']['xyz_center']
        x_condition = np.logical_and(min_xyz[0] <= bbox_center[0], bbox_center[0] <= max_xyz[0])
        y_condition = np.logical_and(min_xyz[1] <= bbox_center[1], bbox_center[1] <= max_xyz[1])
        z_condition = np.logical_and(min_xyz[2] <= bbox_center[2], bbox_center[2] <= max_xyz[2])
        if x_condition and y_condition and z_condition: output.append(lbl_dict)
    data.current_label_list = output