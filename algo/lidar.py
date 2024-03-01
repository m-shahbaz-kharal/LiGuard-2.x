import numpy as np

def crop(data_dict, cfg) -> np.ndarray:
    pcd = data_dict.current_point_cloud_numpy
    min_xyz = cfg.proc.lidar.crop.min_xyz
    max_xyz = cfg.proc.lidar.crop.max_xyz
    x_condition = np.logical_and(min_xyz[0] <= pcd[:, 0], pcd[:, 0] <= max_xyz[0])
    y_condition = np.logical_and(min_xyz[1] <= pcd[:, 1], pcd[:, 1] <= max_xyz[1])
    z_condition = np.logical_and(min_xyz[2] <= pcd[:, 2], pcd[:, 2] <= max_xyz[2])
    data_dict.current_point_cloud_numpy = pcd[x_condition & y_condition & z_condition]
