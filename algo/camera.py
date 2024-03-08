import numpy as np

def project_point_cloud_points(data_dict: dict, cfg_dict: dict):
    if "current_point_cloud_numpy" not in data_dict: return
    if "current_image_numpy" not in data_dict: return
    if 'current_label_list' not in data_dict: return
    if 'calib' not in data_dict['current_label_list'][0]: return
    
    Tr_velo_to_cam = data_dict['current_label_list'][0]['calib']['Tr_velo_to_cam']
    R0_rect = data_dict['current_label_list'][0]['calib']['R0_rect']
    P2 = data_dict['current_label_list'][0]['calib']['P2']
    
    lidar_coords_Nx4 = np.hstack((data_dict['current_point_cloud_numpy'][:,:3], np.ones((data_dict['current_point_cloud_numpy'].shape[0], 1))))
    pixel_coords = P2 @ R0_rect @ Tr_velo_to_cam @ lidar_coords_Nx4.T
    pixel_coords = pixel_coords[:2] / pixel_coords[2]
    pixel_coords = pixel_coords.T
    pixel_coords = pixel_coords.astype(int)
    
    valid_coords = (pixel_coords[:, 0] >= 0) & (pixel_coords[:, 0] < data_dict['current_image_numpy'].shape[1]) & (pixel_coords[:, 1] >= 0) & (pixel_coords[:, 1] < data_dict['current_image_numpy'].shape[0])
    pixel_coords_valid = pixel_coords[valid_coords]
    data_dict['current_image_numpy'][pixel_coords_valid[:, 1], pixel_coords_valid[:, 0]] = [255, 0, 0]
