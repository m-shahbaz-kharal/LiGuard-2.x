import numpy as np

def project_point_cloud_points(data_dict: dict, cfg_dict: dict):
    if "current_point_cloud_numpy" not in data_dict: return
    if "current_image_numpy" not in data_dict: return
    if 'current_calib_data' not in data_dict: return
    
    Tr_velo_to_cam = data_dict['current_calib_data']['Tr_velo_to_cam']
    if 'R0_rect' in data_dict['current_calib_data']: R0_rect = data_dict['current_calib_data']['R0_rect']
    else: R0_rect = np.eye(4,4)
    P2 = data_dict['current_calib_data']['P2']
    
    lidar_coords_Nx4 = np.hstack((data_dict['current_point_cloud_numpy'][:,:3], np.ones((data_dict['current_point_cloud_numpy'].shape[0], 1))))
    
    pixel_coords = P2 @ R0_rect @ Tr_velo_to_cam @ lidar_coords_Nx4.T
    lidar_depths = np.linalg.norm(lidar_coords_Nx4[:, :3], axis=1)
    
    front_pixel_coords = pixel_coords[:, pixel_coords[2] > 0]
    front_lidar_depths = lidar_depths[pixel_coords[2] > 0]
    
    front_pixel_coords = front_pixel_coords[:2] / front_pixel_coords[2]
    front_pixel_coords = front_pixel_coords.T
    front_lidar_depths = front_lidar_depths * 6.0
    front_lidar_depths = 255.0 - np.clip(front_lidar_depths, 0, 255)
    
    front_pixel_coords = front_pixel_coords.astype(int)
    front_lidar_depths = front_lidar_depths.astype(np.uint8)
    
    valid_coords = (front_pixel_coords[:, 0] >= 0) & (front_pixel_coords[:, 0] < data_dict['current_image_numpy'].shape[1]) & (front_pixel_coords[:, 1] >= 0) & (front_pixel_coords[:, 1] < data_dict['current_image_numpy'].shape[0])
    
    pixel_coords_valid = front_pixel_coords[valid_coords]
    pixel_depths_valid = front_lidar_depths[valid_coords]
    
    data_dict['current_image_numpy'][pixel_coords_valid[:, 1], pixel_coords_valid[:, 0]] = np.column_stack((pixel_depths_valid, np.zeros_like(pixel_depths_valid), np.zeros_like(pixel_depths_valid)))
