import numpy as np

def crop(data_dict: dict, cfg_dict: dict):
    if "current_point_cloud_numpy" not in data_dict: return
    pcd = data_dict['current_point_cloud_numpy']
    min_xyz = cfg_dict['proc']['lidar']['crop']['min_xyz']
    max_xyz = cfg_dict['proc']['lidar']['crop']['max_xyz']
    x_condition = np.logical_and(min_xyz[0] <= pcd[:, 0], pcd[:, 0] <= max_xyz[0])
    y_condition = np.logical_and(min_xyz[1] <= pcd[:, 1], pcd[:, 1] <= max_xyz[1])
    z_condition = np.logical_and(min_xyz[2] <= pcd[:, 2], pcd[:, 2] <= max_xyz[2])
    data_dict['current_point_cloud_numpy'] = pcd[x_condition & y_condition & z_condition]
    
def colorize_point_cloud(data_dict: dict, cfg_dict: dict):
    if "current_point_cloud_numpy" not in data_dict: return
    if "current_image_numpy" not in data_dict: return
    if 'current_label_list' not in data_dict: return
    if 'calib' not in data_dict['current_label_list'][0]: return
    
    img_np = data_dict['current_image_numpy']
    Tr_velo_to_cam = data_dict['current_label_list'][0]['calib']['Tr_velo_to_cam']
    R0_rect = data_dict['current_label_list'][0]['calib']['R0_rect']
    P2 = data_dict['current_label_list'][0]['calib']['P2']
    
    data_dict['current_point_cloud_point_colors'] = np.ones((data_dict['current_point_cloud_numpy'].shape[0], 3), dtype=np.uint8) # N X 3(RGB)
    lidar_coords_Nx4 = np.hstack((data_dict['current_point_cloud_numpy'][:,:3], np.ones((data_dict['current_point_cloud_numpy'].shape[0], 1))))
    pixel_coords = P2 @ R0_rect @ Tr_velo_to_cam @ lidar_coords_Nx4.T
    pixel_coords = pixel_coords[:2] / (pixel_coords[2] + 1e-8)
    pixel_coords = pixel_coords.T
    pixel_coords = pixel_coords.astype(int)
    
    valid_coords = np.logical_and.reduce((pixel_coords[:, 0] >= 0, pixel_coords[:, 0] < img_np.shape[1], pixel_coords[:, 1] >= 0, pixel_coords[:, 1] < img_np.shape[0]))
    data_dict['current_point_cloud_point_colors'][valid_coords] = img_np[pixel_coords[valid_coords][:, 1], pixel_coords[valid_coords][:, 0]] / 255.0