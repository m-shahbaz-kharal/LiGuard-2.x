import os
import numpy as np

from algo.utils import gather_point_clouds, skip_frames, combine_gathers
from pcd.utils import fix_number_of_points_in_pcd

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
    
    normalized_pixel_coords_2d = pixel_coords[:2] / (pixel_coords[2] + 1e-8)
    normalized_pixel_coords_2d = normalized_pixel_coords_2d.T
    normalized_pixel_coords_2d = normalized_pixel_coords_2d.astype(int)
    
    valid_coords = np.logical_and.reduce((pixel_coords[2,:] > 0, normalized_pixel_coords_2d[:, 0] >= 0, normalized_pixel_coords_2d[:, 0] < img_np.shape[1], normalized_pixel_coords_2d[:, 1] >= 0, normalized_pixel_coords_2d[:, 1] < img_np.shape[0]))
    data_dict['current_point_cloud_point_colors'][valid_coords] = img_np[normalized_pixel_coords_2d[valid_coords][:, 1], normalized_pixel_coords_2d[valid_coords][:, 0]] / 255.0
    
def BGFilterDHistDPP(data_dict: dict, cfg_dict: dict):
    algo_name = 'BGFilterDHistDPP'
    query_frames_key = f'{algo_name}_query_frames'
    skip_frames_key = f'{algo_name}_skip_frames'
    filter_name = f'{algo_name}_filter'
    
    number_of_frame_gather_iters = cfg_dict['proc']['lidar']['BGFilterDHistDPP']['number_of_frame_gather_iters']
    number_of_frames_in_each_gather_iter = cfg_dict['proc']['lidar']['BGFilterDHistDPP']['number_of_frames_in_each_gather_iter']
    number_of_skip_frames_after_each_iter = cfg_dict['proc']['lidar']['BGFilterDHistDPP']['number_of_skip_frames_after_each_iter']
    number_of_points_per_frame = cfg_dict['proc']['lidar']['BGFilterDHistDPP']['number_of_points_per_frame']
    lidar_range_in_unit_length = cfg_dict['proc']['lidar']['BGFilterDHistDPP']['lidar_range_in_unit_length']
    bins_per_unit_length = cfg_dict['proc']['lidar']['BGFilterDHistDPP']['bins_per_unit_length']
    background_density_threshold = cfg_dict['proc']['lidar']['BGFilterDHistDPP']['background_density_threshold']
    
    if filter_name not in data_dict:
        data_dict[f'{algo_name}_number_of_frame_gather_iters'] = number_of_frame_gather_iters
        data_dict[f'{algo_name}_number_of_frames_in_each_gather_iter'] = number_of_frames_in_each_gather_iter
        data_dict[f'{algo_name}_number_of_skip_frames_after_each_iter'] = number_of_skip_frames_after_each_iter
        data_dict[f'{algo_name}_number_of_points_per_frame'] = number_of_points_per_frame
        data_dict[f'{algo_name}_lidar_range_in_unit_length'] = lidar_range_in_unit_length
        data_dict[f'{algo_name}_bins_per_unit_length'] = bins_per_unit_length
        
        for i in range(number_of_frame_gather_iters):
            gathering_done = gather_point_clouds(data_dict, cfg_dict, f'{query_frames_key}_{i}', number_of_frames_in_each_gather_iter)
            if not gathering_done: return
            skipping_done = skip_frames(data_dict, cfg_dict, f'{skip_frames_key}_{i}', number_of_skip_frames_after_each_iter)
            if not skipping_done: return
        
        gather_keys = [f'{query_frames_key}_{i}' for i in range(number_of_frame_gather_iters)]
        combine_gathers(data_dict, cfg_dict, query_frames_key, gather_keys)
        assert len(data_dict[query_frames_key]) == number_of_frame_gather_iters * number_of_frames_in_each_gather_iter
        
        from algo.background_filters.DHistDPP import DHistDPP
        data_dict[query_frames_key] = [fix_number_of_points_in_pcd(frame, number_of_points_per_frame) for frame in data_dict[query_frames_key]]
        data_dict[filter_name] = DHistDPP(data_dict[query_frames_key], number_of_points_per_frame, lidar_range_in_unit_length, bins_per_unit_length)
    else:
        condition = False
        condition = condition or data_dict[f'{algo_name}_number_of_frame_gather_iters'] != number_of_frame_gather_iters
        condition = condition or data_dict[f'{algo_name}_number_of_frames_in_each_gather_iter'] != number_of_frames_in_each_gather_iter
        condition = condition or data_dict[f'{algo_name}_number_of_skip_frames_after_each_iter'] != number_of_skip_frames_after_each_iter
        condition = condition or data_dict[f'{algo_name}_number_of_points_per_frame'] != number_of_points_per_frame
        condition = condition or data_dict[f'{algo_name}_lidar_range_in_unit_length'] != lidar_range_in_unit_length
        condition = condition or data_dict[f'{algo_name}_bins_per_unit_length'] != bins_per_unit_length
        
        if condition:
            keys_to_remove = [key for key in data_dict.keys() if key.startswith(algo_name)]
            for key in keys_to_remove: data_dict.pop(key)
            return
    
    if filter_name in data_dict:
        data_dict['current_point_cloud_numpy'] = fix_number_of_points_in_pcd(data_dict['current_point_cloud_numpy'], number_of_points_per_frame)
        data_dict['current_point_cloud_numpy'] = data_dict['current_point_cloud_numpy'][data_dict[filter_name](data_dict['current_point_cloud_numpy'], background_density_threshold)]