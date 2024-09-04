# contains point-cloud processing algorithms

import os
import sys

import numpy as np

from gui.logger_gui import Logger

def rotate(data_dict: dict, cfg_dict: dict):
    """
    Rotate the point cloud data by the specified angles.

    Args:
        data_dict (dict): A dictionary containing the data.
        cfg_dict (dict): A dictionary containing the configuration parameters.

    Returns:
        None
    """
    
    # get logger object from data_dict
    if 'logger' in data_dict: logger:Logger = data_dict['logger']
    else: print('[algo->lidar.py->rotate]: No logger object in data_dict. It is abnormal behavior as logger object is created by default. Please check if some script is removing the logger key in data_dict.'); return

    # check if required data is present in data_dict
    if "current_point_cloud_numpy" not in data_dict:
        logger.log('[algo->lidar.py->rotate]: current_point_cloud_numpy not found in data_dict', Logger.ERROR)
        return
    
    # get point cloud and rotation angles
    pcd = data_dict['current_point_cloud_numpy']
    angles = cfg_dict['proc']['lidar']['rotate']['angles']
    angles_rad = np.deg2rad(angles)
    
    # calculate rotation matrix
    rotation_matrix = np.array([[np.cos(angles_rad[1]) * np.cos(angles_rad[2]), np.cos(angles_rad[1]) * np.sin(angles_rad[2]), -np.sin(angles_rad[1])],
                                [np.sin(angles_rad[0]) * np.sin(angles_rad[1]) * np.cos(angles_rad[2]) - np.cos(angles_rad[0]) * np.sin(angles_rad[2]), np.sin(angles_rad[0]) * np.sin(angles_rad[1]) * np.sin(angles_rad[2]) + np.cos(angles_rad[0]) * np.cos(angles_rad[2]), np.sin(angles_rad[0]) * np.cos(angles_rad[1])],
                                [np.cos(angles_rad[0]) * np.sin(angles_rad[1]) * np.cos(angles_rad[2]) + np.sin(angles_rad[0]) * np.sin(angles_rad[2]), np.cos(angles_rad[0]) * np.sin(angles_rad[1]) * np.sin(angles_rad[2]) - np.sin(angles_rad[0]) * np.cos(angles_rad[2]), np.cos(angles_rad[0]) * np.cos(angles_rad[1])]])
    
    # rotate the point cloud
    data_dict['current_point_cloud_numpy'] = np.dot(pcd[:, :3], rotation_matrix.T)

def crop(data_dict: dict, cfg_dict: dict):
    """
    Crop the point cloud data based on the specified limits.

    Args:
        data_dict (dict): A dictionary containing the data.
        cfg_dict (dict): A dictionary containing the configuration parameters.

    Returns:
        None
    """
    
    # get logger object from data_dict
    if 'logger' in data_dict: logger:Logger = data_dict['logger']
    else: print('[algo->lidar.py->crop]: No logger object in data_dict. It is abnormal behavior as logger object is created by default. Please check if some script is removing the logger key in data_dict.'); return

    # check if required data is present in data_dict
    if "current_point_cloud_numpy" not in data_dict:
        logger.log('[algo->lidar.py->crop]: current_point_cloud_numpy not found in data_dict', Logger.ERROR)
        return
    
    # get point cloud and crop limits
    pcd = data_dict['current_point_cloud_numpy']
    min_xyz = cfg_dict['proc']['lidar']['crop']['min_xyz']
    max_xyz = cfg_dict['proc']['lidar']['crop']['max_xyz']
    
    # create conditions for cropping
    x_condition = np.logical_and(min_xyz[0] <= pcd[:, 0], pcd[:, 0] <= max_xyz[0])
    y_condition = np.logical_and(min_xyz[1] <= pcd[:, 1], pcd[:, 1] <= max_xyz[1])
    z_condition = np.logical_and(min_xyz[2] <= pcd[:, 2], pcd[:, 2] <= max_xyz[2])
    
    # Update the point cloud in data_dict
    data_dict['current_point_cloud_numpy'] = pcd[x_condition & y_condition & z_condition]
    data_dict['current_point_cloud_point_colors'] = np.ones((data_dict['current_point_cloud_numpy'].shape[0], 3), dtype=np.float32)
    
def project_image_pixel_colors(data_dict: dict, cfg_dict: dict):
    """
    Projects the colors of image pixels onto the point cloud.

    Args:
        data_dict (dict): A dictionary containing the required data for the operation.
        cfg_dict (dict): A dictionary containing configuration parameters.

    Returns:
        None
    """

    # get logger object from data_dict
    if 'logger' in data_dict: logger:Logger = data_dict['logger']
    else: print('[algo->lidar.py->project_image_pixel_colors]: No logger object in data_dict. It is abnormal behavior as logger object is created by default. Please check if some script is removing the logger key in data_dict.'); return
    
    # check if required data is present in data_dict
    if "current_point_cloud_numpy" not in data_dict:
        logger.log('[algo->lidar.py->project_image_pixel_colors]: current_point_cloud_numpy not found in data_dict', Logger.ERROR)
        return
    if "current_image_numpy" not in data_dict:
        logger.log('[algo->lidar.py->project_image_pixel_colors]: current_image_numpy not found in data_dict', Logger.ERROR)
        return
    if "current_calib_data" not in data_dict:
        logger.log('[algo->lidar.py->project_image_pixel_colors]: current_calib_data not found in data_dict', Logger.ERROR)
        return
    
    # Extract required data
    img_np = data_dict['current_image_numpy']
    Tr_velo_to_cam = data_dict['current_calib_data']['Tr_velo_to_cam']
    R0_rect = data_dict['current_calib_data']['R0_rect']
    P2 = data_dict['current_calib_data']['P2']
    
    data_dict['current_point_cloud_point_colors'] = np.ones((data_dict['current_point_cloud_numpy'].shape[0], 3), dtype=np.float32) # N X 3(RGB)
    # Convert lidar coordinates to homogeneous coordinates
    lidar_coords_Nx4 = np.hstack((data_dict['current_point_cloud_numpy'][:,:3], np.ones((data_dict['current_point_cloud_numpy'].shape[0], 1))))
    
    # Project lidar points onto the image plane
    pixel_coords = P2 @ R0_rect @ Tr_velo_to_cam @ lidar_coords_Nx4.T
    
    # Normalize pixel coordinates
    normalized_pixel_coords_2d = pixel_coords[:2] / (pixel_coords[2] + 1e-8)
    normalized_pixel_coords_2d = normalized_pixel_coords_2d.T
    normalized_pixel_coords_2d = normalized_pixel_coords_2d.astype(int)
    
    # Filter out coordinates that are outside the image boundaries
    valid_coords = np.logical_and.reduce((pixel_coords[2,:] > 0, normalized_pixel_coords_2d[:, 0] >= 0, normalized_pixel_coords_2d[:, 0] < img_np.shape[1], normalized_pixel_coords_2d[:, 1] >= 0, normalized_pixel_coords_2d[:, 1] < img_np.shape[0]))
    # Update the point cloud colors in data_dict corresponding to the valid pixel coordinates
    data_dict['current_point_cloud_point_colors'][valid_coords] = img_np[normalized_pixel_coords_2d[valid_coords][:, 1], normalized_pixel_coords_2d[valid_coords][:, 0]] / 255.0
    
def BGFilterDHistDPP(data_dict: dict, cfg_dict: dict):
    """
    Apply Background Filter using Dynamic Histogram Point Process (DHistDPP) algorithm.

    Args:
        data_dict (dict): A dictionary containing data for processing.
        cfg_dict (dict): A dictionary containing configuration parameters.

    Returns:
        None
    """
    # get logger object from data_dict
    if 'logger' in data_dict: logger:Logger = data_dict['logger']
    else: print('[algo->lidar.py->BGFilterDHistDPP]: No logger object in data_dict. It is abnormal behavior as logger object is created by default. Please check if some script is removing the logger key in data_dict.'); return
    
    # imports
    from algo.non_nn.DHistDPP import calc_DHistDPP_params, save_DHistDPP_params, load_DHistDPP_params, make_DHistDPP_filter
    from algo.utils import gather_point_clouds, skip_frames, combine_gathers
    from pcd.utils import get_fixed_sized_point_cloud

    # algo name
    algo_name = 'BGFilterDHistDPP'

    # dict keys
    query_frames_key = f'{algo_name}_query_frames'
    skip_frames_key = f'{algo_name}_skip_frames'
    params_key = f'{algo_name}_params'
    filter_key = f'{algo_name}_filter'
    filter_loaded_key = f'{algo_name}_filter_loaded'

    # get params
    params = cfg_dict['proc']['lidar']['BGFilterDHistDPP'].copy()
    live_editable_params = ['background_density_threshold'] # list of params that can be live edited and do not require re-computation of filter
    
    # generate keys for query and skip frames
    all_query_frames_keys = [f'{query_frames_key}_{i}' for i in range(params['number_of_frame_gather_iters'])]
    all_skip_frames_keys = [f'{skip_frames_key}_{i}' for i in range(params['number_of_skip_frames_after_each_iter'])]

    # load filter if exists
    if filter_loaded_key not in data_dict and params['load_filter']:
        # add params to data_dict
        data_dict[params_key] = params
        
        filter_params = load_DHistDPP_params(params['filter_path'])
        if filter_params:
            data_dict[filter_key] = lambda pcd, threshold: make_DHistDPP_filter(pcd, threshold, **filter_params)
            data_dict[filter_loaded_key] = True
            logger.log(f'[algo->lidar.py->BGFilterDHistDPP]: Filter loaded from {params["filter_path"]}.', Logger.INFO)
            data_dict['BGFilterDHistDPP_set'] = True
        else:
            data_dict[filter_loaded_key] = False
            logger.log(f'[algo->lidar.py->BGFilterDHistDPP]: Failed to load filter from {params["filter_path"]}. Calculating ...', Logger.WARNING)
            data_dict['BGFilterDHistDPP_set'] = False
    
    # generate filter if not exists
    if filter_key not in data_dict:
        data_dict[params_key] = params
        
        # gather frames
        for i in range(params['number_of_frame_gather_iters']):
            gathering_done = gather_point_clouds(data_dict, cfg_dict, all_query_frames_keys[i], params['number_of_frames_in_each_gather_iter'])
            if not gathering_done: return
            skipping_done = skip_frames(data_dict, cfg_dict, all_skip_frames_keys[i], params['number_of_skip_frames_after_each_iter'])
            if not skipping_done: return
            
        # combine gathered frames
        combine_gathers(data_dict, cfg_dict, query_frames_key, all_query_frames_keys)
        assert len(data_dict[query_frames_key]) == params['number_of_frame_gather_iters'] * params['number_of_frames_in_each_gather_iter']
        
        # generate filter
        logger.log(f'[algo->lidar.py->BGFilterDHistDPP]: Generating filter', Logger.INFO)
        
        data_dict[query_frames_key] = [get_fixed_sized_point_cloud(frame, params['number_of_points_per_frame']) for frame in data_dict[query_frames_key]]
        filter_params = calc_DHistDPP_params(data_dict[query_frames_key], params['number_of_points_per_frame'], params['lidar_range_in_unit_length'], params['bins_per_unit_length'])
        data_dict[filter_key] = lambda pcd, threshold: make_DHistDPP_filter(pcd, threshold, **filter_params)
        # save filter
        filter_saved_path = save_DHistDPP_params(filter_params, params['filter_path'])
        logger.log(f'[algo->lidar.py->BGFilterDHistDPP]: Filter generated and saved in {filter_saved_path}.', Logger.INFO)
        data_dict['BGFilterDHistDPP_set'] = True
    else:
        # recompute filter if non-live-editable params are changed
        condition = False
        for key in params:
            if key in live_editable_params: continue
            condition = condition or data_dict[params_key][key] != params[key]
        # remove all algo keys if params are changed so that filter is re-computed on next call
        if condition:
            keys_to_remove = [key for key in data_dict.keys() if key.startswith(algo_name)]
            for key in keys_to_remove: data_dict.pop(key)
            data_dict['BGFilterDHistDPP_set'] = False
            return
    
    # if filter exists, apply it
    if filter_key in data_dict:
        data_dict['current_point_cloud_numpy'] = get_fixed_sized_point_cloud(data_dict['current_point_cloud_numpy'], params['number_of_points_per_frame'])
        data_dict['current_point_cloud_numpy'] = data_dict['current_point_cloud_numpy'][data_dict[filter_key](data_dict['current_point_cloud_numpy'], params['background_density_threshold'])]

def BGFilterSTDF(data_dict: dict, cfg_dict: dict):
    """
    Applies Background Filter using Spatio-Temporal Density Filtering (BGFilterSTDF) to the point cloud data.

    Args:
        data_dict (dict): A dictionary containing the input data and intermediate results.
        cfg_dict (dict): A dictionary containing the configuration parameters.

    Returns:
        None
    """

    # get logger object from data_dict
    if 'logger' in data_dict: logger:Logger = data_dict['logger']
    else: print('[algo->lidar.py->BGFilterSTDF]: No logger object in data_dict. It is abnormal behavior as logger object is created by default. Please check if some script is removing the logger key in data_dict.'); return

    # imports
    from algo.non_nn.STDF import calc_STDF_params, save_STDF_params, load_STDF_params, make_STDF_filter
    from algo.utils import gather_point_clouds, skip_frames, combine_gathers
    from pcd.utils import get_fixed_sized_point_cloud
    
    # algo name
    algo_name = 'BGFilterSTDF'

    # dict keys
    query_frames_key = f'{algo_name}_query_frames'
    skip_frames_key = f'{algo_name}_skip_frames'
    params_key = f'{algo_name}_params'
    filter_key = f'{algo_name}_filter'
    filter_loaded_key = f'{algo_name}_filter_loaded'

    # get params
    params = cfg_dict['proc']['lidar']['BGFilterSTDF'].copy()
    live_editable_params = ['background_density_threshold'] # list of params that can be live edited and do not require re-computation of filter
    
    # generate keys for query and skip frames
    all_query_frames_keys = [f'{query_frames_key}_{i}' for i in range(params['number_of_frame_gather_iters'])]
    all_skip_frames_keys = [f'{skip_frames_key}_{i}' for i in range(params['number_of_skip_frames_after_each_iter'])]

    # load filter if exists
    if filter_loaded_key not in data_dict and params['load_filter']:
        # add params to data_dict
        data_dict[params_key] = params
        
        filter_params = load_STDF_params(params['filter_path'])
        if filter_params:
            data_dict[filter_key] = lambda pcd, threshold: make_STDF_filter(pcd, threshold, **filter_params)
            data_dict[filter_loaded_key] = True
            logger.log(f'[algo->lidar.py->BGFilterSTDF]: Filter loaded from {params["filter_path"]}.', Logger.INFO)
            data_dict['BGFilterSTDF_set'] = True
        else:
            data_dict[filter_loaded_key] = False
            logger.log(f'[algo->lidar.py->BGFilterSTDF]: Failed to load filter from {params["filter_path"]}. Calculating ...', Logger.WARNING)
            data_dict['BGFilterSTDF_set'] = False
    
    # generate filter if not exists
    if filter_key not in data_dict:
        data_dict[params_key] = params
        # gather frames
        for i in range(params['number_of_frame_gather_iters']):
            gathering_done = gather_point_clouds(data_dict, cfg_dict, all_query_frames_keys[i], params['number_of_frames_in_each_gather_iter'])
            if not gathering_done: return
            skipping_done = skip_frames(data_dict, cfg_dict, all_skip_frames_keys[i], params['number_of_skip_frames_after_each_iter'])
            if not skipping_done: return
            
        # combine gathered frames
        combine_gathers(data_dict, cfg_dict, query_frames_key, all_query_frames_keys)
        assert len(data_dict[query_frames_key]) == params['number_of_frame_gather_iters'] * params['number_of_frames_in_each_gather_iter']
        
        # generate filter
        logger.log(f'[algo->lidar.py->BGFilterSTDF]: Generating filter', Logger.INFO)
        data_dict[query_frames_key] = [get_fixed_sized_point_cloud(frame, params['number_of_points_per_frame']) for frame in data_dict[query_frames_key]]
        filter_params = calc_STDF_params(data_dict[query_frames_key], params['lidar_range_in_unit_length'], params['bins_per_unit_length'])
        data_dict[filter_key] = lambda pcd, threshold: make_STDF_filter(pcd, threshold, **filter_params)
        # save filter
        filter_saved_path = save_STDF_params(filter_params, params['filter_path'])
        logger.log(f'[algo->lidar.py->BGFilterSTDF]: Filter generated and saved in {filter_saved_path}.', Logger.INFO)
        data_dict['BGFilterSTDF_set'] = True
    else:
        # recompute filter if non-live-editable params are changed
        condition = False
        for key in params:
            if key in live_editable_params: continue
            condition = condition or data_dict[params_key][key] != params[key]
        # remove all algo keys if params are changed so that filter is re-computed on next call
        if condition:
            keys_to_remove = [key for key in data_dict.keys() if key.startswith(algo_name)]
            for key in keys_to_remove: data_dict.pop(key)
            data_dict['BGFilterSTDF_set'] = False
            return
    
    # if filter exists, apply it
    if filter_key in data_dict:
        # apply filter
        data_dict['current_point_cloud_numpy'] = get_fixed_sized_point_cloud(data_dict['current_point_cloud_numpy'], params['number_of_points_per_frame'])
        data_dict['current_point_cloud_numpy'] = data_dict['current_point_cloud_numpy'][data_dict[filter_key](data_dict['current_point_cloud_numpy'], params['background_density_threshold'])]

def Clusterer_TEPP_DBSCAN(data_dict: dict, cfg_dict: dict):
    """
    Perform TEPP DBSCAN clustering on the current point cloud.

    Theoretically Efficient and Practical Parallel DBSCAN
    https://dl.acm.org/doi/10.1145/3318464.3380582

    Args:
        data_dict (dict): A dictionary containing data for the algorithm.
        cfg_dict (dict): A dictionary containing configuration parameters.

    Returns:
        None
    """
    # get logger object from data_dict
    if 'logger' in data_dict: logger:Logger = data_dict['logger']
    else: print('[algo->lidar.py->Clusterer_TEPP_DBSCAN]: No logger object in data_dict. It is abnormal behavior as logger object is created by default. Please check if some script is removing the logger key in data_dict.'); return
    
    # clear previous key set
    if 'Clusterer_TEPP_DBSCAN_set' in data_dict: data_dict.pop('Clusterer_TEPP_DBSCAN_set')
    
    # check if required data is present in data_dict
    if "current_point_cloud_numpy" not in data_dict:
        logger.log('[algo->lidar.py->Clusterer_TEPP_DBSCAN]: current_point_cloud_numpy not found in data_dict', Logger.ERROR)
        return
    if cfg_dict['proc']['lidar']['Clusterer_TEPP_DBSCAN']['activate_on_key_set'] not in data_dict: return
    
    try: DBSCAN = __import__('dbscan', fromlist=['DBSCAN']).DBSCAN
    except:
        logger.log('[algo->lidar.py->Clusterer_TEPP_DBSCAN]: dbscan package not found, please install the `dbscan` package using `pip install dbscan`.', Logger.ERROR)
        return
    
    # get params
    params = cfg_dict['proc']['lidar']['Clusterer_TEPP_DBSCAN']

    # perform clustering
    labels, _ = DBSCAN(data_dict['current_point_cloud_numpy'], params['eps'], params['min_samples'])
    
    # create 'current_label_list' if not exists
    if 'current_label_list' not in data_dict:
        data_dict['current_label_list'] = []
        logger.log('[algo->lidar.py->Clusterer_TEPP_DBSCAN]: current_label_list not found in data_dict, creating a new one', Logger.DEBUG)
    
    # update label list
    for label in np.unique(labels):
        if label == -1: continue
        data_dict['current_label_list'].append({'lidar_cluster': {'point_indices': labels == label}})
    data_dict['Clusterer_TEPP_DBSCAN_set'] = True

def O3D_DBSCAN(data_dict: dict, cfg_dict: dict):
    """
    DBSCAN clustering available in Open3D library.

    Args:
        data_dict (dict): A dictionary containing data for processing.
        cfg_dict (dict): A dictionary containing configuration parameters.

    Returns:
        None
    """
    # get logger object from data_dict
    if 'logger' in data_dict: logger:Logger = data_dict['logger']
    else: print('[algo->lidar.py->O3D_DBSCAN]: No logger object in data_dict. It is abnormal behavior as logger object is created by default. Please check if some script is removing the logger key in data_dict.'); return
    
    # clear previous key set
    if 'O3D_DBSCAN_set' in data_dict: data_dict.pop('O3D_DBSCAN_set')
    
    # check if required data is present in data_dict
    if "current_point_cloud_numpy" not in data_dict:
        logger.log('[algo->lidar.py->O3D_DBSCAN]: current_point_cloud_numpy not found in data_dict', Logger.ERROR)
        return
    if cfg_dict['proc']['lidar']['O3D_DBSCAN']['activate_on_key_set'] not in data_dict: return
    
    # get params
    params = cfg_dict['proc']['lidar']['O3D_DBSCAN']

    # perform clustering
    import open3d as o3d
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(data_dict['current_point_cloud_numpy'][:, :3])
    labels = np.array(pcd.cluster_dbscan(eps=params['eps'], min_points=params['min_samples'], print_progress=False))

    # create 'current_label_list' if not exists
    if 'current_label_list' not in data_dict:
        data_dict['current_label_list'] = []
        logger.log('[algo->lidar.py->O3D_DBSCAN]: current_label_list not found in data_dict, creating a new one', Logger.DEBUG)

    # update label list
    for label in np.unique(labels):
        if label == -1: continue
        data_dict['current_label_list'].append({'lidar_cluster': {'point_indices': labels == label}})
    data_dict['O3D_DBSCAN_set'] = True

def Cluster2Object(data_dict: dict, cfg_dict: dict):
    """
    Converts lidar clusters to object labels and adds them to the current label list.

    Args:
        data_dict (dict): A dictionary containing data for processing.
        cfg_dict (dict): A dictionary containing configuration parameters.

    Returns:
        None
    """

    # get logger object from data_dict
    if 'logger' in data_dict: logger:Logger = data_dict['logger']
    else: print('[algo->lidar.py->Cluster2Object]: No logger object in data_dict. It is abnormal behavior as logger object is created by default. Please check if some script is removing the logger key in data_dict.'); return

    # check if required data is present in data_dict
    if "current_point_cloud_numpy" not in data_dict:
        logger.log('[algo->lidar.py->Cluster2Object]: current_point_cloud_numpy not found in data_dict', Logger.ERROR)
        return
    if 'current_label_list' not in data_dict:
        logger.log('[algo->lidar.py->Cluster2Object]: current_label_list not found in data_dict', Logger.ERROR)
        return
    if cfg_dict['proc']['lidar']['Cluster2Object']['activate_on_key_set'] not in data_dict: return

    # imports
    import open3d as o3d

    # algo name and keys used in algo
    algo_name = 'Cluster2Object'
    
    # get params
    params = cfg_dict['proc']['lidar'][algo_name]
    
    for label_dict in data_dict['current_label_list']:
        if 'lidar_cluster' not in label_dict: continue
        
        # get cluster
        lidar_cluster_dict = label_dict['lidar_cluster']
        point_indices = lidar_cluster_dict['point_indices']
        cluster = data_dict['current_point_cloud_numpy'][point_indices][:, :3]

        # get an axis-aligned bounding box
        try:
            bbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(cluster))
            xyz_center = bbox.get_center().astype(np.float32)
            xyz_extent = (bbox.get_max_bound() - bbox.get_min_bound()).astype(np.float32)
            xyz_euler_angles = np.array([0, 0, 0], dtype=np.float32)
        except: continue

        # get bbox params
        base_length = xyz_extent[0] if xyz_extent[0] > xyz_extent[1] else xyz_extent[1]
        height = xyz_extent[2]
        foot = xyz_center[2] - height / 2.0

        # foot can't be too heigh
        if foot > params['ground_level'] + params['max_foot_level']: continue

        # very small height objects are noise
        if height < params['min_height']: continue

        # classify the object based on size constraints
        selected_obj_class = None
        for obj_class, size_constraint in params['size_constraints'].items():
            base_length_range = size_constraint['base_length']
            height_range = size_constraint['height']
            if base_length_range[0] <= base_length < base_length_range[1]: # and height_range[0] <= height < height_range[1]:
                selected_obj_class = obj_class
                break
        
        # if can't classify, skip the cluster
        if selected_obj_class == None: continue

        # get color of the cluster
        if selected_obj_class in params['class_colors']:
            rgb_color = np.array(params['class_colors'][selected_obj_class], dtype=np.float32)
        else:
            logger.log(f'[algo->lidar.py->Cluster2Object]: class color not found for class: {selected_obj_class}, using default color', Logger.WARNING)
            rgb_color = np.array([0, 0, 0], dtype=np.float32)

        # generate label
        label = dict()
        label['class'] = selected_obj_class

        # get orientation of the object
        if selected_obj_class in params['classes_require_orientation']:
            try:
                bbox_o = o3d.geometry.OrientedBoundingBox.create_from_points(o3d.utility.Vector3dVector(cluster))
                xyz_center = bbox_o.get_center().astype(np.float32)
                xyz_extent = bbox_o.extent.astype(np.float32)
                rotation_matrix = bbox_o.R.astype(np.float32)
                xyz_euler_angles = np.array(
                    [0.0, # np.arctan2(rotation_matrix[2,1], rotation_matrix[2,2]),
                    0.0, # np.arctan2(-rotation_matrix[2,0], np.sqrt(rotation_matrix[2,1]**2 + rotation_matrix[2,2]**2)),
                    np.arctan2(rotation_matrix[1,0], rotation_matrix[0,0])], dtype=np.float32)
            except: pass
            
        # complete label dict and add to label list
        label['bbox_3d'] = {'xyz_center': xyz_center, 'xyz_extent': xyz_extent, 'xyz_euler_angles': xyz_euler_angles, 'rgb_color': rgb_color, 'predicted': True, 'added_by': algo_name}
        data_dict['current_label_list'].append(label)

def NNCluster2Object(data_dict: dict, cfg_dict: dict):
    """
    Converts clustered lidar data into object labels using a neural network model.

    Args:
        data_dict (dict): A dictionary containing the required data for processing.
        cfg_dict (dict): A dictionary containing the configuration parameters.

    Returns:
        None
    """
    # get logger object from data_dict
    if 'logger' in data_dict: logger:Logger = data_dict['logger']
    else: print('[algo->lidar.py->NNCluster2Object]: No logger object in data_dict. It is abnormal behavior as logger object is created by default. Please check if some script is removing the logger key in data_dict.'); return

    # algo name and keys used in algo
    algo_name = 'NNCluster2Object'
    model_key = f'{algo_name}_model'
    class_ids_key = f'{algo_name}_class_ids'
    
    # check if required data is present in data_dict
    if "current_point_cloud_numpy" not in data_dict:
        logger.log('[algo->lidar.py->NNCluster2Object]: current_point_cloud_numpy not found in data_dict', Logger.ERROR)
        return
    if 'current_label_list' not in data_dict:
        logger.log('[algo->lidar.py->NNCluster2Object]: current_label_list not found in data_dict', Logger.ERROR)
        return
    if cfg_dict['proc']['lidar']['NNCluster2Object']['activate_on_key_set'] not in data_dict: return
    
    # get params
    import os
    params = cfg_dict['proc']['lidar'][algo_name]
    checkpoint_path = os.path.join(data_dict['root_path'], params['point_nn_bbox_est_checkpoint_path'])
    if os.path.exists(checkpoint_path) == False:
        logger.log(f'[algo->lidar.py->NNCluster2Object]: checkpoint path {checkpoint_path} does not exist', Logger.ERROR)
        return
    num_points = params['point_nn_bbox_est_num_points']
    
    # load model
    try: torch = __import__('torch')
    except:
        logger.log('[algo->lidar.py->NNCluster2Object]: torch package not found, please install the `torch` package using `pip install torch`.', Logger.ERROR)
        return
    device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
    import open3d as o3d
    
    if model_key not in data_dict:
        try:
            from algo.nn.PointNN.models.bbox_est import BBoxEst
            model = BBoxEst(num_points, 512).to(device)
            model.load_state_dict(torch.load(checkpoint_path))
            model.eval()
            data_dict[model_key] = model
            from algo.nn.PointNN.ds_loaders.pcdet import PerObjectDataLoader
            data_dict[class_ids_key] = PerObjectDataLoader.class_id_to_key
        except:
            logger.log('[algo->lidar.py->NNCluster2Object]: failed to load model', Logger.ERROR)
            return
    
    # process clusters
    with torch.no_grad():
        for label_dict in data_dict['current_label_list']:
            if 'lidar_cluster' not in label_dict: continue
            
            # get cluster
            lidar_cluster_dict = label_dict['lidar_cluster']
            point_indices = lidar_cluster_dict['point_indices']
            cluster = data_dict['current_point_cloud_numpy'][point_indices][:, :3]

            # this bbox is used to get the orientation of the object
            orienter_box = None
            try: orienter_box = o3d.geometry.OrientedBoundingBox.create_from_points(o3d.utility.Vector3dVector(cluster))
            except:
                logger.log('[algo->lidar.py->NNCluster2Object]: failed to create an OrientedBoundingBox, skipping ...', Logger.WARNING)
                continue
            
            # prepare cluster for model
            cluster_mean = np.mean(cluster, axis=0)
            cluster -= cluster_mean # subtract mean so that the cluster is centered at origin, model is trained on centered clusters
            
            if cluster.shape[0] < num_points: cluster = np.pad(cluster, ((0, num_points-cluster.shape[0]), (0,0)), mode='constant', constant_values=0)
            elif cluster.shape[0] > num_points: cluster = cluster[:num_points]
            
            cluster = cluster[np.newaxis, ...]
            cluster = torch.tensor(cluster, dtype=torch.float32).transpose(2,1).to(device)
            
            # get predictions and extract center, extent, orientation, and class
            xyz_center, xyz_extent, ry, obj_class, crit_idxs, A_feat = data_dict[model_key](cluster)
            xyz_center = xyz_center.detach().cpu().numpy()[0]
            xyz_extent = xyz_extent.detach().cpu().numpy()[0]
            ry = ry.detach().cpu().numpy()[0][0]
            obj_class = torch.argmax(obj_class[0]).detach().cpu().numpy()
            obj_class = data_dict[class_ids_key][obj_class.item()]
            
            # add cluster mean back to the center to get the actual center
            xyz_center += cluster_mean
            
            # apply orientation from orienter box calculated above, this is much more accurate than the orientation predicted by the model
            if orienter_box != None:
                xyz_euler_angles = np.array(
                    [0.0,
                     0.0,
                     np.arctan2(orienter_box.R[1,0], orienter_box.R[0,0])], dtype=np.float32)
            # if orienter box is not available, use the predicted orientation
            else: xyz_euler_angles = np.array([0, 0, ry], dtype=np.float32)
            
            # select color for the object based on the class
            if obj_class in cfg_dict['proc']['lidar']['NNCluster2Object']['class_colors']:
                rgb_color = np.array(cfg_dict['proc']['lidar']['NNCluster2Object']['class_colors'][obj_class], dtype=np.float32)
            else:
                logger.log(f'[algo->lidar.py->NNCluster2Object]: class color not found for class {obj_class}, using default color', Logger.WARNING)
                rgb_color = np.array([0, 0, 0], dtype=np.float32)
            
            # create label dict
            label = dict()
            label['class'] = obj_class
            label['bbox_3d'] = {'xyz_center': xyz_center, 'xyz_extent': xyz_extent, 'xyz_euler_angles': xyz_euler_angles, 'rgb_color': rgb_color, 'predicted': True, 'added_by': algo_name}
            
            # add label to the label list
            data_dict['current_label_list'].append(label)

def PointPillarDetection(data_dict: dict, cfg_dict: dict):
    """
    Perform object detection using the PointPillar algorithm.

    Original Paper:
    Lang, Alex H., et al. "Pointpillars: Fast encoders for object detection from point clouds."
    Proceedings of the IEEE/CVF conference on computer vision and pattern recognition. 2019.

    The implementation used here is from "https://github.com/zhulf0804/PointPillars".
    
    Args:
        data_dict (dict): A dictionary containing the required data for processing.
        cfg_dict (dict): A dictionary containing the configuration parameters.

    Returns:
        None
    """
    # get logger object from data_dict
    if 'logger' in data_dict: logger:Logger = data_dict['logger']
    else: print('[algo->lidar.py->PointPillarDetection]: No logger object in data_dict. It is abnormal behavior as logger object is created by default. Please check if some script is removing the logger key in data_dict.'); return
    
    # check if required data is present in data_dict
    if "current_point_cloud_numpy" not in data_dict:
        logger.log('[algo->lidar.py->PointPillarDetection]: current_point_cloud_numpy not found in data_dict', Logger.ERROR)
        return
    
    if cfg_dict['proc']['lidar']['PointPillarDetection']['activate_on_key_set'] not in data_dict: return
    

    # algo name and keys used in algo
    algo_name = 'PointPillarDetection'
    model_key = f'{algo_name}_model'
    class_ids_key = f'{algo_name}_class_ids'
    ids_class_key = f'{algo_name}_ids_class'
    pcd_limit_range_key = f'{algo_name}_pcd_limit_range'
    class_color_key = f'{algo_name}_class_color'
    
    # get params
    params = cfg_dict['proc']['lidar'][algo_name]
    
    # imports
    import torch

    # create model if not exists
    if 'PointPillarDetection_model' not in data_dict:
        data_dict[class_ids_key] = {'Pedestrian': 0, 'Cyclist': 1, 'Car': 2}
        data_dict[ids_class_key] = {v:k for k, v in data_dict[class_ids_key].items()}
        min_xyz = cfg_dict['proc']['lidar']['crop']['min_xyz']
        max_xyz = cfg_dict['proc']['lidar']['crop']['max_xyz']
        data_dict[pcd_limit_range_key] = np.array(min_xyz + max_xyz, dtype=np.float32)
        data_dict[class_color_key] = {'Pedestrian': [1, 0, 0], 'Cyclist': [0, 0, 1], 'Car': [0, 1, 0]}

        path = os.path.abspath(os.path.join(data_dict['root_path'], params['path_to_github_repo']))
        if path not in sys.path: sys.path.append(path)
        from algo.nn.PointPillars.model import PointPillars

        data_dict[model_key] = PointPillars(nclasses=len(data_dict[class_ids_key]))
        model_path = os.path.abspath(os.path.join(path, 'pretrained/epoch_160.pth'))
        if torch.cuda.is_available():
            data_dict[model_key].cuda()
            data_dict[model_key].load_state_dict(torch.load(model_path))
        else:
             data_dict[model_key].load_state_dict(torch.load(model_path), map_location=torch.device('cpu'))

        data_dict[model_key].eval()
    
    with torch.no_grad():
        pc_torch = torch.from_numpy(data_dict['current_point_cloud_numpy'])
        if torch.cuda.is_available(): pc_torch = pc_torch.cuda()
        result_filter = data_dict[model_key](batched_pts=[pc_torch], mode='test')[0]

    bbox_3des = result_filter['bbox_3des']
    labels, scores = result_filter['labels'], result_filter['scores']

    for i in range(len(labels)):
        if scores[i] < params['score_threshold']: continue
        bbox_3d = bbox_3des[i]
        xyz_center = bbox_3d[:3]
        xyz_center[2] += bbox_3d[5] / 2
        xyz_extent = bbox_3d[3:6]
        xyz_euler_angles = np.array([0, 0, bbox_3d[6]], dtype=np.float32)
        obj_class = data_dict[ids_class_key][labels[i]]
        rgb_color = np.array(data_dict[class_color_key][obj_class], dtype=np.float32)

        label = dict()
        label['class'] = obj_class
        label['bbox_3d'] = {'xyz_center': xyz_center, 'xyz_extent': xyz_extent, 'xyz_euler_angles': xyz_euler_angles, 'rgb_color': rgb_color, 'predicted': True, 'added_by': algo_name}


        if 'current_label_list' not in data_dict: data_dict['current_label_list'] = []
        data_dict['current_label_list'].append(label)

def gen_bbox_2d(data_dict: dict, cfg_dict: dict):
    """
    Generate 2D bounding boxes from 3D bounding boxes.

    Args:
        data_dict (dict): A dictionary containing the required data for processing.
        cfg_dict (dict): A dictionary containing the configuration parameters.

    Returns:
        None
    """
    # get logger object from data_dict
    if 'logger' in data_dict: logger:Logger = data_dict['logger']
    else: print('[algo->lidar.py->gen_bbox_2d]: No logger object in data_dict. It is abnormal behavior as logger object is created by default. Please check if some script is removing the logger key in data_dict.'); return
    
    # check if required data is present in data_dict
    if 'current_label_list' not in data_dict:
        logger.log('[algo->lidar.py->gen_bbox_2d]: current_label_list not found in data_dict', Logger.ERROR)
        return
    if 'current_calib_data' not in data_dict:
        logger.log('[algo->lidar.py->gen_bbox_2d]: current_calib_data not found in data_dict', Logger.ERROR)
        return
    
    # imports
    import open3d as o3d
    
    # get params
    params = cfg_dict['proc']['lidar']['gen_bbox_2d']
    
    # get calibration data
    calib_data = data_dict['current_calib_data']
    
    # get 2D bounding boxes from 3D bounding boxes
    for label_dict in data_dict['current_label_list']:
        if 'bbox_3d' not in label_dict: continue
        if 'bbox_2d' in label_dict: continue
        
        # get 3D bounding box data
        bbox_3d = label_dict['bbox_3d']
        xyz_center = bbox_3d['xyz_center']
        xyz_extent = bbox_3d['xyz_extent']
        xyz_euler_angles = bbox_3d['xyz_euler_angles']
        
        # get 3D bounding box corners
        rotation_matrix = o3d.geometry.OrientedBoundingBox.get_rotation_matrix_from_xyz(xyz_euler_angles)
        o3d_bbox_3d = o3d.geometry.OrientedBoundingBox(xyz_center, rotation_matrix, xyz_extent)
        corners_3d = np.array(o3d_bbox_3d.get_box_points()).astype(np.float32)
        corners_3d = np.concatenate([corners_3d, np.ones((corners_3d.shape[0], 1))], axis=1) # make it x,y,z,1
        
        # project 3D bounding box corners onto the image plane
        corners_2d = calib_data['P2'] @ calib_data['R0_rect'] @ calib_data['Tr_velo_to_cam'] @ corners_3d.T
        corners_2d = corners_2d[:2] / corners_2d[2]
        corners_2d = corners_2d.T
        
        # get 2D bounding box from 2D corners
        min_xy = np.min(corners_2d, axis=0)
        max_xy = np.max(corners_2d, axis=0)
        
        xy_center = min_xy + (max_xy - min_xy) / 2.0
        xy_extent = max_xy - min_xy
        rgb_color = bbox_3d['rgb_color']
        predicted = bbox_3d['predicted']
        added_by = bbox_3d['added_by'] + '_2d'
        
        # add 2D bounding box to the label dict
        label_dict['bbox_2d'] = {'xy_center': xy_center, 'xy_extent': xy_extent, 'rgb_color': rgb_color, 'predicted': predicted, 'added_by': added_by, 'visualize': params['visualize']}