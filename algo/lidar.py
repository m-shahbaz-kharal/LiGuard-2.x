import os
import numpy as np

from algo.utils import gather_point_clouds, skip_frames, combine_gathers
from pcd.utils import get_fixed_sized_point_cloud

def crop(data_dict: dict, cfg_dict: dict):
    if "current_point_cloud_numpy" not in data_dict: return
    pcd = data_dict['current_point_cloud_numpy']
    min_xyz = cfg_dict['proc']['lidar']['crop']['min_xyz']
    max_xyz = cfg_dict['proc']['lidar']['crop']['max_xyz']
    x_condition = np.logical_and(min_xyz[0] <= pcd[:, 0], pcd[:, 0] <= max_xyz[0])
    y_condition = np.logical_and(min_xyz[1] <= pcd[:, 1], pcd[:, 1] <= max_xyz[1])
    z_condition = np.logical_and(min_xyz[2] <= pcd[:, 2], pcd[:, 2] <= max_xyz[2])
    data_dict['current_point_cloud_numpy'] = pcd[x_condition & y_condition & z_condition]
    
def project_image_pixel_colors(data_dict: dict, cfg_dict: dict):
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
    # algo name
    algo_name = 'BGFilterDHistDPP'

    # dict keys
    query_frames_key = f'{algo_name}_query_frames'
    skip_frames_key = f'{algo_name}_skip_frames'
    params_key = f'{algo_name}_params'
    filter_key = f'{algo_name}_filter'

    # get params
    params = cfg_dict['proc']['lidar']['BGFilterDHistDPP'].copy()
    live_editable_params = ['background_density_threshold'] # list of params that can be live edited and do not require re-computation of filter
    
    # generate keys for query and skip frames
    all_query_frames_keys = [f'{query_frames_key}_{i}' for i in range(params['number_of_frame_gather_iters'])]
    all_skip_frames_keys = [f'{skip_frames_key}_{i}' for i in range(params['number_of_skip_frames_after_each_iter'])]

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
        from algo.non_nn.DHistDPP import DHistDPP
        data_dict[query_frames_key] = [get_fixed_sized_point_cloud(frame, params['number_of_points_per_frame']) for frame in data_dict[query_frames_key]]
        data_dict[filter_key] = DHistDPP(data_dict[query_frames_key], params['number_of_points_per_frame'], params['lidar_range_in_unit_length'], params['bins_per_unit_length'])
    
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
            return
    
    # if filter exists, apply it
    if filter_key in data_dict:
        data_dict['current_point_cloud_numpy'] = get_fixed_sized_point_cloud(data_dict['current_point_cloud_numpy'], params['number_of_points_per_frame'])
        data_dict['current_point_cloud_numpy'] = data_dict['current_point_cloud_numpy'][data_dict[filter_key](data_dict['current_point_cloud_numpy'], params['background_density_threshold'])]

def BGFilterSTDF(data_dict: dict, cfg_dict: dict):
    # algo name
    algo_name = 'BGFilterSTDF'

    # dict keys
    query_frames_key = f'{algo_name}_query_frames'
    skip_frames_key = f'{algo_name}_skip_frames'
    params_key = f'{algo_name}_params'
    filter_key = f'{algo_name}_filter'

    # get params
    params = cfg_dict['proc']['lidar']['BGFilterSTDF'].copy()
    live_editable_params = ['background_density_threshold'] # list of params that can be live edited and do not require re-computation of filter
    
    # generate keys for query and skip frames
    all_query_frames_keys = [f'{query_frames_key}_{i}' for i in range(params['number_of_frame_gather_iters'])]
    all_skip_frames_keys = [f'{skip_frames_key}_{i}' for i in range(params['number_of_skip_frames_after_each_iter'])]

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
        from algo.non_nn.STDF import STDF
        data_dict[query_frames_key] = [get_fixed_sized_point_cloud(frame, params['number_of_points_per_frame']) for frame in data_dict[query_frames_key]]
        data_dict[filter_key] = STDF(data_dict[query_frames_key], params['lidar_range_in_unit_length'], params['bins_per_unit_length'])
    
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
            return
    
    # if filter exists, apply it
    if filter_key in data_dict:
        data_dict['current_point_cloud_numpy'] = get_fixed_sized_point_cloud(data_dict['current_point_cloud_numpy'], params['number_of_points_per_frame'])
        data_dict['current_point_cloud_numpy'] = data_dict['current_point_cloud_numpy'][data_dict[filter_key](data_dict['current_point_cloud_numpy'], params['background_density_threshold'])]

def Clusterer_TEPP_DBSCAN(data_dict: dict, cfg_dict: dict):
    """
        Theoretically Efficient and Practical Parallel DBSCAN
        https://dl.acm.org/doi/10.1145/3318464.3380582
        """
    
    if "current_point_cloud_numpy" not in data_dict: return
    if cfg_dict['proc']['lidar']['Clusterer_TEPP_DBSCAN']['activate_on_key_set'] not in data_dict: return
    
    try: DBSCAN = __import__('dbscan', fromlist=['DBSCAN']).DBSCAN
    except: raise ImportError("Please install the `dbscan` package using `pip install dbscan`.")
    
    params = cfg_dict['proc']['lidar']['Clusterer_TEPP_DBSCAN']

    cluster_label_for_each_point_index, _ = DBSCAN(data_dict['current_point_cloud_numpy'], params['eps'], params['min_samples'])
    point_indices_for_each_cluster_label = [label == cluster_label_for_each_point_index for label in np.unique(cluster_label_for_each_point_index)]

    if 'current_label_list' not in data_dict: data_dict['current_label_list'] = []
    for point_indices in point_indices_for_each_cluster_label:
        data_dict['current_label_list'].append({'lidar_cluster': {'point_indices': point_indices}})

def NNCluster2Object(data_dict: dict, cfg_dict: dict):
    algo_name = 'NNCluster2Object'
    model_key = f'{algo_name}_model'
    class_ids_key = f'{algo_name}_class_ids'
    
    if "current_point_cloud_numpy" not in data_dict: return
    if 'current_label_list' not in data_dict: return
    if cfg_dict['proc']['lidar']['NNCluster2Object']['activate_on_key_set'] not in data_dict: return
    
    checkpoint_path = cfg_dict['proc']['lidar']['NNCluster2Object']['point_nn_bbox_est_checkpoint_path']
    assert os.path.exists(checkpoint_path), f"The checkpoint path: {checkpoint_path} doesn't exist."
    num_points = cfg_dict['proc']['lidar']['NNCluster2Object']['point_nn_bbox_est_num_points']
    
    try: torch = __import__('torch')
    except: raise ImportError("Please install the `pytorch` package using `pip install torch`.")
    device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
    
    if model_key not in data_dict:
        from algo.nn.PointNN.models.bbox_est import BBoxEst
        model = BBoxEst(num_points, 512).to(device)
        model.load_state_dict(torch.load(checkpoint_path))
        model.eval()
        data_dict[model_key] = model
        from algo.nn.PointNN.ds_loaders.pcdet import PerObjectDataLoader
        data_dict[class_ids_key] = PerObjectDataLoader.class_id_to_key
    
    with torch.no_grad():
        for label_dict in data_dict['current_label_list']:
            if 'lidar_cluster' not in label_dict: continue
            
            lidar_cluster_dict = label_dict['lidar_cluster']
            point_indices = lidar_cluster_dict['point_indices']
            cluster = data_dict['current_point_cloud_numpy'][point_indices][:, :3]
            
            cluster_mean = np.mean(cluster, axis=0)
            cluster -= cluster_mean
            
            if cluster.shape[0] < num_points: cluster = np.pad(cluster, ((0, num_points-cluster.shape[0]), (0,0)), mode='constant', constant_values=0)
            elif cluster.shape[0] > num_points: cluster = cluster[:num_points]
            
            cluster = cluster[np.newaxis, ...]
            cluster = torch.tensor(cluster, dtype=torch.float32).transpose(2,1).to(device)
            
            lidar_xyz_center, lidar_xyz_extent, ry, obj_class, crit_idxs, A_feat = data_dict[model_key](cluster)
            lidar_xyz_center = lidar_xyz_center.detach().cpu().numpy()[0]
            lidar_xyz_extent = lidar_xyz_extent.detach().cpu().numpy()[0]
            ry = ry.detach().cpu().numpy()[0][0]
            obj_class = torch.argmax(obj_class[0]).detach().cpu().numpy()
            obj_class = data_dict[class_ids_key][obj_class.item()]
            
            lidar_xyz_center += cluster_mean
            lidar_xyz_euler_angles = [0, 0, ry]
            if obj_class in cfg_dict['proc']['lidar']['NNCluster2Object']['class_colors']:
                lidar_bbox_color = cfg_dict['proc']['lidar']['NNCluster2Object']['class_colors'][obj_class]
            else:
                lidar_bbox_color = [0, 0, 0]
            
            label = dict()
            label['class'] = obj_class
            label['lidar_bbox'] = {'lidar_xyz_center': lidar_xyz_center, 'lidar_xyz_extent': lidar_xyz_extent, 'lidar_xyz_euler_angles': lidar_xyz_euler_angles, 'rgb_bbox_color': lidar_bbox_color}
            
            data_dict['current_label_list'].append(label)