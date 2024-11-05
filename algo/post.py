import inspect
from liguard.gui.config_gui import get_abs_path
from liguard.gui.logger_gui import Logger
from liguard.algo.utils import AlgoType, make_key, get_algo_params
algo_type = AlgoType.post

def Fuse2DPredictedBBoxes(data_dict: dict, cfg_dict: dict, logger: Logger):
    """
    Fuses bbox_2d information among ModalityA and ModalityB.

    Requirements:
    - bbox_2d in current_label_list, predicted by an arbitrary ModalityA's <algo_src>.
    - bbox_2d in current_label_list, predicted by an arbitrary ModalityB's <algo_src>.

    Operation:
    - It uses KD-Tree to find the nearest bbox_2d from ModalityA's <algo_src> and ModalityB's <algo_src>.
    - It assigns the class based on the highest confidence score.
    - It assigns the depth based on the highest confidence score.

    Args:
        data_dict (dict): A dictionary containing the required data.
        cfg_dict (dict): A dictionary containing configuration parameters.
        logger (gui.logger_gui.Logger): A logger object for logging messages and errors in GUI.
    """
    # get name and params
    algo_name = inspect.stack()[0].function
    params = get_algo_params(cfg_dict, algo_type, algo_name, logger)

    # Check if required data is present in data_dict
    if 'current_label_list' not in data_dict:
        logger.log('current_label_list not found in data_dict', Logger.ERROR)
        return
    if 'current_calib_data' not in data_dict:
        logger.log('current_calib_data not found in data_dict', Logger.ERROR)
    
    # imports
    import numpy as np
    from scipy.spatial import KDTree

    # ---------------------- KDTree Matching ---------------------- #
    # Extract info from bbox_2d from modality a and b
    mod_a_list, mod_b_list = [], []
    mod_a_idx, mod_b_idx = [], []
    for i, label_dict in enumerate(data_dict['current_label_list']):
        if 'bbox_2d' not in label_dict: continue
        # get bbox_2d dict
        bbox_2d_dict = label_dict['bbox_2d']
        
        # add xy_center to mod_a_list or mod_b_list
        if bbox_2d_dict['added_by'] == params['ModalityA']['algo_src']:
            mod_a_list.append(bbox_2d_dict['xy_center'])
            mod_a_idx.append(i)
            if params['ModalityA']['img_extras']['show_fusion_dot']:
                label_dict.setdefault('extras', {}).setdefault('img_visualizer', {}).setdefault(algo_name, {}).setdefault('fusion_dot', {'cv2_attr': 'circle', 'params': {'center': tuple(map(int, bbox_2d_dict['xy_center'])), 'radius': 5, 'color': [255, 0, 0], 'thickness': -1}})
        elif bbox_2d_dict['added_by'] == params['ModalityB']['algo_src']:
            mod_b_list.append(bbox_2d_dict['xy_center'])
            mod_b_idx.append(i)
            if params['ModalityB']['img_extras']['show_fusion_dot']:
                label_dict.setdefault('extras', {}).setdefault('img_visualizer', {}).setdefault(algo_name, {}).setdefault('fusion_dot', {'cv2_attr': 'circle', 'params': {'center': tuple(map(int, bbox_2d_dict['xy_center'])), 'radius': 5, 'color': [255, 0, 0], 'thickness': -1}})
                

    if len(mod_a_list) == 0:
        logger.log('No bbox_2d found from ModalityA\'s <algo_src>', Logger.DEBUG)
        return
    if len(mod_b_list) == 0:
        logger.log('No bbox_2d found from ModalityB\'s <algo_src>', Logger.DEBUG)
        return

    # Create a KDTree for xy_center points of bbox_2d from ModalityA <algo_src>
    kd_tree = KDTree(mod_a_list)

    # Calculate the cost matrix based on the smallest distances
    cost_matrix = np.zeros((len(mod_a_list), len(mod_b_list)))

    for i, point in enumerate(mod_b_list):
        distances, indices = kd_tree.query(point, k=len(mod_a_list))
        cost_matrix[indices, i] = distances

    # find the optimal matching
    row_indices = np.arange(len(mod_a_list))
    col_indices = cost_matrix.argmin(axis=1)

    # Output the matching result
    matches_kd_tree = list(zip(row_indices, col_indices))
    already_matched = []
    # ---------------------- KDTree Matching ---------------------- #
    for i, j in matches_kd_tree:
        # check if the current bbox is already matched and cost is within the threshold
        if j in already_matched: continue
        elif cost_matrix[i, j] > params['max_match_distance']: continue
        
        already_matched.append(j)

        mod_a_label = data_dict['current_label_list'][mod_a_idx[i]]
        mod_b_label = data_dict['current_label_list'][mod_b_idx[j]]

        # make the fusion dot green
        if params['ModalityA']['img_extras']['show_fusion_dot']:
            mod_a_label['extras']['img_visualizer'][algo_name]['fusion_dot']['params']['color'] = [0, 255, 0]
        if params['ModalityB']['img_extras']['show_fusion_dot']:
            mod_b_label['extras']['img_visualizer'][algo_name]['fusion_dot']['params']['color'] = [0, 255, 0]

        # placeholder for text_info
        text_info = 'fused: Fuse2DPredictedBBoxes'

        # check availability of confidence score
        conf_in_mod_a = 'confidence' in mod_a_label['bbox_2d']
        conf_in_mod_b = 'confidence' in mod_b_label['bbox_2d']
        if conf_in_mod_a and conf_in_mod_b: confidence_exists = 'both'
        elif conf_in_mod_a: confidence_exists = 'a'
        elif conf_in_mod_b: confidence_exists = 'b'
        else: confidence_exists = 'none'

        # assign class based on the highest confidence score
        # and assign color based on the highest confidence score
        # and assign depth based on the highest confidence score
        if confidence_exists == 'both':
            if mod_a_label['bbox_2d']['confidence'] > mod_b_label['bbox_2d']['confidence']:
                mod_b_label['class'] = mod_a_label['class']
                mod_b_label['bbox_2d']['rgb_color'] = mod_a_label['bbox_2d']['rgb_color']
                if 'depth' in mod_a_label['bbox_2d']: mod_b_label['bbox_2d']['depth'] = mod_a_label['bbox_2d']['depth']
                text_info += f" | conf: {mod_a_label['bbox_2d']['confidence']:.2f}"
                text_info += f" | dist: {mod_b_label['bbox_2d']['depth']:.2f} m"
            else:
                mod_a_label['class'] = mod_b_label['class']
                mod_a_label['bbox_2d']['rgb_color'] = mod_b_label['bbox_2d']['rgb_color']
                if 'depth' in mod_b_label['bbox_2d']: mod_a_label['bbox_2d']['depth'] = mod_b_label['bbox_2d']['depth']
                text_info += f" | conf: {mod_b_label['bbox_2d']['confidence']:.2f}"
                text_info += f" | dist: {mod_a_label['bbox_2d']['depth']:.2f} m"
        elif confidence_exists == 'a':
            mod_b_label['class'] = mod_a_label['class']
            mod_b_label['bbox_2d']['rgb_color'] = mod_a_label['bbox_2d']['rgb_color']
            if 'depth' in mod_a_label['bbox_2d']: mod_b_label['bbox_2d']['depth'] = mod_a_label['bbox_2d']['depth']
            text_info += f" | conf: {mod_a_label['bbox_2d']['confidence']:.2f}"
            text_info += f" | dist: {mod_b_label['bbox_2d']['depth']:.2f} m"
        elif confidence_exists == 'b':
            mod_a_label['class'] = mod_b_label['class']
            mod_a_label['bbox_2d']['rgb_color'] = mod_b_label['bbox_2d']['rgb_color']
            if 'depth' in mod_b_label['bbox_2d']: mod_a_label['bbox_2d']['depth'] = mod_b_label['bbox_2d']['depth']
            text_info += f" | conf: {mod_b_label['bbox_2d']['confidence']:.2f}"
            text_info += f" | dist: {mod_a_label['bbox_2d']['depth']:.2f} m"

        if params['ModalityA']['img_extras']['show_text_info']:
            if 'text_info' not in mod_a_label: mod_a_label['text_info'] = text_info
            else: mod_a_label['text_info'] += f' | {text_info}'
        
        if params['ModalityB']['img_extras']['show_text_info']:
            if 'text_info' not in mod_b_label: mod_b_label['text_info'] = text_info
            else: mod_b_label['text_info'] += f' | {text_info}'

def GenerateKDTreePastTrajectory(data_dict: dict, cfg_dict: dict, logger: Logger):
    """
    Generates past trajectory of objects using KDTree matching.

    Requirements:
    - bbox_3d dict in data_dict['current_label_list'][<index>].
    
    Operation:
    - It uses KDTree to track objects.
    - Stores the past trajectory of objects in data_dict['current_label_list'][<index>]['bbox_3d']['past_trajectory'].

    Args:
        data_dict (dict): A dictionary containing the required data.
        cfg_dict (dict): A dictionary containing configuration parameters.
        logger (gui.logger_gui.Logger): A logger object for logging messages and errors in GUI.
    """
    # get name and params
    algo_name = inspect.stack()[0].function
    params = get_algo_params(cfg_dict, algo_type, algo_name, logger)

    # Check if required data is present in data_dict
    if 'current_label_list' not in data_dict:
        logger.log('current_label_list not found in data_dict', Logger.ERROR)
        return

    # imports
    import numpy as np
    from scipy.spatial import KDTree
    from scipy.optimize import linear_sum_assignment

    # algo name and dict keys
    processed_frames_key = make_key(algo_name, 'processed_frames')
    bbox_history_window_key = make_key(algo_name, 'bbox_history_window')

    # check if the current frame is already processed
    if processed_frames_key in data_dict and data_dict['current_frame_index'] in data_dict[processed_frames_key]:
        logger.log(f'Frame {data_dict["current_frame_index"]} already processed, skipping ...', Logger.WARNING)
        return
    
    current_bbox_3d_labels = []
    for i, label_dict in enumerate(data_dict['current_label_list']):
        if 'bbox_3d' not in label_dict: continue
        bbox_3d_dict = label_dict['bbox_3d']
        current_bbox_3d_labels.append([i, bbox_3d_dict])
    
    # if no bbox_3d found in current frame, skip
    if len(current_bbox_3d_labels) == 0:
        logger.log('No bbox_3d found in current frame', Logger.DEBUG)
        return
    
    # create history window if it does not exist
    if bbox_history_window_key not in data_dict:
        data_dict[bbox_history_window_key] = [current_bbox_3d_labels]
        return

    # Create a KDTree for points_list
    all_history_bbox_3d_xyz_centers = []
    for bbox_3d_labels in data_dict[bbox_history_window_key]:
        xyz_centers = np.array([bbox_3d_dict['xyz_center'] for _, bbox_3d_dict in bbox_3d_labels])
        all_history_bbox_3d_xyz_centers.append(xyz_centers)
    current_bbox_3d_xyz_center = np.array([bbox_3d_dict['xyz_center'] for _, bbox_3d_dict in current_bbox_3d_labels])
    
    # ---------------------- KDTree Tracking ---------------------- #
    kd_tree = KDTree(current_bbox_3d_xyz_center)
    already_matched = []

    for history_idx, xyz_centers in enumerate(all_history_bbox_3d_xyz_centers):
        cost_matrix = np.zeros((len(xyz_centers), len(current_bbox_3d_xyz_center)))

        for i, point in enumerate(xyz_centers):
            distances, indices = kd_tree.query(point, k=len(current_bbox_3d_xyz_center))
            cost_matrix[i, indices] = distances

        # Use the Hungarian algorithm (linear_sum_assignment) to find the optimal matching
        row_indices, col_indices = linear_sum_assignment(cost_matrix)

        # Output the matching result
        matches_kd_tree = list(zip(row_indices, col_indices))

        for i, j in matches_kd_tree:
            # check if the current bbox is already matched
            if j in already_matched: continue
            
            # check if the distance is within the threshold
            if cost_matrix[i, j] > params['max_match_distance']: continue

            # get the bbox_3d dict
            last_bbox_3d = data_dict[bbox_history_window_key][history_idx][i][1]
            current_bbox_3d = current_bbox_3d_labels[j][1]

            # store the past trajectory
            if 'past_trajectory' in last_bbox_3d:
                current_bbox_3d['past_trajectory'] = np.append(last_bbox_3d['past_trajectory'], [last_bbox_3d['xyz_center']], axis=0)
            else:
                current_bbox_3d['past_trajectory'] = np.array([last_bbox_3d['xyz_center']], dtype=np.float32)

            # add text info
            if 'text_info' not in data_dict['current_label_list'][current_bbox_3d_labels[j][0]]: data_dict['current_label_list'][current_bbox_3d_labels[j][0]]['text_info'] = f'traj-: {len(current_bbox_3d["past_trajectory"])}'
            else: data_dict['current_label_list'][current_bbox_3d_labels[j][0]]['text_info'] += f' | traj-: {len(current_bbox_3d["past_trajectory"])}'

            # update the bbox_3d dict
            data_dict['current_label_list'][current_bbox_3d_labels[j][0]]['bbox_3d'] = current_bbox_3d

            # mark the current bbox as matched
            already_matched.append(j)
    # ---------------------- KDTree Tracking ---------------------- #

    # keep record of processed frames
    if processed_frames_key not in data_dict: data_dict[processed_frames_key] = [data_dict['current_frame_index']]
    else: data_dict[processed_frames_key].append(data_dict['current_frame_index'])
    
    # update the last_bbox_3d_labels
    data_dict[bbox_history_window_key].insert(0, current_bbox_3d_labels)
    data_dict[bbox_history_window_key] = data_dict[bbox_history_window_key][:params['history_size']]

def GenerateCubicSplineFutureTrajectory(data_dict: dict, cfg_dict: dict, logger: Logger):
    """
    Generates future trajectory using cubic spline interpolation.

    Requirements:
    - past_trajectory dict in data_dict['current_label_list'][<index>]['bbox_3d'].

    Operation:
    - It uses cubic spline interpolation to predict future trajectory.
    - Stores the future trajectory of objects in data_dict['current_label_list'][<index>]['bbox_3d']['future_trajectory'].

    Args:
        data_dict (dict): A dictionary containing the required data.
        cfg_dict (dict): A dictionary containing configuration parameters.
        logger (gui.logger_gui.Logger): A logger object for logging messages and errors in GUI.
    """
    # get name and params
    algo_name = inspect.stack()[0].function
    params = get_algo_params(cfg_dict, algo_type, algo_name, logger)

    # Check if required data is present in data_dict
    if 'current_label_list' not in data_dict:
        logger.log('current_label_list not found in data_dict', Logger.ERROR)
        return

    # imports
    import numpy as np
    from scipy.interpolate import CubicSpline

    # ---------------------- Cubic Spline Interpolation ---------------------- #
    for label_dict in data_dict['current_label_list']:
        if 'bbox_3d' not in label_dict: continue
        if 'past_trajectory' not in label_dict['bbox_3d']: continue

        past_trajectory = label_dict['bbox_3d']['past_trajectory']
        past_trajectory_x = past_trajectory[:, 0]
        past_trajectory_y = past_trajectory[:, 1]
        past_trajectory_z = past_trajectory[:, 2]
        len_past_trajectory = len(past_trajectory)
        if len_past_trajectory < params['t_minimum']:
            if 'text_info' not in label_dict: label_dict['text_info'] = f'traj+: {params["t_minimum"] - len_past_trajectory}'
            else: label_dict['text_info'] += f' | traj+: {params["t_minimum"] - len_past_trajectory}'
            continue
        cs_x = CubicSpline(np.arange(len_past_trajectory), past_trajectory_x)
        cs_y = CubicSpline(np.arange(len_past_trajectory), past_trajectory_y)
        future_steps = np.arange(len_past_trajectory, len_past_trajectory + params['t_plus_steps'])
        future_trajectory_x = cs_x(future_steps).astype(np.float32)
        future_trajectory_y = cs_y(future_steps).astype(np.float32)
        future_trajectory_z = np.full_like(future_trajectory_x, past_trajectory_z[-1]).astype(np.float32)
        label_dict['bbox_3d']['future_trajectory'] = np.column_stack((future_trajectory_x, future_trajectory_y, future_trajectory_z))
        if 'text_info' not in label_dict: label_dict['text_info'] = f'traj+: locked'
        else: label_dict['text_info'] += f' | traj+: locked'
    # ---------------------- Cubic Spline Interpolation ---------------------- #

def GeneratePolyFitFutureTrajectory(data_dict: dict, cfg_dict: dict, logger: Logger):
    """
    Generates future trajectory using polynomial fit.

    Requirements:
    - past_trajectory dict in data_dict['current_label_list'][<index>]['bbox_3d'].

    Operation:
    - It uses polynomial fit to predict future trajectory.
    - Stores the future trajectory of objects in data_dict['current_label_list'][<index>]['bbox_3d']['future_trajectory'].

    Args:
        data_dict (dict): A dictionary containing the required data.
        cfg_dict (dict): A dictionary containing configuration parameters.
        logger (gui.logger_gui.Logger): A logger object for logging messages and errors in GUI.
    """
    # get name and params
    algo_name = inspect.stack()[0].function
    params = get_algo_params(cfg_dict, algo_type, algo_name, logger)

    # Check if required data is present in data_dict
    if 'current_label_list' not in data_dict:
        logger.log('current_label_list not found in data_dict', Logger.ERROR)
        return

    # imports
    import numpy as np
    from numpy.polynomial import Polynomial

    # ---------------------- Polynomial Fit ---------------------- #
    for label_dict in data_dict['current_label_list']:
        if 'bbox_3d' not in label_dict: continue
        if 'past_trajectory' not in label_dict['bbox_3d']: continue

        past_trajectory = label_dict['bbox_3d']['past_trajectory']
        past_trajectory_x = past_trajectory[:, 0]
        past_trajectory_y = past_trajectory[:, 1]
        past_trajectory_z = past_trajectory[:, 2]
        len_past_trajectory = len(past_trajectory)
        if len_past_trajectory < params['t_minimum']:
            if 'text_info' not in label_dict: label_dict['text_info'] = f'traj+: {params["t_minimum"] - len_past_trajectory}'
            else: label_dict['text_info'] += f' | traj+: {params["t_minimum"] - len_past_trajectory}'
            continue
        poly_x = Polynomial.fit(np.arange(len_past_trajectory), past_trajectory_x, params['poly_degree'])
        poly_y = Polynomial.fit(np.arange(len_past_trajectory), past_trajectory_y, params['poly_degree'])
        future_steps = np.arange(len_past_trajectory, len_past_trajectory + params['t_plus_steps'])
        future_trajectory_x = poly_x(future_steps).astype(np.float32)
        future_trajectory_y = poly_y(future_steps).astype(np.float32)
        future_trajectory_z = np.full_like(future_trajectory_x, past_trajectory_z[-1]).astype(np.float32)
        label_dict['bbox_3d']['future_trajectory'] = np.column_stack((future_trajectory_x, future_trajectory_y, future_trajectory_z))
        if 'text_info' not in label_dict: label_dict['text_info'] = f'traj+: locked'
        else: label_dict['text_info'] += f' | traj+: locked'
    # ---------------------- Polynomial Fit ---------------------- #

def GenerateVelocityFromTrajectory(data_dict: dict, cfg_dict: dict, logger: Logger):
    """
    Generates velocity from trajectory.

    Requirements:
    - past_trajectory dict in data_dict['current_label_list'][<index>]['bbox_3d'].

    Operation:
    - It calculates the velocity from the past trajectory.
    - Stores the velocity of objects in data_dict['current_label_list'][<index>]['bbox_3d']['past_velocity'].

    Args:
        data_dict (dict): A dictionary containing the required data.
        cfg_dict (dict): A dictionary containing configuration parameters.
        logger (gui.logger_gui.Logger): A logger object for logging messages and errors in GUI.
    """
    # get name and params
    algo_name = inspect.stack()[0].function
    params = get_algo_params(cfg_dict, algo_type, algo_name, logger)

    # Check if required data is present in data_dict
    if 'current_label_list' not in data_dict:
        logger.log('current_label_list not found in data_dict', Logger.ERROR)
        return

    # imports
    import numpy as np

    # dict keys
    processed_frames_key = make_key(algo_name, 'processed_frames')

    # check if the current frame is already processed
    if processed_frames_key in data_dict and data_dict['current_frame_index'] in data_dict[processed_frames_key]:
        logger.log(f'Frame {data_dict["current_frame_index"]} already processed, skipping ...', Logger.WARNING)
        return
    
    # ---------------------- Velocity Calculation ---------------------- #
    for label_dict in data_dict['current_label_list']:
        if 'bbox_3d' not in label_dict: continue
        if 'past_trajectory' not in label_dict['bbox_3d']: continue

        past_trajectory = label_dict['bbox_3d']['past_trajectory']
        len_past_trajectory = len(past_trajectory)
        if len_past_trajectory < 2: continue

        # calculate velocity
        past_velocity = np.diff(past_trajectory, axis=0) / params['t_delta']
        past_velocity = np.append(past_velocity, [past_velocity[-1]], axis=0)
        label_dict['bbox_3d']['past_velocity'] = past_velocity
        if 'text_info' not in label_dict: label_dict['text_info'] = f'vel: {np.linalg.norm(past_velocity[-1]):.2f} m/s'
        else: label_dict['text_info'] += f' | vel: {np.linalg.norm(past_velocity[-1]):.2f} m/s'
    # ---------------------- Velocity Calculation ---------------------- #

    # keep record of processed frames
    if processed_frames_key not in data_dict: data_dict[processed_frames_key] = [data_dict['current_frame_index']]
    else: data_dict[processed_frames_key].append(data_dict['current_frame_index'])

def create_per_object_pcdet_dataset(data_dict: dict, cfg_dict: dict, logger: Logger):
    """
    Creates a per-object PCDet dataset by extracting object point clouds and labels from the input data.

    Args:
        data_dict (dict): A dictionary containing the required data.
        cfg_dict (dict): A dictionary containing configuration parameters.
        logger (gui.logger_gui.Logger): A logger object for logging messages and errors in GUI.
    """
    # get name and params
    algo_name = inspect.stack()[0].function
    params = get_algo_params(cfg_dict, algo_type, algo_name, logger)

    # Check if required data is present in data_dict
    if 'current_point_cloud_numpy' not in data_dict:
        logger.log('current_point_cloud_numpy not found in data_dict', Logger.ERROR)
        return
    if "current_label_list" not in data_dict:
        logger.log('current_label_list not found in data_dict', Logger.ERROR)
        return
    
    # imports
    import os
    import numpy as np
    import open3d as o3d
    
    # Get required data from data_dict
    current_point_cloud_path = data_dict['current_point_cloud_path']
    current_point_cloud_numpy = data_dict['current_point_cloud_numpy']
    current_label_list = data_dict['current_label_list']
    
    # Create output directories if they do not exist
    output_path = os.path.join(get_abs_path(cfg_dict['data']['outputs_dir']), 'post', 'per_object_pcdet_dataset')
    pcd_output_dir = os.path.join(output_path, 'point_cloud')
    os.makedirs(pcd_output_dir, exist_ok=True)
    lbl_output_dir = os.path.join(output_path, 'label')
    os.makedirs(lbl_output_dir, exist_ok=True)
    
    idx = 0
    for label_dict in current_label_list:
        if 'bbox_3d' not in label_dict: continue
        
        # Get bounding box center, extent, and euler angles
        bbox_center = label_dict['bbox_3d']['xyz_center'].copy()
        bbox_extent = label_dict['bbox_3d']['xyz_extent']
        bbox_euler_angles = label_dict['bbox_3d']['xyz_euler_angles']
        R = o3d.geometry.OrientedBoundingBox.get_rotation_matrix_from_xyz(bbox_euler_angles)
        
        # Create an oriented bounding box
        try: rotated_bbox = o3d.geometry.OrientedBoundingBox(bbox_center, R, bbox_extent)
        except:
            logger.log(f'Error in creating oriented bounding box for object {idx}', Logger.WARNING)
            continue
        
        # Get points within the bounding box
        inside_points = rotated_bbox.get_point_indices_within_bounding_box(o3d.utility.Vector3dVector(current_point_cloud_numpy[:, :3]))
        object_point_cloud = current_point_cloud_numpy[inside_points]
        
        # Center the point cloud
        point_cloud_mean = np.mean(object_point_cloud[:, :3], axis=0)
        bbox_center -= point_cloud_mean
        object_point_cloud[:, :3] -= point_cloud_mean
        
        # Save the point cloud and label
        npy_path = os.path.join(pcd_output_dir, os.path.basename(current_point_cloud_path).split('.')[0] + f'_{str(idx).zfill(4)}.npy')
        np.save(npy_path, object_point_cloud)
        
        # Save the label
        lbl_path = os.path.join(lbl_output_dir, os.path.basename(current_point_cloud_path).split('.')[0] + f'_{str(idx).zfill(4)}.txt')
        with open(lbl_path, 'w') as f:
            lbl_str = ''
            lbl_str += str(bbox_center[0]) + ' ' + str(bbox_center[1]) + ' ' + str(bbox_center[2]) + ' '
            lbl_str += str(bbox_extent[0]) + ' ' + str(bbox_extent[1]) + ' ' + str(bbox_extent[2]) + ' '
            lbl_str += str(bbox_euler_angles[2]) + ' '
            if 'class' in label_dict: lbl_str += label_dict['class']
            else: lbl_str += 'Unknown'
            f.write(lbl_str)
        
        idx += 1

def create_pcdet_dataset(data_dict: dict, cfg_dict: dict, logger: Logger):
    """
    Creates a PCDet dataset by extracting point clouds and labels from the input data.

    Args:
        data_dict (dict): A dictionary containing the required data.
        cfg_dict (dict): A dictionary containing configuration parameters.
        logger (gui.logger_gui.Logger): A logger object for logging messages and errors in GUI.
    """
    # get name and params
    algo_name = inspect.stack()[0].function
    params = get_algo_params(cfg_dict, algo_type, algo_name, logger)

    # Check if required data is present in data_dict
    if 'current_point_cloud_numpy' not in data_dict:
        logger.log('current_point_cloud_numpy not found in data_dict', Logger.ERROR)
        return
    if "current_label_list" not in data_dict:
        logger.log('current_label_list not found in data_dict', Logger.ERROR)
        return
    
    # imports
    import os
    import numpy as np

    # Get required data from data_dict
    current_point_cloud_path = data_dict['current_point_cloud_path']
    current_point_cloud_numpy = data_dict['current_point_cloud_numpy']
    current_label_list = data_dict['current_label_list']
    
    # Create output directories if they do not exist
    output_path = os.path.join(get_abs_path(cfg_dict['data']['outputs_dir']), 'post', 'pcdet_dataset')
    pcd_output_dir = os.path.join(output_path, 'point_cloud')
    os.makedirs(pcd_output_dir, exist_ok=True)
    lbl_output_dir = os.path.join(output_path, 'label')
    os.makedirs(lbl_output_dir, exist_ok=True)

    # Save the point cloud
    npy_path = os.path.join(pcd_output_dir, os.path.basename(current_point_cloud_path).split('.')[0] + '.npy')
    np.save(npy_path, current_point_cloud_numpy)
    
    lbl_str = ''
    for label_dict in current_label_list:
        if 'bbox_3d' not in label_dict: continue
        # Get bounding box center, extent, and euler angles
        bbox_center = label_dict['bbox_3d']['xyz_center'].copy()
        bbox_extent = label_dict['bbox_3d']['xyz_extent']
        bbox_euler_angles = label_dict['bbox_3d']['xyz_euler_angles']
        lbl_str += str(bbox_center[0]) + ' ' + str(bbox_center[1]) + ' ' + str(bbox_center[2]) + ' '
        lbl_str += str(bbox_extent[0]) + ' ' + str(bbox_extent[1]) + ' ' + str(bbox_extent[2]) + ' '
        lbl_str += str(bbox_euler_angles[2]) + ' '
        if 'class' in label_dict: lbl_str += label_dict['class']
        else: lbl_str += 'Unknown'
        lbl_str += '\n'

    # Save the label
    point_cloud_file_base_name = os.path.basename(current_point_cloud_path).split('.')[0]
    lbl_path = os.path.join(lbl_output_dir, point_cloud_file_base_name + '.txt')
    with open(lbl_path, 'w') as f: f.write(lbl_str)

def visualize_in_vr(data_dict: dict, cfg_dict: dict, logger: Logger):
    """
    Visualizes outputs in the VR environment.

    Args:
        data_dict (dict): A dictionary containing the required data.
        cfg_dict (dict): A dictionary containing configuration parameters.
        logger (gui.logger_gui.Logger): A logger object for logging messages and errors in GUI.
    """
    # get name and params
    algo_name = inspect.stack()[0].function
    params = get_algo_params(cfg_dict, algo_type, algo_name, logger)

    # dict keys
    server_socket_key = make_key(algo_name, 'server_socket')
    client_socket_key = make_key(algo_name, 'client_socket')

    # imports
    import socket
    import struct
    import open3d as o3d
    import numpy as np

    # create a server socket if it does not exist
    if server_socket_key not in data_dict:
        # Create a server socket
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.settimeout(cfg_dict['threads']['net_sleep'])
        server_socket.bind((params['server_ip'], params['server_port']))
        server_socket.listen(1)
        data_dict[server_socket_key] = server_socket
        logger.log(f'Server socket created at {params["server_ip"]}:{params["server_port"]}', Logger.DEBUG)
    
    # accept a client connection
    if client_socket_key not in data_dict:
        server_socket = data_dict[server_socket_key]
        try:
            client_socket, addr = server_socket.accept()
            data_dict[client_socket_key] = client_socket
            logger.log(f'Client connected from {addr}', Logger.DEBUG)
        except socket.timeout: pass
    
    else:
        if 'current_point_cloud_numpy' in data_dict:
            # receive an int32 value, convert to int
            req = struct.unpack('I', data_dict[client_socket_key].recv(4))[0]
            logger.log(f'Received request: {req}', Logger.DEBUG)
            if req == 0: return # do nothing
            if req == 1:
                points = np.asarray(data_dict['current_point_cloud_numpy'][:, :3], dtype=np.float32)
                data = points.flatten().tobytes()
                data_dict[client_socket_key].sendall(struct.pack('I', len(data)))
                data_dict[client_socket_key].sendall(data)
            elif req == 2: # close the client socket
                data_dict[client_socket_key].close()
                data_dict.pop(client_socket_key)
                logger.log(f'Client disconnected', Logger.DEBUG)