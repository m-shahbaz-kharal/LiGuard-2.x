# contains more generic post-processing algorithms for the data

from gui.logger_gui import Logger

def create_per_object_pcdet_dataset(data_dict: dict, cfg_dict: dict):
    """
    Create a per-object PCDet dataset by extracting object point clouds and labels from the input data.

    Args:
        data_dict (dict): A dictionary containing the required data.
        cfg_dict (dict): A dictionary containing configuration parameters.

    Returns:
        None
    """
    # Get logger object from data_dict
    if 'logger' in data_dict: logger:Logger = data_dict['logger']
    else: print('[algo->post.py->create_per_object_pcdet_dataset]: No logger object in data_dict. It is abnormal behavior as logger object is created by default. Please check if some script is removing the logger key in data_dict.'); return

    # Check if required data is present in data_dict
    if 'current_point_cloud_numpy' not in data_dict:
        logger.log('[algo->post.py->create_per_object_pcdet_dataset]: current_point_cloud_numpy not found in data_dict', Logger.ERROR)
        return
    if "current_label_list" not in data_dict:
        logger.log('[algo->post.py->create_per_object_pcdet_dataset]: current_label_list not found in data_dict', Logger.ERROR)
        return
    if "current_label_path" not in data_dict:
        logger.log('[algo->post.py->create_per_object_pcdet_dataset]: current_label_path not found in data_dict', Logger.ERROR)
        return
    
    # imports
    import os
    import numpy as np
    import open3d as o3d
    
    # Get required data from data_dict
    current_point_cloud_numpy = data_dict['current_point_cloud_numpy']
    current_label_list = data_dict['current_label_list']
    current_label_path = data_dict['current_label_path']
    
    # Create output directories if they do not exist
    output_path = os.path.join(cfg_dict['data']['path'], 'output', 'post', 'per_object_pcdet_dataset')
    os.makedirs(output_path, exist_ok=True)
    pcd_output_dir = os.path.join(output_path, 'point_cloud')
    os.makedirs(pcd_output_dir, exist_ok=True)
    lbl_output_dir = os.path.join(output_path, 'label')
    os.makedirs(lbl_output_dir, exist_ok=True)
    
    for idx, label_dict in enumerate(current_label_list):
        if 'bbox_3d' not in label_dict: continue
        # Get bounding box center, extent, and euler angles
        bbox_center = label_dict['bbox_3d']['xyz_center'].copy()
        bbox_extent = label_dict['bbox_3d']['xyz_extent']
        bbox_euler_angles = label_dict['bbox_3d']['xyz_euler_angles']
        R = o3d.geometry.OrientedBoundingBox.get_rotation_matrix_from_xyz(bbox_euler_angles)
        
        # Create an oriented bounding box
        try: rotated_bbox = o3d.geometry.OrientedBoundingBox(bbox_center, R, bbox_extent)
        except:
            logger.log(f'[algo->post.py->create_per_object_pcdet_dataset]: failed to create an OrientedBoundingBox, skipping ...', Logger.WARNING)
            continue
        # Get points within the bounding box
        inside_points = rotated_bbox.get_point_indices_within_bounding_box(o3d.utility.Vector3dVector(current_point_cloud_numpy[:, :3]))
        object_point_cloud = current_point_cloud_numpy[inside_points]
        
        # Center the point cloud
        point_cloud_mean = np.mean(object_point_cloud[:, :3], axis=0)
        bbox_center -= point_cloud_mean
        object_point_cloud[:, :3] -= point_cloud_mean
        
        # Save the point cloud and label
        npy_path = os.path.join(pcd_output_dir, os.path.basename(current_label_path).replace('.txt', f'{str(idx).zfill(4)}.npy'))
        np.save(npy_path, object_point_cloud)
        
        # Save the label
        lbl_path = os.path.join(lbl_output_dir, os.path.basename(current_label_path).replace('.txt', f'{str(idx).zfill(4)}.txt'))
        with open(lbl_path, 'w') as f:
            lbl_str = ''
            lbl_str += str(bbox_center[0]) + ' ' + str(bbox_center[1]) + ' ' + str(bbox_center[2]) + ' '
            lbl_str += str(bbox_extent[0]) + ' ' + str(bbox_extent[1]) + ' ' + str(bbox_extent[2]) + ' '
            lbl_str += str(bbox_euler_angles[2]) + ' '
            if 'class' in label_dict: lbl_str += label_dict['class']
            else: lbl_str += 'Unknown'
            f.write(lbl_str)

def create_pcdet_dataset(data_dict: dict, cfg_dict: dict):
    # Get logger object from data_dict
    if 'logger' in data_dict: logger:Logger = data_dict['logger']
    else: print('[algo->post.py->create_pcdet_dataset]: No logger object in data_dict. It is abnormal behavior as logger object is created by default. Please check if some script is removing the logger key in data_dict.'); return

    # Check if required data is present in data_dict
    if 'current_point_cloud_numpy' not in data_dict:
        logger.log('[algo->post.py->create_pcdet_dataset]: current_point_cloud_numpy not found in data_dict', Logger.ERROR)
        return
    if "current_label_list" not in data_dict:
        logger.log('[algo->post.py->create_pcdet_dataset]: current_label_list not found in data_dict', Logger.ERROR)
        return
    if "current_label_path" not in data_dict:
        logger.log('[algo->post.py->create_pcdet_dataset]: current_label_path not found in data_dict', Logger.ERROR)
        return
    
    # imports
    import os
    import numpy as np

    # Get required data from data_dict
    current_point_cloud_numpy = data_dict['current_point_cloud_numpy']
    current_label_list = data_dict['current_label_list']
    current_label_path = data_dict['current_label_path']
    
    # Create output directories if they do not exist
    output_path = os.path.join(cfg_dict['data']['path'], 'output', 'post', 'pcdet_dataset')
    os.makedirs(output_path, exist_ok=True)
    pcd_output_dir = os.path.join(output_path, 'point_cloud')
    os.makedirs(pcd_output_dir, exist_ok=True)
    lbl_output_dir = os.path.join(output_path, 'label')
    os.makedirs(lbl_output_dir, exist_ok=True)

    # Save the point cloud
    npy_path = os.path.join(pcd_output_dir, os.path.basename(current_label_path).replace('.txt', '.npy'))
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
    lbl_path = os.path.join(lbl_output_dir, os.path.basename(current_label_path))
    with open(lbl_path, 'w') as f: f.write(lbl_str)

def visualize_in_vr(data_dict: dict, cfg_dict: dict):
    """
    Visualize outputs in the VR environment.

    Args:
        data_dict (dict): A dictionary containing the required data.
        cfg_dict (dict): A dictionary containing configuration parameters.

    Returns:
        None
    """
    # Get logger object from data_dict
    if 'logger' in data_dict: logger:Logger = data_dict['logger']
    else: print('[algo->post.py->visualize_in_vr]: No logger object in data_dict. It is abnormal behavior as logger object is created by default. Please check if some script is removing the logger key in data_dict.'); return

    # algo name
    algo_name = 'visualize_in_vr'

    # dict keys
    server_socket_key = f'{algo_name}_server_socket'
    client_socket_key = f'{algo_name}_client_socket'

    # get params
    params = cfg_dict['proc']['post'][algo_name]

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
        logger.log(f'[algo->post.py->visualize_in_vr]: Server socket created', Logger.DEBUG)
    
    # accept a client connection
    if client_socket_key not in data_dict:
        server_socket = data_dict[server_socket_key]
        try:
            client_socket, addr = server_socket.accept()
            data_dict[client_socket_key] = client_socket
            logger.log(f'[algo->post.py->visualize_in_vr]: Client connected from {addr}', Logger.DEBUG)
        except socket.timeout: pass
    
    else:
        if 'current_point_cloud_numpy' in data_dict:
            # receive an int32 value, convert to int
            req = struct.unpack('I', data_dict[client_socket_key].recv(4))[0]
            logger.log(f'[algo->post.py->visualize_in_vr]: Received request: {req}', Logger.DEBUG)
            if req == 0: return # do nothing
            if req == 1:
                points = np.asarray(data_dict['current_point_cloud_numpy'][:, :3], dtype=np.float32)
                data = points.flatten().tobytes()
                data_dict[client_socket_key].sendall(struct.pack('I', len(data)))
                data_dict[client_socket_key].sendall(data)
            elif req == 2: # close the client socket
                data_dict[client_socket_key].close()
                data_dict.pop(client_socket_key)
                logger.log(f'[algo->post.py->visualize_in_vr]: Client disconnected', Logger.DEBUG)