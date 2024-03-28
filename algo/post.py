def create_per_object_pcdet_dataset(data_dict: dict, cfg_dict: dict):
    if 'current_point_cloud_numpy' not in data_dict: return
    if "current_label_list" not in data_dict: return
    if "current_label_path" not in data_dict: return
    
    import os
    import numpy as np
    
    import open3d as o3d
    
    current_point_cloud_numpy = data_dict['current_point_cloud_numpy']
    current_label_list = data_dict['current_label_list']
    current_label_path = data_dict['current_label_path']
    
    output_path = os.path.join(cfg_dict['data']['path'], 'output', 'post', 'per_object_pcdet_dataset')
    os.makedirs(output_path, exist_ok=True)
    pcd_output_dir = os.path.join(output_path, 'point_cloud')
    os.makedirs(pcd_output_dir, exist_ok=True)
    lbl_output_dir = os.path.join(output_path, 'label')
    os.makedirs(lbl_output_dir, exist_ok=True)
    
    for idx, label_dict in enumerate(current_label_list):
        if 'lidar_bbox' not in label_dict: continue
        bbox_center = label_dict['lidar_bbox']['lidar_xyz_center'].copy()
        bbox_extent = label_dict['lidar_bbox']['lidar_xyz_extent']
        bbox_euler_angles = label_dict['lidar_bbox']['lidar_xyz_euler_angles']
        R = o3d.geometry.OrientedBoundingBox.get_rotation_matrix_from_xyz(bbox_euler_angles)
        
        rotated_bbox = o3d.geometry.OrientedBoundingBox(bbox_center, R, bbox_extent)
        inside_points = rotated_bbox.get_point_indices_within_bounding_box(o3d.utility.Vector3dVector(current_point_cloud_numpy[:, :3]))
        object_point_cloud = current_point_cloud_numpy[inside_points]
        
        point_cloud_mean = np.mean(object_point_cloud[:, :3], axis=0)
        bbox_center -= point_cloud_mean
        object_point_cloud[:, :3] -= point_cloud_mean
        
        npy_path = os.path.join(pcd_output_dir, os.path.basename(current_label_path).replace('.txt', f'{str(idx).zfill(4)}.npy'))
        np.save(npy_path, object_point_cloud)
        
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
    if 'current_point_cloud_numpy' not in data_dict: return
    if "current_label_list" not in data_dict: return
    if "current_label_path" not in data_dict: return
    
    import os
    import numpy as np

    current_point_cloud_numpy = data_dict['current_point_cloud_numpy']
    current_label_list = data_dict['current_label_list']
    current_label_path = data_dict['current_label_path']
    
    output_path = os.path.join(cfg_dict['data']['path'], 'output', 'post', 'pcdet_dataset')
    os.makedirs(output_path, exist_ok=True)
    pcd_output_dir = os.path.join(output_path, 'point_cloud')
    os.makedirs(pcd_output_dir, exist_ok=True)
    lbl_output_dir = os.path.join(output_path, 'label')
    os.makedirs(lbl_output_dir, exist_ok=True)

    npy_path = os.path.join(pcd_output_dir, os.path.basename(current_label_path).replace('.txt', '.npy'))
    np.save(npy_path, current_point_cloud_numpy)
    
    lbl_str = ''
    for label_dict in current_label_list:
        if 'lidar_bbox' not in label_dict: continue
        bbox_center = label_dict['lidar_bbox']['lidar_xyz_center'].copy()
        bbox_extent = label_dict['lidar_bbox']['lidar_xyz_extent']
        bbox_euler_angles = label_dict['lidar_bbox']['lidar_xyz_euler_angles']
        
        lbl_str += str(bbox_center[0]) + ' ' + str(bbox_center[1]) + ' ' + str(bbox_center[2]) + ' '
        lbl_str += str(bbox_extent[0]) + ' ' + str(bbox_extent[1]) + ' ' + str(bbox_extent[2]) + ' '
        lbl_str += str(bbox_euler_angles[2]) + ' '
        if 'class' in label_dict: lbl_str += label_dict['class']
        else: lbl_str += 'Unknown'
        lbl_str += '\n'

    lbl_path = os.path.join(lbl_output_dir, os.path.basename(current_label_path))
    with open(lbl_path, 'w') as f: f.write(lbl_str)