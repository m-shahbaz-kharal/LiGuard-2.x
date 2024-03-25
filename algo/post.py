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
    
    for idx, label_dict in enumerate(current_label_list):
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
        
        per_frame_output_path = os.path.join(output_path, os.path.basename(current_label_path).replace('.txt', ''))
        os.makedirs(per_frame_output_path, exist_ok=True)
        os.makedirs(os.path.join(per_frame_output_path, 'point_cloud'), exist_ok=True)
        os.makedirs(os.path.join(per_frame_output_path, 'label'), exist_ok=True)
        
        npy_path = os.path.join(per_frame_output_path, 'point_cloud', f'{idx}.npy')
        np.save(npy_path, object_point_cloud)
        
        lbl_path = os.path.join(per_frame_output_path, 'label', f'{idx}.txt')
        with open(lbl_path, 'w') as f:
            lbl_str = ''
            lbl_str += str(bbox_center[0]) + ' ' + str(bbox_center[1]) + ' ' + str(bbox_center[2]) + ' '
            lbl_str += str(bbox_extent[0]) + ' ' + str(bbox_extent[1]) + ' ' + str(bbox_extent[2]) + ' '
            lbl_str += str(bbox_euler_angles[2]) + ' '
            if 'class' in label_dict: lbl_str += label_dict['class']
            f.write(lbl_str)