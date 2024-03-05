import os
import numpy as np
import open3d as o3d
import json

colors = {
    'Car': [0, 1, 0],
    'Van': [0, 1, 0],
    'Truck': [0, 1, 0],
    'Pedestrian': [1, 0, 0],
    'Person_sitting': [1, 0, 0],
    'Cyclist': [0, 0, 1],
    'Tram': [1, 1, 0],
    'Misc': [1, 1, 0],
    'DontCare': [1, 1, 1]
}

def Handler(label_path: str):
    output = []
    
    # read label file
    if os.path.exists(label_path) == False: return output
    with open(label_path, 'r') as f: lbls = json.load(f)
    for item in lbls:
        # annotator = item['annotator']
        obj_id = int(item['obj_id'])
        obj_type = item['obj_type']
        psr = item['psr']
        psr_position_xyz = [float(psr['position']['x']), float(psr['position']['y']), float(psr['position']['z'])]
        psr_rotation_xyz = [float(psr['rotation']['x']), float(psr['rotation']['y']), float(psr['rotation']['z'])]
        psr_scale_xyz = [float(psr['scale']['x']), float(psr['scale']['y']), float(psr['scale']['z'])]
        
        label = dict()
        # label['annotator'] = annotator
        label['id'] = obj_id
        label['type'] = obj_type
        label['psr'] = psr
        
        lidar_xyz_center = np.array(psr_position_xyz, dtype=np.float32)
        # lidar_xyz_center[2] += psr_scale_xyz[2] / 2.0
        lidar_wlh_extent = np.array(psr_scale_xyz, dtype=np.float32)
        lidar_around_z_rotation_matrix = o3d.geometry.OrientedBoundingBox.get_rotation_matrix_from_xyz(psr_rotation_xyz)
        lidar_bbox_color = colors[obj_type]
        
        label['lidar_bbox'] = {'xyz_center': lidar_xyz_center, 'wlh_extent': lidar_wlh_extent, 'around_z_rotation_matrix': lidar_around_z_rotation_matrix, 'rgb_bbox_color': lidar_bbox_color}
        
        camera_xyz_center = np.array(psr_position_xyz, dtype=np.float32)
        camera_wlh_extent = np.array(psr_scale_xyz, dtype=np.float32)
        camera_around_y_rotation_matrix = o3d.geometry.OrientedBoundingBox.get_rotation_matrix_from_xyz(psr_rotation_xyz)
        camera_bbox_color = [i * 255 for i in colors[obj_type]]
        
        label['camera_bbox'] = {'xyz_center': camera_xyz_center, 'wlh_extent': camera_wlh_extent, 'around_y_rotation_matrix': camera_around_y_rotation_matrix, 'rgb_bbox_color': camera_bbox_color}
        
        output.append(label)
    
    return output