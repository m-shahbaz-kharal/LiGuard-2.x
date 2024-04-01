import os
import numpy as np
import json

colors = {
    "Car":            (0  ,255,0  ),#'#00ff00',
    "Van":            (0  ,255,0  ),#'#00ff00',
    "Bus":            (0  ,255,255),#'#ffff00', 
    "Pedestrian":     (0  ,0  ,255),#'#ff0000',
    "Rider":          (0  ,136,255),#'#ff8800',
    "Cyclist":        (0  ,136,255),#'#ff8800',
    "Bicycle":        (0  ,255,136),#'#88ff00',
    "BicycleGroup":   (0  ,255,136),#'#88ff00',
    "Motor":          (0  ,176,176),#'#aaaa00',
    "Truck":          (255,255,0  ),#'#00ffff',
    "Tram":           (255,255,0  ),#'#00ffff',
    "Animal":         (255,176,0  ),#'#00aaff',
    "Misc":           (136,136,0  ),#'#008888',
    "Unknown":        (136,136,0  ),#'#008888',
}
label_file_extension = '.json'
calib_file_extension = '.json'

def Handler(label_path: str, calib_path: str):
    output = []
    
    # read calib
    calib = __read_calib__(calib_path)
        
    # read label file
    if os.path.exists(label_path) == False: return output
    with open(label_path, 'r') as f: lbls = json.load(f)
    for item in lbls:
        annotator = item['annotator'] if 'annotator' in item else 'Unknown'
        obj_id = int(item['obj_id'])
        obj_type = item['obj_type']
        psr = item['psr']
        psr_position_xyz = np.array([psr['position']['x'], psr['position']['y'], psr['position']['z']], dtype=np.float32)
        psr_rotation_xyz = np.array([psr['rotation']['x'], psr['rotation']['y'], psr['rotation']['z']], dtype=np.float32)
        psr_scale_xyz = np.array([psr['scale']['x'], psr['scale']['y'], psr['scale']['z']], dtype=np.float32)
        
        label = {}
        label['annotator'] = annotator
        label['obj_id'] = obj_id
        label['obj_type'] = obj_type
        label['psr'] = psr
                  
        lidar_xyz_center = psr_position_xyz.copy()
        lidar_xyz_extent = psr_scale_xyz.copy()
        lidar_xyz_euler_angles = psr_rotation_xyz.copy()

        if obj_type in colors: lidar_bbox_color = np.array([i / 255.0 for i in colors[obj_type]], dtype=np.uint8)
        else: lidar_bbox_color = np.array([0, 0, 0], dtype=np.uint8)
        
        label['lidar_bbox'] = {'lidar_xyz_center': lidar_xyz_center, 'lidar_xyz_extent': lidar_xyz_extent, 'lidar_xyz_euler_angles': lidar_xyz_euler_angles, 'rgb_bbox_color': lidar_bbox_color, 'predicted': False}
        
        if obj_type in colors: camera_bbox_color = np.array(colors[obj_type], dtype=np.uint8)
        else: camera_bbox_color = np.array([0, 0, 0], dtype=np.uint8)
        label['camera_bbox'] = {'lidar_xyz_center': lidar_xyz_center, 'lidar_xyz_extent': lidar_xyz_extent, 'lidar_xyz_euler_angles': lidar_xyz_euler_angles, 'rgb_bbox_color': camera_bbox_color, 'predicted': False}
        
        output.append(label)
    
    return output, calib

def __read_calib__(calib_path: str):
    if os.path.exists(calib_path) == False: return None
    calib = {}

    with open(calib_path, 'r') as f: calib = json.load(f)
    extrinsic_matrix  = np.reshape(calib['extrinsic'], [4,4]) # Tr_velo_to_cam
    intrinsic_matrix  = np.reshape(calib['intrinsic'], [3,3]) # P2
    
    calib['P2'] = intrinsic_matrix # 3x3
    calib['P2'] = np.hstack((calib['P2'], np.array([[0], [0], [0]], dtype=np.float32))) # 3x4
    calib['R0_rect'] = np.eye(4, dtype=np.float32)
    calib['Tr_velo_to_cam'] = extrinsic_matrix

    return calib