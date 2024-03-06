import os
import numpy as np
import open3d as o3d
import json

import math
from calib.utils import nx3_to_nx4

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
    calib_file_name = os.path.basename(calib_path).split('.')[0]
    calib_path = calib_path.replace(calib_file_name, 'front') # front camera calib
    calib_exists = os.path.exists(calib_path)
    if calib_exists:
        with open(calib_path, 'r') as f: calib = json.load(f)
        extrinsic_matrix  = np.reshape(calib['extrinsic'], [4,4])
        intrinsic_matrix  = np.reshape(calib['intrinsic'], [3,3])

    # read label file
    if os.path.exists(label_path) == False: return output
    with open(label_path, 'r') as f: lbls = json.load(f)
    for item in lbls:
        annotator = item['annotator'] if 'annotator' in item else 'Unknown'
        obj_id = int(item['obj_id'])
        obj_type = item['obj_type']
        psr = item['psr']
        psr_position_xyz = [float(psr['position']['x']), float(psr['position']['y']), float(psr['position']['z'])]
        psr_rotation_xyz = [float(psr['rotation']['x']), float(psr['rotation']['y']), float(psr['rotation']['z'])]
        psr_scale_xyz = [float(psr['scale']['x']), float(psr['scale']['y']), float(psr['scale']['z'])]
        
        label = dict()
        label['annotator'] = annotator
        label['id'] = obj_id
        label['type'] = obj_type
        label['psr'] = psr
        if calib_exists:
            label['calib'] = calib
            label['calib']['P2'] = nx3_to_nx4(intrinsic_matrix)
        
        lidar_xyz_center = np.array(psr_position_xyz, dtype=np.float32)
        lidar_wlh_extent = np.array(psr_scale_xyz, dtype=np.float32)
        lidar_rotation_matrix = o3d.geometry.OrientedBoundingBox.get_rotation_matrix_from_xyz(psr_rotation_xyz)

        if obj_type in colors: lidar_bbox_color = [i / 255.0 for i in colors[obj_type]]
        else: lidar_bbox_color = [0, 0, 0]
        
        label['lidar_bbox'] = {'xyz_center': lidar_xyz_center, 'wlh_extent': lidar_wlh_extent, 'xyz_rotation_matrix': lidar_rotation_matrix, 'rgb_bbox_color': lidar_bbox_color}
        
        if calib_exists:
            pass
            # To-do: add camera bbox by creating camera_xyz_center, camera_wlh_extent, camera_rotation_matrix, camera_bbox_color
            # camera_xyz_center in camera coordinate system
            # camera_wlh_extent in object coordinate system
            # camera_rotation_matrix in camera coordinate system
            
            # if obj_type in colors: camera_bbox_color = colors[obj_type]
            # else: camera_bbox_color = [0, 0, 0]
            
            # label['camera_bbox'] = {'xyz_center': camera_xyz_center, 'wlh_extent': camera_wlh_extent, 'xyz_rotation_matrix': camera_rotation_matrix, 'rgb_bbox_color': camera_bbox_color}
        
        output.append(label)
    
    return output