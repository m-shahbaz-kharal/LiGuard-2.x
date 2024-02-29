import os
import numpy as np
import open3d as o3d

from calib.utils import inverse_3x4_transform, nx3_to_nx4

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
    
    # read calib
    calib_path = label_path.replace('label', 'calib')
    calib = __read_calib__(calib_path)
    transform_from_lidar_to_image_0 = calib['Tr_velo_to_cam'].reshape(3, 4)
    transform_from_image_0_to_image_2 = calib['P2'].reshape(3, 4)
    transform_from_image_0_to_lidar = inverse_3x4_transform(transform_from_lidar_to_image_0)
    
    # read label file
    if os.path.exists(label_path) == False: return output
    with open(label_path, 'r') as f: lbls = f.readlines()
    for line in lbls:
        parts = line.strip().split(' ')
        obj_class = parts[0] # object class [Car, Van, Truck, Pedestrian, Person_sitting, Cyclist, Tram, Misc, DontCare]
        truncation = float(parts[1]) # truncated pixel ratio [0..1]
        occlusion = int(parts[2]) # 0: fully visible, 1: partly occluded, 2: fully occluded, 3: unknown
        alpha = float(parts[3]) # object observation angle [-pi..pi]
        left, top, right, bottom = np.array([float(x) for x in parts[4:8]], dtype=np.float32) # 0-based 2D bounding box of object in the image
        height, width, length = np.array([float(x) for x in parts[8:11]], dtype=np.float32) # height, width, length in meters
        image_0_xyz = np.array([float(x) for x in parts[11:14]], dtype=np.float32) # location of object center in camera coordinates
        image_0_ry = float(parts[14]) # rotation around Y-axis in camera coordinates [-pi..pi]
        
        label = dict()
        label['class'] = obj_class
        label['truncation'] = truncation
        label['occlusion'] = occlusion
        label['alpha'] = alpha
        label['image_0_bbox2d'] = [left, top, right, bottom]
        label['obj_height'] = height
        label['obj_width'] = width
        label['obj_length'] = length
        label['image_0_xyz'] = image_0_xyz
        label['image_0_ry'] = image_0_ry
        label['transform_from_lidar_to_image_0'] = transform_from_lidar_to_image_0
        label['transform_from_image_0_to_image_2'] = transform_from_image_0_to_image_2
        
        lidar_xyz_center = transform_from_image_0_to_lidar @ nx3_to_nx4(image_0_xyz.reshape(1, 3)).T
        lidar_xyz_center = lidar_xyz_center.T[0]
        lidar_xyz_center[2] += height / 2.0
        lidar_wlh_extent = np.array([width, length, height], dtype=np.float32)
        lidar_around_z_rotation_matrix = o3d.geometry.OrientedBoundingBox.get_rotation_matrix_from_axis_angle([0, 0, -image_0_ry])
        lidar_bbox_color = colors[obj_class]
        
        label['lidar_bbox'] = {'xyz_center': lidar_xyz_center, 'wlh_extent': lidar_wlh_extent, 'around_z_rotation_matrix': lidar_around_z_rotation_matrix, 'rgb_bbox_color': lidar_bbox_color}
        
        camera_xyz_center = image_0_xyz
        camera_wlh_extent = np.array([width, length, height], dtype=np.float32)
        camera_around_y_rotation_matrix = o3d.geometry.OrientedBoundingBox.get_rotation_matrix_from_axis_angle([0, image_0_ry, 0])
        camera_bbox_color = [i * 255 for i in colors[obj_class]]
        
        label['camera_bbox'] = {'xyz_center': camera_xyz_center, 'wlh_extent': camera_wlh_extent, 'around_y_rotation_matrix': camera_around_y_rotation_matrix, 'rgb_bbox_color': camera_bbox_color}
        
        output.append(label)
    
    return output

def __read_calib__(calib_path: str):
    calib = {}
    with open(calib_path) as f:
        for line in f.readlines():
            line = line.strip()
            if len(line) == 0: continue
            k, v = line.split(':')
            calib[k] = np.array([float(x) for x in v.split()])
    return calib