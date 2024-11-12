import os
import numpy as np

colors = {
    'Bus': [0, 1, 0],
    'Car': [0, 1, 0],
    'Cyclist': [0, 0, 1],
    'Engineeringcar': [1, 0, 0],
    'Pedestrian': [1, 0, 0],
    'Tricycle': [1, 1, 0],
    'Truck': [0, 1, 0]
}
label_file_extension = '.txt'

import os
import numpy as np

def Handler(label_path: str, calib_data: dict):
    """
    Process the label file and generate a list of labels.

    Args:
        label_path (str): Path to the label file.
        calib_data (dict): Calibration data.

    Returns:
        list: List of labels.

    """
    output = []
    
    # Read label file
    if not os.path.exists(label_path):
        return output
    with open(label_path, 'r') as f:
        lbls = f.readlines()
    
    for line in lbls:
        parts = line.strip().split(' ')
        obj_type = parts[0]  # Describes the type of object: 'Car', 'Truck', 'Pedestrian', 'Cyclist', 'Tricycle', 'Car', 'Bus','Truck', 'Engineeringcar'
        occluded = float(parts[1])  # Set to -1
        reserved = float(parts[2])  # Set to -1
        alpha = float(parts[3]) # Observation angle of object, ranging [-pi..pi]
        left0, top0, right0, bottom0 = np.array([float(x) for x in parts[4:8]], dtype=np.float32)  # 2D bounding box of object in the image from cam1 (0-based index): contains left, top, right, bottom pixel coordinates.
        left1, top1, right1, bottom1 = np.array([float(x) for x in parts[8:12]], dtype=np.float32)  # 2D bounding box of object in the image from cam2 (0-based index): contains left, top, right, bottom pixel coordinates.
        height, width, length = np.array([float(x) for x in parts[12:15]], dtype=np.float32)  # 3D object dimensions: height, width, length (in meters).
        lidar_0_xyz = np.array([float(x) for x in parts[15:18]], dtype=np.float32) # 3D object location x,y,z in lidar coordinates (in meters).
        image_0_ry = float(parts[18]) # Rotation ry around Y-axis in camera coordinates [-pi..pi].

        label = dict()
        label['class'] = obj_type
        label['occlusion'] = occluded
        label['reserved'] = reserved
        label['alpha'] = alpha
        label['image_0_bbox2d'] = [left0, top0, right0, bottom0]
        label['image_1_bbox2d'] = [left1, top1, right1, bottom1]
        label['obj_height'] = height
        label['obj_width'] = width
        label['obj_length'] = length
        label['lidar_0_xyz'] = lidar_0_xyz
        label['image_0_ry'] = image_0_ry
        
        # project to lidar coordinates
        xyz_center = np.array([lidar_0_xyz[2], -lidar_0_xyz[0], -lidar_0_xyz[1]], dtype=np.float32)
        # Adjust the height of the bounding box since the origin of the lidar coordinates is at the bottom of the vehicle
        xyz_center[2] += width / 2.0
        # w l h -> x y z
        xyz_extent = np.array([width, length, height], dtype=np.float32)
        xyz_euler_angles = np.array([0, 0, -image_0_ry], dtype=np.float32)
        rgb_color = np.array(colors[obj_type], dtype=np.float32)
        
        # visualzer expect bbox_3d to be present in order to visualize the bounding boxes, so we add them here
        label['bbox_3d'] = {'xyz_center': xyz_center, 'xyz_extent': xyz_extent, 'xyz_euler_angles': xyz_euler_angles, 'rgb_color': rgb_color, 'predicted': False}
        
        output.append(label)
    
    return output