import os
import numpy as np

colors = {
    'Car': [0, 1, 0],
    'Van': [0, 1, 0],
    'Truck': [0, 1, 0],
    'Pedestrian': [1, 0, 0],
    'Person_sitting': [1, 0, 0],
    'Cyclist': [0, 0, 1],
    'Tram': [1, 1, 0],
    'Misc': [1, 1, 0],
    'DontCare': [1, 1, 1],
    'Unknown': [1, 1, 1]
}
label_file_extension = '.txt'

import os
import numpy as np

def Handler(label_path: str, calib_data: dict):
    """
    Process the label file and convert it into a list of dictionaries representing the labels.

    Args:
        label_path (str): The path to the label file.
        calib_data (dict): Calibration data.

    Returns:
        list: A list of dictionaries representing the labels.
    """
    output = []
    
    # Check if the label file exists
    if os.path.exists(label_path) == False:
        return output
    
    # Read the label file
    with open(label_path, 'r') as f:
        lbls = f.readlines()
    
    for line in lbls:
        parts = line.strip().split(' ')
        xyz_dxdydz_rz = np.array([float(i) for i in parts[0:7]], dtype=np.float32)
        obj_class = parts[7]
        
        # Create a dictionary to store the label information
        label = dict()
        label['x'] = xyz_dxdydz_rz[0]
        label['y'] = xyz_dxdydz_rz[1]
        label['z'] = xyz_dxdydz_rz[2]
        label['dx'] = xyz_dxdydz_rz[3]
        label['dy'] = xyz_dxdydz_rz[4]
        label['dz'] = xyz_dxdydz_rz[5]
        label['heading_angle'] = xyz_dxdydz_rz[6]
        label['class'] = obj_class
        
        xyz_center = xyz_dxdydz_rz[0:3]
        xyz_extent = xyz_dxdydz_rz[3:6]
        xyz_euler_angles = np.array([0, 0, xyz_dxdydz_rz[6]], dtype=np.float32)
        rgb_color = np.array(colors[obj_class], dtype=np.float32)
        
        # Create a dictionary to store the 3D bounding-box information
        label['bbox_3d'] = {
            'xyz_center': xyz_center,
            'xyz_extent': xyz_extent,
            'xyz_euler_angles': xyz_euler_angles,
            'rgb_color': rgb_color,
            'predicted': False
        }
        
        output.append(label)
    
    return output