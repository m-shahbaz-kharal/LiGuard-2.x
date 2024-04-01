import os
import numpy as np

colors = {'Green': [0, 1, 0]}
label_file_extension = '.txt'
calib_file_extension = '.txt' # not used

def Handler(label_path: str, calib_path: str):
    output = []
    
    # read label file
    if os.path.exists(label_path) == False: return output
    with open(label_path, 'r') as f: lbls = f.readlines()
    for line in lbls:
        parts = line.strip().split(' ')
        xyz_dxdydz_rz = np.array([float(i) for i in parts[0:7]], dtype=np.float32)
        obj_class = parts[7]
        
        label = dict()
        label['x'] = xyz_dxdydz_rz[0]
        label['y'] = xyz_dxdydz_rz[1]
        label['z'] = xyz_dxdydz_rz[2]
        label['dx'] = xyz_dxdydz_rz[3]
        label['dy'] = xyz_dxdydz_rz[4]
        label['dz'] = xyz_dxdydz_rz[5]
        label['heading_angle'] = xyz_dxdydz_rz[6]
        label['category_name'] = obj_class
        
        lidar_xyz_center = xyz_dxdydz_rz[0:3]
        lidar_xyz_extent = xyz_dxdydz_rz[3:6]
        lidar_xyz_euler_angles = np.array([0, 0, xyz_dxdydz_rz[6]], dtype=np.float32)
        lidar_bbox_color = np.array(colors['Green'], dtype=np.uint8)
        
        label['lidar_bbox'] = {'lidar_xyz_center': lidar_xyz_center, 'lidar_xyz_extent': lidar_xyz_extent, 'lidar_xyz_euler_angles': lidar_xyz_euler_angles, 'rgb_bbox_color': lidar_bbox_color, 'predicted': False}
        
        output.append(label)
    
    return output, None