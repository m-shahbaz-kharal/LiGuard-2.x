# colors for visualization, [R, G, B] in range [0.0, 1.0] where 0 is the darkest and 1 is the brightest
colors = {
    'Person': [0.0, 1.0, 0.0],
    'Car': [1.0, 0.0, 0.0],
    'Bicycle': [0.0, 0.0, 1.0],
    'Motorcycle': [1.0, 1.0, 0.0],
    'Bus': [0.0, 1.0, 1.0],
    'Truck': [1.0, 0.0, 1.0],
    'Unknown': [1.0, 1.0, 1.0]
}
id2class = {
    0: 'Person',
    1: 'Car',
    2: 'Bicycle',
    3: 'Motorcycle',
    4: 'Bus',
    5: 'Truck',
    6: 'Unknown'
}
label_file_extension = '.ply' # this tells the extension of the label files in directory given by config['data']['label_subdir']

import os
import numpy as np

from liguard.lbl.lumpi_sdk.LumpiParser import LumpiParser

dataset_path = "E:\\work\\urbanity\\hands_on\\datasets\\lumpi"

measurement_id = 4
data_parser = LumpiParser(dataset_path)
data_parser.read_track(os.path.join(data_parser.path,"Measurement"+str(measurement_id),"Label.csv"))

def Handler(label_path: str, calib_data: dict): # don't change the function signature
    """
    Process the label file and generate a list of labels.

    Args:
        label_path (str): Path to the label file.
        calib_data (dict): Calibration data.

    Returns:
        list: List of labels.

    """
    output = []
    
    ################################################################################
    # basic code snippet to populate the output list, uncomment and modify as needed
    ################################################################################
    index = int(os.path.basename(label_path).split('.')[0])
    for o in data_parser.indexOrdered[index].values():
        obj_class = id2class[o.classId]
        xyz_center = o.position.astype(np.float32)
        xyz_extent = o.dim.astype(np.float32)
        xyz_euler_angles = np.array([0, 0, o.heading], dtype=np.float32)
        rgb_color = np.array(colors[obj_class], dtype=np.float32)

        label = dict()
        label['bbox_3d'] = {
            'xyz_center': xyz_center,
            'xyz_extent': xyz_extent,
            'xyz_euler_angles': xyz_euler_angles,
            'rgb_color': rgb_color,
            'predicted': False # as it is a ground truth label
        }

        output.append(label)
    
    return output # make sure to return the output list, even if it is empty