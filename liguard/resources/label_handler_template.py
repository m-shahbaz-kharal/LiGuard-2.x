import os
import numpy as np

# colors for visualization, [R, G, B] in range [0.0, 1.0] where 0 is the darkest and 1 is the brightest
colors = {
    # 'CLASS_A': [0, 1, 0],
    # 'CLASS_B': [0, 0, 1],
    # ...
}
label_file_extension = '.txt' # this tells the extension of the label files in directory given by config['data']['label_subdir']

import os
import numpy as np

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

    ###############################################################################
    # basic code snippet to read the label file, uncomment and modify as needed
    ###############################################################################
    if calib_data is None: return output
    if not os.path.exists(label_path): return output
    with open(label_path, 'r') as f: lbls = f.readlines()
    
    ################################################################################
    # basic code snippet to populate the output list, uncomment and modify as needed
    ################################################################################
    # for line in lbls:
    #     parts = line.strip().split(' ')
    #     obj_class = ?
    #     # do your processing here
    #     # project to lidar coordinates and calculate xyz_center, xyz_extent, xyz_euler_angles
    #     xyz_center = ?
    #     xyz_extent = ?
    #     xyz_euler_angles = ?
    #     rgb_color = np.array(colors[obj_class], dtype=np.float32)
        
    #     # visualzer expect bbox_3d to be present in order to visualize the bounding boxes, so we add them here
    #     label = dict()
    #     label['bbox_3d'] = {
    #         'xyz_center': xyz_center,
    #         'xyz_extent': xyz_extent,
    #         'xyz_euler_angles': xyz_euler_angles,
    #         'rgb_color': rgb_color,
    #         'predicted': False # as it is a ground truth label
    #     }
        
    #     output.append(label)
    
    return output # make sure to return the output list, even if it is empty