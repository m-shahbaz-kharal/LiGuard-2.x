import open3d.visualization.gui as gui
from liguard.gui.logger_gui import Logger

import numpy as np

def test_project_point_cloud_points():
    # create dummy configuration and data dictionaries
    import os, yaml
    example_config_path = os.path.join('liguard', 'examples', 'simple_pipeline', 'base_config.yml')
    with open(example_config_path, 'r') as f: cfg_dict = yaml.safe_load(f)
    cfg_dict['data']['pipeline_dir'] = os.path.join('liguard', 'examples', 'simple_pipeline')
    data_dict = {}

    # create a logger object as it is required by some algorithms
    logger = Logger()
    logger.reset(cfg_dict)

    # import the function
    func = __import__('liguard.algo.camera', fromlist=['project_point_cloud_points']).project_point_cloud_points
    
    # create dummy calibration data
    P2 = np.eye(3, 4)
    R0_rect = np.eye(4)
    Tr_velo_to_cam = np.eye(4)
    data_dict['current_calib_data'] = {'P2': P2, 'R0_rect': R0_rect, 'Tr_velo_to_cam': Tr_velo_to_cam}
    
    # Create dummy point cloud
    point_cloud = np.zeros((100, 3))

    # tests
    # all points lie in the x-y plane, are in image bounds, and are in front of the camera
    data_dict['current_image_numpy'] = np.zeros((100, 100, 3), dtype=np.uint8)
    x = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
    y = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
    z = 1.
    x, y = np.meshgrid(x, y)
    x = x.flatten()
    y = y.flatten()
    point_cloud[:, 0] = x
    point_cloud[:, 1] = y
    point_cloud[:, 2] = z
    data_dict['current_point_cloud_numpy'] = point_cloud
    
    # run the function
    func(data_dict, cfg_dict, logger)

    # check the number of non-black pixels
    pixel_that_are_not_black_indices = np.where(np.any(data_dict['current_image_numpy'] != 0, axis=-1))
    number_of_non_black_pixels = len(pixel_that_are_not_black_indices[0])
    assert number_of_non_black_pixels == 100, f'Expected 100 non-black pixels, got {number_of_non_black_pixels}'

    # 19 points lie outside the image plane
    data_dict['current_image_numpy'] = np.zeros((100, 100, 3), dtype=np.uint8)
    x = [0, 1, 2, 3, 4, 5, 6, 7, 8, 900]
    y = [0, 1, 2, 3, 4, 5, 6, 7, 8, 900]
    z = 1.0
    x, y = np.meshgrid(x, y)
    x = x.flatten()
    y = y.flatten()
    point_cloud[:, 0] = x
    point_cloud[:, 1] = y
    point_cloud[:, 2] = z
    data_dict['current_point_cloud_numpy'] = point_cloud
    
    # run the function
    func(data_dict, cfg_dict, logger)
    
    # check the number of non-black pixels
    pixel_that_are_not_black_indices = np.where(np.any(data_dict['current_image_numpy'] != 0, axis=-1))
    number_of_non_black_pixels = len(pixel_that_are_not_black_indices[0])
    assert number_of_non_black_pixels == 81, f'Expected 81 non-black pixels, got {number_of_non_black_pixels}'

    # move the half of the points behind the camera
    data_dict['current_image_numpy'] = np.zeros((100, 100, 3), dtype=np.uint8)
    x = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
    y = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
    z = 1.0
    x, y = np.meshgrid(x, y)
    x = x.flatten()
    y = y.flatten()
    point_cloud[:, 0] = x
    point_cloud[:, 1] = y
    point_cloud[50:, 2] = -1.0
    point_cloud[:50, 2] = 1.0
    data_dict['current_point_cloud_numpy'] = point_cloud
    
    # run the function
    func(data_dict, cfg_dict, logger)
    
    # check the number of non-black pixels
    pixel_that_are_not_black_indices = np.where(np.any(data_dict['current_image_numpy'] != 0, axis=-1))
    number_of_non_black_pixels = len(pixel_that_are_not_black_indices[0])
    assert number_of_non_black_pixels == 50, f'Expected 50 non-black pixels, got {number_of_non_black_pixels}'

    # move all the points behind the camera
    data_dict['current_image_numpy'] = np.zeros((100, 100, 3), dtype=np.uint8)
    x = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
    y = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
    z = -1.0
    x, y = np.meshgrid(x, y)
    x = x.flatten()
    y = y.flatten()
    point_cloud[:, 0] = x
    point_cloud[:, 1] = y
    point_cloud[:, 2] = z
    data_dict['current_point_cloud_numpy'] = point_cloud
    
    # run the function
    func(data_dict, cfg_dict, logger)
    
    # check the number of non-black pixels
    pixel_that_are_not_black_indices = np.where(np.any(data_dict['current_image_numpy'] != 0, axis=-1))
    number_of_non_black_pixels = len(pixel_that_are_not_black_indices[0])
    assert number_of_non_black_pixels == 0, f'Expected 0 non-black pixels, got {number_of_non_black_pixels}'