import open3d.visualization.gui as gui
from liguard.gui.logger_gui import Logger

import numpy as np
import os, shutil

def test_create_per_object_pcdet_dataset():
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
    func = __import__('liguard.algo.post', fromlist=['create_per_object_pcdet_dataset']).create_per_object_pcdet_dataset

    # create dummy data
    point_cloud = (np.random.rand(10000, 4) - 0.5) * 10
    point_cloud[:, 3] = 1
    data_dict['current_point_cloud_numpy'] = point_cloud
    
    # run as it is running for bulk data
    for i in range(10):
        # create dummy path
        data_dict['current_point_cloud_path'] = os.path.join('tests', str(i).zfill(6) + '.bin')
        # create dummy label
        data_dict['current_label_list'] = [
            {'bbox_3d': {'xyz_center': (np.random.rand(3)-0.5) * 5, 'xyz_extent': [1, 1, 1], 'xyz_euler_angles': [0, 0, 1]}},
            {'bbox_3d': {'xyz_center': (np.random.rand(3)-0.5) * 5, 'xyz_extent': [2, 1, 0.5], 'xyz_euler_angles': [0, 0, 1]}},
            {'bbox_3d': {'xyz_center': (np.random.rand(3)-0.5) * 5, 'xyz_extent': [1, 2, 0.5], 'xyz_euler_angles': [0, 0, 1]}},
        ]
        data_dict['current_label_path'] = str(i).zfill(4) + '.txt'

        # run the function
        func(data_dict, cfg_dict, logger)

    # check if the output directories are created
    assert os.path.exists(os.path.join(cfg_dict['data']['pipeline_dir'], cfg_dict['data']['outputs_dir'], 'post', 'per_object_pcdet_dataset', 'point_cloud'))
    assert os.path.exists(os.path.join(cfg_dict['data']['pipeline_dir'], cfg_dict['data']['outputs_dir'], 'post', 'per_object_pcdet_dataset', 'label'))

    # count the number of files in the output directories
    point_cloud_files = os.listdir(os.path.join(cfg_dict['data']['pipeline_dir'], cfg_dict['data']['outputs_dir'], 'post', 'per_object_pcdet_dataset', 'point_cloud'))
    label_files = os.listdir(os.path.join(cfg_dict['data']['pipeline_dir'], cfg_dict['data']['outputs_dir'], 'post', 'per_object_pcdet_dataset', 'label'))
    assert len(point_cloud_files) == 30 # 3 labels for each of the 10 point clouds
    assert len(label_files) == 30 # 3 labels for each of the 10 point clouds
    
    # delete the output directories
    shutil.rmtree(os.path.join(cfg_dict['data']['pipeline_dir'], cfg_dict['data']['outputs_dir']))

def test_create_pcdet_dataset():
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
    func = __import__('liguard.algo.post', fromlist=['create_pcdet_dataset']).create_pcdet_dataset

    # create dummy data
    point_cloud = (np.random.rand(10000, 4) - 0.5) * 10
    point_cloud[:, 3] = 1
    data_dict['current_point_cloud_numpy'] = point_cloud
    
    # run as it is running for bulk data
    for i in range(10):
        # create dummy path
        data_dict['current_point_cloud_path'] = os.path.join('tests', str(i).zfill(6) + '.bin')
        # create dummy label
        data_dict['current_label_list'] = [
            {'bbox_3d': {'xyz_center': (np.random.rand(3)-0.5) * 5, 'xyz_extent': [1, 1, 1], 'xyz_euler_angles': [0, 0, 1]}},
            {'bbox_3d': {'xyz_center': (np.random.rand(3)-0.5) * 5, 'xyz_extent': [2, 1, 0.5], 'xyz_euler_angles': [0, 0, 1]}},
            {'bbox_3d': {'xyz_center': (np.random.rand(3)-0.5) * 5, 'xyz_extent': [1, 2, 0.5], 'xyz_euler_angles': [0, 0, 1]}},
        ]
        data_dict['current_label_path'] = str(i).zfill(4) + '.txt'

        # run the function
        func(data_dict, cfg_dict, logger)

    # check if the output directories are created
    assert os.path.exists(os.path.join(cfg_dict['data']['pipeline_dir'], cfg_dict['data']['outputs_dir'], 'post', 'pcdet_dataset', 'point_cloud'))
    assert os.path.exists(os.path.join(cfg_dict['data']['pipeline_dir'], cfg_dict['data']['outputs_dir'], 'post', 'pcdet_dataset', 'label'))

    # count the number of files in the output directories
    point_cloud_files = os.listdir(os.path.join(cfg_dict['data']['pipeline_dir'], cfg_dict['data']['outputs_dir'], 'post', 'pcdet_dataset', 'point_cloud'))
    label_files = os.listdir(os.path.join(cfg_dict['data']['pipeline_dir'], cfg_dict['data']['outputs_dir'], 'post', 'pcdet_dataset', 'label'))
    assert len(point_cloud_files) == 10 # input point clouds = output point clouds
    assert len(label_files) == 10 # each point cloud has one label file containing 3 labels
    
    # delete the output directories
    shutil.rmtree(os.path.join(cfg_dict['data']['pipeline_dir'], cfg_dict['data']['outputs_dir']))