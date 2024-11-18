import open3d.visualization.gui as gui
from liguard.gui.logger_gui import Logger

def test_remove_out_of_bound_labels():
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
    func = __import__('liguard.algo.label', fromlist=['remove_out_of_bound_labels']).remove_out_of_bound_labels

    # crop bound
    cfg_dict['proc']['lidar']['crop']['enabled'] = True
    cfg_dict['proc']['lidar']['crop']['max_xyz'] = [10, 10, 10]
    cfg_dict['proc']['lidar']['crop']['min_xyz'] = [0, 0, 0]

    # enable
    cfg_dict['proc']['label']['remove_out_of_bound_labels']['enabled'] = True
    cfg_dict['proc']['label']['remove_out_of_bound_labels']['use_lidar_range'] = True
    
    # create dummy data
    data_dict['current_label_list'] = []

    # add a label that is out of bound
    data_dict['current_label_list'].append({'bbox_3d': {'xyz_center': [11, 11, 11]}})
    # add anotehr label that is out of bound
    data_dict['current_label_list'].append({'bbox_3d': {'xyz_center': [-1, -1, -1]}})
    # add another label that is out of bound
    data_dict['current_label_list'].append({'bbox_3d': {'xyz_center': [5, 5, 11]}})
    # add another label that is out of bound
    data_dict['current_label_list'].append({'bbox_3d': {'xyz_center': [5, 11, 5]}})
    # add another label that is out of bound
    data_dict['current_label_list'].append({'bbox_3d': {'xyz_center': [11, 5, 5]}})
    # add a label that is within bound
    data_dict['current_label_list'].append({'bbox_3d': {'xyz_center': [5, 5, 5]}})
    # add another label that is within bound
    data_dict['current_label_list'].append({'bbox_3d': {'xyz_center': [0, 0, 0]}})
    # add another label that is within bound
    data_dict['current_label_list'].append({'bbox_3d': {'xyz_center': [10, 10, 10]}})

    # run the function
    func(data_dict, cfg_dict, logger)

    # check if the label list is updated
    assert len(data_dict['current_label_list']) == 3, f'Expected 3 labels, got {len(data_dict["current_label_list"])}'

