import os
import sys
import time

from liguard.gui.gui_utils import resolve_for_application_root, resolve_for_default_workspace

import argparse
import yaml
from tqdm import tqdm

from liguard.pcd.file_io import FileIO as PCD_File_IO
from liguard.img.file_io import FileIO as IMG_File_IO
from liguard.calib.file_io import FileIO as CLB_File_IO
from liguard.lbl.file_io import FileIO as LBL_File_IO

from liguard.gui.logger_gui import Logger

import dask
from dask import delayed, compute
from dask.diagnostics import ProgressBar


def bulk_process(args):
    # load config
    pipeline_dir = args.pipeline_dir
    base_cfg_path = os.path.join(pipeline_dir, 'base_config.yml')
    with open(base_cfg_path) as f:cfg = yaml.safe_load(f)
    cfg['data']['pipeline_dir'] = pipeline_dir

    # load custom algorithms
    custom_algo_dir = os.path.join(pipeline_dir, 'algo')
    custom_algos_cfg = dict()
    if os.path.exists(custom_algo_dir):
        for algo_type_path in os.listdir(custom_algo_dir):
            algo_type_path = os.path.join(custom_algo_dir, algo_type_path)
            if not os.path.isdir(algo_type_path): continue
            if algo_type_path not in sys.path: sys.path.append(algo_type_path)
            algo_type = os.path.basename(algo_type_path)
            if algo_type not in custom_algos_cfg: custom_algos_cfg[algo_type] = dict()
            for algo_file_name in os.listdir(algo_type_path):
                if not algo_file_name.endswith('.yml'): continue
                with open(os.path.join(algo_type_path, algo_file_name)) as f: cust_algo_cfg = yaml.safe_load(f)
                custom_algos_cfg[algo_type].update(cust_algo_cfg)
            cfg['proc'][algo_type].update(custom_algos_cfg[algo_type])

    # init logger
    logger = Logger()
    if cfg['logging']['level'] < Logger.WARNING:
        print('Logging level is too low. Setting to WARNING to prevent spam.')
        cfg['logging']['level'] = Logger.WARNING
    logger.reset(cfg)

    # create output directory
    data_outputs_dir = cfg['data']['outputs_dir']
    if not os.path.isabs(data_outputs_dir): data_outputs_dir = os.path.join(pipeline_dir, data_outputs_dir)
    os.makedirs(data_outputs_dir, exist_ok=True)

    # init data readers
    pcd_reader = PCD_File_IO(cfg) if cfg['data']['lidar']['enabled'] else None
    img_reader = IMG_File_IO(cfg) if cfg['data']['camera']['enabled'] else None
    clb_reader = CLB_File_IO(cfg) if cfg['data']['calib']['enabled'] else None
    lbl_reader = LBL_File_IO(cfg, clb_reader.__getitem__ if clb_reader else None) if cfg['data']['label']['enabled'] else None

    # delayed data readers
    d_pcd_reader = delayed(pcd_reader.__getitem__) if pcd_reader else None
    d_img_reader = delayed(img_reader.__getitem__) if img_reader else None
    d_clb_reader = delayed(clb_reader.__getitem__) if clb_reader else None
    d_lbl_reader = delayed(lbl_reader.__getitem__) if lbl_reader else None

    # delayed data reader tasks
    pcd_reader_tasks = [d_pcd_reader(i) for i in range(len(pcd_reader))] if pcd_reader else None
    img_reader_tasks = [d_img_reader(i) for i in range(len(img_reader))] if img_reader else None
    clb_reader_tasks = [d_clb_reader(i) for i in range(len(clb_reader))] if clb_reader else None
    lbl_reader_tasks = [d_lbl_reader(i) for i in range(len(lbl_reader))] if lbl_reader else None

    # form data dict
    @delayed
    def form_data_dict(pcd_path_and_data=None, img_path_and_data=None, clb_path_and_data=None, lbl_path_and_data=None):
        data_dict = {}
        if pcd_path_and_data:
            data_dict['current_point_cloud_path'] = pcd_path_and_data[0]
            data_dict['current_point_cloud_numpy'] = pcd_path_and_data[1]
        if img_path_and_data:
            data_dict['current_image_path'] = img_path_and_data[0]
            data_dict['current_image_numpy'] = img_path_and_data[1]
        if clb_path_and_data:
            data_dict['current_calib_path'] = clb_path_and_data[0]
            data_dict['current_calib_data'] = clb_path_and_data[1]
        if lbl_path_and_data:
            data_dict['current_label_path'] = lbl_path_and_data[0]
            data_dict['current_label_list'] = lbl_path_and_data[1]
        return data_dict

    min_len = min(
        len(pcd_reader) if pcd_reader else float('inf'),
        len(img_reader) if img_reader else float('inf'),
        len(clb_reader) if clb_reader else float('inf'),
        len(lbl_reader) if lbl_reader else float('inf'),
    )

    merged_tasks = [
        form_data_dict(
            pcd_reader_tasks[i] if pcd_reader else None,
            img_reader_tasks[i] if img_reader else None,
            clb_reader_tasks[i] if clb_reader else None,
            lbl_reader_tasks[i] if lbl_reader else None,
        )
        for i in range(min_len)
    ]

    # pre process
    pre_processes_dict = dict()
    built_in_pre_modules = __import__('liguard.algo.pre', fromlist=['*']).__dict__
    for proc in cfg['proc']['pre']:
        if not cfg['proc']['pre'][proc]['enabled']: continue
        order = cfg['proc']['pre'][proc]['order']
        if proc in built_in_pre_modules: process = built_in_pre_modules[proc]
        else: process = __import__(proc, fromlist=['*']).__dict__[proc]
        pre_processes_dict[order] = process
    pre_processes = [pre_processes_dict[order] for order in sorted(pre_processes_dict.keys())]

    @delayed
    def run_pre_processing(data):
        for func in pre_processes:
            func(data, cfg, logger)
        return data
    pre_processed_tasks = [run_pre_processing(task) for task in merged_tasks]

    # lidar process
    lidar_processes_dict = dict()
    built_in_lidar_modules = __import__('liguard.algo.lidar', fromlist=['*']).__dict__
    for proc in cfg['proc']['lidar']:
        if not cfg['proc']['lidar'][proc]['enabled']: continue
        order = cfg['proc']['lidar'][proc]['order']
        if proc in built_in_lidar_modules: process = built_in_lidar_modules[proc]
        else: process = __import__(proc, fromlist=['*']).__dict__[proc]
        lidar_processes_dict[order] = process
    lidar_processes = [lidar_processes_dict[order] for order in sorted(lidar_processes_dict.keys())]

    @delayed
    def run_lidar_processing(data):
        for func in lidar_processes:
            func(data, cfg, logger)
        return data

    lidar_processed_tasks = [run_lidar_processing(task) for task in pre_processed_tasks]

    # camera process
    camera_processes_dict = dict()
    built_in_camera_modules = __import__('liguard.algo.camera', fromlist=['*']).__dict__
    for proc in cfg['proc']['camera']:
        if not cfg['proc']['camera'][proc]['enabled']: continue
        order = cfg['proc']['camera'][proc]['order']
        if proc in built_in_camera_modules: process = built_in_camera_modules[proc]
        else: process = __import__(proc, fromlist=['*']).__dict__[proc]
        camera_processes_dict[order] = process
    camera_processes = [camera_processes_dict[order] for order in sorted(camera_processes_dict.keys())]

    @delayed
    def run_camera_processing(data):
        for func in camera_processes:
            func(data, cfg, logger)
        return data

    camera_processed_tasks = [run_camera_processing(task) for task in lidar_processed_tasks]

    # calib process
    calib_processes_dict = dict()
    built_in_calib_modules = __import__('liguard.algo.calib', fromlist=['*']).__dict__
    for proc in cfg['proc']['calib']:
        if not cfg['proc']['calib'][proc]['enabled']: continue
        order = cfg['proc']['calib'][proc]['order']
        if proc in built_in_calib_modules: process = built_in_calib_modules[proc]
        else: process = __import__(proc, fromlist=['*']).__dict__[proc]
        calib_processes_dict[order] = process
    calib_processes = [calib_processes_dict[order] for order in sorted(calib_processes_dict.keys())]

    @delayed
    def run_calib_processing(data):
        for func in calib_processes:
            func(data, cfg, logger)
        return data

    calib_processed_tasks = [run_calib_processing(task) for task in camera_processed_tasks]

    # label process
    label_processes_dict = dict()
    built_in_label_modules = __import__('liguard.algo.label', fromlist=['*']).__dict__
    for proc in cfg['proc']['label']:
        if not cfg['proc']['label'][proc]['enabled']: continue
        order = cfg['proc']['label'][proc]['order']
        if proc in built_in_label_modules: process = built_in_label_modules[proc]
        else: process = __import__(proc, fromlist=['*']).__dict__[proc]
        label_processes_dict[order] = process
    label_processes = [label_processes_dict[order] for order in sorted(label_processes_dict.keys())]

    @delayed
    def run_label_processing(data):
        for func in label_processes:
            func(data, cfg, logger)
        return data

    label_processed_tasks = [run_label_processing(task) for task in calib_processed_tasks]

    # post process
    post_processes_dict = dict()
    built_in_post_modules = __import__('liguard.algo.post', fromlist=['*']).__dict__
    for proc in cfg['proc']['post']:
        if not cfg['proc']['post'][proc]['enabled']: continue
        order = cfg['proc']['post'][proc]['order']
        if proc in built_in_post_modules: process = built_in_post_modules[proc]
        else: process = __import__(proc, fromlist=['*']).__dict__[proc]
        post_processes_dict[order] = process
    post_processes = [post_processes_dict[order] for order in sorted(post_processes_dict.keys())]

    @delayed
    def run_postprocessing(data):
        for func in post_processes:
            func(data, cfg, logger)
        return data

    post_processed_tasks = [run_postprocessing(task) for task in label_processed_tasks]

    # results
    with ProgressBar():
        results = compute(*post_processed_tasks, scheduler='threads', num_workers=args.num_workers)
        if args.save_raw_results:
            save_raw_results(results, data_outputs_dir)
            
def save_raw_results(results, data_outputs_dir):
    # data related imports
    import open3d as o3d
    import cv2

    # create output dirs
    results_dir = os.path.join(data_outputs_dir, 'raw_results', time.strftime("%Y%m%d-%H%M%S"))
    print(f'Saving raw results to {results_dir} ...')
    results_pcd_dir = os.path.join(results_dir, 'point_clouds')
    os.makedirs(results_pcd_dir, exist_ok=True)
    results_img_dir = os.path.join(results_dir, 'images')
    os.makedirs(results_img_dir, exist_ok=True)
    results_clb_dir = os.path.join(results_dir, 'calibs')
    os.makedirs(results_clb_dir, exist_ok=True)
    results_lbl_dir = os.path.join(results_dir, 'labels')
    os.makedirs(results_lbl_dir, exist_ok=True)
    os.makedirs(results_dir, exist_ok=True)

    # save results
    for i, result in tqdm(enumerate(results), total=len(results), unit='frames'):
        # point clouds
        current_point_cloud_path = result.get('current_point_cloud_path', None)
        if current_point_cloud_path is not None:
            output_point_cloud_path = os.path.join(results_pcd_dir, os.path.basename(current_point_cloud_path).split('.')[0] + '.pcd')
            current_point_cloud_numpy = result.get('current_point_cloud_numpy', None)
            if current_point_cloud_numpy is not None:
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(current_point_cloud_numpy[:, :3])
                o3d.io.write_point_cloud(output_point_cloud_path, pcd)
                
        # images
        current_image_path = result.get('current_image_path', None)
        if current_image_path is not None:
            output_image_path = os.path.join(results_img_dir, os.path.basename(current_image_path).split('.')[0] + '.png')
            current_image_numpy = result.get('current_image_numpy', None)
            if current_image_numpy is not None:
                if current_image_numpy.shape[2] == 3:
                    current_image_numpy = cv2.cvtColor(current_image_numpy, cv2.COLOR_RGB2BGR)
                    cv2.imwrite(output_image_path, current_image_numpy)
                elif current_image_numpy.shape[2] == 4:
                    current_image_numpy = cv2.cvtColor(current_image_numpy, cv2.COLOR_RGBA2BGR)
                    cv2.imwrite(output_image_path, current_image_numpy)

        # calibration data
        current_calib_path = result.get('current_calib_path', None)
        if current_calib_path is not None:
            output_calib_path = os.path.join(results_clb_dir, os.path.basename(current_calib_path).split('.')[0] + '.yaml')
            current_calib_data = result.get('current_calib_data', None)
            if current_calib_data is not None:
                with open(output_calib_path, 'w') as f:
                    yaml.dump(current_calib_data, f)

        # label data
        current_label_path = result.get('current_label_path', None)
        if current_label_path is not None:
            output_label_path = os.path.join(results_lbl_dir, os.path.basename(current_label_path).split('.')[0] + '.yaml')
            current_label_list = result.get('current_label_list', None)
            if current_label_list is not None:
                with open(output_label_path, 'w') as f:
                    yaml.dump(current_label_list, f)

def main():
    # banner
    banner = \
    """
    #######################################################
        _      _  _____                     _   ___   
        | |    (_)/ ____|                   | | |__ \  
        | |     _| |  __ _   _  __ _ _ __ __| |    ) |
        | |    | | | |_ | | | |/ _` | '__/ _` |   / / 
        | |____| | |__| | |_| | (_| | | | (_| |  / /_
        |______|_|\_____|\__,_|\__,_|_|  \__,_| |____|
                           Headless Bulk Processing Utility
    #######################################################
    LiGuard's utility for no-GUI bulk data processing.
    """
    print(banner)
    description = \
    """
    Description:
    Once you have created a pipeline using LiGuard's interactive
    interface, you can pass your pipeline folder path and use this script
    to process the entire dataset faster. This script processes the data faster
    by utilizing dask for parallelism, and removing GUI and other interactive elements.

    Note 1: Currently, this doesn't work with live sensor data streams.
    Note 2: Currently, this doesn't work with multi-frame dependent algorithms
    such as calculating background filters using multiple frames (you can use a pre-calculated filter though), tracking, etc.
    """

    # argument parse
    parser = argparse.ArgumentParser(description=f'{description}')
    parser.add_argument('pipeline_dir', type=str, help='Path to the pipleine directory.')
    parser.add_argument('--num_workers', type=int, default=4, help='Number of workers to use. Default: 4')
    parser.add_argument('--save_raw_results', action='store_true', help='Save raw processed data dictionary to the output directory. Default: False')
    args = parser.parse_args()

    # copy examples
    default_workspace_dir = resolve_for_default_workspace('')
    if not os.path.exists(default_workspace_dir): os.makedirs(default_workspace_dir)
    if not os.path.exists(os.path.join(default_workspace_dir, 'examples')):
        import shutil
        shutil.copytree(resolve_for_application_root('examples'), resolve_for_default_workspace('examples'))
    
    # bulk process
    try: bulk_process(args)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt: Exiting, Please Wait ...")
        sys.exit(1)
    
    # complete
    print('Processing complete.')

if __name__ == '__main__':
    main()