import os
import sys
from liguard.gui.config_gui import resolve_for_application_root, resolve_for_default_workspace
import yaml

from liguard.pcd.file_io import FileIO as PCD_File_IO
from liguard.img.file_io import FileIO as IMG_File_IO
from liguard.calib.file_io import FileIO as CLB_File_IO
from liguard.lbl.file_io import FileIO as LBL_File_IO

from liguard.gui.logger_gui import Logger

import time
import signal
from threading import Thread, Event
from queue import Queue

from tqdm import tqdm

stop_event = Event()

def reader2queue(reader, size, target_queue):
    for idx in range(size):
        if stop_event.is_set():
            target_queue.put(None)
            break
        target_queue.put(reader[idx])

def queue2dict2queue(source_queue, key1, key2, target_queue):
    while True:
        data = source_queue.get()
        if data is None:
            target_queue.put(None)
            source_queue.task_done()
            break
        data_dict = {key1: data[0], key2: data[1]}
        target_queue.put(data_dict)
        source_queue.task_done()
    
def dicts2singledict(source_queues, target_queue, p_bar):
    while True:
        data = [source_queues[i].get() for i in range(len(source_queues))]
        if None in data:
            target_queue.put(None)
            for i in range(len(source_queues)): source_queues[i].task_done()
            break
        data_dict = dict()
        for d in data: data_dict.update(d)
        target_queue.put(data_dict)
        for i in range(len(source_queues)): source_queues[i].task_done()
        p_bar.update(1)

def dict2proc2dict(source_queue, cfg, logger, processes, target_queue, p_bar):
    while True:
        data = source_queue.get()
        if data is None:
            if target_queue: target_queue.put(None)
            source_queue.task_done()
            break
        for process in processes: process(data, cfg, logger)
        if target_queue: target_queue.put(data)
        source_queue.task_done()
        p_bar.update(1)

def signal_handler(sig, frame):
    print("\nCtrl + C detected! Stopping ...")
    stop_event.set()  # Signal all threads to stop

def bulk_process(args):
    pipeline_dir = args.pipeline_dir
    base_cfg_path = os.path.join(pipeline_dir, 'base_config.yml')
    with open(base_cfg_path) as f:cfg = yaml.safe_load(f)
    cfg['data']['pipeline_dir'] = pipeline_dir
    
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
    
    logger = Logger()
    if cfg['logging']['level'] < Logger.WARNING:
        print('Logging level is too low. Setting to WARNING to prevent spam.')
        cfg['logging']['level'] = Logger.WARNING
    logger.reset(cfg)

    # create dirs
    data_outputs_dir = cfg['data']['outputs_dir']
    if not os.path.isabs(data_outputs_dir): data_outputs_dir = os.path.join(pipeline_dir, data_outputs_dir)
    os.makedirs(data_outputs_dir, exist_ok=True)

    # signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # reader queues
    pcd_input_queue = Queue(maxsize=args.max_queue_size)
    img_input_queue = Queue(maxsize=args.max_queue_size)
    clb_input_queue = Queue(maxsize=args.max_queue_size)
    lbl_input_queue = Queue(maxsize=args.max_queue_size)

    # readers
    pcd_reader = PCD_File_IO(cfg) if cfg['data']['lidar']['enabled'] else None
    img_reader = IMG_File_IO(cfg) if cfg['data']['camera']['enabled'] else None
    clb_reader = CLB_File_IO(cfg) if cfg['data']['calib']['enabled'] else None
    lbl_reader = LBL_File_IO(cfg, clb_reader.__getitem__ if clb_reader else None) if cfg['data']['label']['enabled'] else None

    # reader threads
    if pcd_reader:
        pcd_io_thread = Thread(target=reader2queue, args=(pcd_reader, len(pcd_reader), pcd_input_queue))
        pcd_io_thread.start()
    if img_reader:
        img_io_thread = Thread(target=reader2queue, args=(img_reader, len(img_reader), img_input_queue))
        img_io_thread.start()
    if clb_reader:
        clb_io_thread = Thread(target=reader2queue, args=(clb_reader, len(clb_reader), clb_input_queue))
        clb_io_thread.start()
    if lbl_reader:
        lbl_io_thread = Thread(target=reader2queue, args=(lbl_reader, len(lbl_reader), lbl_input_queue))
        lbl_io_thread.start()

    # data dict queues
    pcd_data_dict_queue = Queue(maxsize=args.max_queue_size)
    img_data_dict_queue = Queue(maxsize=args.max_queue_size)
    clb_data_dict_queue = Queue(maxsize=args.max_queue_size)
    lbl_data_dict_queue = Queue(maxsize=args.max_queue_size)
    common_data_dict_queue = Queue(maxsize=args.max_queue_size)

    # queue to dict threads
    if pcd_reader:
        pcd_io_to_data_dict_thread = Thread(target=queue2dict2queue, args=(pcd_input_queue, 'current_point_cloud_path', 'current_point_cloud_numpy', pcd_data_dict_queue))
        pcd_io_to_data_dict_thread.start()
    if img_reader:
        img_io_to_data_dict_thread = Thread(target=queue2dict2queue, args=(img_input_queue, 'current_image_path', 'current_image_numpy', img_data_dict_queue))
        img_io_to_data_dict_thread.start()
    if clb_reader:
        clb_io_to_data_dict_thread = Thread(target=queue2dict2queue, args=(clb_input_queue, 'current_calib_path', 'current_calib_data', clb_data_dict_queue))
        clb_io_to_data_dict_thread.start()
    if lbl_reader:
        lbl_io_to_data_dict_thread = Thread(target=queue2dict2queue, args=(lbl_input_queue, 'current_label_path', 'current_label_list', lbl_data_dict_queue))
        lbl_io_to_data_dict_thread.start()
    
    # dict to single dict thread
    data_dicts = []
    min_len = 1e9
    if pcd_reader:
        if len(pcd_reader) < min_len: min_len = len(pcd_reader)
        data_dicts.append(pcd_data_dict_queue)
    if img_reader:
        if len(img_reader) < min_len: min_len = len(img_reader)
        data_dicts.append(img_data_dict_queue)
    if clb_reader:
        if len(clb_reader) < min_len: min_len = len(clb_reader)
        data_dicts.append(clb_data_dict_queue)
    if lbl_reader:
        if len(lbl_reader) < min_len: min_len = len(lbl_reader)
        data_dicts.append(lbl_data_dict_queue)
    reader_tqdm = tqdm(total=min_len, desc='Reading data', position=0)
    data_dict_thread = Thread(target=dicts2singledict, args=(data_dicts, common_data_dict_queue, reader_tqdm))
    data_dict_thread.start()

    # processes
    pre_processes_dict = dict()
    built_in_pre_modules = __import__('liguard.algo.pre', fromlist=['*']).__dict__
    for proc in cfg['proc']['pre']:
        if not cfg['proc']['pre'][proc]['enabled']: continue
        priority = cfg['proc']['pre'][proc]['priority']
        if proc in built_in_pre_modules: process = built_in_pre_modules[proc]
        else: process = __import__(proc, fromlist=['*']).__dict__[proc]
        pre_processes_dict[priority] = process
    pre_processes = [pre_processes_dict[priority] for priority in sorted(pre_processes_dict.keys())]

    lidar_processes_dict = dict()
    built_in_lidar_modules = __import__('liguard.algo.lidar', fromlist=['*']).__dict__
    for proc in cfg['proc']['lidar']:
        if not cfg['proc']['lidar'][proc]['enabled']: continue
        priority = cfg['proc']['lidar'][proc]['priority']
        if proc in built_in_lidar_modules: process = built_in_lidar_modules[proc]
        else: process = __import__(proc, fromlist=['*']).__dict__[proc]
        lidar_processes_dict[priority] = process
    lidar_processes = [lidar_processes_dict[priority] for priority in sorted(lidar_processes_dict.keys())]

    camera_processes_dict = dict()
    built_in_camera_modules = __import__('liguard.algo.camera', fromlist=['*']).__dict__
    for proc in cfg['proc']['camera']:
        if not cfg['proc']['camera'][proc]['enabled']: continue
        priority = cfg['proc']['camera'][proc]['priority']
        if proc in built_in_camera_modules: process = built_in_camera_modules[proc]
        else: process = __import__(proc, fromlist=['*']).__dict__[proc]
        camera_processes_dict[priority] = process
    camera_processes = [camera_processes_dict[priority] for priority in sorted(camera_processes_dict.keys())]

    calib_processes_dict = dict()
    built_in_calib_modules = __import__('liguard.algo.calib', fromlist=['*']).__dict__
    for proc in cfg['proc']['calib']:
        if not cfg['proc']['calib'][proc]['enabled']: continue
        priority = cfg['proc']['calib'][proc]['priority']
        if proc in built_in_calib_modules: process = built_in_calib_modules[proc]
        else: process = __import__(proc, fromlist=['*']).__dict__[proc]
        calib_processes_dict[priority] = process
    calib_processes = [calib_processes_dict[priority] for priority in sorted(calib_processes_dict.keys())]

    label_processes_dict = dict()
    built_in_label_modules = __import__('liguard.algo.label', fromlist=['*']).__dict__
    for proc in cfg['proc']['label']:
        if not cfg['proc']['label'][proc]['enabled']: continue
        priority = cfg['proc']['label'][proc]['priority']
        if proc in built_in_label_modules: process = built_in_label_modules[proc]
        else: process = __import__(proc, fromlist=['*']).__dict__[proc]
        label_processes_dict[priority] = process
    label_processes = [label_processes_dict[priority] for priority in sorted(label_processes_dict.keys())]

    post_processes_dict = dict()
    built_in_post_modules = __import__('liguard.algo.post', fromlist=['*']).__dict__
    for proc in cfg['proc']['post']:
        if not cfg['proc']['post'][proc]['enabled']: continue
        priority = cfg['proc']['post'][proc]['priority']
        if proc in built_in_post_modules: process = built_in_post_modules[proc]
        else: process = __import__(proc, fromlist=['*']).__dict__[proc]
        post_processes_dict[priority] = process
    post_processes = [post_processes_dict[priority] for priority in sorted(post_processes_dict.keys())]

    # preprocess
    preprocessed_data_dict_queue = Queue(maxsize=args.max_queue_size)
    preprocess_tqdm = tqdm(total=min_len, desc='Preprocessing data', position=1)
    preprocess_thread = Thread(target=dict2proc2dict, args=(common_data_dict_queue, cfg, logger, pre_processes, preprocessed_data_dict_queue, preprocess_tqdm))
    preprocess_thread.start()

    # sequential processing
    seq_processed_data_dict_queue = Queue(maxsize=args.max_queue_size)
    seq_process_tqdm = tqdm(total=min_len, desc='Processing data', position=2)
    lidar_thread = Thread(target=dict2proc2dict, args=(preprocessed_data_dict_queue, cfg, logger, lidar_processes + camera_processes + calib_processes, seq_processed_data_dict_queue, seq_process_tqdm))
    lidar_thread.start()

    # label processing
    label_processed_data_dict_queue = Queue(maxsize=args.max_queue_size)
    label_process_tqdm = tqdm(total=min_len, desc='Processing labels', position=3)
    label_thread = Thread(target=dict2proc2dict, args=(seq_processed_data_dict_queue, cfg, logger, label_processes, label_processed_data_dict_queue, label_process_tqdm))
    label_thread.start()

    # postprocess
    postprocessed_data_dict_queue = Queue(maxsize=args.max_queue_size)
    postprocess_tqdm = tqdm(total=min_len, desc='Postprocessing data', position=4)
    postprocess_thread = Thread(target=dict2proc2dict, args=(label_processed_data_dict_queue, cfg, logger, post_processes, None, postprocess_tqdm))
    postprocess_thread.start()

    # signal handler
    # sigint handler
    try:
        while not stop_event.is_set(): time.sleep(0.1)  # Sleep for a short time to keep the loop efficient
    except KeyboardInterrupt:
        # Handle Ctrl + C pressed in the main thread
        signal_handler(None, None)

    # cleanup
    if pcd_reader:
        pcd_io_thread.join()
        pcd_io_to_data_dict_thread.join()
    if img_reader:
        img_io_thread.join()
        img_io_to_data_dict_thread.join()
    if clb_reader:
        clb_io_thread.join()
        clb_io_to_data_dict_thread.join()
    if lbl_reader:
        lbl_io_thread.join()
        lbl_io_to_data_dict_thread.join()
    
    data_dict_thread.join()
    print('Data reading complete.')
    preprocess_thread.join()
    print('Preprocessing complete.')
    lidar_thread.join()
    print('Sequential processing complete.')
    label_thread.join()
    print('Label processing complete.')
    postprocess_thread.join()
    print('Postprocessing complete.')

    reader_tqdm.close()
    preprocess_tqdm.close()
    seq_process_tqdm.close()
    label_process_tqdm.close()
    postprocess_tqdm.close()

    logger.log('Processing complete.', Logger.INFO)

def main():
    banner = \
    """
    #########################################################
        _      _  _____                     _   ___    ___  
        | |    (_)/ ____|                   | | |__ \  / _ \ 
        | |     _| |  __ _   _  __ _ _ __ __| |    ) || | | |
        | |    | | | |_ | | | |/ _` | '__/ _` |   / / | | | |
        | |____| | |__| | |_| | (_| | | | (_| |  / /_ | |_| |
        |______|_|\_____|\__,_|\__,_|_|  \__,_| |____(_)___/ 
                                       Headless Bulk Processor
    ##########################################################
    LiGuard's utility for no-GUI bulk data processing.
    """
    print(banner)
    description = \
    """
    Description:
    Once you have created a pipeline using LiGuard's interactive
    interface, you can take your configuration file (.yml) and use
    this script to process entire datasets faster. This script processes
    the data faster by utilizing multiple threads and removing GUI and
    other interactive elements.

    Note 1: Currently, this doesn't work with live sensor data streams.
    Note 2: Currently, this doesn't work with multi-frame dependent algorithms
    such as calculating background filters using multiple frames (you can use a pre-calculated filter though), tracking, etc.
    """
    import argparse
    parser = argparse.ArgumentParser(description=f'{description}')
    parser.add_argument('pipeline_dir', type=str, help='Path to the pipleine directory.')
    parser.add_argument('--max_queue_size', type=int, default=10, help='Maximum size of the queues.')
    args = parser.parse_args()
    bulk_process(args)

if __name__ == '__main__':
    main()