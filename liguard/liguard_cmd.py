import os
from liguard.gui.config_gui import get_abs_path
import yaml

from liguard.pcd.file_io import FileIO as PCD_File_IO
from liguard.img.file_io import FileIO as IMG_File_IO
from liguard.calib.file_io import FileIO as CLB_File_IO
from liguard.lbl.file_io import FileIO as LBL_File_IO

from liguard.gui.logger_gui import Logger

import threading
from queue import Queue

def reader2queue(reader, size, target_queue):
    for idx in range(size): target_queue.put(reader[idx])
    target_queue.put(None)

def queue2dict2queue(source_queue, key1, key2, target_queue):
    while True:
        data = source_queue.get()
        if data is None: break
        data_dict = {key1: data[0], key2: data[1]}
        target_queue.put(data_dict)
        source_queue.task_done()
    target_queue.put(None)

def dicts2singledict(source_queues, target_queue):
    while True:
        data = [source_queues[i].get() for i in range(len(source_queues))]
        if None in data: break
        data_dict = dict()
        for d in data: data_dict.update(d)
        target_queue.put(data_dict)
        for i in range(len(source_queues)): source_queues[i].task_done()
    target_queue.put(None)

def dict2proc2dict(source_queue, cfg, logger, processes, target_queue):
    while True:
        data = source_queue.get()
        if data is None: break
        for process in processes: process(data, cfg, logger)
        if target_queue: target_queue.put(data)
        source_queue.task_done()
    if target_queue: target_queue.put(None)

def bulk_process(args):
    import yaml
    with open(args.config) as f:cfg = yaml.safe_load(f)
    logger = Logger()
    if cfg['logging']['level'] < Logger.WARNING:
        print('Logging level is too low. Setting to WARNING to prevent spam.')
        cfg['logging']['level'] = Logger.WARNING
    logger.reset(cfg)

    # create dirs
    if not os.path.exists(cfg['data']['outputs_dir']): os.makedirs(get_abs_path(cfg['data']['outputs_dir']), exist_ok=True)
    
    # reader queues
    pcd_input_queue = Queue(maxsize=args.max_queue_size)
    img_input_queue = Queue(maxsize=args.max_queue_size)
    clb_input_queue = Queue(maxsize=args.max_queue_size)
    lbl_input_queue = Queue(maxsize=args.max_queue_size)

    # readers
    pcd_reader = PCD_File_IO(cfg) if cfg['data']['lidar']['enabled'] else None
    img_reader = IMG_File_IO(cfg) if cfg['data']['camera']['enabled'] else None
    clb_reader = CLB_File_IO(cfg) if cfg['data']['calib']['enabled'] else None
    lbl_reader = LBL_File_IO(cfg) if cfg['data']['label']['enabled'] else None

    # reader threads
    if pcd_reader:
        pcd_io_thread = threading.Thread(target=reader2queue, args=(pcd_reader, len(pcd_reader), pcd_input_queue))
        pcd_io_thread.start()
    if img_reader:
        img_io_thread = threading.Thread(target=reader2queue, args=(img_reader, len(img_reader), img_input_queue))
        img_io_thread.start()
    if clb_reader:
        clb_io_thread = threading.Thread(target=reader2queue, args=(clb_reader, len(clb_reader), clb_input_queue))
        clb_io_thread.start()
    if lbl_reader:
        lbl_io_thread = threading.Thread(target=reader2queue, args=(lbl_reader, len(lbl_reader), lbl_input_queue))
        lbl_io_thread.start()

    # data dict queues
    pcd_data_dict_queue = Queue(maxsize=args.max_queue_size)
    img_data_dict_queue = Queue(maxsize=args.max_queue_size)
    clb_data_dict_queue = Queue(maxsize=args.max_queue_size)
    lbl_data_dict_queue = Queue(maxsize=args.max_queue_size)
    common_data_dict_queue = Queue(maxsize=args.max_queue_size)

    # queue to dict threads
    if pcd_reader:
        pcd_io_to_data_dict_thread = threading.Thread(target=queue2dict2queue, args=(pcd_input_queue, 'current_point_cloud_path', 'current_point_cloud_numpy', pcd_data_dict_queue))
        pcd_io_to_data_dict_thread.start()
    if img_reader:
        img_io_to_data_dict_thread = threading.Thread(target=queue2dict2queue, args=(img_input_queue, 'current_image_path', 'current_image_numpy', img_data_dict_queue))
        img_io_to_data_dict_thread.start()
    if clb_reader:
        clb_io_to_data_dict_thread = threading.Thread(target=queue2dict2queue, args=(clb_input_queue, 'current_calib_path', 'current_calib_data', clb_data_dict_queue))
        clb_io_to_data_dict_thread.start()
    if lbl_reader:
        lbl_io_to_data_dict_thread = threading.Thread(target=queue2dict2queue, args=(lbl_input_queue, 'current_label_path', 'current_label_list', lbl_data_dict_queue))
        lbl_io_to_data_dict_thread.start()
    
    # dict to single dict thread
    data_dicts = []
    if pcd_reader: data_dicts.append(pcd_data_dict_queue)
    if img_reader: data_dicts.append(img_data_dict_queue)
    if clb_reader: data_dicts.append(clb_data_dict_queue)
    if lbl_reader: data_dicts.append(lbl_data_dict_queue)
    data_dict_thread = threading.Thread(target=dicts2singledict, args=(data_dicts, common_data_dict_queue))
    data_dict_thread.start()

    # processes
    pre_processes_dict = dict()
    for proc in cfg['proc']['pre']:
        if not cfg['proc']['pre'][proc]['enabled']: continue
        priority = cfg['proc']['pre'][proc]['priority']
        process = __import__('liguard.algo.pre', fromlist=[proc]).__dict__[proc]
        pre_processes_dict[priority] = process
    pre_processes = [pre_processes_dict[priority] for priority in sorted(pre_processes_dict.keys())]

    lidar_processes_dict = dict()
    for proc in cfg['proc']['lidar']:
        if not cfg['proc']['lidar'][proc]['enabled']: continue
        priority = cfg['proc']['lidar'][proc]['priority']
        process = __import__('liguard.algo.lidar', fromlist=[proc]).__dict__[proc]
        lidar_processes_dict[priority] = process
    lidar_processes = [lidar_processes_dict[priority] for priority in sorted(lidar_processes_dict.keys())]

    camera_processes_dict = dict()
    for proc in cfg['proc']['camera']:
        if not cfg['proc']['camera'][proc]['enabled']: continue
        priority = cfg['proc']['camera'][proc]['priority']
        process = __import__('liguard.algo.camera', fromlist=[proc]).__dict__[proc]
        camera_processes_dict[priority] = process
    camera_processes = [camera_processes_dict[priority] for priority in sorted(camera_processes_dict.keys())]

    calib_processes_dict = dict()
    for proc in cfg['proc']['calib']:
        if not cfg['proc']['calib'][proc]['enabled']: continue
        priority = cfg['proc']['calib'][proc]['priority']
        process = __import__('liguard.algo.calib', fromlist=[proc]).__dict__[proc]
        calib_processes_dict[priority] = process
    calib_processes = [calib_processes_dict[priority] for priority in sorted(calib_processes_dict.keys())]

    label_processes_dict = dict()
    for proc in cfg['proc']['label']:
        if not cfg['proc']['label'][proc]['enabled']: continue
        priority = cfg['proc']['label'][proc]['priority']
        process = __import__('liguard.algo.label', fromlist=[proc]).__dict__[proc]
        label_processes_dict[priority] = process
    label_processes = [label_processes_dict[priority] for priority in sorted(label_processes_dict.keys())]

    post_processes_dict = dict()
    for proc in cfg['proc']['post']:
        if not cfg['proc']['post'][proc]['enabled']: continue
        priority = cfg['proc']['post'][proc]['priority']
        process = __import__('liguard.algo.post', fromlist=[proc]).__dict__[proc]
        post_processes_dict[priority] = process
    post_processes = [post_processes_dict[priority] for priority in sorted(post_processes_dict.keys())]

    # preprocess
    preprocessed_data_dict_queue = Queue(maxsize=args.max_queue_size)
    preprocess_thread = threading.Thread(target=dict2proc2dict, args=(common_data_dict_queue, cfg, logger, pre_processes, preprocessed_data_dict_queue))
    preprocess_thread.start()

    # sequential processing
    seq_processed_data_dict_queue = Queue(maxsize=args.max_queue_size)
    lidar_thread = threading.Thread(target=dict2proc2dict, args=(preprocessed_data_dict_queue, cfg, logger, lidar_processes + camera_processes + calib_processes, seq_processed_data_dict_queue))
    lidar_thread.start()

    # label processing
    label_processed_data_dict_queue = Queue(maxsize=args.max_queue_size)
    label_thread = threading.Thread(target=dict2proc2dict, args=(seq_processed_data_dict_queue, cfg, logger, label_processes, label_processed_data_dict_queue))
    label_thread.start()

    # postprocess
    postprocessed_data_dict_queue = Queue(maxsize=args.max_queue_size)
    postprocess_thread = threading.Thread(target=dict2proc2dict, args=(label_processed_data_dict_queue, cfg, logger, post_processes, None))
    postprocess_thread.start()

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
    preprocess_thread.join()
    lidar_thread.join()
    label_thread.join()
    postprocess_thread.join()

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
    parser.add_argument('config', type=str, help='Path to the configuration (.yml) file.')
    parser.add_argument('--max_queue_size', type=int, default=10, help='Maximum size of the queues.')
    args = parser.parse_args()
    bulk_process(args)

if __name__ == '__main__':
    main()