import os
import traceback

import open3d.visualization.gui as gui

from liguard.gui.config_gui import BaseConfiguration as BaseConfigurationGUI
from liguard.gui.config_gui import resolve_for_application_root, resolve_for_default_workspace
from liguard.gui.logger_gui import Logger
from liguard.liguard_profiler import Profiler

from liguard.pcd.file_io import FileIO as PCD_File_IO
from liguard.pcd.sensor_io import SensorIO as PCD_Sensor_IO
from liguard.pcd.viz import PointCloudVisualizer

from liguard.img.file_io import FileIO as IMG_File_IO
from liguard.img.sensor_io import SensorIO as IMG_Sensor_IO
from liguard.img.viz import ImageVisualizer

from liguard.calib.file_io import FileIO as CLB_File_IO
from liguard.lbl.file_io import FileIO as LBL_File_IO

import pynput, threading, time

profiler = Profiler('main')

class LiGuard:
    def __init__(self):
        # initialize the application
        self.app = gui.Application.instance
        self.app.initialize()
        
        # initialize the configuration GUI
        self.config = BaseConfigurationGUI(self.app)
        # initialize the logger GUI
        self.logger = Logger(self.app)
        # set the callbacks for the configuration GUI
        config_call_backs = BaseConfigurationGUI.get_callbacks_dict()
        config_call_backs['apply_config'] = [self.reset, self.start]
        config_call_backs['save_config'] = [lambda cfg: self.pcd_visualizer.save_view_status() if self.pcd_visualizer else None]
        config_call_backs['save_as_config'] = [lambda cfg: self.pcd_visualizer.save_view_status() if self.pcd_visualizer else None]
        config_call_backs['quit_config'] = [self.quit]
        self.config.update_callbacks(config_call_backs)
        
        # initialize the data sources
        self.pcd_io = None
        self.pcd_visualizer = None
        
        self.img_io = None
        self.img_visualizer = None

        self.clb_io = None
        
        self.lbl_io = None
        
        # initialize the main lock
        self.lock = threading.Lock()
        self.pynput_listener = None
        self.is_running = False # if the app is running
        self.is_focused = False # if the app is focused
        self.is_playing = False # if the frames are playing
        # initialize the data dictionary
        self.data_dict = dict()
        self.data_dict['logger'] = self.logger
        self.data_dict['current_frame_index'] = 0
        self.data_dict['previous_frame_index'] = -1
        self.data_dict['maximum_frame_index'] = 0
        
        # start the application loop
        self.app.run()
        
    # handle the key events of right, left, and space keys
    def handle_key_event(self, key):
        with self.lock:
            if key == pynput.keyboard.Key.right:
                self.is_playing = False
                if self.data_dict['current_frame_index'] < self.data_dict['maximum_frame_index']:
                    self.data_dict['current_frame_index'] += 1
            elif key == pynput.keyboard.Key.left:
                self.is_playing = False
                if self.data_dict['current_frame_index'] > 0:
                    self.data_dict['current_frame_index'] -= 1
            elif key == pynput.keyboard.Key.space:
                self.is_playing = not self.is_playing
            elif key == pynput.keyboard.Key.delete:
                self.is_playing = False
                self.data_dict['current_frame_index'] = 0
                if self.pcd_visualizer: self.pcd_visualizer.load_view_status()
            elif key == pynput.keyboard.KeyCode(char='['):
                self.is_playing = False
                self.config.show_input_dialog('Enter the frame index:', f'Jump to Frame (0-{self.data_dict["maximum_frame_index"]})', 'jump_to_frame')
                    
    def reset(self, cfg):
        """
        Resets the LiGuard with the given configuration.

        Parameters:
        - cfg (dict): The configuration dictionary containing the settings for LiGuard.

        Returns:
        - None
        """
        # Get the logger object from the data dictionary
        logger:Logger = self.data_dict['logger']
        # Check if the data path or logging level has changed
        need_reset = False
        need_level_change = False
        
        current_data_path = cfg['data']['main_dir']
        if not os.path.isabs(current_data_path): current_data_path = os.path.join(cfg['data']['pipeline_dir'], current_data_path)
        if not hasattr(self, 'last_data_path'):
            self.last_data_path = current_data_path
            need_reset = True
        if self.last_data_path != current_data_path: need_reset = True
        
        current_logging_level = cfg['logging']['level']
        if not hasattr(self, 'last_logging_level'):
            self.last_logging_level = current_logging_level
            need_level_change = True
        if self.last_logging_level != current_logging_level: need_level_change = True
        
        current_logging_path = cfg['logging']['logs_dir']
        if not os.path.isabs(current_logging_path): current_logging_path = os.path.join(cfg['data']['pipeline_dir'], current_logging_path)
        if not hasattr(self, 'last_logging_path'):
            self.last_logging_path = current_logging_path
            need_reset = True
        if self.last_logging_path != current_logging_path: need_reset = True
        
        # Reset the logger if the data path or logging level has changed
        if need_reset:
            self.last_data_path = current_data_path
            self.last_logging_path = current_logging_path
            logger.reset(cfg)
        # Change the logging level if it has changed
        if need_level_change:
            self.last_logging_level = current_logging_level
            logger.change_level(current_logging_level)

        # Make sure the required directories exist
        self.outputs_dir = cfg['data']['outputs_dir']
        if not os.path.isabs(self.outputs_dir): self.outputs_dir = os.path.join(cfg['data']['pipeline_dir'], self.outputs_dir)
        os.makedirs(self.outputs_dir, exist_ok=True)

        # unlock the keyboard keys right, left, and space
        if self.pynput_listener: self.pynput_listener.stop()
        # pause at the start
        with self.lock: self.is_running = False
        # reset the frame index
        self.data_dict['previous_frame_index'] = -1
        
        # manage pcd reading
        if self.pcd_io != None: self.pcd_io.close()
        # if files are enabled
        if cfg['data']['lidar']['enabled']:
            try:
                self.pcd_io = PCD_File_IO(cfg)
                self.logger.log('PCD_File_IO created', Logger.DEBUG)
            except Exception:
                self.logger.log(f'PCD_File_IO creation failed:\n{traceback.format_exc()}', Logger.CRITICAL)
                self.pcd_io = None
        # if sensors are enabled
        elif 'sensors' in cfg and 'lidar' in cfg['sensors'] and cfg['sensors']['lidar']['enabled']:
            try:
                self.pcd_io = PCD_Sensor_IO(cfg)
                self.logger.log('PCD_Sensor_IO created', Logger.DEBUG)
            except Exception:
                self.logger.log(f'PCD_Sensor_IO creation failed:\n{traceback.format_exc()}', Logger.CRITICAL)
                self.pcd_io = None
        else: self.pcd_io = None
        # get the total number of pcd frames
        self.data_dict['total_pcd_frames'] = len(self.pcd_io) if self.pcd_io else 0
        self.logger.log(f'total_pcd_frames: {self.data_dict["total_pcd_frames"]}', Logger.DEBUG)
        
        # manage pcd visualization
        if self.pcd_io:
            if self.pcd_visualizer != None: self.pcd_visualizer.reset(cfg)
            else:
                try:
                    self.pcd_visualizer = PointCloudVisualizer(self.app, cfg)
                    self.pcd_visualizer.load_view_status()
                    self.logger.log('PointCloudVisualizer created', Logger.DEBUG)
                except Exception:
                    self.logger.log(f'PointCloudVisualizer creation failed:\n{traceback.format_exc()}', Logger.CRITICAL)
                    self.pcd_visualizer = None
        # manage image reading
        if self.img_io != None: self.img_io.close()
        # if files are enabled
        if cfg['data']['camera']['enabled']:
            try:
                self.img_io = IMG_File_IO(cfg)
                self.logger.log('IMG_File_IO created', Logger.DEBUG)
            except Exception:
                self.logger.log(f'IMG_File_IO creation failed:\n{traceback.format_exc()}', Logger.CRITICAL)
                self.img_io = None
        # if sensors are enabled
        elif 'sensors' in cfg and 'camera' in cfg['sensors'] and cfg['sensors']['camera']['enabled']:
            try:
                self.img_io = IMG_Sensor_IO(cfg)
                self.logger.log('IMG_Sensor_IO created', Logger.DEBUG)
            except Exception:
                self.logger.log(f'IMG_Sensor_IO creation failed:\n{traceback.format_exc()}', Logger.CRITICAL)
                self.img_io = None
        else: self.img_io = None
        # get the total number of image frames
        self.data_dict['total_img_frames'] = len(self.img_io) if self.img_io else 0
        self.logger.log(f'total_img_frames: {self.data_dict["total_img_frames"]}', Logger.DEBUG)
        
        # manage image visualization
        if self.img_io:
            try:
                if self.img_visualizer != None: self.img_visualizer.reset(cfg)
                else: self.img_visualizer = ImageVisualizer(self.app, cfg)
                self.logger.log('ImageVisualizer created', Logger.DEBUG)
            except Exception:
                self.logger.log(f'ImageVisualizer creation failed:\n{traceback.format_exc()}', Logger.CRITICAL)
                self.img_visualizer = None

        # manage calibration reading
        if self.clb_io != None: self.clb_io.close()
        if cfg['data']['calib']['enabled']:
            try:
                self.clb_io = CLB_File_IO(cfg)
                self.logger.log('CLB_File_IO created', Logger.DEBUG)
            except Exception:
                self.logger.log(f'CLB_File_IO creation failed:\n{traceback.format_exc()}', Logger.CRITICAL)
                self.clb_io = None
        else: self.clb_io = None
        # get the total number of calibration frames
        self.data_dict['total_clb_frames'] = len(self.clb_io) if self.clb_io else 0
        self.logger.log(f'total_clb_frames: {self.data_dict["total_clb_frames"]}', Logger.DEBUG)
        
        # manage label reading
        if self.lbl_io != None: self.lbl_io.close()
        if cfg['data']['label']['enabled']:
            try:
                self.lbl_io = LBL_File_IO(cfg, self.clb_io.__getitem__ if self.clb_io else None)
                self.logger.log('LBL_File_IO created', Logger.DEBUG)
            except Exception:
                self.logger.log(f'LBL_File_IO creation failed:\n{traceback.format_exc()}', Logger.CRITICAL)
                self.lbl_io = None
        else: self.lbl_io = None
        # get the total number of label frames
        self.data_dict['total_lbl_frames'] = len(self.lbl_io) if self.lbl_io else 0
        self.logger.log(f'total_lbl_frames: {self.data_dict["total_lbl_frames"]}', Logger.DEBUG)
        
        # get the maximum frame index
        self.data_dict['maximum_frame_index'] = max(self.data_dict['total_pcd_frames'], self.data_dict['total_img_frames'], self.data_dict['total_lbl_frames']) - 1
        self.logger.log(f'maximum_frame_index: {self.data_dict["maximum_frame_index"]}', Logger.DEBUG)
        ##########################################
        # processes
        ##########################################
        # pre processes
        self.pre_processes = dict()
        built_in_pre_modules = __import__('liguard.algo.pre', fromlist=['*']).__dict__
        for proc in cfg['proc']['pre']:
            enabled = cfg['proc']['pre'][proc]['enabled']
            if enabled:
                try:
                    priority = cfg['proc']['pre'][proc]['priority']
                    if proc in built_in_pre_modules: process = built_in_pre_modules[proc]
                    else: process = __import__(proc, fromlist=['*']).__dict__[proc]
                    self.pre_processes[priority] = process
                except Exception:
                    self.logger.log(f'pre_processes creation failed for {proc.__name__}:\n{traceback.format_exc()}', Logger.CRITICAL)
        self.pre_processes = [self.pre_processes[priority] for priority in sorted(self.pre_processes.keys())]
        self.logger.log(f'enabled pre_processes: {[f.__name__ for f in self.pre_processes]}', Logger.DEBUG)
        
        # lidar processes
        self.lidar_processes = dict()
        built_in_lidar_modules = __import__('liguard.algo.lidar', fromlist=['*']).__dict__
        for proc in cfg['proc']['lidar']:
            enabled = cfg['proc']['lidar'][proc]['enabled']
            if enabled:
                try:
                    priority = cfg['proc']['lidar'][proc]['priority']
                    if proc in built_in_lidar_modules: process = built_in_lidar_modules[proc]
                    else: process = __import__(proc, fromlist=['*']).__dict__[proc]
                    self.lidar_processes[priority] = process
                except Exception:
                    self.logger.log(f'lidar_processes creation failed for {proc.__name__}:\n{traceback.format_exc()}', Logger.CRITICAL)
        self.lidar_processes = [self.lidar_processes[priority] for priority in sorted(self.lidar_processes.keys())]
        self.logger.log(f'enabled lidar_processes: {[f.__name__ for f in self.lidar_processes]}', Logger.DEBUG)
        
        # camera processes
        self.camera_processes = dict()
        built_in_camera_modules = __import__('liguard.algo.camera', fromlist=['*']).__dict__
        for proc in cfg['proc']['camera']:
            enabled = cfg['proc']['camera'][proc]['enabled']
            if enabled:
                try:
                    priority = cfg['proc']['camera'][proc]['priority']
                    if proc in built_in_camera_modules: process = built_in_camera_modules[proc]
                    else: process = __import__(proc, fromlist=['*']).__dict__[proc]
                    self.camera_processes[priority] = process
                except Exception:
                    self.logger.log(f'camera_processes creation failed for {proc.__name__}:\n{traceback.format_exc()}', Logger.CRITICAL)
        self.camera_processes = [self.camera_processes[priority] for priority in sorted(self.camera_processes.keys())]
        self.logger.log(f'enabled camera_processes: {[f.__name__ for f in self.camera_processes]}', Logger.DEBUG)

        # calib processes
        self.calib_processes = dict()
        built_in_calib_modules = __import__('liguard.algo.calib', fromlist=['*']).__dict__
        for proc in cfg['proc']['calib']:
            enabled = cfg['proc']['calib'][proc]['enabled']
            if enabled:
                try:
                    priority = cfg['proc']['calib'][proc]['priority']
                    if proc in built_in_calib_modules: process = built_in_calib_modules[proc]
                    else: process = __import__(proc, fromlist=['*']).__dict__[proc]
                    self.calib_processes[priority] = process
                except Exception:
                    self.logger.log(f'calib_processes creation failed for {proc.__name__}:\n{traceback.format_exc()}', Logger.CRITICAL)
        self.calib_processes = [self.calib_processes[priority] for priority in sorted(self.calib_processes.keys())]
        self.logger.log(f'enabled calib_processes: {[f.__name__ for f in self.calib_processes]}', Logger.DEBUG)

        # label processes
        self.label_processes = dict()
        built_in_label_modules = __import__('liguard.algo.label', fromlist=['*']).__dict__
        for proc in cfg['proc']['label']:
            enabled = cfg['proc']['label'][proc]['enabled']
            if enabled:
                try:
                    priority = cfg['proc']['label'][proc]['priority']
                    if proc in built_in_label_modules: process = built_in_label_modules[proc]
                    else: process = __import__(proc, fromlist=['*']).__dict__[proc]
                    self.label_processes[priority] = process
                except Exception:
                    self.logger.log(f'label_processes creation failed for {proc.__name__}:\n{traceback.format_exc()}', Logger.CRITICAL)
        self.label_processes = [self.label_processes[priority] for priority in sorted(self.label_processes.keys())]
        self.logger.log(f'enabled label_processes: {[f.__name__ for f in self.label_processes]}', Logger.DEBUG)
        
        # post processes
        self.post_processes = dict()
        built_in_post_modules = __import__('liguard.algo.post', fromlist=['*']).__dict__
        for proc in cfg['proc']['post']:
            enabled = cfg['proc']['post'][proc]['enabled']
            if enabled:
                try:
                    priority = cfg['proc']['post'][proc]['priority']
                    if proc in built_in_post_modules: process = built_in_post_modules[proc]
                    else: process = __import__(proc, fromlist=['*']).__dict__[proc]
                    self.post_processes[priority] = process
                except Exception:
                    self.logger.log(f'post_processes creation failed for {proc.__name__}:\n{traceback.format_exc()}', Logger.CRITICAL)
            
        self.post_processes = [self.post_processes[priority] for priority in sorted(self.post_processes.keys())]
        self.logger.log(f'enabled post_processes: {[f.__name__ for f in self.post_processes]}', Logger.DEBUG)
        
    def start(self, cfg):
        # start the LiGuard
        with self.lock: self.is_running = True
        
        # start key event handling
        if self.pcd_visualizer or self.img_visualizer:
            self.pynput_listener = pynput.keyboard.Listener(on_press=self.handle_key_event)
            self.pynput_listener.start()
        
        # the main loop
        while True:
            # check if the app is running
            with self.lock:
                if not self.is_running: break
                # check if the frames are playing, if yes increment the frame index
                elif self.is_playing and self.data_dict['current_frame_index'] < self.data_dict['maximum_frame_index']: self.data_dict['current_frame_index'] += 1
            
            # check if user has jumped to a frame
            jump = self.config.get_input_dialog_value('jump_to_frame')
            if jump and (jump >= 0 and jump <= self.data_dict['maximum_frame_index']): self.data_dict['current_frame_index'] = jump
            # check if the frame has changed
            frame_changed = self.data_dict['previous_frame_index'] != self.data_dict['current_frame_index']
            
            # if the frame has changed, update the data dictionary with the new frame data
            if frame_changed:
                profiler.add_target('Total Time / Step')
                self.logger.set_status_frame_idx(self.data_dict['current_frame_index'] + cfg['data']['start']['global_zero'])
                self.data_dict['previous_frame_index'] = self.data_dict['current_frame_index']
                
                if self.pcd_io:
                    profiler.add_target('pcd_io')
                    current_point_cloud_path, current_point_cloud_numpy = self.pcd_io[self.data_dict['current_frame_index']]
                    self.data_dict['current_point_cloud_path'] = current_point_cloud_path
                    self.data_dict['current_point_cloud_numpy'] = current_point_cloud_numpy
                    profiler.end_target('pcd_io')
                elif 'current_point_cloud_numpy' in self.data_dict:
                    self.logger.log(f'current_point_cloud_numpy found in data_dict while pcd_io is None, removing ...', Logger.DEBUG)
                    self.data_dict.pop('current_point_cloud_numpy')
                
                if self.img_io:
                    profiler.add_target('img_io')
                    current_image_path, current_image_numpy = self.img_io[self.data_dict['current_frame_index']]
                    self.data_dict['current_image_path'] = current_image_path
                    self.data_dict['current_image_numpy'] = current_image_numpy
                    profiler.end_target('img_io')
                elif 'current_image_numpy' in self.data_dict:
                    self.logger.log(f'current_image_numpy found in data_dict while img_io is None, removing ...', Logger.DEBUG)
                    self.data_dict.pop('current_image_numpy')

                if self.clb_io:
                    profiler.add_target('clb_io')
                    current_calib_path, current_calib_data = self.clb_io[self.data_dict['current_frame_index']]
                    self.data_dict['current_calib_path'] = current_calib_path
                    self.data_dict['current_calib_data'] = current_calib_data
                    profiler.end_target('clb_io')
                elif 'current_calib_data' in self.data_dict:
                    self.logger.log(f'current_calib_data found in data_dict while clb_io is None, removing ...', Logger.DEBUG)
                    self.data_dict.pop('current_calib_data')
                
                if self.lbl_io:
                    profiler.add_target('lbl_io')
                    current_label_path, current_label_list = self.lbl_io[self.data_dict['current_frame_index']]
                    self.data_dict['current_label_path'] = current_label_path
                    self.data_dict['current_label_list'] = current_label_list
                    profiler.end_target('lbl_io')
                elif 'current_label_list' in self.data_dict:
                    self.logger.log(f'current_label_list found in data_dict while lbl_io is None, removing ...', Logger.DEBUG)
                    self.data_dict.pop('current_label_list')

                # if non of the data sources are available, exit the app in 5 seconds
                if not any([self.pcd_io, self.img_io, self.clb_io, self.lbl_io]):
                    self.logger.log(f'no data source is available, exiting in 5 seconds...', Logger.CRITICAL)
                    time.sleep(5)
                    break

                # apply the processes
                for proc in self.pre_processes:
                    try:
                        profiler.add_target(f'pre_{proc.__name__}')
                        proc(self.data_dict, cfg, self.logger)
                        profiler.end_target(f'pre_{proc.__name__}')
                    except Exception:
                        profiler.end_target(f'pre_{proc.__name__}')
                        self.logger.log(f'pre_processes failed for {proc.__name__}:\n{traceback.format_exc()}', Logger.ERROR)
            
                if self.pcd_io:
                    for proc in self.lidar_processes:
                        try:
                            profiler.add_target(f'lidar_{proc.__name__}')
                            proc(self.data_dict, cfg, self.logger)
                            profiler.end_target(f'lidar_{proc.__name__}')
                        except Exception:
                            profiler.end_target(f'lidar_{proc.__name__}')
                            self.logger.log(f'lidar_processes failed for {proc.__name__}:\n{traceback.format_exc()}', Logger.ERROR)
                if self.img_io:
                    for proc in self.camera_processes:
                        try:
                            profiler.add_target(f'camera_{proc.__name__}')
                            proc(self.data_dict, cfg, self.logger)
                            profiler.end_target(f'camera_{proc.__name__}')
                        except Exception:
                            profiler.end_target(f'camera_{proc.__name__}')
                            self.logger.log(f'camera_processes failed for {proc.__name__}:\n{traceback.format_exc()}', Logger.ERROR)
                if self.clb_io:
                    for proc in self.calib_processes:
                        try:
                            profiler.add_target(f'calib_{proc.__name__}')
                            proc(self.data_dict, cfg, self.logger)
                            profiler.end_target(f'calib_{proc.__name__}')
                        except Exception:
                            profiler.end_target(f'calib_{proc.__name__}')
                            self.logger.log(f'calib_processes failed for {proc.__name__}:\n{traceback.format_exc()}', Logger.ERROR)
                if self.lbl_io:
                    for proc in self.label_processes:
                        try:
                            profiler.add_target(f'label_{proc.__name__}')
                            proc(self.data_dict, cfg, self.logger)
                            profiler.end_target(f'label_{proc.__name__}')
                        except Exception:
                            profiler.end_target(f'label_{proc.__name__}')
                            self.logger.log(f'label_processes failed for {proc.__name__}:\n{traceback.format_exc()}', Logger.ERROR)
                
                for proc in self.post_processes:
                    try:
                        profiler.add_target(f'post_{proc.__name__}')
                        proc(self.data_dict, cfg, self.logger)
                        profiler.end_target(f'post_{proc.__name__}')
                    except Exception:
                        profiler.end_target(f'post_{proc.__name__}')
                        self.logger.log(f'post_processes failed for {proc.__name__}:\n{traceback.format_exc()}', Logger.ERROR)

                # update the visualizers
                if self.pcd_io:
                    # update and redraw
                    profiler.add_target('pcd_visualizer_update')
                    if cfg['visualization']['enabled']:
                        self.pcd_visualizer.update(self.data_dict)
                    self.pcd_visualizer.redraw()
                    profiler.end_target('pcd_visualizer_update')
                    # save image of the view if enabled
                    if cfg['visualization']['lidar']['save_images']:
                        profiler.add_target('pcd_visualizer_save_current_view')
                        self.pcd_visualizer.save_current_view(self.data_dict['current_frame_index'])
                        profiler.end_target('pcd_visualizer_save_current_view')
                if self.img_io:
                    profiler.add_target('img_visualizer_update')
                    if cfg['visualization']['enabled']:
                        self.img_visualizer.update(self.data_dict)
                    self.img_visualizer.redraw()
                    profiler.end_target('img_visualizer_update')
                    if cfg['visualization']['camera']['save_images']:
                        profiler.add_target('img_visualizer_save_current_view')
                        self.img_visualizer.save_current_view(self.data_dict['current_frame_index'])
                        profiler.end_target('img_visualizer_save_current_view')

                if cfg['visualization']['enabled'] == False:
                    self.logger.log(f'Processed frame {self.data_dict["current_frame_index"]}', Logger.INFO)
                profiler.end_target('Total Time / Step')
                    
            else:
                # if the frame has not changed, redraw the visualizers only no processing is required
                if self.pcd_io: self.pcd_visualizer.redraw()
                if self.img_io: self.img_visualizer.redraw()
            
            # sleep for a while
            time.sleep(cfg['threads']['vis_sleep'])
            profiler.save(os.path.join(self.outputs_dir, 'LiGuard_main.profile'))
            
    def quit(self, cfg):
        # stop the app
        with self.lock: self.is_running = False
        # unhook the keyboard keys
        if self.pynput_listener: self.pynput_listener.stop()
        
        # close the data sources
        if self.pcd_io: self.pcd_io.close()
        if self.img_io: self.img_io.close()
        if self.lbl_io: self.lbl_io.close()
        
        # close the visualizers
        if self.pcd_visualizer: self.pcd_visualizer.quit()
        if self.img_visualizer: self.img_visualizer.quit()
        
        # close app
        self.app.quit()
        
def main():
    try:
        default_workspace_dir = resolve_for_default_workspace('')
        if not os.path.exists(default_workspace_dir):
            os.makedirs(default_workspace_dir)
            import shutil
            shutil.copytree(resolve_for_application_root('examples'), resolve_for_default_workspace('examples'))
        LiGuard()
        print('LiGuard exited successfully.')
    except Exception:
        print(f'LiGuard exited with an error:\n{traceback.format_exc()}')

if __name__ == '__main__': main()