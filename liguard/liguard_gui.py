import os
import platform
import traceback

import open3d.visualization.gui as gui

from liguard.gui.config_gui import BaseConfiguration as BaseConfigurationGUI
from liguard.gui.gui_utils import resolve_for_application_root, resolve_for_default_workspace, import_module
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

# Quartz imports (only valid on macOS)
try:
    from Quartz import (
        CGEventTapCreate, CGEventTapEnable, kCGHeadInsertEventTap,
        CGEventMaskBit, kCGEventKeyDown,
        kCGSessionEventTap, kCGEventTapOptionDefault,
        kCGKeyboardEventKeycode
    )
    from Quartz.CoreGraphics import CGEventGetIntegerValueField
    import CoreFoundation
except ImportError:
    CGEventTapCreate = None  # Quartz not available

profiler = Profiler('main')

class LiGuard:
    def __init__(self):
        # initialize the application
        self.app = gui.Application.instance
        self.app.initialize()
        
        # initialize gui
        self.logger = Logger(self.app)
        self.config = BaseConfigurationGUI(self.app, self.logger)
        self.pcd_viz = PointCloudVisualizer(self.app, self.logger)
        self.pcd_viz.hide()
        self.img_viz = ImageVisualizer(self.app, self.logger)
        self.img_viz.hide()
        
        # remove menubar
        self.config.mwin.show_menu(False)
        self.config.mwin.post_redraw()
        self.logger.mwin.show_menu(False)
        self.logger.mwin.post_redraw()
        
        # set the callbacks for the configuration GUI
        config_call_backs = BaseConfigurationGUI.get_callbacks_dict()
        config_call_backs['apply_config'] = [self.reset, self.start]
        config_call_backs['quit_config'] = [self.quit]
        self.config.update_callbacks(config_call_backs)
        
        # initialize the data sources
        self.pcd_io = None
        self.img_io = None
        self.clb_io = None
        self.lbl_io = None
        
        # initialize the main lock
        self.pynput_listener = None
        self.quartz_thread = None
        self.is_running = False # if the app is running
        self.is_focused = False # if the app is focused
        self.is_playing = False # if the frames are playing

        # initialize the data dictionary
        self.data_dict = dict()
        self.data_dict['current_frame_index'] = 0
        self.data_dict['previous_frame_index'] = -1
        self.data_dict['maximum_frame_index'] = 0
        
        # start the application loop
        self.app.run()
        
    # handle the key events of right, left, and space keys
    def handle_key_event(self, key):
        if self.pcd_viz.viz.is_visible or self.img_viz.viz.is_visible:
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
                self.pcd_viz.load_view_status()
            elif key == pynput.keyboard.KeyCode(char='['):
                self.is_playing = False
                self.config.show_input_dialog('Enter the frame index:', f'Jump to Frame (0-{self.data_dict["maximum_frame_index"]})', 'jump_to_frame')
    
    def start_keyboard_listener(self):
        self.stop_keyboard_listener()

        if platform.system() == 'Darwin' and CGEventTapCreate:
            # macOS Quartz fallback
            self._start_quartz_listener()
        else:
            # cross‑platform pynput
            self.pynput_listener = pynput.keyboard.Listener(
                on_press=self.handle_key_event
            )
            self.pynput_listener.start()

    def stop_keyboard_listener(self):
        # stop pynput if running
        if self.pynput_listener:
            self.pynput_listener.stop()
            self.pynput_listener = None

        # stop Quartz thread if running
        if self.quartz_thread and self.quartz_thread.is_alive():
            # Note: CFRunLoopRun() doesn’t exit unless you explicitly stop it.
            # You can store the tap and call CGEventTapEnable(tap, False), or just leave it.
            # For simplicity we won’t stop it here.
            pass

    def _start_quartz_listener(self):
        def quartz_loop():
            # mask only key‐down events
            mask = CGEventMaskBit(kCGEventKeyDown)

            def callback(proxy, type_, event, refcon):
                keycode = CGEventGetIntegerValueField(event, kCGKeyboardEventKeycode)
                # Map raw keycodes to your pynput Key/KeyCode:
                mapping = {
                    124: pynput.keyboard.Key.right,
                    123: pynput.keyboard.Key.left,
                    49:  pynput.keyboard.Key.space,
                    117:  pynput.keyboard.Key.delete,
                    33:  pynput.keyboard.KeyCode(char='['),  # may vary by layout
                }
                key = mapping.get(keycode)
                if key:
                    self.handle_key_event(key)
                return event

            tap = CGEventTapCreate(
                kCGSessionEventTap,
                kCGHeadInsertEventTap,
                kCGEventTapOptionDefault,
                mask,
                callback,
                None
            )
            source = CoreFoundation.CFMachPortCreateRunLoopSource(
                None, tap, 0
            )
            CoreFoundation.CFRunLoopAddSource(
                CoreFoundation.CFRunLoopGetCurrent(),
                source,
                CoreFoundation.kCFRunLoopCommonModes
            )
            CGEventTapEnable(tap, True)
            CoreFoundation.CFRunLoopRun()

        self.quartz_thread = threading.Thread(
            target=quartz_loop,
            daemon=True
        )
        self.quartz_thread.start()
                    
    def reset(self, cfg):
        """
        Resets the LiGuard with the given configuration.

        Args:
            cfg: A dictionary containing configuration parameters.
        """
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
            self.logger.reset(cfg)
            
        # Change the logging level if it has changed
        if need_level_change:
            self.last_logging_level = current_logging_level
            self.logger.change_level(current_logging_level)

        # Make sure the required directories exist
        self.outputs_dir = cfg['data']['outputs_dir']
        if not os.path.isabs(self.outputs_dir): self.outputs_dir = os.path.join(cfg['data']['pipeline_dir'], self.outputs_dir)
        os.makedirs(self.outputs_dir, exist_ok=True)

        # unlock the keyboard keys right, left, and space
        if self.pynput_listener: self.stop_keyboard_listener()
        
        # pause at the start
        self.is_running = False
        
        # manage pcd reading
        if self.pcd_io != None: self.pcd_io.close()
        # if files are enabled
        if cfg['data']['lidar']['enabled']:
            try:
                self.pcd_viz.show()
                self.pcd_io = PCD_File_IO(cfg)
                self.logger.log('PCD_File_IO created', Logger.DEBUG)
            except Exception:
                self.logger.log(f'PCD_File_IO creation failed:\n{traceback.format_exc()}', Logger.CRITICAL)
                self.pcd_io = None
        # if sensors are enabled
        elif 'sensors' in cfg and 'lidar' in cfg['sensors'] and cfg['sensors']['lidar']['enabled']:
            try:
                self.pcd_viz.show()
                self.pcd_io = PCD_Sensor_IO(cfg)
                self.logger.log('PCD_Sensor_IO created', Logger.DEBUG)
            except Exception:
                self.logger.log(f'PCD_Sensor_IO creation failed:\n{traceback.format_exc()}', Logger.CRITICAL)
                self.pcd_io = None
        else:
            self.pcd_viz.hide()
            self.pcd_io = None
        # get the total number of pcd frames
        self.data_dict['total_pcd_frames'] = len(self.pcd_io) if self.pcd_io else 0
        self.logger.log(f'total_pcd_frames: {self.data_dict["total_pcd_frames"]}', Logger.DEBUG)
        
        # manage pcd visualization
        if self.pcd_io: self.pcd_viz.reset(cfg)
        
        # manage image reading
        if self.img_io != None: self.img_io.close()
        # if files are enabled
        if cfg['data']['camera']['enabled']:
            try:
                self.img_viz.show()
                self.img_io = IMG_File_IO(cfg)
                self.logger.log('IMG_File_IO created', Logger.DEBUG)
            except Exception:
                self.logger.log(f'IMG_File_IO creation failed:\n{traceback.format_exc()}', Logger.CRITICAL)
                self.img_io = None
        # if sensors are enabled
        elif 'sensors' in cfg and 'camera' in cfg['sensors'] and cfg['sensors']['camera']['enabled']:
            try:
                self.img_viz.show()
                self.img_io = IMG_Sensor_IO(cfg)
                self.logger.log('IMG_Sensor_IO created', Logger.DEBUG)
            except Exception:
                self.logger.log(f'IMG_Sensor_IO creation failed:\n{traceback.format_exc()}', Logger.CRITICAL)
                self.img_io = None
        else:
            self.img_viz.hide()
            self.img_io = None
        # get the total number of image frames
        self.data_dict['total_img_frames'] = len(self.img_io) if self.img_io else 0
        self.logger.log(f'total_img_frames: {self.data_dict["total_img_frames"]}', Logger.DEBUG)
        
        # manage image visualization
        if self.img_io: self.img_viz.reset(cfg)

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
        self.data_dict['previous_frame_index'] = -1
        if self.data_dict['current_frame_index'] > self.data_dict['maximum_frame_index']: self.data_dict['current_frame_index'] = self.data_dict['maximum_frame_index']
        ##########################################
        # processes
        ##########################################
        # pre processes
        self.pre_processes = dict()
        built_in_pre_modules = import_module('liguard.algo.pre', fromlist=['*']).__dict__
        for proc in cfg['proc']['pre']:
            enabled = cfg['proc']['pre'][proc]['enabled']
            if enabled:
                try:
                    order = cfg['proc']['pre'][proc]['order']
                    if proc in built_in_pre_modules: process = built_in_pre_modules[proc]
                    else: process = import_module(proc, fromlist=['*']).__dict__[proc]
                    self.pre_processes[order] = process
                except Exception:
                    self.logger.log(f'pre_processes creation failed for {proc}:\n{traceback.format_exc()}', Logger.CRITICAL)
        self.pre_processes = [self.pre_processes[order] for order in sorted(self.pre_processes.keys())]
        self.logger.log(f'enabled pre_processes: {[f.__name__ for f in self.pre_processes]}', Logger.DEBUG)
        
        # lidar processes
        self.lidar_processes = dict()
        built_in_lidar_modules = import_module('liguard.algo.lidar', fromlist=['*']).__dict__
        for proc in cfg['proc']['lidar']:
            enabled = cfg['proc']['lidar'][proc]['enabled']
            if enabled:
                try:
                    order = cfg['proc']['lidar'][proc]['order']
                    if proc in built_in_lidar_modules: process = built_in_lidar_modules[proc]
                    else: process = import_module(proc, fromlist=['*']).__dict__[proc]
                    self.lidar_processes[order] = process
                except Exception:
                    self.logger.log(f'lidar_processes creation failed for {proc}:\n{traceback.format_exc()}', Logger.CRITICAL)
        self.lidar_processes = [self.lidar_processes[order] for order in sorted(self.lidar_processes.keys())]
        self.logger.log(f'enabled lidar_processes: {[f.__name__ for f in self.lidar_processes]}', Logger.DEBUG)
        
        # camera processes
        self.camera_processes = dict()
        built_in_camera_modules = import_module('liguard.algo.camera', fromlist=['*']).__dict__
        for proc in cfg['proc']['camera']:
            enabled = cfg['proc']['camera'][proc]['enabled']
            if enabled:
                try:
                    order = cfg['proc']['camera'][proc]['order']
                    if proc in built_in_camera_modules: process = built_in_camera_modules[proc]
                    else: process = import_module(proc, fromlist=['*']).__dict__[proc]
                    self.camera_processes[order] = process
                except Exception:
                    self.logger.log(f'camera_processes creation failed for {proc}:\n{traceback.format_exc()}', Logger.CRITICAL)
        self.camera_processes = [self.camera_processes[order] for order in sorted(self.camera_processes.keys())]
        self.logger.log(f'enabled camera_processes: {[f.__name__ for f in self.camera_processes]}', Logger.DEBUG)

        # calib processes
        self.calib_processes = dict()
        built_in_calib_modules = import_module('liguard.algo.calib', fromlist=['*']).__dict__
        for proc in cfg['proc']['calib']:
            enabled = cfg['proc']['calib'][proc]['enabled']
            if enabled:
                try:
                    order = cfg['proc']['calib'][proc]['order']
                    if proc in built_in_calib_modules: process = built_in_calib_modules[proc]
                    else: process = import_module(proc, fromlist=['*']).__dict__[proc]
                    self.calib_processes[order] = process
                except Exception:
                    self.logger.log(f'calib_processes creation failed for {proc}:\n{traceback.format_exc()}', Logger.CRITICAL)
        self.calib_processes = [self.calib_processes[order] for order in sorted(self.calib_processes.keys())]
        self.logger.log(f'enabled calib_processes: {[f.__name__ for f in self.calib_processes]}', Logger.DEBUG)

        # label processes
        self.label_processes = dict()
        built_in_label_modules = import_module('liguard.algo.label', fromlist=['*']).__dict__
        for proc in cfg['proc']['label']:
            enabled = cfg['proc']['label'][proc]['enabled']
            if enabled:
                try:
                    order = cfg['proc']['label'][proc]['order']
                    if proc in built_in_label_modules: process = built_in_label_modules[proc]
                    else: process = import_module(proc, fromlist=['*']).__dict__[proc]
                    self.label_processes[order] = process
                except Exception:
                    self.logger.log(f'label_processes creation failed for {proc}:\n{traceback.format_exc()}', Logger.CRITICAL)
        self.label_processes = [self.label_processes[order] for order in sorted(self.label_processes.keys())]
        self.logger.log(f'enabled label_processes: {[f.__name__ for f in self.label_processes]}', Logger.DEBUG)
        
        # post processes
        self.post_processes = dict()
        built_in_post_modules = import_module('liguard.algo.post', fromlist=['*']).__dict__
        for proc in cfg['proc']['post']:
            enabled = cfg['proc']['post'][proc]['enabled']
            if enabled:
                try:
                    order = cfg['proc']['post'][proc]['order']
                    if proc in built_in_post_modules: process = built_in_post_modules[proc]
                    else: process = import_module(proc, fromlist=['*']).__dict__[proc]
                    self.post_processes[order] = process
                except Exception:
                    self.logger.log(f'post_processes creation failed for {proc}:\n{traceback.format_exc()}', Logger.CRITICAL)
            
        self.post_processes = [self.post_processes[order] for order in sorted(self.post_processes.keys())]
        self.logger.log(f'enabled post_processes: {[f.__name__ for f in self.post_processes]}', Logger.DEBUG)
        
    def start(self, cfg):
        # start the LiGuard
        self.is_running = True
        
        # start key event handling
        if self.pcd_io or self.img_io:
            self.start_keyboard_listener()
        
        # the main loop
        while self.is_running:
            if self.is_playing and self.data_dict['current_frame_index'] < self.data_dict['maximum_frame_index']: self.data_dict['current_frame_index'] += 1
            
            # check if user has jumped to a frame
            jump = self.config.get_input_dialog_value('jump_to_frame')
            if jump and (jump >= 0 and jump <= self.data_dict['maximum_frame_index']): self.data_dict['current_frame_index'] = jump
            # check if the frame has changed
            frame_changed = self.data_dict['previous_frame_index'] != self.data_dict['current_frame_index']
            
            # if the frame has changed, update the data dictionary with the new frame data
            if frame_changed:
                profiler.add_target('Total Time / Step')
                self.logger.gui_set_frame(self.data_dict['current_frame_index'] + cfg['data']['start']['global_zero'])
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
                    self.logger.log(f'No data source available', Logger.CRITICAL)
                    time.sleep(0.1)
                    continue

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
                        self.pcd_viz.gui_update(self.data_dict)
                    profiler.end_target('pcd_visualizer_update')
                if self.img_io:
                    profiler.add_target('img_visualizer_update')
                    if cfg['visualization']['enabled']:
                        self.img_viz.gui_update(self.data_dict)
                    profiler.end_target('img_visualizer_update')

                if cfg['visualization']['enabled'] == False:
                    self.logger.log(f'Processed frame {self.data_dict["current_frame_index"]}', Logger.INFO)
                profiler.end_target('Total Time / Step')
            
            # sleep for a while
            time.sleep(0.1)
            profiler.save(os.path.join(self.outputs_dir, 'LiGuard_main.profile'))
            
    def quit(self):
        # stop the app
        if self.is_running: self.is_running = False
        # unhook the keyboard keys
        if self.pynput_listener: self.stop_keyboard_listener()
        
        # close the data sources
        if self.pcd_io: self.pcd_io.close()
        if self.img_io: self.img_io.close()
        if self.lbl_io: self.lbl_io.close()
        
        # close the visualizers
        self.pcd_viz.quit()
        self.img_viz.quit()
        
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