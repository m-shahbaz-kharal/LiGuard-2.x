import open3d.visualization.gui as gui

from config.gui import BaseConfiguration as BaseConfigurationGUI

from pcd.file_io import FileIO as PCD_File_IO
from pcd.sensor_io import SensorIO as PCD_Sensor_IO
from pcd.viz import PointCloudVisualizer

from img.file_io import FileIO as IMG_File_IO
from img.sensor_io import SensorIO as IMG_Sensor_IO
from img.viz import ImageVisualizer

from lbl.file_io import FileIO as LBL_File_IO

import keyboard, threading, time

class LiGuard:
    def __init__(self):
        self.app = gui.Application.instance
        self.app.initialize()
        
        self.config = BaseConfigurationGUI(self.app)
        config_call_backs = BaseConfigurationGUI.get_callbacks_dict()
        config_call_backs['apply_and_save_config'] = [self.reset, self.start]
        config_call_backs['apply_and_save_as_config'] = [self.reset, self.start]
        config_call_backs['apply_only_config'] = [self.reset, self.start]
        config_call_backs['quit_config'] = [self.quit]
        self.config.update_callbacks(config_call_backs)
        
        self.pcd_io = None
        self.pcd_visualizer = None
        
        self.img_io = None
        self.img_visualizer = None
        
        self.lbl_io = None
        
        self.lock = threading.Lock()
        self.is_running = False # if the app is running
        self.is_playing = False # if the frames are playing
        self.frame_index = 0
        self.last_frame_index = -1
        self.data = dict()
        
        self.app.run()
        
    def handle_key_event(self, event:keyboard.KeyboardEvent):
        if event.event_type == keyboard.KEY_DOWN:
            with self.lock:
                if event.name == 'right':
                    self.is_playing = False
                    if self.frame_index < self.data['max_frame_index']:
                        self.frame_index += 1
                elif event.name == 'left':
                    self.is_playing = False
                    if self.frame_index > 0:
                        self.frame_index -= 1
                elif event.name == 'space':
                    self.is_playing = not self.is_playing
        
    def reset(self, cfg):
        keyboard.unhook_all()
        with self.lock: self.is_running = False
        
        if self.pcd_io != None: self.pcd_io.close()
        if cfg['data']['lidar']['enabled']: self.pcd_io = PCD_File_IO(cfg)
        elif cfg['sensors']['lidar']['enabled']: self.pcd_io = PCD_Sensor_IO(cfg)
        else: self.pcd_io = None
        self.data['total_pcd_frames'] = len(self.pcd_io) if self.pcd_io else 0
        
        if self.pcd_io and cfg['visualization']['enabled']:
            if self.pcd_visualizer != None: self.pcd_visualizer.reset(cfg)
            else: self.pcd_visualizer = PointCloudVisualizer(self.app, cfg)
        
        if self.img_io != None: self.img_io.close()
        if cfg['data']['camera']['enabled']: self.img_io = IMG_File_IO(cfg)
        elif cfg['sensors']['camera']['enabled']: self.img_io = IMG_Sensor_IO(cfg)
        else: self.img_io = None
        self.data['total_img_frames'] = len(self.img_io) if self.img_io else 0
        
        if self.img_io and cfg['visualization']['enabled']:
            if self.img_visualizer != None: self.img_visualizer.reset(cfg)
            else: self.img_visualizer = ImageVisualizer(self.app, cfg)
        
        if self.lbl_io != None: self.lbl_io.close()
        if cfg['data']['label']['enabled']: self.lbl_io = LBL_File_IO(cfg)
        else: self.lbl_io = None
        self.data['total_lbl_frames'] = len(self.lbl_io) if self.lbl_io else 0
        
        self.data['max_frame_index'] = min(self.data['total_pcd_frames'], self.data['total_img_frames'], self.data['total_lbl_frames']) - 1
        
        self.lidar_procs = [__import__('algo.lidar', fromlist=[proc]).__dict__[proc] for proc in cfg['proc']['lidar'] if cfg['proc']['lidar'][proc]['enabled']]
        self.camera_procs = [__import__('algo.camera', fromlist=[proc]).__dict__[proc] for proc in cfg['proc']['camera'] if cfg['proc']['camera'][proc]['enabled']]
        self.label_procs = [__import__('algo.label', fromlist=[proc]).__dict__[proc] for proc in cfg['proc']['label'] if cfg['proc']['label'][proc]['enabled']]
        
    def start(self, cfg):
        with self.lock: self.is_running = True
        
        if self.pcd_visualizer or self.img_visualizer: keyboard.hook(self.handle_key_event)
        
        while True:
            with self.lock:
                if not self.is_running: break
                elif self.is_playing and self.frame_index < self.data['max_frame_index']: self.frame_index += 1
            
            frame_changed = self.last_frame_index != self.frame_index
            
            if frame_changed:
                self.last_frame_index = self.frame_index
                if self.pcd_io: self.data['current_point_cloud_numpy'] = self.pcd_io[self.frame_index]
                elif 'current_point_cloud_numpy' in self.data: self.data.pop('current_point_cloud_numpy')
                if self.img_io: self.data['current_image_numpy'] = self.img_io[self.frame_index]
                elif 'current_image_numpy' in self.data: self.data.pop('current_image_numpy')
                if self.lbl_io: self.data['current_label_list'] = self.lbl_io[self.frame_index]
                elif 'current_label_list' in self.data: self.data.pop('current_label_list')
            
                if self.pcd_io:
                    for proc in self.lidar_procs: proc(self.data, cfg)
                if self.img_io:
                    for proc in self.camera_procs: proc(self.data, cfg)
                if self.lbl_io:
                    for proc in self.label_procs: proc(self.data, cfg)
            
                if self.pcd_io:
                    self.pcd_visualizer.update(self.data)
                    self.pcd_visualizer.redraw()
                if self.img_io:
                    self.img_visualizer.update(self.data)
                    self.img_visualizer.redraw()

            else:
                if self.pcd_io: self.pcd_visualizer.redraw()
                if self.img_io: self.img_visualizer.redraw()
            
            time.sleep(cfg['threads']['vis_sleep'])
            
    def quit(self, cfg):
        with self.lock: self.is_running = False
        keyboard.unhook_all()
        
        if self.pcd_io: self.pcd_io.close()
        if self.img_io: self.img_io.close()
        if self.lbl_io: self.lbl_io.close()
        
        if self.pcd_visualizer: self.pcd_visualizer.quit()
        if self.img_visualizer: self.img_visualizer.quit()
        
        self.app.quit()
        
def main():
    LiGuard()
    
main()