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
from easydict import EasyDict

class LiGuard:
    def __init__(self):
        self.app = gui.Application.instance
        self.app.initialize()
        
        self.config = BaseConfigurationGUI(self.app)
        config_call_backs = BaseConfigurationGUI.get_callbacks_dict()
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
        self.data = EasyDict()
        
        keyboard.hook(self.handle_key_event)
        self.app.run()
        
    def handle_key_event(self, event:keyboard.KeyboardEvent):
        if event.event_type == keyboard.KEY_DOWN:
            with self.lock:
                if event.name == 'right':
                    self.is_playing = False
                    if self.frame_index < len(self.pcd_io) - 1: self.frame_index += 1
                elif event.name == 'left':
                    self.is_playing = False
                    if self.frame_index > 0: self.frame_index -= 1
                elif event.name == 'space':
                    self.is_playing = not self.is_playing
        
    def reset(self, cfg):
        with self.lock: self.is_running = False
        
        if self.pcd_io != None: self.pcd_io.close()
        if cfg.sensors.lidar.enabled: self.pcd_io = PCD_Sensor_IO(cfg)
        else: self.pcd_io = PCD_File_IO(cfg)
        
        if self.img_io != None: self.img_io.close()
        if cfg.sensors.camera.enabled: self.img_io = IMG_Sensor_IO(cfg)
        else: self.img_io = self.img_io = IMG_File_IO(cfg)
        
        if self.lbl_io != None: self.lbl_io.close()
        self.lbl_io = LBL_File_IO(cfg)
             
        if self.pcd_visualizer != None: self.pcd_visualizer.reset(cfg)
        else: self.pcd_visualizer = PointCloudVisualizer(self.app, cfg)
                    
        if self.img_visualizer != None: self.img_visualizer.reset(cfg)
        else: self.img_visualizer = ImageVisualizer(self.app, cfg)
        
        self.lidar_procs = [__import__('algo.lidar', fromlist=[proc]).__dict__[proc] for proc in cfg.proc.lidar if cfg.proc.lidar[proc].enabled]
        self.camera_procs = []
        self.label_procs = [__import__('algo.label', fromlist=[proc]).__dict__[proc] for proc in cfg.proc.label if cfg.proc.label[proc].enabled]
        self.fusion_procs = [__import__('algo.fusion', fromlist=[proc]).__dict__[proc] for proc in cfg.proc.fusion if cfg.proc.fusion[proc].enabled]
        
    def start(self, cfg):
        with self.lock: self.is_running = True
        while True:
            with self.lock:
                if not self.is_running: break
                elif self.is_playing and self.frame_index < len(self.pcd_io) - 1: self.frame_index += 1
            
            self.data.current_point_cloud_numpy = self.pcd_io[self.frame_index]
            self.data.current_image_numpy = self.img_io[self.frame_index]
            self.data.current_label_list = self.lbl_io[self.frame_index]
            
            for proc in self.lidar_procs: proc(self.data, cfg)
            for proc in self.camera_procs: proc(self.data, cfg)
            for proc in self.label_procs: proc(self.data, cfg)
            
            # fusion_input = {'pcd_np': pcd_np, 'img_np': img_np, 'lbl_list': lbl_list}
            # fusion_output = dict()
            # for proc in self.fusion_procs: proc(fusion_input, cfg, fusion_output)
            
            self.pcd_visualizer.update(self.data)
            self.img_visualizer.update(self.data)
            
            self.pcd_visualizer.redraw()
            self.img_visualizer.redraw()
            
            time.sleep(cfg.debug.asyncio_sleep)
            
    def quit(self, cfg):
        keyboard.unhook_all()
        with self.lock: self.is_running = False
        
        self.pcd_io.close()
        self.img_io.close()
        
        self.pcd_visualizer.quit()
        self.img_visualizer.quit()
        
        self.app.quit()
        
def main():
    LiGuard()
    
main()