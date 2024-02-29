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
        
    def start(self, cfg):
        with self.lock: self.is_running = True
        while True:
            with self.lock:
                if not self.is_running: break
                elif self.is_playing and self.frame_index < len(self.pcd_io) - 1: self.frame_index += 1
            
            pcd_np = self.pcd_io[self.frame_index]
            img_np = self.img_io[self.frame_index]
            lbl_list = self.lbl_io[self.frame_index]
            
            self.pcd_visualizer.update_pcd(pcd_np)
            self.pcd_visualizer.clear_bboxes()
            for lbl in lbl_list: self.pcd_visualizer.add_bbox(lbl)
            self.img_visualizer.update_img(img_np)
            for lbl in lbl_list: self.img_visualizer.add_bbox(lbl)
            
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