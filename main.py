import open3d.visualization.gui as gui

from config.gui import BaseConfiguration as BaseConfigurationGUI

from pcd.file_io import FileIO as PCD_File_IO
from pcd.sensor_io import SensorIO as PCD_Sensor_IO
from pcd.viz import PointCloudVisualizer

from rgb.file_io import FileIO as RGB_File_IO
from rgb.sensor_io import SensorIO as RGB_Sensor_IO
from rgb.viz import ImageVisualizer

import keyboard, threading, time

class LiGuard:
    def __init__(self):
        self.app = gui.Application.instance
        self.app.initialize()
        
        self.config = BaseConfigurationGUI(self.app)
        config_call_backs = BaseConfigurationGUI.get_callbacks_dict()
        config_call_backs['apply_only_config'] = [self.reset]
        config_call_backs['quit_config'] = [self.quit]
        self.config.update_callbacks(config_call_backs)
        
        self.pcd_io = None
        self.pcd_visualizer = None
        
        self.rgb_io = None
        self.rgb_visualizer = None
        
        self.is_running = threading.Event()
        self.app.run()
        
    def reset(self, cfg):
        self.is_running.clear()
        keyboard.unhook_all()
        
        if self.pcd_io != None: self.pcd_io.close()
        if cfg.sensors.lidar.enabled: self.pcd_io = PCD_Sensor_IO(cfg)
        else: self.pcd_io = PCD_File_IO(cfg)
        
        if self.rgb_io != None: self.rgb_io.close()
        if cfg.sensors.camera.enabled: self.rgb_io = RGB_Sensor_IO(cfg)
        else: self.rgb_io = self.rgb_io = RGB_File_IO(cfg)
             
        if self.pcd_visualizer != None: self.pcd_visualizer.reset(cfg, self.pcd_io)
        else: self.pcd_visualizer = PointCloudVisualizer(self.app, cfg, self.pcd_io)
        keyboard.hook(self.pcd_visualizer.key_handler)
                    
        if self.rgb_visualizer != None: self.rgb_visualizer.reset(cfg, self.rgb_io)
        else: self.rgb_visualizer = ImageVisualizer(self.app, cfg, self.rgb_io)
        keyboard.hook(self.rgb_visualizer.key_handler)
        
        pcd_viz_begin_fn, pcd_viz_loop_fn, pcd_viz_end_fn =  self.pcd_visualizer.get_main_functions()
        rgb_viz_begin_fn, rgb_viz_loop_fn, rgb_viz_end_fn =  self.rgb_visualizer.get_main_functions()
        
        # main loop
        pcd_viz_begin_fn()
        rgb_viz_begin_fn()
        self.is_running.set()
        while self.is_running.is_set():
            pcd_viz_loop_fn()
            rgb_viz_loop_fn()
            time.sleep(cfg.debug.asyncio_sleep)
        pcd_viz_end_fn()
        rgb_viz_end_fn()
            
    def quit(self, cfg):
        keyboard.unhook_all()
        if self.pcd_io != None: self.pcd_io.close()
        if self.rgb_io != None: self.rgb_io.close()
        self.is_running.clear()
        self.app.quit()
        
def main():
    liguard = LiGuard()
    
main()