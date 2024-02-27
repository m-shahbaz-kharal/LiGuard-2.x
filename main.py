import open3d.visualization.gui as gui

from config.config_gui import BaseConfiguration

from pcd.file_io import FileIO as PCD_File_IO
from pcd.sensor_io import SensorIO as PCD_Sensor_IO
from pcd.viz import PointCloudVisualizer
from pcd.utils import colorize_pcd

from rgb.file_io import FileIO as RGB_File_IO
from rgb.sensor_io import SensorIO as RGB_Sensor_IO
from rgb.viz import ImageVisualizer

import time, threading

class LiGuard:
    def __init__(self):
        self.app = gui.Application.instance
        self.app.initialize()
        
        self.config = BaseConfiguration(self.app)
        
        self.pcd_io = None
        self.pcd_visualizer = None
        
        self.rgb_io = None
        self.rgb_visualizer = None
        
        self.callbacks = dict()
        self.__set_callbacks__()
        
        self.is_running = threading.Event()
        self.app.run()
        
    def __set_callbacks__(self):
        config_call_backs = BaseConfiguration.get_callbacks_dict()
        config_call_backs['apply_only_config'] = [self.reset]
        config_call_backs['quit_config'] = [self.quit]
        self.config.update_callbacks(config_call_backs)
        self.callbacks['config'] = config_call_backs
        
        pcd_visualizer_callbacks = PointCloudVisualizer.get_callbacks_dict()
        self.callbacks['pcd_visualizer'] = pcd_visualizer_callbacks
        
        rgb_visualizer_callbacks = ImageVisualizer.get_callbacks_dict()
        self.callbacks['rgb_visualizer'] = rgb_visualizer_callbacks
        
    def reset(self, cfg):
        self.is_running.clear()
        if self.pcd_io != None: self.pcd_io.close()
        if cfg.sensors.lidar.enabled: self.pcd_io = PCD_Sensor_IO(cfg)
        else: self.pcd_io = PCD_File_IO(cfg)
        
        if self.rgb_io != None: self.rgb_io.close()
        if cfg.sensors.camera.enabled: self.rgb_io = RGB_Sensor_IO(cfg)
        else: self.rgb_io = self.rgb_io = RGB_File_IO(cfg)
             
        if self.pcd_visualizer != None: self.pcd_visualizer.reset(cfg, self.pcd_io, self.callbacks['pcd_visualizer'])
        else: self.pcd_visualizer = PointCloudVisualizer(self.app, cfg, self.pcd_io)
            
        if self.rgb_visualizer != None: self.rgb_visualizer.reset(cfg, self.rgb_io, self.callbacks['rgb_visualizer'])
        else: self.rgb_visualizer = ImageVisualizer(self.app, cfg, self.rgb_io)
        
        def pcd_vis_key_space_callback(): self.rgb_visualizer.is_playing = not self.rgb_visualizer.is_playing
        self.callbacks['pcd_visualizer']['key_space'] = [pcd_vis_key_space_callback]
        self.pcd_visualizer.update_callbacks(self.callbacks['pcd_visualizer'])
        
        def rgb_vis_key_space_callback(): self.pcd_visualizer.is_playing = not self.pcd_visualizer.is_playing
        self.callbacks['rgb_visualizer']['key_space'] = [rgb_vis_key_space_callback]
        self.rgb_visualizer.update_callbacks(self.callbacks['rgb_visualizer'])
        
        pcd_viz_begin_fn, pcd_viz_loop_fn, pcd_viz_end_fn =  self.pcd_visualizer.get_main_functions()
        rgb_viz_begin_fn, rgb_viz_loop_fn, rgb_viz_end_fn =  self.rgb_visualizer.get_main_functions()
        
        # main loop
        pcd_viz_begin_fn()
        rgb_viz_begin_fn()
        self.is_running.set()
        while self.is_running.is_set():
            pcd_viz_loop_fn()
            rgb_viz_loop_fn()
            colorize_pcd(self.pcd_visualizer.geoms['point_cloud'], self.rgb_visualizer.img_np, self.config.cfg.sensors.camera.camera_matrix, self.config.cfg.sensors.camera.distortion_coeffs, self.config.cfg.sensors.camera.T_lidar_camera)
            time.sleep(cfg.debug.asyncio_sleep)
        pcd_viz_end_fn()
        rgb_viz_end_fn()
            
    def quit(self, cfg):
        if self.pcd_io != None: self.pcd_io.close()
        if self.rgb_io != None: self.rgb_io.close()
        self.is_running.clear()
        self.app.quit()
        
def main():
    liguard = LiGuard()
    
if __name__ == "__main__":
    main()