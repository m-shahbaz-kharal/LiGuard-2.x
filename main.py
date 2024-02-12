import open3d.visualization.gui as gui

from config.config_gui import BaseConfiguration
from pcd.file_io import FileIO as PCD_IO
from pcd.sensor_io import SensorIO as PCD_Sensor_IO
from pcd.viz import PointCloudVisualizer

class LiGuard:
    def __init__(self):
        self.app = gui.Application.instance
        self.app.initialize()
        
        self.config = BaseConfiguration(self.app)
        self.pcd_io = None
        self.sensor_io = None
        self.visualizer = None
        
        self.callbacks = dict()
        self.__set_callbacks__()
        
        self.app.run()
        
    def __set_callbacks__(self):
        config_call_backs = BaseConfiguration.get_callbacks_dict()
        config_call_backs['apply_only_config'] = [self.reset]
        config_call_backs['quit_config'] = [self.quit]
        
        self.callbacks['config'] = config_call_backs
        self.config.set_callbacks(config_call_backs)
        
        visualizer_callbacks = PointCloudVisualizer.get_callbacks_dict()
        self.callbacks['visualizer'] = visualizer_callbacks
        
    def reset(self, cfg):
        if self.pcd_io != None:
            self.pcd_io.close()
            if cfg.data.streaming.enabled: self.pcd_io = PCD_Sensor_IO(cfg)
            else: self.pcd_io = PCD_IO(cfg)
        else:
            if cfg.data.streaming.enabled: self.pcd_io = PCD_Sensor_IO(cfg)
            else: self.pcd_io = PCD_IO(cfg)
            
        if self.visualizer != None: self.visualizer.reset(cfg, self.pcd_io, self.callbacks['visualizer'])
        else:
            self.visualizer = PointCloudVisualizer(self.app, cfg, self.pcd_io)
            self.visualizer.start()
            
    def quit(self, cfg):
        if self.pcd_io != None: self.pcd_io.close()
        if self.visualizer != None: self.visualizer.close()
        self.app.quit()
        
def main():
    liguard = LiGuard()
    
if __name__ == "__main__":
    main()