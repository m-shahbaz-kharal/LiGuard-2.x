import open3d.visualization.gui as gui

from config.config_gui import BaseConfiguration
from pcd.io import IO as PCD_IO
from pcd.viz import PointCloudVisualizer

class LiGuard:
    def __init__(self):
        self.app = gui.Application.instance
        self.app.initialize()
        
        self.config = BaseConfiguration(self.app)
        self.pcd_io = None
        self.visualizer = None
        
        self.callbacks = dict()
        self.__set_callbacks()
        
        self.app.run()
        
    def __set_callbacks(self):
        config_call_backs = BaseConfiguration.get_callbacks_dict()
        config_call_backs['apply_only_config'] = [self.reset]
        config_call_backs['quit_config'] = [self.quit]
        self.config.set_callbacks(config_call_backs)
        
    def reset(self, cfg):
        if self.pcd_io != None:
            self.pcd_io.close()
            self.pcd_io = PCD_IO(cfg)
        else:
            self.pcd_io = PCD_IO(cfg)
        if self.visualizer != None: self.visualizer.reset(cfg, self.pcd_io, self.callbacks)
        else:
            self.visualizer = PointCloudVisualizer(self.app, cfg, self.pcd_io)
            self.visualizer.start()
            
    def quit(self, cfg):
        if self.visualizer != None: self.visualizer.close()
        self.app.quit()
        
def main():
    liguard = LiGuard()
    
if __name__ == "__main__":
    main()