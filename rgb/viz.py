import open3d as o3d

from easydict import EasyDict
import time
import numpy as np
import threading

if __name__ == '__main__':
    from sensor_io import SensorIO
else:
    from rgb.sensor_io import SensorIO

class ImageVisualizer:
    def get_callbacks_dict(): return {'key_right_arrow': [], 'key_left_arrow': [], 'key_space': []}
    
    def __init__(self, app, cfg: EasyDict, io: SensorIO, callbacks: dict = get_callbacks_dict()):
        # set vars
        self.__set_vars__(app, cfg, io, callbacks)
        # create visualizer
        self.viz = o3d.visualization.VisualizerWithKeyCallback()
        self.viz.create_window("Image Feed", width=int(1440/4), height=int(1080/4), left=480 - int(1440/4), top=30)
        # key callbacks
        self.viz.register_key_callback(262, self.__key_right_arrow__) # right arrow key
        self.viz.register_key_callback(263, self.__key_left_arrow__) # left arrow key
        self.viz.register_key_callback(32, self.__key_space__) # space bar key
        # add default geometries
        self.__init_default_geoms__()

    def __set_vars__(self, app, cfg, io, callbacks):
        self.app = app
        self.cfg = cfg
        self.io = io
        self.callbacks = callbacks
        
        self.geoms = dict()
        self.running = False
        self.is_playing = False
        self.index = 0
        
    def update_callbacks(self, callbacks): self.callbacks = callbacks
        
    def __init_default_geoms__(self, reset_bounding_box=True):
        self.viz.clear_geometries()
        # global point cloud
        self.img = o3d.geometry.Image(np.zeros((1080, 1440, 3), dtype=np.uint8))
        self.add_geometry('image', self.img, reset_bounding_box=reset_bounding_box)
        
    def add_geometry(self, name, geom, reset_bounding_box=True):
        if name in self.geoms: self.viz.remove_geometry(self.geoms[name], reset_bounding_box=False)
        self.geoms[name] = geom
        self.viz.add_geometry(geom, reset_bounding_box=reset_bounding_box)
        
    def update_geometry(self, name, geom):
        if name in self.geoms: self.viz.update_geometry(geom)
        
    def reset(self, cfg, io, callbacks):
        self.__set_vars__(self.app, cfg, io, callbacks)
        self.__init_default_geoms__(False)
        
    def __process_single_frame__(self):
        if self.is_playing and self.index < len(self.io) - 1: self.index += 1
        img = self.io.__get_item__(self.index)
        self.img = o3d.geometry.Image(img)
        self.add_geometry('image', self.img)
    
    def get_main_functions(self):
        def begin_fn(): self.running = True
        def loop_fn():
            self.__process_single_frame__()
            self.viz.poll_events()
            self.viz.update_renderer()
        def end_fn():
            self.running = False
            self.viz.destroy_window()
        return begin_fn, loop_fn, end_fn
            
    def __key_right_arrow__(self, viz):
        self.is_playing = False
        if self.index < len(self.io) - 1: self.index += 1
        for callback in self.callbacks['key_right_arrow']: callback()
        
    def __key_left_arrow__(self, viz):
        self.is_playing = False
        if self.index > 0: self.index -= 1
        for callback in self.callbacks['key_left_arrow']: callback()
        
    def __key_space__(self, viz):
        self.is_playing = not self.is_playing
        for callback in self.callbacks['key_space']: callback()
        
if __name__ == "__main__":
    cfg = EasyDict({
        'sensors': {
            'camera': {
                'hostname': 'localhost',
                'manufacturer': 'Flir',
                'model': 'BFS-PGE-16S2C-CS',
                'serial_number': '23422874'
            }
        }
    })
    sensor_io = SensorIO(cfg)
    app = o3d.visualization.gui.Application.instance
    img_visualizer = ImageVisualizer(app, cfg, sensor_io)
    running = threading.Event()
    running.set()
    img_visualizer.viz.register_key_callback(ord('Q'), lambda viz: running.clear())
    img_viz_begin_fn, img_viz_loop_fn, img_viz_end_fn =  img_visualizer.get_main_functions()
    img_viz_begin_fn()
    while running.is_set():
        img_viz_loop_fn()
        time.sleep(0.01)
    img_viz_end_fn()
    sensor_io.close()
    print("Done.")