import open3d as o3d

from easydict import EasyDict
import time
import numpy as np
import threading

if __name__ == "__main__":
    from utils import create_pcd
    from file_io import FileIO
    from sensor_io import SensorIO
else:
    from pcd.utils import create_pcd
    from pcd.file_io import FileIO
    from pcd.sensor_io import SensorIO

class PointCloudVisualizer:
    def get_callbacks_dict(): return {'key_right_arrow': [], 'key_left_arrow': [], 'key_space': []}
    
    def __init__(self, app, cfg: EasyDict, io: [FileIO, SensorIO], callbacks: dict = get_callbacks_dict()):
        # set vars
        self.__set_vars__(app, cfg, io, callbacks)
        # create visualizer
        self.viz = o3d.visualization.VisualizerWithKeyCallback()
        self.viz.create_window("PointCloud Feed", width=1440, height=1080, left=480, top=30)
        # set rendering options
        self.__set_render_options(cfg)
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
        
    def __set_render_options(self, cfg):
        render_options = self.viz.get_render_option()
        render_options.point_size = cfg.visualize.point_size
        render_options.background_color = cfg.visualize.space_color
        
    def __init_default_geoms__(self, reset_bounding_box=True):
        self.viz.clear_geometries()
        # status text
        status_text = o3d.geometry.OrientedBoundingBox()
        # add coordinate frame
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame()
        self.add_geometry('coordinate_frame', coordinate_frame, reset_bounding_box=reset_bounding_box)

        # add range bounds
        bound = o3d.geometry.AxisAlignedBoundingBox(self.cfg.data.range[0:3], self.cfg.data.range[3:6])
        bound.color = self.cfg.visualize.bound_color
        self.add_geometry('bound', bound, reset_bounding_box=reset_bounding_box)
        
        # global point cloud
        self.point_cloud = create_pcd(np.zeros((1000, 4)))
        self.add_geometry('point_cloud', self.point_cloud, reset_bounding_box=reset_bounding_box)
        
    def add_geometry(self, name, geom, reset_bounding_box=True):
        if name in self.geoms: self.viz.remove_geometry(self.geoms[name], reset_bounding_box=False)
        self.geoms[name] = geom
        self.viz.add_geometry(geom, reset_bounding_box=reset_bounding_box)
        
    def update_geometry(self, name, geom):
        if name in self.geoms: self.viz.update_geometry(geom)
        
    def reset(self, cfg, io, callbacks):
        self.__set_vars__(self.app, cfg, io, callbacks)
        self.__set_render_options(cfg)
        self.__init_default_geoms__(False)
        
    def __process_single_frame__(self):
        if self.is_playing and self.index < len(self.io) - 1: self.index += 1
        pcd = self.io.__get_item__(self.index)
        self.point_cloud.points = o3d.utility.Vector3dVector(pcd[:, 0:3])
        self.update_geometry('point_cloud', self.point_cloud)
    
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
        'data': {
            'range': [-10, -10, -10, 10, 10, 10]
        },
        'visualize': {
            'point_size': 1,
            'bound_color': [0, 0, 1],
            'space_color': [0, 0, 0]
        },
        'sensors': { # lidar and camera configurations
            'lidar': {
                'enabled': False, # set True to stream point clouds from sensor
                'hostname': '192.168.1.12', # sensor ip address or hostname
                'manufacturer': 'Ouster', # sensor manufacturer
                'model': 'OS1-64', # sensor model
                'serial_number': '122204001078' # sensor serial number
            }
        }
    })
    sensor_io = SensorIO(cfg)
    app = o3d.visualization.gui.Application.instance
    pcd_visualizer = PointCloudVisualizer(app, cfg, sensor_io)
    running = threading.Event()
    running.set()
    pcd_visualizer.viz.register_key_callback(ord('Q'), lambda viz: running.clear())
    pcd_viz_begin_fn, pcd_viz_loop_fn, pcd_viz_end_fn =  pcd_visualizer.get_main_functions()
    pcd_viz_begin_fn()
    while running.is_set():
        pcd_viz_loop_fn()
        time.sleep(0.01)
    pcd_viz_end_fn()
    sensor_io.close()
    print("Done.")