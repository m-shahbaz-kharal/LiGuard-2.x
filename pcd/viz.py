import open3d as o3d

from easydict import EasyDict
import numpy as np

from pcd.proc.utils import create_pcd
from pcd.file_io import FileIO
from pcd.sensor_io import SensorIO

class PointCloudVisualizer:
    def __init__(self, app, cfg: EasyDict, io):
        # set vars
        self.__set_vars__(app, cfg, io)
        # create visualizer
        self.viz = o3d.visualization.Visualizer()
        self.viz.create_window("PointCloud Feed", width=1440, height=1080, left=480, top=30)
        # set rendering options
        self.__set_render_options(cfg)
        # add default geometries
        self.__init_default_geoms__()

    def __set_vars__(self, app, cfg, io):
        self.app = app
        self.cfg = cfg
        self.io = io
        
        self.geoms = dict()
        self.running = False
        self.is_playing = False
        self.index = 0
        
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
        
    def reset(self, cfg, io):
        self.__set_vars__(self.app, cfg, io)
        self.__set_render_options(cfg)
        self.__init_default_geoms__(False)
        
    def __process_single_frame__(self):
        if self.is_playing and self.index < len(self.io) - 1: self.index += 1
        self.pcd_np = self.io[self.index]
        self.point_cloud.points = o3d.utility.Vector3dVector(self.pcd_np[:, 0:3])
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
    
    def key_handler(self, key_event): # key_event: keyboard.KeyboardEvent
        if key_event.event_type == 'down':
            if key_event.name == 'left': self.__key_left_arrow__()
            elif key_event.name == 'right': self.__key_right_arrow__()
            elif key_event.name == 'space': self.__key_space__()
            
    def __key_right_arrow__(self):
        self.is_playing = False
        if self.index < len(self.io) - 1: self.index += 1
        
    def __key_left_arrow__(self):
        self.is_playing = False
        if self.index > 0: self.index -= 1
        
    def __key_space__(self):
        self.is_playing = not self.is_playing