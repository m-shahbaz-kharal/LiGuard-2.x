import open3d as o3d

from easydict import EasyDict
import numpy as np

class ImageVisualizer:
    def __init__(self, app, cfg: EasyDict):
        self.app = app
        # create visualizer
        self.viz = o3d.visualization.Visualizer()
        self.viz.create_window("Image Feed", width=int(1440/4), height=int(1080/4), left=480 - int(1440/4), top=30)
        # init
        self.reset(cfg, True)
        
    def reset(self, cfg, reset_bounding_box=False):
        self.cfg = cfg
        # clear geometries
        self.geometries = dict()
        self.viz.clear_geometries()
        # add default geometries
        self.__add_default_geometries__(reset_bounding_box)
        
    def __add_default_geometries__(self, reset_bounding_box):
        # global image
        self.img = o3d.geometry.Image(np.zeros((1080, 1440, 3), dtype=np.uint8))
        self.__add_geometry__('image', self.img, reset_bounding_box)
        
    def __add_geometry__(self, name, geometry, reset_bounding_box):
        if name in self.geometries: self.viz.remove_geometry(self.geometries[name], reset_bounding_box=False)
        self.geometries[name] = geometry
        self.viz.add_geometry(geometry, reset_bounding_box=reset_bounding_box)
        
    def __update_geometry__(self, name, geometry):
        if name in self.geometries: self.viz.update_geometry(geometry)
        
    def update_img(self, img_np: np.ndarray):
        self.img = o3d.geometry.Image(img_np)
        self.__add_geometry__('image', self.img, False)
        
    def redraw(self):
        self.viz.poll_events()
        self.viz.update_renderer()
        
    def quit(self):
        self.viz.destroy_window()