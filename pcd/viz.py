import open3d as o3d

from easydict import EasyDict
import numpy as np

from pcd.proc.utils import create_pcd

class PointCloudVisualizer:
    def __init__(self, app, cfg: EasyDict):
        self.app = app
        # create visualizer
        self.viz = o3d.visualization.Visualizer()
        self.viz.create_window("PointCloud Feed", width=1440, height=1080, left=480, top=30)
        # init
        self.reset(cfg, True)
        
    def reset(self, cfg, reset_bounding_box=False):
        self.cfg = cfg
        # clear geometries
        self.geometries = dict()
        self.viz.clear_geometries()
        # set render options
        render_options = self.viz.get_render_option()
        render_options.point_size = cfg.visualize.point_size
        render_options.background_color = cfg.visualize.space_color
        # add default geometries
        self.__add_default_geometries__(reset_bounding_box)
        
    def __add_default_geometries__(self, reset_bounding_box):
        # add coordinate frame
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame()
        self.__add_geometry__('coordinate_frame', coordinate_frame, reset_bounding_box)

        # add range bounds
        bound = o3d.geometry.AxisAlignedBoundingBox(self.cfg.data.range[0:3], self.cfg.data.range[3:6])
        bound.color = self.cfg.visualize.bound_color
        self.__add_geometry__('bound', bound, reset_bounding_box)
        
        # global point cloud
        self.point_cloud = create_pcd(np.zeros((1000, 4)))
        self.__add_geometry__('point_cloud', self.point_cloud, reset_bounding_box)
        
        # bboxes
        self.bboxes = []
        
    def __add_geometry__(self, name, geometry, reset_bounding_box):
        if name in self.geometries: self.viz.remove_geometry(self.geometries[name], reset_bounding_box=False)
        self.geometries[name] = geometry
        self.viz.add_geometry(geometry, reset_bounding_box=reset_bounding_box)
        
    def __update_geometry__(self, name, geometry):
        if name in self.geometries:
            self.viz.update_geometry(geometry)
            return True
        return False
        
    def update_pcd(self, pcd_np: np.ndarray):
        self.point_cloud.points = o3d.utility.Vector3dVector(pcd_np[:, 0:3])
        self.__update_geometry__('point_cloud', self.point_cloud)
        
    def add_bbox(self, bbox):
        self.bboxes.append(bbox)
        self.__add_geometry__(f'bbox_{str(len(self.bboxes)+1).zfill(4)}', bbox, False)
        
    def clear_bboxes(self):
        for bbox in self.bboxes: self.viz.remove_geometry(bbox, False)
        self.bboxes.clear()
        
    def redraw(self):
        self.viz.poll_events()
        self.viz.update_renderer()
        
    def quit(self):
        self.viz.destroy_window()