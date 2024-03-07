import open3d as o3d
import numpy as np

from pcd.utils import create_pcd

class PointCloudVisualizer:
    def __init__(self, app, cfg: dict):
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
        render_options.point_size = cfg['visualization']['lidar']['point_size']
        render_options.background_color = cfg['visualization']['lidar']['space_color']
        # add default geometries
        self.__add_default_geometries__(reset_bounding_box)
        
    def __add_default_geometries__(self, reset_bounding_box):
        # add coordinate frame
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame()
        self.__add_geometry__('coordinate_frame', coordinate_frame, reset_bounding_box)

        # add range bounds
        bound = o3d.geometry.AxisAlignedBoundingBox(self.cfg['proc']['lidar']['crop']['min_xyz'], self.cfg['proc']['lidar']['crop']['max_xyz'])
        bound.color = self.cfg['visualization']['lidar']['bound_color']
        self.__add_geometry__('bound', bound, reset_bounding_box)
        
        # global point cloud
        self.point_cloud = create_pcd(np.zeros((1000, 4)))
        self.__add_geometry__('point_cloud', self.point_cloud, reset_bounding_box)
        
        # bboxes
        self.bboxes = []
        
    def __add_geometry__(self, name, geometry, reset_bounding_box):
        if name in self.geometries: self.viz.remove_geometry(self.geometries[name], reset_bounding_box=False)
        else: self.geometries[name] = geometry
        self.viz.add_geometry(geometry, reset_bounding_box=reset_bounding_box)
        
    def __update_geometry__(self, name, geometry):
        if name in self.geometries:
            self.viz.update_geometry(geometry)
            return True
        return False
        
    def update(self, data_dict):
        if "current_point_cloud_numpy" not in data_dict: return
        self.point_cloud.points = o3d.utility.Vector3dVector(data_dict['current_point_cloud_numpy'][:, 0:3])
        self.__update_geometry__('point_cloud', self.point_cloud)
        self.__clear_bboxes__()

        if "current_label_list" not in data_dict: return
        for lbl in data_dict['current_label_list']: self.__add_bbox__(lbl)

    def update_colors(self, pcd_colors: np.ndarray):
        self.point_cloud.colors = o3d.utility.Vector3dVector(pcd_colors)
        self.__update_geometry__('point_cloud', self.point_cloud)

    def __add_bbox__(self, label_dict: dict):
        # bbox params
        lidar_bbox_dict = label_dict['lidar_bbox']
        lidar_xyz_center = lidar_bbox_dict['lidar_xyz_center']
        lidar_xyz_extent = lidar_bbox_dict['lidar_xyz_extent']
        lidar_xyz_euler_angles = lidar_bbox_dict['lidar_xyz_euler_angles']
        color = lidar_bbox_dict['rgb_bbox_color']

        # calculating bbox
        rotation_matrix = o3d.geometry.OrientedBoundingBox.get_rotation_matrix_from_xyz(lidar_xyz_euler_angles)
        lidar_xyz_bbox = o3d.geometry.OrientedBoundingBox(lidar_xyz_center, rotation_matrix, lidar_xyz_extent)
        lidar_xyz_bbox.color = color
        
        self.bboxes.append(lidar_xyz_bbox)
        self.__add_geometry__(f'bbox_{str(len(self.bboxes)+1).zfill(4)}', lidar_xyz_bbox, False)
        
    def __clear_bboxes__(self):
        for bbox in self.bboxes: self.viz.remove_geometry(bbox, False)
        self.bboxes.clear()
        
    def redraw(self):
        self.viz.poll_events()
        self.viz.update_renderer()
        
    def quit(self):
        self.viz.destroy_window()