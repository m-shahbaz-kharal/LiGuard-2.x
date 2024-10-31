import os
from gui.config_gui import get_abs_path

import open3d as o3d
import numpy as np

from pcd.utils import create_pcd

from gui.logger_gui import Logger

import open3d as o3d
import numpy as np

import pygetwindow as gw

class PointCloudVisualizer:
    """
    Class for visualizing point clouds and bounding boxes using Open3D.
    """

    def __init__(self, app, cfg: dict):
        """
        Initializes the PointCloudVisualizer.

        Args:
            app: The application object.
            cfg: A dictionary containing configuration parameters.
        """
        self.app = app
        # create visualizer
        self.viz = o3d.visualization.Visualizer()
        self.win_created = self.viz.create_window("PointCloud Feed", width=1000, height=1080, left=480, top=30)
        # init
        # create necessary paths
        if cfg['visualization']['lidar']['save_images']:
            self.lidar_save_path = os.path.join(get_abs_path(cfg['data']['outputs_dir']), 'pcd_viz')
            os.makedirs(self.lidar_save_path, exist_ok=True)
        # reset
        self.reset(cfg, True)

    def __is_focused__(self):
        """
        Checks if the visualizer is focused.

        Returns:
            bool: True if the visualizer is focused, False otherwise.
        """
        window = gw.getWindowsWithTitle('PointCloud Feed')
        if window: return window[0].isActive
        return False
        
    def reset(self, cfg, reset_bounding_box=False):
        """
        Resets the visualizer.

        Args:
            cfg: A dictionary containing configuration parameters.
            reset_bounding_box: Whether to reset the bounding box or not.
        """
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
        """
        Adds default geometries to the visualizer.

        Args:
            reset_bounding_box: Whether to reset the bounding box or not.
        """
        # add coordinate frame
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame()
        self.__add_geometry__('coordinate_frame', coordinate_frame, reset_bounding_box)

        # add default range bound
        default_bound = o3d.geometry.AxisAlignedBoundingBox([-50, -50, -5], [+50, +50, +5])
        default_bound.color = self.cfg['visualization']['lidar']['bound_color']
        self.__add_geometry__('bound', default_bound, reset_bounding_box)

        # if crop enabled, remove default bound and add bound according to crop params
        if self.cfg['proc']['lidar']['crop']['enabled']:
            crop_bound = o3d.geometry.AxisAlignedBoundingBox(self.cfg['proc']['lidar']['crop']['min_xyz'], self.cfg['proc']['lidar']['crop']['max_xyz'])
            crop_bound.color = self.cfg['visualization']['lidar']['bound_color']
            self.__add_geometry__('bound', crop_bound, reset_bounding_box)
        else:
            self.viz.remove_geometry(default_bound, False)
        
        # global point cloud
        self.point_cloud = create_pcd(np.zeros((1000, 4)))
        self.__add_geometry__('point_cloud', self.point_cloud, reset_bounding_box)
        
        # bboxes
        self.bboxes = []

        # trajectories
        self.trajectories = []
        
    def __add_geometry__(self, name, geometry, reset_bounding_box):
        """
        Adds a geometry to the visualizer.

        Args:
            name: The name of the geometry.
            geometry: The geometry object.
            reset_bounding_box: Whether to reset the bounding box or not.
        """
        if name in self.geometries:
            self.viz.remove_geometry(self.geometries[name], reset_bounding_box=False)
        else:
            self.geometries[name] = geometry
        self.viz.add_geometry(geometry, reset_bounding_box=reset_bounding_box)
        
    def __update_geometry__(self, name, geometry):
        """
        Updates a geometry in the visualizer.

        Args:
            name: The name of the geometry.
            geometry: The updated geometry object.

        Returns:
            bool: True if the geometry was updated successfully, False otherwise.
        """
        if name in self.geometries:
            self.viz.update_geometry(geometry)
            return True
        return False
        
    def update(self, data_dict):
        """
        Updates the visualizer with new data.

        Args:
            data_dict: A dictionary containing the data to be visualized.
        """
        if 'logger' in data_dict: logger:Logger = data_dict['logger']
        else: print('[CRITICAL ERROR]: No logger object in data_dict. It is abnormal behavior as logger object is created by default. Please check if some script is removing the logger key in data_dict.'); return
        if "current_point_cloud_numpy" not in data_dict:
            logger.log(f'current_point_cloud_numpy not found in data_dict', Logger.DEBUG)
            return
        self.point_cloud.points = o3d.utility.Vector3dVector(data_dict['current_point_cloud_numpy'][:, 0:3])
        if 'current_point_cloud_point_colors' in data_dict:
            self.point_cloud.colors = o3d.utility.Vector3dVector(data_dict['current_point_cloud_point_colors'][:, 0:3])
        else:
            self.point_cloud.paint_uniform_color([1,1,1])
        self.__update_geometry__('point_cloud', self.point_cloud)
        
        self.__clear_bboxes__()
        self.__clear_trajectories__()
        
        if "current_label_list" not in data_dict:
            return
        for lbl in data_dict['current_label_list']:
            self.__add_bbox__(lbl)
            self.__add_cluster__(lbl)
            self.__add_trajectory__(lbl)

    def __add_bbox__(self, label_dict: dict):
        """
        Adds a bounding box to the visualizer.

        Args:
            label_dict: A dictionary containing the label information.
        """
        if 'bbox_3d' not in label_dict or not self.cfg['visualization']['lidar']['draw_bbox_3d']:
            return
        # bbox params
        bbox_3d_dict = label_dict['bbox_3d']
        xyz_center = bbox_3d_dict['xyz_center']
        xyz_extent = bbox_3d_dict['xyz_extent']
        xyz_euler_angles = bbox_3d_dict['xyz_euler_angles']
        if bbox_3d_dict['predicted']:
            color = bbox_3d_dict['rgb_color']
        else:
            color = bbox_3d_dict['rgb_color'] * 0.5 # darken the color for ground truth

        # calculating bbox
        rotation_matrix = o3d.geometry.OrientedBoundingBox.get_rotation_matrix_from_xyz(xyz_euler_angles)
        lidar_xyz_bbox = o3d.geometry.OrientedBoundingBox(xyz_center, rotation_matrix, xyz_extent)
        lidar_xyz_bbox.color = color

        self.bboxes.append(lidar_xyz_bbox)
        self.__add_geometry__(f'bbox_{str(len(self.bboxes)+1).zfill(4)}', lidar_xyz_bbox, False)
        
    def __clear_bboxes__(self):
        """
        Clears all the bounding boxes from the visualizer.
        """
        for bbox in self.bboxes:
            self.viz.remove_geometry(bbox, False)
        self.bboxes.clear()

    def __add_cluster__(self, label_dict: dict):
        """
        Adds a cluster to the visualizer.

        Args:
            label_dict: A dictionary containing the label information.
        """
        if 'lidar_cluster' not in label_dict or not self.cfg['visualization']['lidar']['draw_cluster']:
            return
        # cluster params
        lidar_cluster_dict = label_dict['lidar_cluster']
        point_indices = lidar_cluster_dict['point_indices']
        colors = np.asarray(self.point_cloud.colors)
        if colors.shape[0] != point_indices.shape[0]:
            colors = np.zeros_like(self.point_cloud.points)
        colors[point_indices] = np.random.rand(3) # ToDO: use consistent color if tracking is enabled
        self.point_cloud.colors = o3d.utility.Vector3dVector(colors)

    def __add_trajectory__(self, label_dict: dict):
        """
        Adds a trajectory to the visualizer.

        Args:
            trajectory: The trajectory to be added.
            color: The color of the trajectory.
        """
        if 'bbox_3d' not in label_dict or not self.cfg['visualization']['lidar']['draw_trajectory']:
            return

        color = label_dict['bbox_3d']['rgb_color']
        
        if 'past_trajectory' in label_dict['bbox_3d']: past_trajectory = label_dict['bbox_3d']['past_trajectory']
        else: past_trajectory = []
        
        if 'future_trajectory' in label_dict['bbox_3d']: future_trajectory = label_dict['bbox_3d']['future_trajectory']
        else: future_trajectory = []
        
        if len(past_trajectory) >= 2:
            lines = []
            for i in range(len(past_trajectory) - 1): lines.append([i, i + 1])
            line_set = o3d.geometry.LineSet()
            line_set.points = o3d.utility.Vector3dVector(past_trajectory)
            line_set.lines = o3d.utility.Vector2iVector(lines)
            line_set.colors = o3d.utility.Vector3dVector([color for _ in range(len(lines))])
            self.trajectories.append(line_set)
            self.__add_geometry__(f'past_trajectory_{str(len(self.trajectories)+1).zfill(4)}', line_set, False)
        
        if len(future_trajectory) >= 2:
            lines = []
            for i in range(len(future_trajectory) - 1): lines.append([i, i + 1])
            line_set = o3d.geometry.LineSet()
            line_set.points = o3d.utility.Vector3dVector(future_trajectory)
            line_set.lines = o3d.utility.Vector2iVector(lines)
            line_set.colors = o3d.utility.Vector3dVector([color for _ in range(len(lines))])
            self.trajectories.append(line_set)
            self.__add_geometry__(f'future_trajectory_{str(len(self.trajectories)+1).zfill(4)}', line_set, False)

    def __clear_trajectories__(self):
        """
        Clears all the trajectories from the visualizer.
        """
        for trajectory in self.trajectories:
            self.viz.remove_geometry(trajectory, False)
        self.trajectories.clear()
        
    def redraw(self):
        """
        Redraws the visualizer.
        """
        self.viz.poll_events()
        self.viz.update_renderer()

    def save_current_view(self, frame_idx):
        """
        Saves the current view of the visualizer to file.

        Args:
            frame_idx: The index of the frame.
        """
        file_path = os.path.join(self.lidar_save_path, f'{frame_idx:08d}.png')
        self.viz.capture_screen_image(file_path)

    def save_view_status(self):
        """
        Saves the view status (parameters of looking camera) of the visualizer.
        """
        if self.win_created:
            save_path = os.path.join(get_abs_path(self.cfg['data']['outputs_dir']), 'view_status.txt')
            with open(save_path, 'w') as file: file.write(str(self.viz.get_view_status()))
    
    def load_view_status(self):
        """
        Loads the view status (parameters of looking camera) of the visualizer from the configuration.
        """
        try:
            load_path = os.path.join(get_abs_path(self.cfg['data']['outputs_dir']), 'view_status.txt')
            with open(load_path, 'r') as file: self.viz.set_view_status(file.read())
        except: pass

    def quit(self):
        """ 
        Quits the visualizer.
        """
        self.viz.destroy_window()