import os
import time

import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import numpy as np

from liguard.gui.gui_utils import resolve_for_application_root, resolve_for_default_workspace
from liguard.gui.logger_gui import Logger
from liguard.pcd.utils import create_pcd

import threading

class PointCloudVisualizer:
    """
    Class for visualizing point clouds and bounding boxes using Open3D.
    """

    def __init__(self, app: gui.Application, logger: Logger=None):
        """
        Initializes the PointCloudVisualizer.

        Args:
            app (gui.Application): The application object.
            logger (Logger): The logger object.
        """
        self.app = app
        if logger: self.log = logger.log
        else: self.log = lambda msg,lvl: print(f'[Point Cloud Visualizer][{Logger.__level_string__[lvl]}] {msg}')
        
        # create visualizer
        self.viz = o3d.visualization.O3DVisualizer(title="PointCloud Feed", width=1000, height=1080)
        self.viz.set_on_close(lambda: False)

        # scaling factor between OS pixels and device pixels
        self.scaling = self.viz.scaling

        self.viz.show_menu(False)
        self.viz.show_settings = True
        self.viz.show_skybox(False)
        self.viz.ground_plane = rendering.Scene.GroundPlane.XY
        self.viz.enable_raw_mode(True)

        #
        self.materials = dict()
        self.geometries = dict()
        
        #
        self.lock = threading.Lock()

        # add to window
        self.app.add_window(self.viz)

        # reset
        self.reset()

    def show(self):
        """
        Shows the visualizer window.
        """
        self.app.post_to_main_thread(self.viz, lambda: self.viz.show(True))

    def hide(self):
        """
        Hides the visualizer window.
        """
        self.app.post_to_main_thread(self.viz, lambda: self.viz.show(False))
        
    def reset(self, cfg=None):
        """
        Resets the visualizer.

        Args:
            cfg: A dictionary containing configuration parameters.
        """
        # acquire lock
        self.lock.acquire()

        # update cfg
        self.cfg = cfg
        
        #
        def clear_and_init_settings_gui():
            self.clear()
            self._init_settings()

        self.app.post_to_main_thread(self.viz, clear_and_init_settings_gui)

        # release lock
        self.lock.release()

    def clear(self):
        self.viz.clear_3d_labels()
        self.materials.clear()
        for name in self.geometries: self.viz.remove_geometry(name)
        self.geometries.clear()

    def _init_settings(self):
        # set render options
        if self.cfg:
            self.viz.point_size = int(self.scaling * self.cfg['visualization']['lidar']['point_size'])
            self.viz.set_background(np.append(self.cfg['visualization']['lidar']['space_color'], 1), None)
        else:
            self.viz.add_3d_label([0, 0, 0], "No Data")
        
        # add coordinate frame
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame()
        self.__add_geometry__('coordinate_frame', coordinate_frame)

        # global point cloud
        self.default_points = np.random.rand(1000, 3) * [100, 100, 10]
        self.default_points -= [50, 50, 5]
        self.point_cloud = create_pcd(self.default_points)
        self.__add_geometry__('point_cloud', self.point_cloud)

        # bboxes and trajectories
        self.bboxes = []
        self.bbox_counter = 0
        self.trajectories = []
        self.past_trajectory_counter = 0
        self.future_trajectory_counter = 0

        # set camera
        center = np.array([0, 0, 0], dtype=np.float32)
        eye = np.array([0, -100, 50], dtype=np.float32)
        up = np.array([0, 0, 1], dtype=np.float32)
        self.viz.setup_camera(60, center, eye, up)
        
    def __add_geometry__(self, name, geometry):
        """
        Adds a geometry to the visualizer.

        Args:
            name: The name of the geometry.
            geometry: The geometry object.

        """
        if name in self.geometries: self.viz.remove_geometry(name)
        else:
            mat = rendering.MaterialRecord()
            mat.shader = "defaultUnlit"
            mat.point_size = self.viz.point_size # default point size
            self.materials[name] = mat
            self.geometries[name] = geometry
        
        # handle dynamic update in point size from O3DVisualizer's settings
        if name =='point_cloud': self.materials[name].point_size = self.viz.point_size
        
        self.viz.add_geometry(name, self.geometries[name], self.materials[name])

    def gui_update(self, data_dict):
        if not hasattr(self, 'viz'): return
        self.app.post_to_main_thread(self.viz, lambda: self._update(data_dict))

    def _update(self, data_dict):
        """
        Updates the visualizer with new data.

        Args:
            data_dict: A dictionary containing the data to be visualized.
        """
        # acquire lock
        self.lock.acquire()

        # update frame name
        try: self.frame_name = os.path.basename(data_dict['current_point_cloud_path'])
        except: self.frame_name = 'last_frame'

        # update pcd points
        if "current_point_cloud_numpy" in data_dict:
            try: self.point_cloud.points = o3d.utility.Vector3dVector(data_dict['current_point_cloud_numpy'][:, 0:3])
            except Exception as e:
                self.log(f"Failed to update point cloud: {e}", Logger.ERROR)
                self.point_cloud.points = o3d.utility.Vector3dVector(self.default_points)
        
        # updates pcd colors
        if 'current_point_cloud_point_colors' in data_dict:
            try:
                self.point_cloud.colors = o3d.utility.Vector3dVector(data_dict['current_point_cloud_point_colors'][:, 0:3])
            except:
                self.log("Failed to update point cloud colors", Logger.ERROR)
                self.point_cloud.paint_uniform_color([1,1,1])

        # update pcd
        self.__add_geometry__('point_cloud', self.point_cloud)
        
        # update other geometries
        self.viz.clear_3d_labels()
        self.__clear_bboxes__()
        self.__clear_trajectories__()
        
        if "current_label_list" in data_dict:
            for lbl in data_dict['current_label_list']:
                self.__add_bbox__(lbl)
                self.__add_cluster__(lbl)
                self.__add_trajectory__(lbl)
        
        # release lock
        self.lock.release()

    def __add_bbox__(self, label_dict: dict):
        """
        Adds a bounding box to the visualizer.

        Args:
            label_dict: A dictionary containing the label information.
        """
        enabled = self.cfg['visualization']['lidar']['draw_bbox_3d']
        enabled_draw_class = self.cfg['visualization']['lidar']['draw_bbox_3d_class']
        have_bbox = 'bbox_3d' in label_dict
        if not enabled or not have_bbox: return
        
        try:
            # bbox params
            bbox_3d_dict = label_dict['bbox_3d']
            xyz_center = bbox_3d_dict['xyz_center']
            xyz_extent = bbox_3d_dict['xyz_extent']
            xyz_euler_angles = bbox_3d_dict['xyz_euler_angles']
            if 'rgb_color' in bbox_3d_dict: color = bbox_3d_dict['rgb_color']
            else: color = np.array([1,1,1], dtype=np.float32)
            if 'predicted' in bbox_3d_dict and not bbox_3d_dict['predicted']: color *= 0.5 # darken the color for ground truth

            # calculating bbox
            rotation_matrix = o3d.geometry.OrientedBoundingBox.get_rotation_matrix_from_xyz(xyz_euler_angles)
            lidar_xyz_bbox = o3d.geometry.OrientedBoundingBox(xyz_center, rotation_matrix, xyz_extent)
            lidar_xyz_bbox.color = color

            self.bbox_counter += 1
            box_name = f'bbox_{str(self.bbox_counter).zfill(4)}'
            self.bboxes.append(box_name)
            self.__add_geometry__(box_name, lidar_xyz_bbox)
            if enabled_draw_class and 'class' in label_dict:
                self.viz.add_3d_label(xyz_center, label_dict['class'].lower().title())
        except Exception as e:
            self.log(f"Failed to add bounding box: {e}", Logger.ERROR)
        
    def __clear_bboxes__(self):
        """
        Clears all the bounding boxes from the visualizer.
        """
        for bbox_name in self.bboxes: self.viz.remove_geometry(bbox_name)
        self.bboxes.clear()

    def __add_cluster__(self, label_dict: dict):
        """
        Adds a cluster to the visualizer.

        Args:
            label_dict: A dictionary containing the label information.
        """
        enabled = self.cfg['visualization']['lidar']['draw_cluster']
        has_cluster = 'lidar_cluster' in label_dict
        if not enabled or not has_cluster: return

        try:
            # cluster params
            lidar_cluster_dict = label_dict['lidar_cluster']
            point_indices = lidar_cluster_dict['point_indices']
            colors = np.asarray(self.point_cloud.colors)
            assert colors.shape[0] == self.point_indices.points.shape[0]
            colors[point_indices] = np.random.rand(3) # ToDO: use consistent color if tracking is enabled
            self.point_cloud.colors = o3d.utility.Vector3dVector(colors)
        except Exception as e:
            self.log(f"Failed to add cluster: {e}", Logger.ERROR)

    def __add_trajectory__(self, label_dict: dict):
        """
        Adds a trajectory to the visualizer.

        Args:
            trajectory: The trajectory to be added.
        """
        enabled = self.cfg['visualization']['lidar']['draw_trajectory']
        has_box = 'bbox_3d' in label_dict # trajectories in lidar are associated to bounding boxes
        if not enabled or not has_box: return

        box = label_dict['bbox_3d']
        if 'rgb_color' in box: color = box['rgb_color']
        else: color = np.array([1,1,1], dtype=np.float32)
        
        if 'past_trajectory' in box: past_trajectory = box['past_trajectory']
        else: past_trajectory = []
        
        if 'future_trajectory' in box: future_trajectory = box['future_trajectory']
        else: future_trajectory = []
        
        try:
            if len(past_trajectory) >= 2:
                lines = []
                for i in range(len(past_trajectory) - 1): lines.append([i, i + 1])
                line_set = o3d.geometry.LineSet()
                line_set.points = o3d.utility.Vector3dVector(past_trajectory)
                line_set.lines = o3d.utility.Vector2iVector(lines)
                line_set.colors = o3d.utility.Vector3dVector([color for _ in range(len(lines))])
                self.past_trajectory_counter += 1
                trajectory_name = f'past_trajectory_{str(self.past_trajectory_counter).zfill(4)}'
                self.trajectories.append(trajectory_name)
                self.__add_geometry__(trajectory_name, line_set)
        except Exception as e:
            self.log(f"Failed to add past trajectory: {e}", Logger.ERROR)
        
        try:
            if len(future_trajectory) >= 2:
                lines = []
                for i in range(len(future_trajectory) - 1): lines.append([i, i + 1])
                line_set = o3d.geometry.LineSet()
                line_set.points = o3d.utility.Vector3dVector(future_trajectory)
                line_set.lines = o3d.utility.Vector2iVector(lines)
                line_set.colors = o3d.utility.Vector3dVector([color for _ in range(len(lines))])
                self.future_trajectory_counter += 1
                trajectory_name = f'future_trajectory_{str(self.future_trajectory_counter).zfill(4)}'
                self.trajectories.append(trajectory_name)
                self.__add_geometry__(trajectory_name, line_set)
        except Exception as e:
            self.log(f"Failed to add future trajectory: {e}", Logger.ERROR)

    def __clear_trajectories__(self):
        """
        Clears all the trajectories from the visualizer.
        """
        for trajectory in self.trajectories: self.viz.remove_geometry(trajectory)
        self.trajectories.clear()

    def save_image(self):
        """
        Saves the current visualization as an image.
        """
        # check if save_image is enabled
        enabled = hasattr(self, 'cfg') and self.cfg['visualization']['lidar']['save_images']
        if not enabled: return

        # create output directory
        pipeline_dir = self.cfg['data']['pipeline_dir']
        data_outputs_dir = self.cfg['data']['outputs_dir']
        if not os.path.isabs(data_outputs_dir): data_outputs_dir = os.path.join(pipeline_dir, data_outputs_dir)
        pcd_imgs_dir = os.path.join(data_outputs_dir, 'pcd_viz')
        os.makedirs(pcd_imgs_dir, exist_ok=True)

        # save image
        file_path = os.path.join(pcd_imgs_dir, self.frame_name)
        self.app.post_to_main_thread(self.viz, lambda: self.viz.export_current_image(file_path))

    def save_view_status(self):
        """
        Saves the view status (parameters of looking camera) of the visualizer.
        """
        # make sure the outputs_dir is created
        data_outputs_dir = self.cfg['data']['outputs_dir']
        if not os.path.isabs(data_outputs_dir): data_outputs_dir = os.path.join(self.cfg['data']['pipeline_dir'], data_outputs_dir)
        save_path = os.path.join(data_outputs_dir, 'view_status.txt')
        with open(save_path, 'w') as file: file.write(str(self.viz.get_view_status()))
    
    def load_view_status(self):
        """
        Loads the view status (parameters of looking camera) of the visualizer from the configuration.
        """
        try:
            data_outputs_dir = self.cfg['data']['outputs_dir']
            if not os.path.isabs(data_outputs_dir): data_outputs_dir = os.path.join(self.cfg['data']['pipeline_dir'], data_outputs_dir)
            load_path = os.path.join(data_outputs_dir, 'view_status.txt')
            with open(load_path, 'r') as file: self.viz.set_view_status(file.read())
        except: pass

    def quit(self):
        """ 
        Quits the visualizer.
        """
        del self.viz