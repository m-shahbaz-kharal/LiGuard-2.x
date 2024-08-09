import open3d as o3d
import cv2
import numpy as np

from gui.logger_gui import Logger

import pygetwindow as gw

class ImageVisualizer:
    """
    A class for visualizing images and bounding boxes using Open3D.

    Args:
        app: The application object.
        cfg (dict): A dictionary containing configuration parameters.

    Attributes:
        app: The application object.
        viz: The Open3D visualizer object.
        img: The image to be visualized.
        geometries (dict): A dictionary to store the geometries added to the visualizer.

    Methods:
        reset: Resets the visualizer and clears all geometries.
        update: Updates the visualizer with new data.
        redraw: Redraws the visualizer.
        quit: Destroys the visualizer window.
    """

    def __init__(self, app, cfg: dict):
        self.app = app
        # create visualizer
        self.viz = o3d.visualization.Visualizer()
        self.viz.create_window("Image Feed", width=int(1440/4), height=int(1080/4), left=480 - int(1440/4), top=30)
        # init
        self.reset(cfg, True)

    def __is_focused__(self):
        """
        Checks if the visualizer is focused.

        Returns:
            bool: True if the visualizer is focused, False otherwise.
        """
        window = gw.getWindowsWithTitle('Image Feed')
        if window: return window[0].isActive
        return False
        
    def reset(self, cfg, reset_bounding_box=False):
        """
        Resets the visualizer and clears all geometries.

        Args:
            cfg: The configuration parameters.
            reset_bounding_box (bool): Whether to reset the bounding box or not.
        """
        self.cfg = cfg
        # clear geometries
        self.geometries = dict()
        self.viz.clear_geometries()
        # add default geometries
        self.__add_default_geometries__(reset_bounding_box)
        
    def __add_default_geometries__(self, reset_bounding_box):
        """
        Adds default geometries to the visualizer.

        Args:
            reset_bounding_box (bool): Whether to reset the bounding box or not.
        """
        # global image
        self.img = o3d.geometry.Image(np.zeros((1080, 1440, 3), dtype=np.uint8))
        self.__add_geometry__('image', self.img, reset_bounding_box)
        
    def __add_geometry__(self, name, geometry, reset_bounding_box):
        """
        Adds a geometry to the visualizer.

        Args:
            name (str): The name of the geometry.
            geometry: The geometry object.
            reset_bounding_box (bool): Whether to reset the bounding box or not.
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
            name (str): The name of the geometry.
            geometry: The updated geometry object.
        """
        if name in self.geometries:
            self.viz.update_geometry(geometry)
        
    def update(self, data_dict):
        """
        Updates the visualizer with new data.

        Args:
            data_dict (dict): A dictionary containing the data to be visualized.
        """
        if 'logger' in data_dict: logger:Logger = data_dict['logger']
        else: print('[CRITICAL ERROR]: No logger object in data_dict. It is abnormal behavior as logger object is created by default. Please check if some script is removing the logger key in data_dict.'); return

        if "current_image_numpy" not in data_dict:
            logger.log(f'[img->viz.py->ImageVisualizer->update]: current_image_numpy not found in data_dict', Logger.DEBUG)
            return
        self.img = o3d.geometry.Image(data_dict['current_image_numpy'])
        self.__add_geometry__('image', self.img, False)
        
        if "current_label_list" not in data_dict: return
        
        if "current_calib_data" in data_dict: clb = data_dict['current_calib_data']
        else: clb = None

        for lbl in data_dict['current_label_list']:
            self.__add_bbox__(lbl, clb)
        
    def __add_bbox__(self, label_dict, calib_dict):
        """
        Adds a bounding box to the visualizer.

        Args:
            label_dict (dict): A dictionary containing the label information.
            calib_dict (dict): A dictionary containing the calibration information.
        """
        if 'bbox_3d' in label_dict and calib_dict != None:
            # bbox parameters
            bbox_3d_dict = label_dict['bbox_3d']
            xyz_center = bbox_3d_dict['xyz_center']
            xyz_extent = bbox_3d_dict['xyz_extent']
            xyz_euler_angles = bbox_3d_dict['xyz_euler_angles']
            rgb_color = bbox_3d_dict['rgb_color'] * 255.0
            if not bbox_3d_dict['predicted']: rgb_color *= 0.5 # darken
            
            # calib parameters
            P2 = calib_dict['P2']
            if 'R0_rect' in calib_dict: R0_rect = calib_dict['R0_rect']
            Tr_velo_to_cam = calib_dict['Tr_velo_to_cam']
            
            # transforms
            rotation_matrix = o3d.geometry.OrientedBoundingBox.get_rotation_matrix_from_xyz(xyz_euler_angles)
            translation_matrix = xyz_center.reshape([-1,1])
            Rt = np.concatenate([rotation_matrix,translation_matrix], axis=-1)
            Rt_4x4 = np.concatenate([Rt, np.array([0,0,0,1], dtype=np.float32).reshape([1,-1])], axis=0)
            
            # 3D bbox
            pt_x = xyz_extent[0]/2
            pt_y = xyz_extent[1]/2
            pt_z = xyz_extent[2]/2
            # define 3D bounding box
            bbox_in_local_coords = np.array([
                pt_x, pt_y, -pt_z, 1,   pt_x, -pt_y, -pt_z, 1,  #front-left-bottom, front-right-bottom
                pt_x, -pt_y, pt_z, 1,   pt_x, pt_y, pt_z, 1,    #front-right-top,   front-left-top

                -pt_x, pt_y, -pt_z, 1,   -pt_x, -pt_y, -pt_z, 1,#rear-left-bottom, rear-right-bottom
                -pt_x, -pt_y, pt_z, 1,   -pt_x, pt_y, pt_z, 1,  #rear-right-top,   rear-left-top
                
                #middle plane
                #0, y, -z, 1,   0, -y, -z, 1,  #rear-left-bottom, rear-right-bottom
                #0, -y, z, 1,   0, y, z, 1,    #rear-right-top,   rear-left-top
            ], dtype=np.float32).reshape((-1,4))
            
            # add rotation and translation
            bbox_in_world_coords = Rt_4x4 @ bbox_in_local_coords.T
            # project to camera coordinates
            bbox_pts_in_camera_coords = Tr_velo_to_cam @ bbox_in_world_coords
            # rect matrix shall be applied here, for kitti
            if 'R0_rect' in calib_dict: bbox_pts_in_camera_coords = R0_rect @ bbox_pts_in_camera_coords
            # if any point is behind camera, return
            if np.any(bbox_pts_in_camera_coords[2] < 0): return None
            # project to image pixel coordinates
            bbox_pts_in_image_pixel_coords = P2 @ bbox_pts_in_camera_coords
            # normalize
            points_in_image = bbox_pts_in_image_pixel_coords[0:2,:] / bbox_pts_in_image_pixel_coords[2:,:]
            
            # the image to draw on
            img_np = np.asarray(self.img)
            
            # draw
            points_in_image = points_in_image.T
            pts = np.int32(points_in_image)
            rgb_color = rgb_color.tolist()
            cv2.line(img_np,tuple(pts[0]),tuple(pts[1]),rgb_color,self.cfg['visualization']['camera']['bbox_line_width'])
            cv2.line(img_np,tuple(pts[1]),tuple(pts[2]),rgb_color,self.cfg['visualization']['camera']['bbox_line_width'])
            cv2.line(img_np,tuple(pts[2]),tuple(pts[3]),rgb_color,self.cfg['visualization']['camera']['bbox_line_width'])
            cv2.line(img_np,tuple(pts[3]),tuple(pts[0]),rgb_color,self.cfg['visualization']['camera']['bbox_line_width'])
            # cv2.fillPoly(img_np, [pts[0:4].reshape((-1,1,2))],color)
            cv2.line(img_np,tuple(pts[4]),tuple(pts[5]),rgb_color,self.cfg['visualization']['camera']['bbox_line_width'])
            cv2.line(img_np,tuple(pts[5]),tuple(pts[6]),rgb_color,self.cfg['visualization']['camera']['bbox_line_width'])
            cv2.line(img_np,tuple(pts[6]),tuple(pts[7]),rgb_color,self.cfg['visualization']['camera']['bbox_line_width'])
            cv2.line(img_np,tuple(pts[7]),tuple(pts[4]),rgb_color,self.cfg['visualization']['camera']['bbox_line_width'])
            cv2.line(img_np,tuple(pts[0]),tuple(pts[4]),rgb_color,self.cfg['visualization']['camera']['bbox_line_width'])
            cv2.line(img_np,tuple(pts[1]),tuple(pts[5]),rgb_color,self.cfg['visualization']['camera']['bbox_line_width'])
            cv2.line(img_np,tuple(pts[2]),tuple(pts[6]),rgb_color,self.cfg['visualization']['camera']['bbox_line_width'])
            cv2.line(img_np,tuple(pts[3]),tuple(pts[7]),rgb_color,self.cfg['visualization']['camera']['bbox_line_width'])
            
            # add to visualizer
            self.img = o3d.geometry.Image(img_np)
            self.__add_geometry__('image', self.img, False)
        
    def redraw(self):
        """
        Redraws the visualizer.
        """
        self.viz.poll_events()
        self.viz.update_renderer()
        
    def quit(self):
        """
        Destroys the visualizer window.
        """
        self.viz.destroy_window()