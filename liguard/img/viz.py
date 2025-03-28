import os
import time

import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import cv2
import numpy as np

from liguard.gui.gui_utils import resolve_for_application_root, resolve_for_default_workspace
from liguard.gui.logger_gui import Logger

import threading

class ImageVisualizer:
    """
    Initializes the ImageVisualizer.
        Args:
            app (gui.Application): The application object.
            logger (Logger): The logger object.
    """

    def __init__(self, app: gui.Application, logger: Logger=None):
        self.app = app
        if logger: self.log = logger.log
        else: self.log = lambda msg,lvl: print(f'[Image Visualizer][{Logger.__level_string__[lvl]}] {msg}')
        
        # create visualizer
        self.viz = o3d.visualization.O3DVisualizer(title="Image Feed", width=int(1440/4), height=int(1080/4))
        self.viz.set_on_close(lambda: False)
        self.viz.show_menu(False)
        self.viz.show_settings = False
        self.viz.show_skybox(False)

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
        self.app.post_to_main_thread(self.viz, clear_and_init_settings_gui)

        # release lock
        self.lock.release()

    def clear(self):
        self.viz.clear_3d_labels()
        black = np.zeros((1080, 1440, 3), dtype=np.uint8)
        image = o3d.geometry.Image(black)
        self.viz.set_background(np.array([0,0,0,1], dtype=np.float32), image)
        
    def __add_geometry__(self, name, geometry):
        """
        Adds a geometry to the visualizer.

        Args:
            name: The name of the geometry.
            geometry: The geometry object.

        """
        self.viz.set_background([1,1,1,1],geometry)

    def gui_update(self, data_dict):
        if not hasattr(self, 'viz'): return
        self.app.post_to_main_thread(self.viz, lambda: self._update(data_dict))
        
    def _update(self, data_dict):
        """
        Updates the visualizer with new data.

        Args:
            data_dict: A dictionary containing the data to be visualized.
        """
        self.lock.acquire()

        # update frame name
        try: self.frame_name = os.path.basename(data_dict['current_image_path'])
        except: self.frame_name = 'last_frame'

        if "current_image_numpy" in data_dict:
            try:
                self.img = o3d.geometry.Image(data_dict['current_image_numpy'])
            except Exception as e:
                self.log("Failed to read image data: " + str(e), Logger.ERROR)
                self.img = o3d.geometry.Image(self.default_image)
            self.__add_geometry__('image', self.img)
        
        # add labels
        has_label_list = "current_label_list" in data_dict
        has_calib_data = "current_calib_data" in data_dict
        
        # return if there are no labels to paint
        if has_label_list:
            # check if calibration data is available
            if has_calib_data: clb = data_dict['current_calib_data']
            else: clb = None

            # paint labels
            for lbl in data_dict['current_label_list']:
                self.__add_bbox__(lbl, clb)
        
        # release lock
        self.lock.release()
        
    def __add_bbox__(self, label_dict, calib_dict):
        """
        Adds a bounding box to the visualizer.

        Args:
            label_dict: A dictionary containing the label information.
            calib_dict: A dictionary containing the calibration information.
        """
        # the image to paint on
        img_np = np.asarray(self.img)

        # project 3d bboxes on image
        enabled_3d_bboxes = self.cfg['visualization']['camera']['draw_bbox_3d']
        has_bbox_and_calib = 'bbox_3d' in label_dict and calib_dict != None
        if enabled_3d_bboxes and has_bbox_and_calib:
            try:
                # bbox parameters
                bbox_3d_dict = label_dict['bbox_3d']
                xyz_center = bbox_3d_dict['xyz_center']
                xyz_extent = bbox_3d_dict['xyz_extent']
                xyz_euler_angles = bbox_3d_dict['xyz_euler_angles']
                if 'rgb_color' in bbox_3d_dict: rgb_color = bbox_3d_dict['rgb_color'] * 255.0
                else: rgb_color = np.array([255, 255, 255], dtype=np.float32)
                if 'predicted' in bbox_3d_dict and not bbox_3d_dict['predicted']: rgb_color *= 0.5 # darken
                
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

                # add text info if available
                if 'text_info' in label_dict and self.cfg['visualization']['camera']['draw_text_info'] and 'text_info_drawn' not in label_dict:
                    text = 'bbox 3d->2d | ' + label_dict['text_info']
                    img_np = self.__highlighted_text__(img_np, text, (pts[0][0], pts[0][1] - 5))
                    label_dict['text_info_drawn'] = True
                
                # add past bbox_3d trajectories if available
                if 'past_trajectory' in bbox_3d_dict and self.cfg['visualization']['camera']['draw_trajectory']:
                    past_trajectory = bbox_3d_dict['past_trajectory']
                    past_trajectory = np.concatenate([past_trajectory, np.ones((past_trajectory.shape[0], 1))], axis=1)
                    past_trajectory = calib_dict['P2'] @ calib_dict['R0_rect'] @ calib_dict['Tr_velo_to_cam'] @ past_trajectory.T
                    past_trajectory = past_trajectory[:, past_trajectory[2] > 0]
                    past_trajectory = past_trajectory[:2] / past_trajectory[2]
                    past_trajectory = past_trajectory.T
                    past_trajectory = past_trajectory.astype(np.int32)
                    
                    for i in range(len(past_trajectory) - 1): cv2.line(img_np,
                                                                    tuple(past_trajectory[i]),
                                                                    tuple(past_trajectory[i + 1]),
                                                                    (bbox_3d_dict['rgb_color'] * 127).tolist(),
                                                                    self.cfg['visualization']['camera']['trajectory_line_width'])
                
                # add future bbox_3d trajectories if available
                if 'future_trajectory' in bbox_3d_dict and self.cfg['visualization']['camera']['draw_trajectory']:
                    future_trajectory = bbox_3d_dict['future_trajectory']
                    future_trajectory = np.concatenate([future_trajectory, np.ones((future_trajectory.shape[0], 1))], axis=1)
                    future_trajectory = calib_dict['P2'] @ calib_dict['R0_rect'] @ calib_dict['Tr_velo_to_cam'] @ future_trajectory.T
                    future_trajectory = future_trajectory[:, future_trajectory[2] > 0]
                    future_trajectory = future_trajectory[:2] / future_trajectory[2]
                    future_trajectory = future_trajectory.T
                    future_trajectory = future_trajectory.astype(np.int32)
                    
                    for i in range(len(future_trajectory) - 1): cv2.line(img_np,
                                                                        tuple(future_trajectory[i]),
                                                                        tuple(future_trajectory[i + 1]),
                                                                        (bbox_3d_dict['rgb_color'] * 255).tolist(),
                                                                        self.cfg['visualization']['camera']['trajectory_line_width'])
            except Exception as e:
                self.log("Failed to draw 3D bbox: " + str(e), Logger.ERROR)

        # draw 2d bbox
        enabled_2d_bboxes = self.cfg['visualization']['camera']['draw_bbox_2d']
        has_2d_box = 'bbox_2d' in label_dict
        if enabled_2d_bboxes and has_2d_box \
            and not ('visualize' in label_dict['bbox_2d'] and not label_dict['bbox_2d']['visualize']):
            try:
                # bbox parameters
                bbox_2d_dict = label_dict['bbox_2d']
                xy_center = bbox_2d_dict['xy_center']
                xy_extent = bbox_2d_dict['xy_extent']
                rgb_color = bbox_2d_dict['rgb_color'] * 255.0
                if not bbox_2d_dict['predicted']: rgb_color *= 0.5 # darken

                rgb_color = rgb_color.tolist()

                # draw box
                start_point = (xy_center - xy_extent / 2.0).astype(int)
                end_point = (xy_center + xy_extent / 2.0).astype(int)
                cv2.rectangle(img_np, start_point, end_point, rgb_color, self.cfg['visualization']['camera']['bbox_line_width'])

                # add text info
                if 'text_info' in label_dict and self.cfg['visualization']['camera']['draw_text_info'] and 'text_info_drawn' not in label_dict:
                    text = 'bbox 2d | ' + label_dict['text_info']
                    img_np = self.__highlighted_text__(img_np, text, (start_point[0], start_point[1] - 5))
                    label_dict['text_info_drawn'] = True
            except Exception as e:
                self.log("Failed to draw 2D bbox: " + str(e), Logger.ERROR)

        # draw extras if available
        enabled_extras = self.cfg['visualization']['camera']['draw_extras']
        has_extras = 'extras' in label_dict and 'img_visualizer' in label_dict['extras']
        if enabled_extras and has_extras:
            try:
                for algo_name, algo_extras in label_dict['extras']['img_visualizer'].items():
                    for name, extra in algo_extras.items():
                        getattr(cv2, extra['cv2_attr'])(img_np, **extra['params'])
                        if 'text_info' in label_dict and self.cfg['visualization']['camera']['draw_text_info'] and 'text_info_drawn' not in label_dict:
                            text = label_dict['text_info']
                            img_np = self.__highlighted_text__(img_np, text, (extra['params']['center'][0], extra['params']['center'][1] - 5))
                            label_dict['text_info_drawn'] = True
            except Exception as e:
                self.log("Failed to draw extras: " + str(e), Logger.ERROR)
                        
        # add to visualizer
        self.img = o3d.geometry.Image(img_np)
        self.__add_geometry__('image', self.img)

        # remove text_info_drawn flag
        label_dict.pop('text_info_drawn', None)

    def __highlighted_text__(self, image, text, position):
        # keep copy
        highlight_img = image.copy()
        
        # Define text properties
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        font_thickness = 1
        text_color = (255, 255, 255)  # White

        # Define highlight properties
        highlight_color = (0, 0, 255)  # Red

        # Get the size of the text
        (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, font_thickness)

        # Define the position of the text
        text_x, text_y = position

        # Draw the filled rectangle (highlight)
        rect_start = (text_x - 10, text_y + 10)
        rect_end = (text_x + text_width + 10, text_y - text_height - 10)
        cv2.rectangle(image, rect_start, rect_end, highlight_color, -1)

        # Blend the highlight image with the original image using alpha
        cv2.addWeighted(highlight_img, 0.75, image, 1 - 0.75, 0, image)

        # Draw the text
        cv2.putText(image, text, (text_x, text_y), font, font_scale, text_color, font_thickness, cv2.LINE_AA)

        return image

    def save_image(self):
        """
        Saves the current visualization as an image.
        """
        # check if save_image is enabled
        enabled = hasattr(self, 'cfg') and self.cfg['visualization']['camera']['save_images']
        if not enabled: return

        # create output directory
        pipeline_dir = self.cfg['data']['pipeline_dir']
        data_outputs_dir = self.cfg['data']['outputs_dir']
        if not os.path.isabs(data_outputs_dir): data_outputs_dir = os.path.join(pipeline_dir, data_outputs_dir)
        img_imgs_dir = os.path.join(data_outputs_dir, 'img_viz')
        os.makedirs(img_imgs_dir, exist_ok=True)

        img = np.asarray(self.img)
        image_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        
        path = os.path.join(self.img_imgs_dir, self.frame_name)
        cv2.imwrite(path, image_bgr)
        
    def quit(self):
        """
        Destroys the visualizer window.
        """
        del self.viz