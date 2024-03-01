import open3d as o3d
import cv2

from easydict import EasyDict
import numpy as np

from calib.utils import nx3_to_nx4

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
        else: self.geometries[name] = geometry
        self.viz.add_geometry(geometry, reset_bounding_box=reset_bounding_box)
        
    def __update_geometry__(self, name, geometry):
        if name in self.geometries: self.viz.update_geometry(geometry)
        
    def update(self, data):
        self.img = o3d.geometry.Image(data.current_image_numpy)
        self.__add_geometry__('image', self.img, False)
        for lbl in data.current_label_list: self.__add_bbox__(lbl)
        
    def __add_bbox__(self, label_dict: dict):
        camera_bbox_dict = label_dict['camera_bbox']
        center = camera_bbox_dict['xyz_center']
        w, l, h = camera_bbox_dict['wlh_extent']
        rotation_matrix = camera_bbox_dict['around_y_rotation_matrix']
        color = camera_bbox_dict['rgb_bbox_color']
        
        # define 3D bounding box
        x_corners = [l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2]
        y_corners = [0, 0, 0, 0, -h, -h, -h, -h]
        z_corners = [w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2]
        # rotate and translate 3D bounding box
        corners_3d = np.dot(rotation_matrix, np.vstack([x_corners, y_corners, z_corners]))
        # moving the center to object center
        corners_3d[0, :] = corners_3d[0, :] + center[0]
        corners_3d[1, :] = corners_3d[1, :] + center[1]
        corners_3d[2, :] = corners_3d[2, :] + center[2]
        # if any corner is behind camera, return
        if np.any(corners_3d[2, :] < 0.1): return
        # project 3D bounding box to 2D image
        corners_2d = label_dict['calib']['P2'].reshape(3, 4) @ nx3_to_nx4(corners_3d.T).T
        corners_2d = corners_2d.T # 3x8 -> 8x3
        corners_2d = corners_2d[:, 0:2] / corners_2d[:, 2:3]
        corners_2d = corners_2d[:, 0:2].astype(np.int32)
        # draw 2D bounding box
        img_np = np.asarray(self.img)
        for k in range(0, 4):
            i, j = k, (k + 1) % 4
            cv2.line(img_np, (corners_2d[i, 0], corners_2d[i, 1]), (corners_2d[j, 0], corners_2d[j, 1]), color, 2)
            i, j = k + 4, (k + 1) % 4 + 4
            cv2.line(img_np, (corners_2d[i, 0], corners_2d[i, 1]), (corners_2d[j, 0], corners_2d[j, 1]), color, 2)
            i, j = k, k + 4
            cv2.line(img_np, (corners_2d[i, 0], corners_2d[i, 1]), (corners_2d[j, 0], corners_2d[j, 1]), color, 2)
        
        self.img = o3d.geometry.Image(img_np)
        self.__add_geometry__('image', self.img, False)
        
    def redraw(self):
        self.viz.poll_events()
        self.viz.update_renderer()
        
    def quit(self):
        self.viz.destroy_window()