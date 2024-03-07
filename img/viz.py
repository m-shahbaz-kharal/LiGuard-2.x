import open3d as o3d
import cv2

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
        else: self.geometries[name] = geometry
        self.viz.add_geometry(geometry, reset_bounding_box=reset_bounding_box)
        
    def __update_geometry__(self, name, geometry):
        if name in self.geometries: self.viz.update_geometry(geometry)
        
    def update(self, data):
        if "current_image_numpy" not in data: return
        self.img = o3d.geometry.Image(data.current_image_numpy)
        self.__add_geometry__('image', self.img, False)
        
        if "current_label_list" not in data: return
        for lbl in data.current_label_list: self.__add_bbox__(lbl)
        
    def __add_bbox__(self, label_dict: dict):
        if 'camera_bbox' not in label_dict: return
        # bbox parameters
        camera_bbox_dict = label_dict['camera_bbox']
        lidar_xyz_center = camera_bbox_dict['lidar_xyz_center']
        lidar_xyz_extent = camera_bbox_dict['lidar_xyz_extent']
        lidar_xyz_euler_angles = camera_bbox_dict['lidar_xyz_euler_angles']
        color = camera_bbox_dict['rgb_bbox_color']
        
        # calib parameters
        Tr_velo_to_cam = label_dict['calib']['Tr_velo_to_cam']
        if Tr_velo_to_cam.shape[0] == 3: Tr_velo_to_cam = np.vstack((Tr_velo_to_cam, np.array([0,0,0,1])))
        P2 = label_dict['calib']['P2']
        if P2.shape[1] == 3: P2 = np.hstack((P2, np.array([[0], [0], [0]])))
        if 'R0_rect' in label_dict['calib']:
            R0_rect = label_dict['calib']['R0_rect']
            if R0_rect.shape == (3, 3):
                R0_rect = np.pad(R0_rect, ((0, 1), (0, 1)), mode='constant', constant_values=0)
                R0_rect[3, 3] = 1
            label_dict['calib']['R0_rect'] = R0_rect
            
        # transforms
        rotation_matrix = o3d.geometry.OrientedBoundingBox.get_rotation_matrix_from_xyz(lidar_xyz_euler_angles)
        translation_matrix = lidar_xyz_center.reshape([-1,1])
        Rt = np.concatenate([rotation_matrix,translation_matrix], axis=-1)
        Rt_4x4 = np.concatenate([Rt, np.array([0,0,0,1]).reshape([1,-1])], axis=0)
        
        # 3D bbox
        pt_x = lidar_xyz_extent[0]/2
        pt_y = lidar_xyz_extent[1]/2
        pt_z = lidar_xyz_extent[2]/2
        # define 3D bounding box
        bbox_in_local_coords = np.array([
            pt_x, pt_y, -pt_z, 1,   pt_x, -pt_y, -pt_z, 1,  #front-left-bottom, front-right-bottom
            pt_x, -pt_y, pt_z, 1,   pt_x, pt_y, pt_z, 1,    #front-right-top,   front-left-top

            -pt_x, pt_y, -pt_z, 1,   -pt_x, -pt_y, -pt_z, 1,#rear-left-bottom, rear-right-bottom
            -pt_x, -pt_y, pt_z, 1,   -pt_x, pt_y, pt_z, 1,  #rear-right-top,   rear-left-top
            
            #middle plane
            #0, y, -z, 1,   0, -y, -z, 1,  #rear-left-bottom, rear-right-bottom
            #0, -y, z, 1,   0, y, z, 1,    #rear-right-top,   rear-left-top
        ]).reshape((-1,4))
        
        # add rotation and translation
        bbox_in_world_coords = Rt_4x4 @ bbox_in_local_coords.T
        # project to camera coordinates
        bbox_pts_in_camera_coords = Tr_velo_to_cam @ bbox_in_world_coords
        # rect matrix shall be applied here, for kitti
        if 'R0_rect' in label_dict['calib']: bbox_pts_in_camera_coords = R0_rect @ bbox_pts_in_camera_coords
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
        color = color.tolist()
        cv2.line(img_np,tuple(pts[0]),tuple(pts[1]),color,self.cfg.visualization.camera.bbox_line_width)
        cv2.line(img_np,tuple(pts[1]),tuple(pts[2]),color,self.cfg.visualization.camera.bbox_line_width)
        cv2.line(img_np,tuple(pts[2]),tuple(pts[3]),color,self.cfg.visualization.camera.bbox_line_width)
        cv2.line(img_np,tuple(pts[3]),tuple(pts[0]),color,self.cfg.visualization.camera.bbox_line_width)
        cv2.fillPoly(img_np, [pts[0:4].reshape((-1,1,2))],color)
        cv2.line(img_np,tuple(pts[4]),tuple(pts[5]),color,self.cfg.visualization.camera.bbox_line_width)
        cv2.line(img_np,tuple(pts[5]),tuple(pts[6]),color,self.cfg.visualization.camera.bbox_line_width)
        cv2.line(img_np,tuple(pts[6]),tuple(pts[7]),color,self.cfg.visualization.camera.bbox_line_width)
        cv2.line(img_np,tuple(pts[7]),tuple(pts[4]),color,self.cfg.visualization.camera.bbox_line_width)
        cv2.line(img_np,tuple(pts[0]),tuple(pts[4]),color,self.cfg.visualization.camera.bbox_line_width)
        cv2.line(img_np,tuple(pts[1]),tuple(pts[5]),color,self.cfg.visualization.camera.bbox_line_width)
        cv2.line(img_np,tuple(pts[2]),tuple(pts[6]),color,self.cfg.visualization.camera.bbox_line_width)
        cv2.line(img_np,tuple(pts[3]),tuple(pts[7]),color,self.cfg.visualization.camera.bbox_line_width)
        
        # add to visualizer
        self.img = o3d.geometry.Image(img_np)
        self.__add_geometry__('image', self.img, False)
        
    def redraw(self):
        self.viz.poll_events()
        self.viz.update_renderer()
        
    def quit(self):
        self.viz.destroy_window()