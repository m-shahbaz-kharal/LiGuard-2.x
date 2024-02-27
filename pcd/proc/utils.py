import open3d as o3d
import numpy as np
import cv2

def create_pcd(points: np.ndarray) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, 0:3])
    return pcd

def update_pcd(pcd: o3d.geometry.PointCloud, points: np.ndarray) -> o3d.geometry.PointCloud:
    pcd.points = o3d.utility.Vector3dVector(points[:, 0:3])
    return pcd

def project_img_to_pcd(pcd: o3d.geometry.PointCloud, img: np.ndarray, camera_matrix: np.ndarray, distortion_coeffs: np.ndarray, T_lidar_camera: np.ndarray):
    # undistort image
    h, w = img.shape[:2]
    camera_matrix = np.array(camera_matrix).reshape(3, 3)
    distortion_coeffs = np.array(distortion_coeffs)
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coeffs, (w, h), 1, (w, h))
    img_undistorted = cv2.undistort(img, camera_matrix, distortion_coeffs, None, newcameramtx)
    # project image to point cloud using T_lidar_camera which is 4x4 transformation matrix from camera to lidar
    T_lidar_camera = np.array(T_lidar_camera).reshape(4, 4)
    T_camera_lidar = np.linalg.inv(T_lidar_camera)
    pcd_np = np.asarray(pcd.points)
    colors = np.ones((pcd_np.shape[0], 3))
    for i in range(pcd_np.shape[0]):
        camera_pt = T_camera_lidar @ np.append(pcd_np[i], 1)
        camera_pt = camera_pt[0:3] / camera_pt[3] # normalize
        camera_pt = camera_matrix @ camera_pt
        camera_pt = camera_pt[0:2] / camera_pt[2]
        camera_pt = np.round(camera_pt).astype(int)
        if 0 <= camera_pt[0] < w and 0 <= camera_pt[1] < h: colors[i] = img_undistorted[camera_pt[1], camera_pt[0]] / 255.0
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd


