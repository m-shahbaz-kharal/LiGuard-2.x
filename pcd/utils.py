import open3d as o3d
import numpy as np

def create_pcd(points: np.ndarray) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, 0:3])
    return pcd

def update_pcd(pcd: o3d.geometry.PointCloud, points: np.ndarray) -> o3d.geometry.PointCloud:
    pcd.points = o3d.utility.Vector3dVector(points[:, 0:3])
    return pcd