import open3d as o3d
import numpy as np

def create_pcd(points: np.ndarray) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, 0:3])
    return pcd

def colorize_pcd(pcd: o3d.geometry.PointCloud, colors: np.ndarray) -> o3d.geometry.PointCloud:
    pcd.colors = o3d.utility.Vector3dVector(colors[:, 0:3])
    return pcd

def colorize_pcd(pcd: o3d.geometry.PointCloud, color: np.ndarray) -> o3d.geometry.PointCloud:
    pcd.paint_uniform_color(color)
    return pcd