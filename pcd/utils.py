import open3d as o3d
import numpy as np

def create_pcd(points: np.ndarray) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, 0:3])
    return pcd

def update_pcd(pcd: o3d.geometry.PointCloud, points: np.ndarray) -> o3d.geometry.PointCloud:
    pcd.points = o3d.utility.Vector3dVector(points[:, 0:3])
    return pcd

def fix_number_of_points_in_pcd(point_cloud: np.ndarray, number_of_points: int) -> np.ndarray:
    # pad if less points
    if point_cloud.shape[0] < number_of_points: point_cloud = np.pad(point_cloud, ((0, number_of_points-point_cloud.shape[0]), (0,0)), mode='constant', constant_values=0)
    # crop if more points
    elif point_cloud.shape[0] > number_of_points: point_cloud = point_cloud[:number_of_points]
    return point_cloud