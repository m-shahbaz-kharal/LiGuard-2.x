import numpy as np

"""
Author: Muhammad Shahbaz (m.shahbaz.kharal@outlook.com)
Github: github.com/m-shahbaz-kharal

Discrete Histogram of Distances-Per-Point (D_HistDPP) is a background filter for roadside LIDAR (sequential) structured point cloud data, that means, the point cloud is represented by a fixed size HXV (horizontal resolution x vertical resolution) matrix.
"""

def DHistDPP(point_cloud_sequence: list, # a list of sequentially captrued point clouds, points in each frame must be equal
             number_of_points_per_frame: int, # number of points in each point cloud
             lidar_range_in_unit_length: float, # maximum range of lidar in lidar unit length
             bins_per_unit_length: int, # number of bins per unit length
):
    number_of_frames = len(point_cloud_sequence)
    point_cloud_sequence = np.array(point_cloud_sequence, dtype=np.float32)
    # calculate euclidean distances of all points from origin
    distances_per_point = np.linalg.norm(point_cloud_sequence[:, :,:3], ord=2, axis=2).transpose()
    # get the most abundant distances
    bins_per_point = int(lidar_range_in_unit_length * bins_per_unit_length)
    bins = np.linspace(0, lidar_range_in_unit_length, bins_per_point+1)
    histograms_per_point = np.apply_along_axis(lambda x: np.histogram(x, bins_per_point, range=(0, lidar_range_in_unit_length))[0], axis=1, arr=distances_per_point)
    normalized_histogram_per_point = histograms_per_point.astype(np.float32) / number_of_frames
    
    def filter(point_cloud: np.ndarray, # Nx4,
               background_density_threshold: float # if the point falls in a bin with density less than this threshold, it is considered as foreground
    ):
        
        dists = np.linalg.norm(point_cloud[:,:3], ord=2, axis=1)
        bin_indices = np.digitize(dists, bins) - 1
        bin_indices = np.clip(bin_indices, 0, bins_per_point-1)
        mask = normalized_histogram_per_point[np.arange(number_of_points_per_frame), bin_indices] < background_density_threshold
        return mask

    return filter