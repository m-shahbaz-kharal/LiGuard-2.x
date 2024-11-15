import pickle
import numpy as np

"""
Author: Muhammad Shahbaz (m.shahbaz.kharal@outlook.com)
Github: github.com/m-shahbaz-kharal

Discrete Histograms of Distances-Per-Point (D_HistDPP)
=======================================================

It is a background filter for roadside LIDAR "structured" point cloud data. The term "structured", here, means that the point cloud can be represented by a fixed size HXV (horizontal resolution x vertical resolution) matrix. For example, if a LiDAR has a 1024 horizontal and 64 vertical resolution, and outputs a point cloud containing 1024x64 points where each point is from a fixed azimuth and elevation ray, then the point cloud is structured. The algorithm is designed to work with such structured point clouds.

The algorithm is summmarized in the following steps:
Since the point cloud is structured, each point index represents a unique ray in the LIDAR's field of view.
1. So, for each ray, a histogram of distance is calculated across all frames, resulting in N histograms, where N is the number of points in the point cloud.
2. The histograms are then normalized by the number of frames.
3. The normalized histograms are then used to filter out the background points as follows:
    a. For each point index in the query point cloud, the distance of the point is calculated.
    b. The distance is then used to find the bin index in the corresponding histogram (indexed by the same point index).
    c. The density of the bin is then compared with a threshold:
        - If the density is less than the threshold, the point is considered as foreground,
        - otherwise background.
"""

def calc_DHistDPP_params(point_cloud_set: list, # a list of point clouds, points in each frame must be equal
             number_of_points_per_frame: int, # number of points in each point cloud
             lidar_range_in_unit_length: float, # maximum range of lidar in lidar unit length
             bins_per_unit_length: int, # number of bins per unit length
):
    number_of_frames = len(point_cloud_set)
    point_cloud_set = np.array(point_cloud_set, dtype=np.float32)
    # calculate euclidean distances of all points from origin
    distances_per_point = np.linalg.norm(point_cloud_set[:, :,:3], ord=2, axis=2).transpose()
    # get the most abundant distances
    bins_per_point = int(lidar_range_in_unit_length * bins_per_unit_length)
    bins = np.linspace(0, lidar_range_in_unit_length, bins_per_point+1)
    histograms_per_point = np.apply_along_axis(lambda x: np.histogram(x, bins_per_point, range=(0, lidar_range_in_unit_length))[0], axis=1, arr=distances_per_point)
    normalized_histogram_per_point = histograms_per_point.astype(np.float32) / number_of_frames

    filter_params = dict(
        bins=bins,
        bins_per_point=bins_per_point,
        normalized_histogram_per_point=normalized_histogram_per_point,
        number_of_points_per_frame=number_of_points_per_frame
    )

    return filter_params
    
def make_DHistDPP_filter(point_cloud: np.ndarray, # Nx4,
           background_density_threshold: float, # if the point falls in a bin with density less than this threshold, it is considered as foreground
           bins: np.ndarray, # bins for histogram
           bins_per_point: int, # number of bins per point
           normalized_histogram_per_point: np.ndarray, # normalized histogram per point
           number_of_points_per_frame: int # number of points in each frame
    ):
    dists = np.linalg.norm(point_cloud[:,:3], ord=2, axis=1)
    bin_indices = np.digitize(dists, bins) - 1
    bin_indices = np.clip(bin_indices, 0, bins_per_point-1)
    mask = normalized_histogram_per_point[np.arange(number_of_points_per_frame), bin_indices] < background_density_threshold
    return mask

def save_DHistDPP_params(filter_params, filename):
    filename = filename.split('.')[0] + '.pkl'
    with open(filename, 'wb') as f: pickle.dump(filter_params, f)
    return filename

def load_DHistDPP_params(filename):
    try:
        filename = filename.split('.')[0] + '.pkl'
        with open(filename, 'rb') as f: return pickle.load(f)
    except:
        return None