import numpy as np

def D_HistDPP(point_cloud_sequence: list, # a list of sequentially captrued point clouds
              number_of_points_per_frame: int, # number of points in each point cloud
              lidar_range_in_unit_length: float, # maximum range of lidar in lidar unit length
              bins_per_unit_length: int, # number of bins per unit length
              background_density_threshold: int, # threshold that tells if a bin is dense enough to be considered as background
):
    number_of_frames = len(point_cloud_sequence)
    buffer = []
    for pcd in point_cloud_sequence:
        # pad if less points
        if pcd.shape[0] < number_of_points_per_frame: pcd = np.pad(pcd, ((0, number_of_points_per_frame-pcd.shape[0]), (0,0)), mode='constant', constant_values=0)
        # crop if more points
        elif pcd.shape[0] > number_of_points_per_frame: pcd = pcd[:number_of_points_per_frame]
        buffer.append(pcd)
    buffer = np.array(buffer, dtype=np.float32)
    # calculate euclidean distances of all points from origin
    distances_per_point = np.zeros((number_of_frames, number_of_points_per_frame), dtype=np.float32)
    for i, frame in enumerate(buffer):
        dist = np.linalg.norm(frame[:,:3], ord=2, axis=1)
        distances_per_point[i] = dist
    # get the most abundant distances
    distances_per_point = distances_per_point.transpose()
    bins_per_point = int(lidar_range_in_unit_length * bins_per_unit_length)
    histogram_per_point = []
    for i, distance_set_per_point in enumerate(distances_per_point):
        hist, bins = np.histogram(distance_set_per_point, bins_per_point, range=(0, lidar_range_in_unit_length))
        histogram_per_point.append(hist)
    histogram_per_point = np.array(histogram_per_point, dtype=np.float32)
    normalized_histogram_per_point = histogram_per_point / number_of_frames
    
    def filter(point_cloud: np.ndarray):
        # pad if less points
        if point_cloud.shape[0] < number_of_points_per_frame: point_cloud = np.pad(point_cloud, ((0, number_of_points_per_frame-point_cloud.shape[0]), (0,0)), mode='constant', constant_values=0)
        # crop if more points
        elif point_cloud.shape[0] > number_of_points_per_frame: point_cloud = point_cloud[:number_of_points_per_frame]
        dists = np.linalg.norm(point_cloud[:,:3], ord=2, axis=1)
        bin_indices = np.digitize(dists, bins)-1
        bin_indices = np.clip(bin_indices, 0, bins_per_point-1)
        mask = normalized_histogram_per_point[np.arange(number_of_points_per_frame), bin_indices] < background_density_threshold
        return mask

    return filter