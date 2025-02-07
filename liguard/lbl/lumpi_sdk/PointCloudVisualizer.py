import open3d as o3d
from time import sleep
class PointCloudVisualizer:
    def __init__(self):
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.lines = [
            [0, 1], [1, 2], [2, 3], [0, 3],
            [4, 5], [5, 6], [6, 7], [4, 7],
            [0, 4], [1, 5], [2, 6], [3, 7]
        ]
        self.colors = [[0, 0, 1] for _ in range(len(self.lines))]

    def init_camera(self, points, duration):
        self.vis.create_window()
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        self.vis.add_geometry(pcd)
        for i in range(duration):
            self.vis.poll_events()
            self.vis.get_render_option().point_size = 3
            self.vis.update_renderer()
            sleep(0.05)
        self.camera_parameters = self.vis.get_view_control().convert_to_pinhole_camera_parameters()
    def update_view(self):
        self.vis.get_view_control().convert_from_pinhole_camera_parameters(self.camera_parameters, True)
        self.vis.poll_events()
        self.vis.update_renderer()
        self.camera_parameters=self.vis.get_view_control().convert_to_pinhole_camera_parameters()
    def add_bounding_box(self, bounding_box):
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(bounding_box.corners.transpose())
        line_set.lines = o3d.utility.Vector2iVector(self.lines)
        line_set.colors = o3d.utility.Vector3dVector(self.colors)
        self.vis.add_geometry(line_set)
    def add_colored_cloud(self, points, point_colors):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(point_colors)     
        self.vis.add_geometry(pcd)