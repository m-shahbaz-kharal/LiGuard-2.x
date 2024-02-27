import open3d as o3d
from easydict import EasyDict
import numpy as np
import os
import glob
import threading, time

supported_file_types = ['.bin', '.npy', '.ply', '.pcd']

class FileIO:
    def __init__(self, cfg: EasyDict):
        self.pcd_dir = os.path.join(cfg.data.path, 'points')
        self.pcd_type = cfg.data.pcd_type
        self.pcd_count = cfg.data.size
        files = glob.glob(os.path.join(self.pcd_dir, '*' + self.pcd_type))
        file_basenames = [os.path.splitext(os.path.basename(file))[0] for file in files]
        file_basenames.sort(key=lambda file_name: int(''.join(filter(str.isdigit, file_name))))
        self.files_basenames = file_basenames[:self.pcd_count]
        
        if self.pcd_type not in supported_file_types: raise NotImplementedError("File type not supported. Supported file types: " + ', '.join(supported_file_types) + ".")
        self.reader = getattr(self, '__read_' + self.pcd_type[1:] + '__')
        
        self.data_lock = threading.Lock()
        self.data = []
        self.stop = threading.Event()
        threading.Thread(target=self.__async_read_fn__).start()
    
    def __read_bin__(self, file_abs_path: str):
        return np.fromfile(file_abs_path, dtype=np.float32).reshape(-1,4)

    def __read_npy__(self, file_abs_path: str):
        return np.load(file_abs_path)

    def __read_ply__(self, file_abs_path: str):
        points = np.asarray(o3d.io.read_point_cloud(file_abs_path).points, dtype=np.float32)
        points = np.hstack((points, np.ones((points.shape[0], 1), dtype=np.float32)))
        return points
    
    def __read_pcd__(self, file_abs_path: str):
        points = np.asarray(o3d.io.read_point_cloud(file_abs_path).points, dtype=np.float32)
        points = np.hstack((points, np.ones((points.shape[0], 1), dtype=np.float32)))
        return points
        
    def get_abs_path(self, idx: int):
        return os.path.join(self.pcd_dir, self.files_basenames[idx] + self.pcd_type)
        
    def __async_read_fn__(self):
        for idx in range(len(self.files_basenames)):
            if self.stop.is_set(): break
            file_abs_path = self.get_abs_path(idx)
            pcd_np = self.reader(file_abs_path)
            with self.data_lock: self.data.append(pcd_np)
        
    def __len__(self): return len(self.files_basenames)
    
    def __getitem__(self, idx):
        try:
            with self.data_lock: return self.data[idx]
        except:
            file_abs_path = self.get_abs_path(idx)
            return self.reader(file_abs_path)
        
    def close(self):
        self.stop.set()