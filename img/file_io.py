import cv2
from easydict import EasyDict
import numpy as np
import os
import glob
import threading, time

class FileIO:
    def __init__(self, cfg: EasyDict):
        self.img_dir = os.path.join(cfg.data.path, cfg.data.camera_subdir)
        self.img_type = cfg.data.camera.img_type
        self.img_count = cfg.data.size
        files = glob.glob(os.path.join(self.img_dir, '*' + self.img_type))
        file_basenames = [os.path.splitext(os.path.basename(file))[0] for file in files]
        file_basenames.sort(key=lambda file_name: int(''.join(filter(str.isdigit, file_name))))
        self.files_basenames = file_basenames[:self.img_count]
        self.reader = self.__read_img__
        
        self.data_lock = threading.Lock()
        self.data = []
        self.stop = threading.Event()
        threading.Thread(target=self.__async_read_fn__).start()
    
    def __read_img__(self, file_abs_path: str):
        img_bgr = cv2.imread(file_abs_path)
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        return img_rgb
        
    def get_abs_path(self, idx: int): return os.path.join(self.img_dir, self.files_basenames[idx] + self.img_type)
        
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