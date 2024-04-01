import os
import glob
import time
import threading

supported_calib_types = [clb_handler.split('_')[1].replace('.py','') for clb_handler in os.listdir('calib') if 'handler' in clb_handler]

class FileIO:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.clb_dir = os.path.join(cfg['data']['path'], cfg['data']['calib_subdir'])
        self.clb_type = cfg['data']['calib']['clb_type']
        self.clb_count = cfg['data']['size']
        
        if self.clb_type not in supported_calib_types: raise NotImplementedError("Calib type not supported. Supported file types: " + ', '.join(supported_calib_types) + ".")
        h = __import__('calib.handler_'+self.clb_type, fromlist=['calib_file_extension', 'Handler'])
        self.clb_ext, self.reader = h.calib_file_extension, h.Handler
        
        files = glob.glob(os.path.join(self.clb_dir, '*' + self.clb_ext))
        file_basenames = [os.path.splitext(os.path.basename(file))[0] for file in files]
        file_basenames.sort(key=lambda file_name: int(''.join(filter(str.isdigit, file_name))))
        self.files_basenames = file_basenames[:self.clb_count]
        
        self.data_lock = threading.Lock()
        self.data = []
        self.stop = threading.Event()
        threading.Thread(target=self.__async_read_fn__).start()
        
    def get_abs_path(self, idx: int):
        clb_path = os.path.join(self.clb_dir, self.files_basenames[idx] + self.clb_ext)
        return clb_path
        
    def __async_read_fn__(self):
        for idx in range(len(self.files_basenames)):
            if self.stop.is_set(): break
            clb_abs_path = self.get_abs_path(idx)
            calib = self.reader(clb_abs_path)
            with self.data_lock: self.data.append((clb_abs_path, calib))
            time.sleep(self.cfg['threads']['io_sleep'])
        
    def __len__(self): return len(self.files_basenames)
    
    def __getitem__(self, idx):
        try:
            with self.data_lock: return self.data[idx]
        except:
            clb_abs_path = self.get_abs_path(idx)
            calib = self.reader(clb_abs_path)
            return (clb_abs_path, calib)
        
    def close(self):
        self.stop.set()