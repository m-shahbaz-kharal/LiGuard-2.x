from easydict import EasyDict
import os
import glob
import threading

supported_label_types = [lbl_handler.split('_')[1].replace('.py','') for lbl_handler in os.listdir('lbl') if 'handler' in lbl_handler]

class FileIO:
    def __init__(self, cfg: EasyDict):
        self.lbl_dir = os.path.join(cfg.data.path, cfg.data.label_subdir)
        self.clb_dir = os.path.join(cfg.data.path, cfg.data.calib_subdir)
        self.lbl_type = cfg.data.label.lbl_type
        self.lbl_count = cfg.data.size
        
        if self.lbl_type not in supported_label_types: raise NotImplementedError("Label type not supported. Supported file types: " + ', '.join(supported_label_types) + ".")
        self.lbl_ext, self.clb_ext, self.reader = __import__('lbl.handler_'+self.lbl_type, fromlist=['label_file_extention', 'calib_file_extension', 'Handler']).Handler
        
        files = glob.glob(os.path.join(self.lbl_dir, '*' + self.lbl_ext))
        file_basenames = [os.path.splitext(os.path.basename(file))[0] for file in files]
        file_basenames.sort(key=lambda file_name: int(''.join(filter(str.isdigit, file_name))))
        self.files_basenames = file_basenames[:self.lbl_count]
        
        self.data_lock = threading.Lock()
        self.data = []
        self.stop = threading.Event()
        threading.Thread(target=self.__async_read_fn__).start()
        
    def get_abs_path(self, idx: int):
        lbl_path = os.path.join(self.lbl_dir, self.files_basenames[idx] + self.lbl_ext)
        clb_path = os.path.join(self.clb_dir, self.files_basenames[idx] + self.clb_ext)
        return lbl_path, clb_path
        
    def __async_read_fn__(self):
        for idx in range(len(self.files_basenames)):
            if self.stop.is_set(): break
            lbl_abs_path, clb_abs_path = self.get_abs_path(idx)
            annotation = self.reader(lbl_abs_path, clb_abs_path)
            with self.data_lock: self.data.append(annotation)
        
    def __len__(self): return len(self.files_basenames)
    
    def __getitem__(self, idx):
        try:
            with self.data_lock: return self.data[idx]
        except:
            lbl_abs_path, clb_abs_path = self.get_abs_path(idx)
            return self.reader(lbl_abs_path, clb_abs_path)
        
    def close(self):
        self.stop.set()