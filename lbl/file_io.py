import os
from gui.config_gui import get_abs_path
import glob
import time
import threading

lbl_dir = os.path.dirname(os.path.realpath(__file__))

supported_label_types = [lbl_handler.split('_')[1].replace('.py','') for lbl_handler in os.listdir(lbl_dir) if 'handler' in lbl_handler]

class FileIO:
    """
    Class for handling file input/output operations.

    Args:
        cfg (dict): Configuration dictionary.
        calib_reader (callable): Callable object for reading calibration data.

    Attributes:
        cfg (dict): Configuration dictionary.
        lbl_dir (str): Directory path for label files.
        lbl_type (str): Type of label files.
        lbl_start_idx (int): Index of the first label file to process.
        lbl_count (int): Number of label files to process.
        lbl_ext (str): Extension of label files.
        reader (class): Handler class for reading label files.
        clb_reader (callable): Callable object for reading calibration data.
        files_basenames (list): List of file basenames.
        data_lock (threading.Lock): Lock for thread safety.
        data (list): List of tuples containing label file paths and annotations.
        stop (threading.Event): Event for stopping the async read thread.

    Methods:
        get_abs_path(idx: int) -> str: Returns the absolute path of the label file at the given index.
        __async_read_fn__(): Asynchronously reads label files and annotations.
        __len__() -> int: Returns the number of label files.
        __getitem__(idx) -> tuple: Returns the label file path and annotation at the given index.
        close(): Stops the async read thread.

    """
    def __init__(self, cfg: dict, calib_reader: callable):
        self.cfg = cfg
        self.lbl_dir = os.path.join(get_abs_path(cfg['data']['main_dir']), cfg['data']['label_subdir'])
        self.lbl_type = cfg['data']['label']['lbl_type']
        self.lbl_start_idx = cfg['data']['start']['label']
        self.global_zero = cfg['data']['start']['global_zero']
        self.lbl_end_idx = self.lbl_start_idx + cfg['data']['count']
        
        # Check if the label type is supported
        if self.lbl_type not in supported_label_types: raise NotImplementedError("Label type not supported. Supported file types: " + ', '.join(supported_label_types) + ".")
        # Import the handler for the label type
        h = __import__('lbl.handler_'+self.lbl_type, fromlist=['label_file_extension', 'Handler'])
        self.lbl_ext, self.reader = h.label_file_extension, h.Handler
        self.clb_reader = calib_reader
        
        files = glob.glob(os.path.join(self.lbl_dir, '*' + self.lbl_ext))
        file_basenames = [os.path.splitext(os.path.basename(file))[0] for file in files]
        # Sort the file basenames based on the numbers in the filenames
        file_basenames.sort(key=lambda file_name: int(''.join(filter(str.isdigit, file_name))))
        self.files_basenames = file_basenames[self.lbl_start_idx:self.lbl_end_idx][self.global_zero:]
        
        self.data_lock = threading.Lock()
        self.data = []
        self.stop = threading.Event()
        threading.Thread(target=self.__async_read_fn__).start()
        
    def get_abs_path(self, idx: int) -> str:
        """
        Returns the absolute path of the label file at the given index.

        Args:
            idx (int): Index of the label file.

        Returns:
            str: Absolute path of the label file.

        """
        lbl_path = os.path.join(self.lbl_dir, self.files_basenames[idx] + self.lbl_ext)
        return lbl_path
        
    def __async_read_fn__(self):
        """
        Asynchronously reads label files and annotations.

        """
        for idx in range(len(self.files_basenames)):
            if self.stop.is_set(): break
            lbl_abs_path = self.get_abs_path(idx)
            annotation = self.reader(lbl_abs_path, self.clb_reader(idx)[1] if self.clb_reader else None)
            with self.data_lock: self.data.append((lbl_abs_path, annotation))
            time.sleep(self.cfg['threads']['io_sleep'])
        
    def __len__(self) -> int:
        """
        Returns the number of label files.

        Returns:
            int: Number of label files.

        """
        return len(self.files_basenames)
    
    def __getitem__(self, idx) -> tuple:
        """
        Returns the label file path and annotation at the given index.

        Args:
            idx: Index of the label file.

        Returns:
            tuple: Label file path and annotation.

        """
        try:
            with self.data_lock: return self.data[idx]
        except:
            lbl_abs_path = self.get_abs_path(idx)
            annotation = self.reader(lbl_abs_path, self.clb_reader(idx)[1] if self.clb_reader else None)
            return (lbl_abs_path, annotation)
        
    def close(self):
        """
        Stops the async read thread.

        """
        self.stop.set()