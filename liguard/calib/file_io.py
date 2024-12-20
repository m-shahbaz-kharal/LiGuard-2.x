import os
from liguard.gui.config_gui import resolve_for_application_root, resolve_for_default_workspace
import glob
import time
import threading

calib_dir = os.path.dirname(os.path.realpath(__file__))
supported_calib_types = [clb_handler.split('_')[1].replace('.py','') for clb_handler in os.listdir(calib_dir) if 'handler' in clb_handler]

class FileIO:
    """
    Class for handling file input/output operations related to calibration files.
    """

    def __init__(self, cfg: dict):
        """
        Initializes a FileIO object.

        Args:
            cfg (dict): Configuration dictionary containing parameters for file IO.

        Raises:
            NotImplementedError: If the calibration type is not supported.
        """
        # get params from config
        self.cfg = cfg
        main_dir = cfg['data']['main_dir']
        if not os.path.isabs(main_dir): main_dir = os.path.join(self.cfg['data']['pipeline_dir'], main_dir)
        self.clb_dir = os.path.join(main_dir, cfg['data']['calib_subdir'])
        self.clb_type = cfg['data']['calib']['clb_type']
        self.clb_start_idx = cfg['data']['start']['calib']
        self.global_zero = cfg['data']['start']['global_zero']
        self.clb_end_idx = self.clb_start_idx + cfg['data']['count']
        
        # Add custom supported calibration types to the list
        custom_calib_data_handlers_dir = os.path.join(cfg['data']['pipeline_dir'], 'data_handler', 'calib')
        if os.path.exists(custom_calib_data_handlers_dir):
            custom_supported_calib_types = [clb_handler.split('_')[1].replace('.py','') for clb_handler in os.listdir(custom_calib_data_handlers_dir) if 'handler' in clb_handler]
            supported_calib_types.extend(custom_supported_calib_types)
        else:
            custom_supported_calib_types = []
        # Check if the calibration type is supported
        if self.clb_type not in supported_calib_types: raise NotImplementedError("Calib type not supported. Supported file types: " + ', '.join(supported_calib_types) + ".")
        # Import the calibration handler
        if self.clb_type in custom_supported_calib_types:
            h = __import__(f'handler_{self.clb_type}', fromlist=['calib_file_extension', 'Handler'])
        else:
            h = __import__('liguard.calib.handler_'+self.clb_type, fromlist=['calib_file_extension', 'Handler'])
        self.clb_ext, self.reader = h.calib_file_extension, h.Handler
        
        # read all the calibration files
        files = glob.glob(os.path.join(self.clb_dir, '*' + self.clb_ext))
        file_basenames = [os.path.splitext(os.path.basename(file))[0] for file in files]
        file_basenames.sort(key=lambda file_name: int(''.join(filter(str.isdigit, file_name))))
        self.files_basenames = file_basenames[self.clb_start_idx:self.clb_end_idx][self.global_zero:]
        
        # read the calibration files in async mode
        self.data_lock = threading.Lock()
        self.data = []
        self.stop = threading.Event()
        threading.Thread(target=self.__async_read_fn__).start()
        
    def get_abs_path(self, idx: int):
        """
        Get the absolute path of the calibration file at the specified index.

        Args:
            idx (int): Index of the calibration file.

        Returns:
            str: Absolute path of the calibration file.
        """
        clb_path = os.path.join(self.clb_dir, self.files_basenames[idx] + self.clb_ext)
        return clb_path
    
    def __async_read_fn__(self):
        """
        Asynchronously read all the calibration files.
        """
        for idx in range(len(self.files_basenames)):
            if self.stop.is_set(): break
            clb_abs_path = self.get_abs_path(idx)
            calib = self.reader(clb_abs_path)
            with self.data_lock: self.data.append((clb_abs_path, calib))
            time.sleep(self.cfg['threads']['io_sleep'])
        
    def __len__(self):
        """
        Get the number of calibration files.

        Returns:
            int: Number of calibration files.
        """
        return len(self.files_basenames)
    
    def __getitem__(self, idx):
        """
        Get the calibration file at the specified index.

        Args:
            idx: Index of the calibration file.

        Returns:
            tuple: Tuple containing the absolute path of the calibration file and the calibration data.
        """
        try:
            with self.data_lock: return self.data[idx]
        except:
            clb_abs_path = self.get_abs_path(idx)
            calib = self.reader(clb_abs_path)
            return (clb_abs_path, calib)
        
    def close(self):
        """
        Closes the calibration files reading thread.
        """
        self.stop.set()