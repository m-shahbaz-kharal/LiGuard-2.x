import os
from liguard.gui.gui_utils import resolve_for_application_root, resolve_for_default_workspace
import glob
import time
import threading

import cv2
import numpy as np

class FileIO:
    """
    Class for reading and managing a collection of image files.

    Args:
        cfg (dict): Configuration dictionary containing the necessary parameters.

    Attributes:
        cfg (dict): Configuration dictionary.
        img_dir (str): Directory path where the image files are located.
        img_type (str): File extension of the image files.
        img_start_idx (int): Index of the first image file to read.
        img_count (int): Number of image files to read.
        files_basenames (list): List of file basenames (without extension) of the image files.
        reader (function): Function to read an image file.
        data_lock (threading.Lock): Lock for thread-safe access to the data list.
        data (list): List of tuples containing the file absolute path and the image data.
        stop (threading.Event): Event to signal the thread to stop.

    Methods:
        __init__(self, cfg: dict): Initializes the FileIO object.
        __read_img__(self, file_abs_path: str): Reads an image file and returns the image data.
        __read_raw_img__(self, file_abs_path: str): Reads a raw image file and returns the image data.
        get_abs_path(self, idx: int): Returns the absolute path of the image file at the given index.
        __async_read_fn__(self): Asynchronously reads the image files and populates the data list.
        __len__(self): Returns the number of image files.
        __getitem__(self, idx): Returns the image data and file absolute path at the given index.
        close(self): Stops the asynchronous reading process.

    """

    def __init__(self, cfg: dict):
        self.cfg = cfg
        main_dir = cfg['data']['main_dir']
        if not os.path.isabs(main_dir): main_dir = os.path.join(self.cfg['data']['pipeline_dir'], main_dir)
        self.img_dir = os.path.join(main_dir, cfg['data']['camera_subdir'])
        if not os.path.exists(self.img_dir): raise FileNotFoundError(f'Directory {self.img_dir} does not exist.')
        self.img_type = cfg['data']['camera']['img_type']
        self.img_start_idx = cfg['data']['start']['camera']
        self.global_zero = cfg['data']['start']['global_zero']
        self.img_end_idx = self.img_start_idx + cfg['data']['count']
        files = glob.glob(os.path.join(self.img_dir, '*' + self.img_type))
        if len(files) == 0: raise FileNotFoundError(f'No image files found in {self.img_dir}.')
        file_basenames = [os.path.splitext(os.path.basename(file))[0] for file in files]
        # Sort the file basenames based on the numerical part
        file_basenames.sort(key=lambda file_name: int(''.join(filter(str.isdigit, file_name))))
        self.files_basenames = file_basenames[self.img_start_idx:self.img_end_idx][self.global_zero:]
        self.reader = self.__read_img__

        self.data_lock = threading.Lock()
        self.data = []
        self.stop = threading.Event()
        # Start the asynchronous reading thread
        threading.Thread(target=self.__async_read_fn__).start()

    def __read_img__(self, file_abs_path: str):
        """
        Reads an image file and returns the image data in RGB format.
        Supports standard image formats through OpenCV and raw image formats.

        Args:
            file_abs_path (str): Absolute path of the image file.

        Returns:
            numpy.ndarray: Image data in RGB format.

        """
        # Check if it's a raw image file
        if file_abs_path.lower().endswith('.raw'):
            return self.__read_raw_img__(file_abs_path)
        
        # Use OpenCV for standard image formats
        img_bgr = cv2.imread(file_abs_path, cv2.IMREAD_UNCHANGED)
        if img_bgr is None:
            raise ValueError(f"Failed to read image file: {file_abs_path}")
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        return img_rgb

    def __read_raw_img__(self, file_abs_path: str):
        """
        Reads a raw image file and returns the image data in RGB format.
        
        Args:
            file_abs_path (str): Absolute path of the raw image file.
            
        Returns:
            numpy.ndarray: Image data in RGB format.
            
        Note:
            This method assumes the raw image is stored as a 3-channel RGB image.
            For different raw formats, you may need to modify the parameters below.
        """
        try:
            # Get file size to determine image dimensions
            file_size = os.path.getsize(file_abs_path)
            
            # Determine number of channels based on configuration or file size analysis
            if hasattr(self, 'cfg') and 'camera' in self.cfg.get('data', {}) and 'raw_format' in self.cfg['data']['camera']:
                raw_format = self.cfg['data']['camera']['raw_format'].lower()
                if raw_format == 'grayscale':
                    channels = 1
                else:
                    channels = 3  # RGB or BGR
            else:
                # Try to determine channels from file size
                # Test common channel counts
                for test_channels in [1, 3, 4]:  # grayscale, RGB, RGBA
                    if file_size % test_channels == 0:
                        channels = test_channels
                        break
                else:
                    # Default to 3 channels if no clear match
                    channels = 3
            
            bits_per_channel = 8
            bytes_per_pixel = channels * bits_per_channel // 8
            
            # Calculate total pixels
            total_pixels = file_size // bytes_per_pixel
            
            # Common image size presets (width, height) - add more as needed
            common_sizes = [
                (640, 480),      # VGA
                (800, 600),      # SVGA
                (1024, 768),     # XGA
                (1280, 720),     # HD
                (1280, 960),     # SXGA
                (1440, 1080),    # HD+
                (1600, 1200),    # UXGA
                (1920, 1080),    # Full HD
                (1920, 1200),    # WUXGA
                (2560, 1440),    # QHD
                (2560, 1600),    # WQXGA
                (3840, 2160),    # 4K UHD
                (4096, 2160),    # 4K DCI
                (7680, 4320),    # 8K UHD
            ]
            
            # Try to find matching dimensions from common presets
            width = height = None
            for w, h in common_sizes:
                if w * h == total_pixels:
                    width, height = w, h
                    break
            
            # If no matching preset found, use square dimensions as fallback
            if width is None:
                side_length = int(np.sqrt(total_pixels))
                width = height = side_length
            
            # Validate that the dimensions make sense
            if width <= 0 or height <= 0 or width > 10000 or height > 10000:
                raise ValueError(f"Invalid image dimensions calculated: {width}x{height}")
            
            # Read raw data
            with open(file_abs_path, 'rb') as f:
                raw_data = f.read()
            
            # Convert to numpy array
            raw_array = np.frombuffer(raw_data, dtype=np.uint8)
            
            # Calculate expected size
            expected_size = width * height * channels
            
            # Validate data size
            if len(raw_array) < expected_size:
                raise ValueError(f"Raw file too small: expected {expected_size} bytes, got {len(raw_array)} bytes")
            
            # Reshape to image dimensions
            if len(raw_array) >= expected_size:
                # Truncate to exact size if there's extra data
                raw_array = raw_array[:expected_size]
                img = raw_array.reshape((height, width, channels))
            else:
                # This should not happen due to validation above, but handle just in case
                required_size = expected_size
                padded_array = np.zeros(required_size, dtype=np.uint8)
                padded_array[:len(raw_array)] = raw_array
                img = padded_array.reshape((height, width, channels))
            
            # Ensure the image is in RGB format
            if img.shape[2] == 3:
                # If it's BGR, convert to RGB
                if hasattr(self, 'cfg') and 'camera' in self.cfg.get('data', {}) and 'raw_format' in self.cfg['data']['camera']:
                    if self.cfg['data']['camera']['raw_format'].lower() == 'bgr':
                        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                else:
                    # Default assumption: raw data is in RGB format
                    pass
            elif img.shape[2] == 1:
                # If it's grayscale, convert to RGB
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
            elif img.shape[2] == 4:
                # If it's RGBA, convert to RGB
                img = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)
            
            return img
            
        except Exception as e:
            raise ValueError(f"Failed to read raw image file {file_abs_path}: {str(e)}")

    def get_abs_path(self, idx: int):
        """
        Returns the absolute path of the image file at the given index.

        Args:
            idx (int): Index of the image file.

        Returns:
            str: Absolute path of the image file.

        """
        return os.path.join(self.img_dir, self.files_basenames[idx] + self.img_type)

    def __async_read_fn__(self):
        """
        Asynchronously reads the image files and populates the data list.

        """
        for idx in range(len(self.files_basenames)):
            if self.stop.is_set():
                break
            file_abs_path = self.get_abs_path(idx)
            pcd_np = self.reader(file_abs_path)
            with self.data_lock:
                # append the file absolute path and the image data to the data list
                self.data.append((file_abs_path, pcd_np))
            time.sleep(self.cfg['threads']['io_sleep'])

    def __len__(self):
        """
        Returns the number of image files.

        Returns:
            int: Number of image files.

        """
        return len(self.files_basenames)

    def __getitem__(self, idx):
        """
        Returns the image data and file absolute path at the given index.

        Args:
            idx (int): Index of the image file.

        Returns:
            tuple: Tuple containing the file absolute path and the image data.

        """
        try:
            with self.data_lock:
                return self.data[idx]
        except:
            file_abs_path = self.get_abs_path(idx)
            # return the file absolute path and the image data
            return (file_abs_path, self.reader(file_abs_path))

    def close(self):
        """
        Closes the image files reading thread.

        """
        self.stop.set()