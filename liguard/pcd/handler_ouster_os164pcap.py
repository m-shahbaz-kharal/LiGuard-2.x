import numpy as np

from more_itertools import consume, nth
import tkinter as tk
from tkinter import filedialog

class Handler:
    """
    A class that handles the Ouster OS1-64 LiDAR sensor stream stored in PCAP file.

    Args:
        cfg (dict): Configuration dictionary containing sensor information.

    Attributes:
        cfg (dict): Configuration dictionary containing sensor information.
        manufacturer (str): Manufacturer of the LiDAR sensor.
        model (str): Model of the LiDAR sensor.
        serial_no (str): Serial number of the LiDAR sensor.
        hostname (str): Hostname of the LiDAR sensor.
        client (ouster.client): Ouster client object.
        stream (ouster.client.Scans): Scans stream object.
        xyz_lut (ouster.client.XYZLut): XYZ lookup table object.
        reader (generator): Generator that yields point cloud data.

    """

    def __init__(self, cfg: dict):
        try:
            self.ouster = __import__('ouster.sdk', fromlist=['open_source', 'client'])
        except:
            print("Ouster-SDK not installed, please install it using 'pip install ouster-sdk'.")
            return
        
        self.cfg = cfg
        
        # Extract sensor information from the configuration dictionary
        self.manufacturer = self.cfg['sensors']['lidar']['manufacturer'].lower()
        self.model = self.cfg['sensors']['lidar']['model'].lower().replace('-','')
        self.serial_no = self.cfg['sensors']['lidar']['serial_number']
        self.hostname = self.cfg['sensors']['lidar']['hostname']

        self.pcd_start_idx = self.cfg['data']['start']['lidar']
        self.pcd_end_idx = self.pcd_start_idx + cfg['data']['count']
        
        root = tk.Tk()
        root.withdraw()
        self.pcap_file = filedialog.askopenfilename(title="Select PCAP file", filetypes=(("PCAP files", "*.pcap"), ("All files", "*.*")))
        self.json_file = filedialog.askopenfilename(title="Select JSON file", filetypes=(("JSON files", "*.json"), ("All files", "*.*")))

        self.source = self.ouster.open_source(self.pcap_file, meta=[self.json_file], index=True)
        self.source = self.source[self.pcd_start_idx:self.pcd_end_idx]
        self.scans = self.ouster.client.Scans(self.source)
        self.xyzlut = self.ouster.XYZLut(self.source.metadata)
        self.reader = self.__get_reader__()

    def __get_reader__(self):
        """
        Generator function that yields point cloud data.

        Yields:
            np.ndarray: Numpy array containing point cloud data.

        """
        for scan in self.scans:
            pcd_xyz = self.xyzlut(scan).reshape(-1, 3)
            intensity = self.ouster.client.destagger(self.source.metadata, scan.field(self.ouster.client.ChanField.REFLECTIVITY)).reshape(-1, 1)
            pcd_intensity_np = np.hstack((pcd_xyz, intensity))
            yield pcd_intensity_np
                
    def close(self):
        """
        Closes the reader and stream objects.

        """
        self.reader.close()