try: ouster = __import__('ouster', fromlist=['client'])
except: raise ImportError("Ouster-SDK not installed, please install it using 'pip install ouster-sdk'.")

import numpy as np

class Handler:
    def __init__(self, cfg):
        self.cfg = cfg
        
        self.manufacturer = self.cfg.sensors.lidar.manufacturer.lower()
        self.model = self.cfg.sensors.lidar.model.lower().replace('-','')
        self.serial_no = self.cfg.sensors.lidar.serial_number
        self.hostname = self.cfg.sensors.lidar.hostname
        
        self.client = ouster.client
        
        # set the sensor config
        config = self.client.SensorConfig()
        config.udp_port_lidar = 7502
        config.udp_port_imu = 7503
        config.operating_mode = self.client.OperatingMode.OPERATING_NORMAL
        if self.hostname == 'localhost': self.hostname = 'os-'+self.serial_no+'.local'
        self.client.set_config(self.hostname, config, udp_dest_auto=True)
        
        self.stream = self.client.Scans.stream(hostname=self.hostname, lidar_port=config.udp_port_lidar)
        self.xyz_lut = self.client.XYZLut(self.stream.metadata)
        self.reader = self.__get_reader__()

    def __get_reader__(self):
        while True:
            for scan in self.stream:
                pcd_xyz = self.xyz_lut(scan).reshape(-1, 3)
                intensity = self.client.destagger(self.stream.metadata, scan.field(self.client.ChanField.REFLECTIVITY)).reshape(-1, 1)
                pcd_intensity_np = np.hstack((pcd_xyz, intensity))
                yield pcd_intensity_np
                
    def close(self):
        self.reader.close()
        self.stream.close()