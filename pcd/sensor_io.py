from easydict import EasyDict
import os
import numpy as np

supported_manufacturers = ['ouster']
supported_models = ['os1']

class SensorIO:
    def __init__(self, cfg: EasyDict):
        self.manufacturer = cfg.data.streaming.sensor_manufacturer.lower()
        self.model = cfg.data.streaming.sensor_model.lower()
        self.serial_no = cfg.data.streaming.sensor_serial_number
        
        if self.manufacturer not in supported_manufacturers: raise NotImplementedError("Manufacturer not supported. Supported manufacturers: " + ', '.join(supported_manufacturers) + ".")
        if self.model not in supported_models: raise NotImplementedError("Model not supported. Supported models: " + ', '.join(supported_models) + ".")
        
        self.cfg = cfg
        
        self.reader = self.get_reader_by_sensor(self.manufacturer, self.model)
        self.idx = -1
        
    def get_reader_by_sensor(self, manufacturer, model):
        if manufacturer == 'ouster':
            if model == 'os1': return self.__read_ouster__()
        
    def __read_ouster__(self, hostname:str = 'localhost'):
        try: ouster = __import__('ouster', fromlist=['client'])
        except: raise ImportError("Ouster-SDK not installed, please install it using 'pip install ouster-sdk'.")
        client = ouster.client
        config = client.SensorConfig()
        config.udp_port_lidar = 7502
        config.udp_port_imu = 7503
        config.operating_mode = client.OperatingMode.OPERATING_NORMAL
        
        if hostname == 'localhost': hostname = 'os-'+self.serial_no+'.local'
        client.set_config(hostname, config, udp_dest_auto=True)
        
        stream = client.Scans.stream(hostname=hostname, lidar_port=config.udp_port_lidar)
        xyz_lut = client.XYZLut(stream.metadata)

        def reader():
            while True:
                for scan in stream:
                    pcd_xyz = xyz_lut(scan).reshape(-1, 3)
                    intensity = client.destagger(stream.metadata, scan.field(client.ChanField.REFLECTIVITY)).reshape(-1, 1)
                    pcd_intensity_np = np.hstack((pcd_xyz, intensity))
                    yield pcd_intensity_np
                    
        return reader()
    
    def __get_item__(self, idx):
        if idx > self.idx:
            self.pcd_intensity_np = next(self.reader)
            self.idx = idx
        return self.pcd_intensity_np
        
    def __len__(self): return 1000000 # a large number
    
    def close(self): self.reader.close()

if __name__ == "__main__":
    cfg = EasyDict({
        'data': {
            'streaming': {
                'sensor_manufacturer': 'ouster',
                'sensor_model': 'os1',
                'sensor_serial_number': '122204001078'
            }
        }
    })
    sensor_io = SensorIO(cfg)
    for i in range(10):
        pcd = sensor_io.__get_item__(i)
        print(pcd.shape)
    sensor_io.close()
    print("Done.")
    
    