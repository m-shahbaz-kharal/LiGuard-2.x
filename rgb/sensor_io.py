from easydict import EasyDict
import os
import numpy as np

supported_manufacturers = [sm.split('_')[1] for sm in os.listdir('rgb') if 'handler' in sm]
supported_models = [sm.split('_')[2].replace('.py','') for sm in os.listdir('rgb') if 'handler' in sm]

class SensorIO:
    def __init__(self, cfg: EasyDict):
        self.manufacturer = cfg.sensors.camera.sensor_manufacturer.lower()
        self.model = cfg.sensors.camera.sensor_model.lower().replace('-','')
        self.serial_no = cfg.sensors.camera.sensor_serial_number
        
        if self.manufacturer not in supported_manufacturers: raise NotImplementedError("Manufacturer not supported. Supported manufacturers: " + ', '.join(supported_manufacturers) + ".")
        if self.model not in supported_models: raise NotImplementedError("Model not supported. Supported models: " + ', '.join(supported_models) + ".")
        
        self.cfg = cfg
        
        self.io = self.get_reader_by_sensor(self.manufacturer, self.model)
        self.reader = self.io.reader
        self.idx = -1
        
    def get_reader_by_sensor(self, manufacturer, model):
        handler = __import__('rgb.handler_'+manufacturer+'_'+model, fromlist=['Handler']).Handler
        handle = handler(self.manufacturer, self.model, self.serial_no)
        return handle
        
    def __get_item__(self, idx):
        if idx > self.idx:
            img_bgr = next(self.reader)
            self.img_rgb = img_bgr [...,::-1].copy()
            self.idx = idx
        return self.img_rgb
        
    def __len__(self): return 1000000 # a large number
    
    def close(self): self.io.close()

if __name__ == "__main__":
    cfg = EasyDict({
        'sensors': {
            'camera': {
                'sensor_hostname': 'localhost',
                'sensor_manufacturer': 'Flir',
                'sensor_model': 'BFS-PGE-16S2C-CS',
                'sensor_serial_number': '23422874'
            }
        }
    })
    sensor_io = SensorIO(cfg)
    import open3d as o3d
    viz = o3d.visualization.Visualizer()
    for i in range(10):
        img = sensor_io.__get_item__(i)
        img_o3d = o3d.geometry.Image(img)
        print(img.shape)
        o3d.visualization.draw_geometries([img_o3d])
    sensor_io.close()
    print("Done.")
    
    