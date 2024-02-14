from easydict import EasyDict
import os

supported_manufacturers = [sm.split('_')[1] for sm in os.listdir('rgb') if 'handler' in sm]
supported_models = [sm.split('_')[2].replace('.py','') for sm in os.listdir('rgb') if 'handler' in sm]

class SensorIO:
    def __init__(self, cfg: EasyDict):
        self.manufacturer = cfg.sensors.camera.manufacturer.lower()
        self.model = cfg.sensors.camera.model.lower().replace('-','')
        self.serial_no = cfg.sensors.camera.serial_number
        
        if self.manufacturer not in supported_manufacturers: raise NotImplementedError("Manufacturer not supported. Supported manufacturers: " + ', '.join(supported_manufacturers) + ".")
        if self.model not in supported_models: raise NotImplementedError("Model not supported. Supported models: " + ', '.join(supported_models) + ".")
        
        self.cfg = cfg
        
        handler = __import__('rgb.handler_'+self.manufacturer+'_'+self.model, fromlist=['Handler']).Handler
        self.handle = handler(self.cfg)
        self.reader = self.handle.reader
        self.idx = -1
        
    def __get_item__(self, idx):
        if idx > self.idx:
            img_bgr = next(self.reader)
            self.img_rgb = img_bgr
            self.idx = idx
        return self.img_rgb
        
    def __len__(self): return 1000000 # a large number
    
    def close(self): self.handle.close()

if __name__ == "__main__":
    cfg = EasyDict({
        'sensors': {
            'camera': {
                'hostname': 'localhost',
                'manufacturer': 'Flir',
                'model': 'BFS-PGE-16S2C-CS',
                'serial_number': '23422874'
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
    
    