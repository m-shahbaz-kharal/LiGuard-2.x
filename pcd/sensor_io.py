import os

supported_manufacturers = [sm.split('_')[1] for sm in os.listdir('pcd') if 'handler' in sm]
supported_models = [sm.split('_')[2].replace('.py','') for sm in os.listdir('pcd') if 'handler' in sm]

class SensorIO:
    def __init__(self, cfg: dict):
        self.manufacturer = cfg['sensors']['lidar']['manufacturer'].lower()
        self.model = cfg['sensors']['lidar']['model'].lower().replace('-', '')
        self.serial_no = cfg['sensors']['lidar']['serial_number']
        
        if self.manufacturer not in supported_manufacturers: raise NotImplementedError("Manufacturer not supported. Supported manufacturers: " + ', '.join(supported_manufacturers) + ".")
        if self.model not in supported_models: raise NotImplementedError("Model not supported. Supported models: " + ', '.join(supported_models) + ".")
        
        self.cfg = cfg
        
        handler = __import__('pcd.handler_'+self.manufacturer+'_'+self.model, fromlist=['Handler']).Handler
        
        self.handle = handler(self.cfg)
        self.reader = self.handle.reader
        self.idx = -1
        
    def __getitem__(self, idx):
        if idx > self.idx:
            self.pcd_intensity_np = next(self.reader)
            self.idx = idx
        return self.pcd_intensity_np
        
    def __len__(self): return float('inf')
    
    def close(self): self.handle.close()