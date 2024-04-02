import os

supported_manufacturers = [sm.split('_')[1] for sm in os.listdir('img') if 'handler' in sm]
supported_models = [sm.split('_')[2].replace('.py','') for sm in os.listdir('img') if 'handler' in sm]

class SensorIO:
    def __init__(self, cfg: dict):
        self.manufacturer = cfg['sensors']['camera']['manufacturer'].lower()
        self.model = cfg['sensors']['camera']['model'].lower().replace('-','')
        self.serial_no = cfg['sensors']['camera']['serial_number']
        
        if self.manufacturer not in supported_manufacturers: raise NotImplementedError("Manufacturer not supported. Supported manufacturers: " + ', '.join(supported_manufacturers) + ".")
        if self.model not in supported_models: raise NotImplementedError("Model not supported. Supported models: " + ', '.join(supported_models) + ".")
        
        self.cfg = cfg
        self.img_count = cfg['data']['size']
        
        handler = __import__('img.handler_'+self.manufacturer+'_'+self.model, fromlist=['Handler']).Handler
        self.handle = handler(self.cfg)
        self.reader = self.handle.reader
        self.idx = -1
        
    def __getitem__(self, idx):
        if idx > self.idx:
            img_bgr = next(self.reader)
            self.img_rgb = img_bgr[:,:,::-1].copy()
            self.idx = idx
        return None, self.img_rgb
        
    def __len__(self): return self.img_count
    
    def close(self): self.handle.close()
    
    