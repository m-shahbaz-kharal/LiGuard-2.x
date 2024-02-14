from easydict import EasyDict

try: pyspin = __import__('PySpin')
except: raise ImportError("Spinnaker SDK not installed.\n \
    Please download resource at from https://flir.netx.net/file/asset/59493/original/attachment and please install the wheel using `pip install spinnaker_python-4.0.0.116-cp310-cp310-win_amd64.whl.")

class Handler:
    def __init__(self, manufacturer, model, serial_no):
        self.manufacturer = manufacturer
        self.model = model
        self.serial_no = serial_no
        
        self.system = pyspin.System.GetInstance()
        camera_list = self.system.GetCameras()
        
        if camera_list.GetSize() == 0:
            self.system.ReleaseInstance()
            raise Exception("There is no flir camera connected to the system.")
        
        self.camera = camera_list.GetBySerial(self.serial_no)
        self.camera.Init()
        self.camera.BeginAcquisition()
        
        self.reader = self.__get__reader__()
            
    def __get__reader__(self):
        while True:
            image_result = self.camera.GetNextImage(1000) # 1000 ms timeout
            if image_result.IsIncomplete(): print('An incomplete image is received. Dropping ...')
            else:
                img_np = image_result.GetNDArray()
                image_result.Release()
                yield img_np
                    
    def close(self):
        self.reader.close()
        self.camera.EndAcquisition()
        self.camera.DeInit()
        del self.camera
        self.system.ReleaseInstance()