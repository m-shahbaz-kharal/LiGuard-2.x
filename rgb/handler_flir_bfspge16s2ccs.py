try: pyspin = __import__('PySpin')
except: raise ImportError("Spinnaker SDK not installed.\n \
    Please download resource at from https://flir.netx.net/file/asset/59493/original/attachment and please install the wheel using `pip install spinnaker_python-4.0.0.116-cp310-cp310-win_amd64.whl.")

class Handler:
    def __init__(self, cfg):
        self.manufacturer = cfg.sensors.camera.manufacturer.lower()
        self.model = cfg.sensors.camera.model.lower().replace('-','')
        self.serial_no = cfg.sensors.camera.serial_number.lower()
        
        self.system = pyspin.System.GetInstance()
        camera_list = self.system.GetCameras()
        
        if camera_list.GetSize() == 0:
            self.system.ReleaseInstance()
            raise Exception("There is no flir camera connected to the system.")
        
        self.camera = camera_list.GetBySerial(self.serial_no)
        self.camera.Init()
        
        # set it to continuous 
        nodemap = self.camera.GetNodeMap()
        node_acquisition_mode = pyspin.CEnumerationPtr(nodemap.GetNode('AcquisitionMode'))
        node_acquisition_mode_continuous = node_acquisition_mode.GetEntryByName('Continuous')
        acquisition_mode_continuous = node_acquisition_mode_continuous.GetValue()
        node_acquisition_mode.SetIntValue(acquisition_mode_continuous)
        
        # & latest frame only mode
        sNodemap = self.camera.GetTLStreamNodeMap()
        node_bufferhandling_mode = pyspin.CEnumerationPtr(sNodemap.GetNode('StreamBufferHandlingMode'))
        node_newestonly = node_bufferhandling_mode.GetEntryByName('NewestOnly')
        node_newestonly_mode = node_newestonly.GetValue()
        node_bufferhandling_mode.SetIntValue(node_newestonly_mode)
        
        # set to BGR8 pixel format
        node_pixel_format = pyspin.CEnumerationPtr(nodemap.GetNode('PixelFormat'))
        node_bgr8 = node_pixel_format.GetEntryByName('BGR8')
        pixel_format_bgr8 = node_bgr8.GetValue()
        node_pixel_format.SetIntValue(pixel_format_bgr8)
        
        # start the camera
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