import numpy as np

class Handler:
    """
    A class that handles the CARLA camera sensor.

    Args:
        cfg (dict): Configuration dictionary containing sensor information.

    Attributes:
        cfg (dict): Configuration dictionary containing sensor information.
        manufacturer (str): Manufacturer of the LiDAR sensor.
        model (str): Model of the LiDAR sensor.
        serial_no (str): Serial number of the LiDAR sensor.
        hostname (str): Hostname of the LiDAR sensor.
        client (carla.Client): Carla client object.
        world (carla.World): Carla world object.
        camera (carla.Sensor): Carla camera object.
        data_queue (queue.Queue): A queue to store RGB image data.
        reader (generator): Generator that yields point RGB image data.

    """
    def __init__(self, cfg: dict):
        try: carla = __import__('carla')
        except:
            print("CARLA not installed, please install it using 'pip install carla'.")
            return
        self.cfg = cfg
        
        # params
        self.manufacturer = self.cfg['sensors']['camera']['manufacturer'].lower()
        self.model = self.cfg['sensors']['camera']['model'].lower().replace('-','')
        self.serial_no = self.cfg['sensors']['camera']['serial_number']
        self.hostname = self.cfg['sensors']['camera']['hostname']
        
        # connection to carla
        self.client = carla.Client(self.hostname, 2000)
        self.world = self.client.get_world()
        
        # traffic manager
        traffic_manager = self.client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)

        # world settings
        settings = self.world.get_settings()
        settings.fixed_delta_seconds = self.cfg['threads']['io_sleep']
        settings.synchronous_mode = True
        self.world.apply_settings(settings)
        
        # get a lidar
        cameras = self.world.get_actors().filter('sensor.camera.rgb')
        if cameras: self.camera = cameras[0]
        else: raise Exception("No camera.rgb sensor found in the world.")
        
        from queue import Queue
        self.data_queue = Queue()
        self.camera.listen(self.data_queue.put)
        
        # reader    
        self.reader = self.__get_reader__()

    def __get_reader__(self):
        """
        Generator function that yields RGB image data.

        Yields:
            np.ndarray: Numpy array containing RGB image data.

        """
        while True:
            if 'temporary' not in self.cfg: self.cfg['temporary'] = {}
            elif 'carla_frame_idx' not in self.cfg['temporary']:
                self.world.tick()
                self.cfg['temporary']['carla_frame_idx'] = self.world.get_snapshot().frame
            data = self.data_queue.get(timeout=self.cfg['threads']['io_sleep'])
            assert self.cfg['temporary']['carla_frame_idx'] == data.frame, "carla.Sensor frame mismatch."
            im_array = np.copy(np.frombuffer(data.raw_data, dtype=np.dtype("uint8")))
            im_array = np.reshape(im_array, (data.height, data.width, 4))
            im_array = im_array[:, :, :3][:, :, ::-1]
            yield im_array
            
    def close(self):
        """
        Closes the generator function.

        """
        self.reader.close()