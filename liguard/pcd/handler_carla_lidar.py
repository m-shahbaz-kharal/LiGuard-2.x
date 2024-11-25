import os
import numpy as np

class Handler:
    """
    A class that handles the CARLA lidar sensor.

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
        lidar (carla.Sensor): Carla lidar object.
        data_queue (queue.Queue): A queue to store point cloud data.
        reader (generator): Generator that yields point cloud data.

    """

    def __init__(self, cfg: dict):
        try: carla = __import__('carla')
        except:
            print("CARLA not installed, please install it using 'pip install carla'.")
            return
        self.cfg = cfg
        
        # params
        self.manufacturer = self.cfg['sensors']['lidar']['manufacturer'].lower()
        self.model = self.cfg['sensors']['lidar']['model'].lower().replace('-','')
        self.serial_no = self.cfg['sensors']['lidar']['serial_number']
        self.hostname = self.cfg['sensors']['lidar']['hostname']
        
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
        lidars = self.world.get_actors().filter('sensor.lidar.ray_cast')
        if lidars: self.lidar = lidars[0]
        else: raise Exception("No lidar (ray_cast) sensor found in the world.")
        
        from queue import Queue
        self.data_queue = Queue()
        self.lidar.listen(self.data_queue.put)
        
        # reader    
        self.reader = self.__get_reader__()

    def __get_reader__(self):
        """
        Generator function that yields point cloud data.

        Yields:
            np.ndarray: Numpy array containing point cloud data.

        """
        while True:
            if 'temporary' not in self.cfg: self.cfg['temporary'] = {}
            elif 'carla_frame_idx' not in self.cfg['temporary']:
                self.world.tick()
                self.cfg['temporary']['carla_frame_idx'] = self.world.get_snapshot().frame
            data = self.data_queue.get(timeout=self.cfg['threads']['io_sleep'])
            assert self.cfg['temporary']['carla_frame_idx'] == data.frame, "carla.Sensor frame mismatch."
            pcd_intensity_np = np.copy(np.frombuffer(data.raw_data, dtype=np.dtype('f4')))
            pcd_intensity_np = np.reshape(pcd_intensity_np, (int(pcd_intensity_np.shape[0] / 4), 4))
            yield pcd_intensity_np
            
    def close(self):
        """
        Closes the generator function.

        """
        self.reader.close()