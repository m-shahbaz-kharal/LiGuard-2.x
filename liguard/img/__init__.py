"""
img Package
===========
This package contains modules responsible for reading and displaying image data. It comprises of three kinds of modules:

1. **file_io.py**: Contains the boilerplate code for reading image files supported by `opencv-python` library.
2. **sensor_io.py**: Contains the boilerplate code for reading live stream from camera sensor. This module is experimental.
3. **handler_<manufacturer>_<model>.py**: Contains the handler for reading live stream from a specific camera sensor from a specific manufacturer.

The `file_io.py` and `sensor_io.py` modules should not be modified except for contributions to the framework application logic.

Contributing a New Sensor Stream Handler
----------------------------------------
If you think that a new sensor stream handler can be beneficial for a vast majority of users, you can follow the contribution guidelines and add your handler to the `img` package as described below.

1. Create a new Python file named `handler_<manufacturer>_<model>.py` in the `img` package.
2. Replace `<manufacturer>` and `<model>` with the respective manufacturer and model of the camera.
3. The manufacturer and model are passed from, respectively, `sensors->camera->manufacturer` and `sensors->camera->model` in `base_config.yml`, and are used to select the appropriate handler.

Handler File Structure
-----------------------
The `handler_<manufacturer>_<model>.py` file should contain a class named `Handler` with the following structure:

.. code-block:: python

    class Handler:
        '''
        Args:
            cfg (dict): Configuration dictionary containing camera settings.

        Attributes:
            manufacturer (str): The manufacturer of the camera.
            model (str): The model of the camera.
            serial_no (str): The serial number of the camera.
            reader (generator): A generator that yields image arrays.

        Raises:
            Exception: If fails to connect to the camera or receive images from it.
        '''

        def __init__(self, cfg):
            self.manufacturer = cfg['sensors']['camera']['manufacturer'].lower()
            self.model = cfg['sensors']['camera']['model'].lower().replace('-', '')
            self.serial_no = cfg['sensors']['camera']['serial_number'].lower()

            # Connect to the camera and initialize the reader
            self.reader = self.__get_reader__()

        def __get_reader__(self):
            # Return a generator that yields image arrays
            pass

        def close(self):
            # Release the camera resources
            self.reader.close()

"""