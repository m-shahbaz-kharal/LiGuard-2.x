"""
calib Package
=============
The calib package manages calibration file handlers for reading calibration files. The `file_io.py` module should not be modified except for contributions to the framework application logic.

Contributing a New Calibration File Handler
-------------------------------------------
If you think that a new calibration file handler can be beneficial for a vast majority of users, you can follow the contribution guidelines and add your handler to the `calib` package as described below.

1. Create a new Python file named `handler_<clb_type>.py` in the `calib` directory.
2. Replace `<clb_type>` with the dataset name. For example, if the calibration file handler is for reading calibration files in `KITTI` dataset, the handler file should be named `handler_kitti.py`.
3. The `clb_type` is passed from `data->calib->clb_type` in `base_config.yml` that is used to select the appropriate handler.

Handler File Structure
-----------------------

.. code-block:: python

    import os
    import numpy as np

    calib_file_extension = '.txt' # this tells the extension of the calibration files in directory given by config['data']['calib_subdir']

    def Handler(calib_path: str): # don't change the function signature
        '''
        Process a calibration file and generate a dictionary containing the calibration data in KITTI format.

        Args:
            calib_path (str): The path to the calibration file.

        Returns:
            dict: A dictionary containing the calibration data.
        '''
        calib = {}

        #################################################################################
        # basic code snippet to read the calibration file, uncomment and modify as needed
        #################################################################################
        # if not os.path.exists(calib_path): return calib
        # with open(calib_path, 'r') as f: clbs = f.readlines()
        # for line in clbs:
        #     line = line.strip()
        #     if len(line) == 0 or line.startswith('#'): continue
        #     # parse your calibs
        #     calib['P2'] = ? # 3x4 projection matrix
        #     calib['R0_rect'] = ? # 4x4 rectification matrix
        #     calib['Tr_velo_to_cam'] = ? # 4x4 transformation matrix from lidar to camera

        return calib # make sure to return the calibration data dictionary, even if it is empty
"""