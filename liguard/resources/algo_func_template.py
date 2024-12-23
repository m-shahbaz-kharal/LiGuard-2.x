#########################################################################################################################
# usually, the following imports are common for all the algorithms, so it is recommended to not remove them
import inspect
from liguard.gui.config_gui import resolve_for_application_root, resolve_for_default_workspace
from liguard.gui.logger_gui import Logger
from liguard.algo.utils import AlgoType, algo_func, get_algo_params, make_key
algo_type = AGLO_TYPE
#########################################################################################################################

@algo_func(required_data=[]) # add required keys in the list -- necessary decorator, don't remove
# following keys are standard to `LiGuard`:
# `current_point_cloud_path`, `current_point_cloud_numpy`, `current_image_path`, `current_image_numpy`, `current_calib_path`, `current_calib_data`, `current_label_path`, `current_label_list`
# one or more of the `LiGuard` standard keys can be added to `keys_required_in_data_dict` decorator, for example:
# @keys_required_in_data_dict(['current_point_cloud_numpy', 'current_image_numpy'])
# @keys_required_in_data_dict(['current_calib_data'])
# custom keys can also be added to `keys_required_in_data_dict` decorator if those are generated by any previous algorithm(s) in the pipeline, for example:
# @keys_required_in_data_dict(['custom_key_1', 'custom_key_2'])
def FUNCTION_NAME(data_dict: dict, cfg_dict: dict, logger: Logger):
    """
    A function to perform the algorithmic operations on the data.

    Args:
        data_dict (dict): A dictionary containing the data.
        cfg_dict (dict): A dictionary containing the configuration parameters.
        logger (gui.logger_gui.Logger): A logger object for logging messages and errors in GUI.
    """
    #########################################################################################################################
    # standard code snippet that gets the parameters from the config file and checks if required data is present in data_dict
    # usually, this snippet is common for all the algorithms, so it is recommended to not remove it
    algo_name = inspect.stack()[0].function
    params = get_algo_params(cfg_dict, algo_type, algo_name, logger)
    
    # check if required data is present in data_dict
    for key in FUNCTION_NAME.required_data:
        if key not in data_dict:
            logger.log(f'{key} not found in data_dict', Logger.ERROR)
            return
    # standard code snippet ends here
    #########################################################################################################################
    # imports
    # import numpy as np
    # ...
    
    # your code
    # def add(a, b): return a + b
    # result = add(params['a'], params['b'])

    # add results to data_dict
    # data_dict[f'{algo_name}_result'] = result