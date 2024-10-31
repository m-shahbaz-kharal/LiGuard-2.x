import inspect
from gui.config_gui import get_abs_path
from gui.logger_gui import Logger
from algo.utils import AlgoType, make_key, get_algo_params
algo_type = AlgoType.calib

def dummy(data_dict: dict, cfg_dict: dict, logger: Logger):
    """
    Write a description of the function here.

    Args:
        data_dict (dict): The dictionary containing the data.
        cfg_dict (dict): The dictionary containing the configuration data.
        logger (gui.logger_gui.Logger): A logger object for logging messages and errors in GUI.
    """
    # get name and params
    algo_name = inspect.stack()[0].function
    params = get_algo_params(cfg_dict, algo_type, algo_name, logger)

    # do stuff
    # ...
    test_key = make_key(algo_name, 'test')

    # logging any operation performed
    logger.log('Dummy operation performed', Logger.INFO)
    
    # update the data_dict
    data_dict[test_key] = 'test'