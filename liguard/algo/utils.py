from liguard.gui.logger_gui import Logger
from enum import Enum, auto

class AlgoType(Enum):
    """
    An enumeration of the categories of algorithms.
    """
    pre = auto()
    lidar = auto()
    camera = auto()
    calib = auto()
    label = auto()
    post = auto()

def algo_func(required_data: list = []):
    """
    A decorator to specify the required data for an algorithm.
    """
    def decorator(func):
        func.required_data = required_data
        return func
    return decorator

def make_key(algo_name: str, key: str) -> str:
    """
    Creates a standard key for the data dictionary used in LiGuard.
    
    Args:
        algo_name (str): The name of the algorithm.
        key (str): The key to create.
    
    Returns:
        str: The created key.
    """
    return f'{algo_name}_{key}'

def get_algo_params(cfg_dict: dict, algo_type: AlgoType, algo_name: str, logger: Logger) -> dict:
    """
    Gets the configuration parameters for the specified algorithm.
    
    Args:
        cfg_dict (dict): The dictionary containing the configuration data.
        algo_type (AlgoType): The category of the algorithm.
        algo_name (str): The name of the algorithm.
        logger (Logger): The logger object for logging messages.
    
    Returns:
        dict: The configuration parameters for the specified algorithm.
    """
    if algo_name not in cfg_dict['proc'][algo_type.name]:
        logger.log(f'"{algo_name}" not found in cfg_dict["proc"]["{algo_type.name}"]', Logger.ERROR)
        return {}
    return cfg_dict['proc'][algo_type.name][algo_name]

def gather_point_clouds(data_dict: dict, cfg_dict: dict, key: str, count: int, global_index_key: str = None):
    """
    Gathers point clouds until a specified count is reached.
    
    Args:
        data_dict (dict): The dictionary containing the data.
        cfg_dict (dict): The dictionary containing the configuration data.
        key (str): The key to store the gathered point clouds in the data dictionary.
        count (int): The desired count of point clouds to gather.
        global_index_key (str, optional): The key to store the indices of the gathered frames in the data dictionary. Defaults to None.
    
    Returns:
        bool: True if the gathering is completed, False otherwise.
    """
    logger: Logger = data_dict['logger']
    
    gathering_not_started = key not in data_dict
    if gathering_not_started:
        data_dict[key] = []
        logger.log(f'Gathering {count} point clouds into data_dict["{key}"]', Logger.INFO)
    
    if global_index_key is None:
        global_index_key = f'{key}_gathered_frames_indices'
    if global_index_key not in data_dict:
        data_dict[global_index_key] = []
    
    # Check if gathering is completed
    gathering_completed = len(data_dict[key]) >= count
    point_cloud_is_present = 'current_point_cloud_numpy' in data_dict
    point_cloud_is_novel = data_dict['current_frame_index'] not in data_dict[global_index_key]
    
    # Gather the point cloud if gathering is not completed and the point cloud is present and novel
    if not gathering_completed and point_cloud_is_present and point_cloud_is_novel:
        data_dict[key].append(data_dict['current_point_cloud_numpy'])
        data_dict[global_index_key].append(data_dict['current_frame_index'])
    
    gathering_completed = len(data_dict[key]) >= count
    return gathering_completed


def combine_gathers(data_dict: dict, cfg_dict: dict, key: str, gather_keys: list):
    """
    Combines multiple gathers into a single gather.
    
    Args:
        data_dict (dict): The dictionary containing the data.
        cfg_dict (dict): The dictionary containing the configuration data.
        key (str): The key to store the combined gather in the data dictionary.
        gather_keys (list): The keys of the gathers to combine.
    """
    # Get logger object from data_dict
    logger: Logger = data_dict['logger']

    # Check if combining has started
    combining_not_started = key not in data_dict
    if combining_not_started:
        data_dict[key] = []
        logger.log(f'Combining {len(gather_keys)} gathers into data_dict["{key}"]', Logger.INFO)
        for gather_key in gather_keys:
            data_dict[key].extend(data_dict[gather_key])


def skip_frames(data_dict: dict, cfg_dict: dict, key: str, skip: int, global_index_key: str = None):
    """
    Skips frames until a specified count is reached.
    
    Args:
        data_dict (dict): The dictionary containing the data.
        cfg_dict (dict): The dictionary containing the configuration data.
        key (str): The key to store the skipped frames count in the data dictionary.
        skip (int): The desired count of frames to skip.
        global_index_key (str, optional): The key to store the indices of the skipped frames in the data dictionary. Defaults to None.
    
    Returns:
        bool: True if the skipping is completed, False otherwise.
    """
    # Get logger object from data_dict
    logger: Logger = data_dict['logger']

    # Check if skipping has started
    skipping_not_started = key not in data_dict
    if skipping_not_started:
        data_dict[key] = 0
        logger.log(f'Skipping {skip} frames', Logger.INFO)
    
    if global_index_key is None:
        global_index_key = f'{key}_skipped_frames_indices'
    if global_index_key not in data_dict:
        data_dict[global_index_key] = []
    
    # Check if skipping is completed
    skipping_completed = data_dict[key] >= skip
    point_cloud_is_present = 'current_point_cloud_numpy' in data_dict
    point_cloud_is_novel = data_dict['current_frame_index'] not in data_dict[global_index_key]
    
    # Skip the frame if skipping is not completed and the point cloud is present and novel
    if not skipping_completed and point_cloud_is_present and point_cloud_is_novel:
        data_dict[key] += 1
        data_dict[global_index_key].append(data_dict['current_frame_index'])
        
    skipping_completed = data_dict[key] >= skip
    return skipping_completed