from gui.logger_gui import Logger

def dummy(data_dict: dict, cfg_dict: dict):
    logger:Logger = data_dict['logger']

    if 'current_calib_path' not in data_dict:
        logger.log('[algo->calib.py->dummy]: current_calib_path not found in data_dict', Logger.DEBUG)
    
    if 'current_calib_data' not in data_dict:
        logger.log('[algo->calib.py->dummy]: current_calib_data not found in data_dict', Logger.ERROR)
        return
    
    current_calib_path = data_dict['current_calib_path']
    current_calib_data = data_dict['current_calib_data']

    # do ops on current_calib_data as per requirement