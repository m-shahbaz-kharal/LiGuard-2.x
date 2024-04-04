from gui.logger_gui import Logger

def dummy(data_dict: dict, cfg_dict: dict):
    logger:Logger = data_dict['logger']

    # do stuff
    logger.log('[algo->pre.py->dummy]: A dummy function is called.', Logger.DEBUG)