def gather_point_clouds(data_dict: dict, cfg_dict: dict, key: str, count: int):
    gathering_not_started = key not in data_dict
    gathering_not_completed = len(data_dict[key]) < count
    novel_point_cloud_is_present = 'current_point_cloud_numpy' in data_dict and data_dict['current_point_cloud_numpy'] not in data_dict[key]
    
    if gathering_not_started: data_dict[key] = []
    elif gathering_not_completed and novel_point_cloud_is_present: data_dict[key].append(data_dict['current_point_cloud_numpy'])
    return len(data_dict[key]) == count

def combine_gathers(data_dict: dict, cfg_dict: dict, key:str, gather_keys: list):
    combining_not_started = key not in data_dict
    if combining_not_started:
        data_dict[key] = []
        for gather_key in gather_keys: data_dict[key].extend(data_dict[gather_key])
    

def skip_frames(data_dict: dict, cfg_dict: dict, key: str, skip: int):
    skipping_not_started = key not in data_dict
    skipping_not_completed = len(data_dict[key]) < skip
    novel_point_cloud_is_present = 'current_point_cloud_numpy' in data_dict and data_dict['current_point_cloud_numpy'] not in data_dict[key]
    
    if skipping_not_started: data_dict[key] = 0
    elif skipping_not_completed and novel_point_cloud_is_present: data_dict[key] += 1
    return data_dict[key] == skip