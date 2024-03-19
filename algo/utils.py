def gather_point_clouds(data_dict: dict, cfg_dict: dict, key: str, count: int, global_index_key: str = None):
    gathering_not_started = key not in data_dict
    if gathering_not_started: data_dict[key] = []
    
    if global_index_key == None: global_index_key = f'{key}_gathered_frames_indices'
    if global_index_key not in data_dict: data_dict[global_index_key] = []
    
    gathering_completed = len(data_dict[key]) >= count
    point_cloud_is_present = 'current_point_cloud_numpy' in data_dict
    point_cloud_is_novel = data_dict['current_frame_index'] not in data_dict[global_index_key]
    
    if not gathering_completed and point_cloud_is_present and point_cloud_is_novel:
        data_dict[key].append(data_dict['current_point_cloud_numpy'])
        data_dict[global_index_key].append(data_dict['current_frame_index'])
    
    gathering_completed = len(data_dict[key]) >= count
    return gathering_completed

def combine_gathers(data_dict: dict, cfg_dict: dict, key:str, gather_keys: list):
    combining_not_started = key not in data_dict
    if combining_not_started:
        data_dict[key] = []
        for gather_key in gather_keys: data_dict[key].extend(data_dict[gather_key])
    
def skip_frames(data_dict: dict, cfg_dict: dict, key: str, skip: int, global_index_key: str = None):
    skipping_not_started = key not in data_dict
    if skipping_not_started: data_dict[key] = 0
    
    if global_index_key == None: global_index_key = f'{key}_skipped_frames_indices'
    if global_index_key not in data_dict: data_dict[global_index_key] = []
    
    skipping_completed = data_dict[key] >= skip
    point_cloud_is_present = 'current_point_cloud_numpy' in data_dict
    point_cloud_is_novel = data_dict['current_frame_index'] not in data_dict[global_index_key]
    
    if not skipping_completed and point_cloud_is_present and point_cloud_is_novel:
        data_dict[key] += 1
        data_dict[global_index_key].append(data_dict['current_frame_index'])
        
    skipping_completed = data_dict[key] >= skip 
    return skipping_completed