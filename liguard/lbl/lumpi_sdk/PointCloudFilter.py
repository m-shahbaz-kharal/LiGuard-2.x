import numpy as np
import os
import glob
import csv
class PointCloudFilter:
    def __init__(self):
        self.background_paths={}
        self.background_data={}
        self.metadata={}
        self.meter_to_distance=1
    def read_background(self, path):
        with open(os.path.join(path,"meta_background.txt"), 'r') as csvfile:
            reader = csv.reader(csvfile)
            for i, row in enumerate(reader):
                if(row[0]=="azimuth normalizer"):
                    self.metadata[row[1]]=float(row[2])
                if(row[0]=="meter to distance"):
                    self.meter_to_distance=float(row[1])
        files = glob.glob(os.path.join(path,"*.npy"))
        for f in files:
          file_id=f.split(os.sep)[-1].split(".")[0]
          self.background_data[file_id]=np.load(f)
          self.background_paths[file_id]=np.load(f)
          self.background_data[file_id] -= self.meter_to_distance * 0.1  # add distance to background 0.1m
    def filter_background(self,pc):
        f=[]
        b=[]
        for i in range(pc.shape[0]):
            pc_id = str(pc[i, 0])
            pc_y = int(pc[i, 1])
            pc_z = int(pc[i, 2] * self.metadata[pc_id])
            background_value = self.background_data[pc_id][pc_y, pc_z]
            if pc[i, 3] < background_value:
                f.append(i)
            else:
                b.append(i)
        return f, b
    
    def filter_points_by_time(self, pc, min_time, max_time):
        min_time = min_time * 1000000
        max_time = max_time * 1000000
        return np.where((pc["time"] < max_time) & (pc["time"] > min_time))[0]
    def filter_points_by_time(self, pc, minTime,maxTime):
        minTime=minTime*1000000
        maxTime=maxTime*1000000
        return np.where((pc["time"]<maxTime) & (pc["time"]>minTime))[0]
