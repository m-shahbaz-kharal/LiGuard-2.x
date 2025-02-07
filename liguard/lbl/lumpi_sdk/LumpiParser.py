import pandas as pd
import numpy as np
from tqdm import tqdm
import os
import json
import glob
from plyfile import PlyData
from liguard.lbl.lumpi_sdk.CameraViewer import CameraViewer
class Pose:
    def __init__(self,id,x,y,z,heading,l,w,h,shape,indices,time,classId=-1):
        self.position = np.asarray([x, y, z])
        self.heading = heading
        self.shape = shape
        self.dim = np.asarray([l, w, h]).astype(np.float64)
        self.time = time
        self.pc = []
        self.id = int(id)
        self.index = int(np.floor(float(time) * 10))
        self.indices = np.asarray(indices).astype(np.int64)
        self.classId = int(classId)
        self.u = [np.cos(self.heading), np.sin(self.heading), 0]
        self.corners = np.array([
        [-l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2],
        [w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2],
        [-h / 2, -h / 2, -h / 2, -h / 2, h / 2, h / 2, h / 2, h / 2]])
        self.R = np.array([[self.u[0], -self.u[1], 0], [self.u[1], self.u[0], 0], [0, 0, 1]])
        # Rotate the corners of the bounding box to align with the object's heading
        self.corners = np.dot(self.R, self.corners)
        eight_points = np.tile(self.position, (8, 1))
        self.corners = self.corners + eight_points.transpose()
    def interpolate(self, nextPose, time):
        """
        Interpolates the position, heading, and dimensions between the current pose and the next pose at a given time.

        Parameters:
        nextPose (Pose): The next pose to interpolate towards.
        # Interpolate heading and normalize it to be within the range [-pi, pi]
        heading = np.fmod(nextPose.heading * weight_next + self.heading * weight_current + np.pi, np.pi * 2) - np.pi

        Returns:
        tuple: A tuple containing the interpolated position, heading, and dimensions.
        """
        pose_time_difference = nextPose.time - self.time
        time_difference = time - self.time
        weight_next = time_difference / pose_time_difference
        weight_current = 1 - weight_next
        position = nextPose.position * weight_next + self.position * weight_current
        heading = np.fmod(nextPose.heading * weight_next + self.heading * weight_current + np.pi, np.pi * 2) - np.pi
        dim = self.dim * weight_current + nextPose.dim * weight_next
        return position,heading,dim
class LumpiParser:
    """
    LUMPI parser will parse the lumpi data
    Track format is :frame_number, id, bbox_left, bbox_top, bbox_width, bbox_height, score, class, 1, loc_x, loc_y, loc_z, dim_1, dim_2, dim_3, rotation_y, [shape,3](if classId =1),[indices]

    """
    helptext="Track format is :frame_number, id, bbox_left, bbox_top, bbox_width, bbox_height, score, class, 1, loc_x, loc_y, loc_z, dim_1, dim_2, dim_3, rotation_y, [shape,3](if classId =1),[indices]"
    def __init__(self, path):
        self.path=path
        self.tracks={}
        self.indexOrdered={}
        self.meta={}
        self.cameras=[]
        self.lidarPath=""
        self.point_cloud_files=[]
        self.pc=""
        self.read_meta(self.path)
        self.index=0
        """
        The Tracks will be stored in a dictonary first key is object id second key is point cloud index and the value is a Pose
        """
    def read_track(self, path):
        """
        Reads track data from a CSV file and populates the tracks dictionary.

        Parameters:
        path (str): The file path to the CSV file containing track data.
        """
        print("reading tracks")
        df=pd.read_csv(path)
        data = pd.DataFrame(df[list(df)[:-1]]).to_numpy().astype(np.float64)
        for i in tqdm(range(data.shape[0])):
            p=Pose(id=data[i,1],classId=data[i,7],time=data[i,0],x=data[i,9],y=data[i,10],z=data[i,11],heading=data[i,15],l=data[i,12],w=data[i,13],h=data[i,14],shape=[],indices=[])
            if p.id not in self.tracks:
                self.tracks[p.id]={}
            self.tracks[p.id][p.index]=p 
        for o in self.tracks:
            for pk in self.tracks[o]:
               index= self.tracks[o][pk].index
               if index not in self.indexOrdered:
                   self.indexOrdered[index]={}
               self.indexOrdered[index][o]=self.tracks[o][pk]   
               
    def read_meta(self,data_path):
        with open(os.path.join(data_path,"meta.json"), "r") as f:
            self.meta = json.load(f)   
            
    def read_point_cloud_file_list(self,exp_id):
        self.lidarPath=os.path.join(self.path,"Measurement"+str(exp_id),"lidar")
        self.point_cloud_files = glob.glob(os.path.join(self.lidarPath,"*.ply"))
        self.point_cloud_files.sort()
        
    def read_point_cloud(self,index):
        try:
            self.pc = PlyData.read(self.point_cloud_files[index])
            self.pc = self.pc['vertex']
            self.index=index
            return True
        except:
           print("File:\""+self.point_cloud_files[index]+ "\" could not be loaded, check ply file integrety") 
           return False
       
    def get_xyz(self):
        return pd.DataFrame(self.pc[["x","y","z"]]).to_numpy()
    
    def get_points_meta(self):
        return pd.DataFrame(self.pc[["id","ray","azimuth","distance"]]).to_numpy()
    
    def get_bounding_boxes_at(self,time):
        index=int(np.floor(time*10))
        if index not in self.indexOrdered:
            return []
        obs=self.indexOrdered[index]
        boxes={}
        for o in obs:
            if index+1 in self.tracks[o]:
                nextp=self.tracks[o][index+1]
                p,h,d=obs[o].interpolate(nextp,time)
                boxes[o]=(p,h, d)
        return boxes 
    
    def read_all_cameras(self,exp_id,mask_flag):
         for s in self.meta["session"]:
            if self.meta["session"][s]["experimentId"]!=exp_id:
               continue
            if self.meta["session"][s]["type"]=="lidar":
                continue
            data_path=os.path.join(self.path,"Measurement"+str(exp_id))
            try:
                self.cameras.append(CameraViewer(data_path=data_path,meta=self.meta,session=s,mask_flag=mask_flag))
            except:
                print("Could not load camera with session: "+s)