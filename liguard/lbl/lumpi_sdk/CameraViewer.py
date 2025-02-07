import numpy as np
import cv2
import os
class CameraViewer:
    def __init__(self, data_path, meta, session, mask_flag):
        print("loading camera "+str(session))
        try:
            self.rvec=np.array(meta["session"][session]['rvec'])
            self.intrinsic=np.array(meta["session"][session]['intrinsic'])
            self.tvec=np.array(meta["session"][session]['tvec'])
            self.inverse_extrinsic=np.array(meta["session"][session]['extrinsic'])
            self.extrinsic=np.linalg.inv(self.inverse_extrinsic)
            self.distortion=np.array(meta["session"][session]['distortion'])
            self.fps=meta["session"][session]['fps']
            self.decive_id=meta["session"][session]['deviceId']
            self.img_path=os.path.join(data_path,"cam",str(meta["session"][session]["deviceId"]),"video.mp4")
            self.mask_path=os.path.join(data_path,"cam",str(meta["session"][session]["deviceId"]),"mask.mp4")
        except:
            print("Camera meta "+str(session)+" not found")
        self.frame_number=0
        self.size= np.array([0, 0])
        self.img= np.array([0, 0])
        try:
            self.img_capture=cv2.VideoCapture(self.img_path)
            self.img_capture.set(cv2.CAP_PROP_POS_FRAMES, int(self.frame_number))
            ret, self.img = self.img_capture.read()
            self.size=self.img.shape
            self.last_frame_number=self.img_capture.get(cv2.CAP_PROP_BUFFERSIZE)
        except:
            print("Camera "+str(session)+" video not found")
        self.mask=[]
        self.mask_capture=[]  
        if mask_flag:
            try:
                self.mask_capture=cv2.VideoCapture(self.mask_path)
                ret, self.mask = self.mask_capture.read()
                self.mask_capture.set(cv2.CAP_PROP_POS_FRAMES, int(self.frame_number))
                if self.size[0]!=self.mask.shape[0] or self.size[1]!=self.mask.shape[1]:
                    self.size=self.mask.shape
            except:
                print("Camera "+str(session)+" mask not found")
        self.lines= [[0, 1], [1, 2], [2, 3], [0, 3],
        [4, 5], [5, 6], [6, 7], [4, 7],
        [0, 4], [1, 5], [2, 6], [3, 7]]
        self.fov_x =1
        self.fov_y =1
        self.planes={}
        self.transformed_planes = {}
        try:
            self.calculate_fov()
            self.get_frustum_planes(1,200)
        except:
            print("Could not calculate frustum")
       
    def calculate_fov(self):
        self.fov_x = 2 * np.degrees(np.arctan2(self.size[1] , (2 * self.intrinsic[0, 0])))
        self.fov_y = 2 * np.degrees(np.arctan2(self.size[0] , (2 * self.intrinsic[1, 1])))
    def get_frustum_planes(self, near, far):
        tan_fov_y = np.tan(np.radians(self.fov_y) / 2)
        tan_fov_x = np.tan(np.radians(self.fov_x) / 2)
        nh = tan_fov_y * near
        nw = tan_fov_x*near
        self.planes = {
            "near": np.array([0, 0, 1, -near]),
            "far": np.array([0, 0, -1, far]),
            "left": np.array([1, 0, nw / near, 0]),
            "right": np.array([-1, 0, nw / near, 0]),
            "top": np.array([0, 1, nh / near, 0]),
            "bottom": np.array([0, -1, nh / near, 0])
        }
        for key, plane in self.planes.items():
            plane[:3]=plane[:3]/np.linalg.norm(plane[:3])
            normal = self.inverse_extrinsic[:3, :3] @ plane[:3]
            d = plane[3] + np.dot(self.inverse_extrinsic[:3, 3], normal)
            self.transformed_planes[key] = np.append(normal, d)
    def segment_plane_intersection(self,p1, p2, plane):
        segment_dir = p2 - p1
        numerator = np.dot(plane[:3], p1) + plane[3]  # a*x1 + b*y1 + c*z1 + d
        denominator = np.dot(plane[:3], segment_dir)  # a*(x2-x1) + b*(y2-y1) + c*(z2-z1)
        po=[0,0,0]
        if(  np.fabs(plane[0])>0):
            po[0]=-plane[3]/plane[0]
        elif( np.fabs(plane[1])>0):
            po[1]=-plane[3]/plane[1]
        elif(np.fabs(plane[2])>0):
            po[2]=-plane[3]/plane[2]
        numerator = np.dot(po-p1, plane[:3])  
        if denominator == 0:
        
            return []  
        t = -numerator / denominator
        if t < 0 or t > 1:
            return [] 
        return  p1 + t * segment_dir
    def point_in_frustum(self,point):
        for key, plane in self.planes.items():
            if np.dot(plane[:3], point[:3]) + plane[3] <0:
                return False
        return True  
    
    def box_culling(self,pts):
        valid_lines=[]
        lines=[]
        for l in self.lines:
            lines.append([pts[:,l[0]],pts[:,l[1]]])
        for l in lines:
            v1=self.point_in_frustum(l[0])
            v2=self.point_in_frustum(l[1])
            if not v1 and not v2:
                continue
            if(v1 and v2):
                valid_lines.append(l)
            for p in self.planes.values():
                    tmp=self.segment_plane_intersection(p1=l[0],p2=l[1],plane=p)
                    if (not v1 and len(tmp)>0 ):
                        l[0]=tmp
                        if(self.point_in_frustum(l[0])):
                            break
                    elif(not v2 and len(tmp)>0 ):
                        l[1]=tmp
                        if(self.point_in_frustum(l[1])):
                            break
            valid_lines.append(l)
        return valid_lines        
    def project_pts(self,pts):
        if(len(pts)<1):
            return []
        return  cv2.projectPoints(pts, self.rvec, self.tvec, self.intrinsic,self.distortion)[0][:, 0, :].astype(int)
    def project_line(self,pts):
        return  cv2.projectPoints(pts, np.zeros((1,3)),np.zeros((1,3)), self.intrinsic,self.distortion)[0][:, 0, :].astype(int)
    def project_box(self,position,heading,dim):
        u=[np.cos(heading), np.sin(heading),0]
        l,w,h=dim
        corners=np.array([
        [-l/2, -l/2, l/2, l/2, -l/2, -l/2, l/2, l/2],
        [w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2],
        [-h/2, -h/2, -h/2, -h/2, h/2, h/2, h/2, h/2]])
        R=np.array([[u[0], -u[1],0],[u[1], u[0],0],[0,0,1]])
        corners=np.dot(R,corners)
        eight_points = np.tile(position, (8, 1))
        corners=corners+eight_points.transpose()
        corners= np.append(corners, np.ones((1,corners.shape[1])),axis=0)
        corners=self.extrinsic.dot(corners)
        lines=self.box_culling(np.asarray(corners)[:3,:])
        imgLines=[]
        for l in lines:
           imgLines.append(self.project_line(np.array(l).transpose()))
        return imgLines
        
    def filter_invalid_pts(self,pts):
         if(len(pts)<1):
            return []
         id=np.where((pts[:,0]< self.size[1]) &(pts[:, 0] >0)&(pts[:,1]< self.size[0]) &(pts[:, 1] >0))
         return pts[id[0],:]
    def set_frame(self, frame_number, mask_flag):
        self.frame_number=frame_number
        self.img_capture.set(cv2.CAP_PROP_POS_FRAMES, int(self.frame_number))
        ret, self.img = self.img_capture.read()
        if(mask_flag ):
            self.mask_capture.set(cv2.CAP_PROP_POS_FRAMES, int(self.frame_number))
            ret, self.mask = self.mask_capture.read()
            if(self.img is None):
                self.img=self.mask
            elif(self.mask is not None):
                mask=np.sum(self.mask,axis=2)   
                id=np.where(mask>0)
                self.img[id[0],id[1],:]=self.img[id[0],id[1],:]*0.5+0.5*self.mask[id[0],id[1],:]
    def set_frame_to_point_cloud_index(self,index):
           self.set_frame(int(index/10.*self.fps))
           cv2.putText(img=self.img, text="frame: "+str(self.frame_number)+" time: "+"{:.2f}".format(self.frame_number/self.fps), org=(10, 50), fontFace=1, fontScale=1.4, color=[0, 255, 0], thickness=2)
    def plot_frame(self,wait_time_ms):
        if(self.img is None):
            print("empty Image"+str(self.frame_number))
            return
        cv2.imshow("LUMPI CAM"+str(self.decive_id),self.img )
        key = cv2.waitKey(wait_time_ms)
        return key
    def plot_point_cloud(self,pc,color):
        if(len(pc)<1):
            return
        pc=self.project_pts(pc)
        tmp=self.filter_invalid_pts(pc)
        for i in range(tmp.shape[0]):
            cv2.circle(self.img, (tmp[i,0], tmp[i, 1]), 4, color,-1)
    def plot_bounding_boxes_3D(self,boxes,color): 
        for o in boxes:
            p,h,d=boxes[o]
            lines=self.project_box(p,h,d)
            if len(lines)<1:
                continue
            cv2.putText(self.img,str(o),lines[0][0],1,1,[0,0,0],1)
            for l in lines:
                cv2.line(self.img,l[0],l[1],color,2) 