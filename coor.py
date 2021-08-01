import cv2
import numpy as np
import math
import time
import pickle
from dlt import DLT

class Points():
    def __init__(self, theta, phi, sqSize) -> None:
        self.theta = theta
        self.phi = phi
        self.sqSize = sqSize
        self.points = []
        
    def add(self, idx, u, v):
        self.points.append(Point(idx, self.theta, self.phi, self.sqSize, u, v))
        
    def getAllXYZ(self):
        return [point.getXYZ() for point in self.points]
    
    def getAllUV(self):
        return [point.getUV() for point in self.points]
    
    def getAll(self):
        """
        This will return array of coordinate\n
        [[[x1, y1, z1], [u1, v1]],\n
         [[x2, y2, z2], [u2, v2]],\n
          ...\n
         [[xn, yn, zn], [un, vn]]]\n
        """
        return [[point.getXYZ(), point.getUV()] for point in self.points]
        
class Point():
    def __init__(self, idx, theta, phi, sqSize, u, v) -> None:
        self.idx = idx
        self.theta = theta
        self.phi = phi
        self.sqSize = sqSize
        self.u = u
        self.v = v
        self.x = ((self.idx // 6) + 1) * self.sqSize
        self.y = ((self.idx % 6) + 1) * self.sqSize * math.cos(math.radians(self.theta))
        self.z = ((self.idx % 6) + 1) * self.sqSize * math.sin(math.radians(self.theta))
        self.rotate()
        
    def rotate(self):
        tx = self.x
        ty = self.y
        self.x = (tx * math.cos(math.radians(self.phi))) - (ty*math.sin(math.radians(self.phi)))
        self.y = (tx * math.sin(math.radians(self.phi))) + (ty*math.cos(math.radians(self.phi)))
    
    def getXYZ(self):
        return [self.x, self.y, self.z]
    
    def getUV(self):
        return [self.u, self.v]

def __findCorners(filename:str):
    image = cv2.imread(filename)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    success, corners = cv2.findChessboardCorners(gray, (6, 9), None)
    
    if success == True:
        return np.array(corners).reshape(54, 2).tolist()
    else:
        return None

def __findChessCoor(filename:str, theta:float, phi:float, sqSize:float):  
    uv_list = __findCorners(filename)
    points = Points(theta, phi, sqSize)
    if uv_list is not None:
        for idx, uv in enumerate(uv_list):
            u, v = uv
            points.add(idx, u, v)
    return points

def save_to_file(obj:DLT, filename:str):
    outfile = open(filename + '.dlt','wb')
    pickle.dump(obj,outfile)
    outfile.close()
    print("save the calibrated parameter to file: ", filename)
        
def load_from_file(filename:str):
    if not filename.endswith('.dlt'):
        raise ValueError("invalid filename DLT file should end with \".dlt\"")
    with open(filename,'rb') as param:
        obj = pickle.load(param)
    return obj

def EuclidDistance(xyz, uvw):
    return math.hypot(xyz[0]-uvw[0], xyz[1]-uvw[1], xyz[2]-uvw[2])
     
def findBestDLT(l_file:str, r_file:str, theta:float, phi:float, sqSize:float, silent:bool=False, num_iterate:int = 1500) -> DLT:
    if not silent:
        print("Finding L and R matrices...")
        print("With Error (mm)")
        
    files = [l_file, r_file]
    
    uvRL = []
    for file in files:
        points = __findChessCoor(file, theta, phi, sqSize)
        xyz = points.getAllXYZ()
        uv = points.getAllUV()
        uvRL.append(uv)
    
    uvRL = np.array(uvRL)
    xyz = np.array(xyz)
    
    num_calib_points = [8]
    lowest_mean_plus_std = np.Inf

    tStart = time.time()
    
    best_dlt = None
    
    for i, num_calib_point in enumerate(num_calib_points):
        for iterate in range(num_iterate):
            all_idx = np.array(range(0,54))
            rand_idx = np.random.choice(all_idx, size=num_calib_point, replace=False)
            remain_idx = np.delete(all_idx, rand_idx)
            
            dlt = DLT()
            dlt.xyz = xyz[rand_idx]
            dlt.uvL = uvRL[0, rand_idx]
            dlt.uvR = uvRL[1, rand_idx]
            dlt.calibrate()
            
            uds = []
            for idx in remain_idx:
                theory_coordinate = Point(idx, theta, phi, sqSize, -1, -1).getXYZ()
                dlt_coordinate = dlt.getXYZ(uvRL[0, idx], uvRL[1, idx])
                ud = EuclidDistance(dlt_coordinate, theory_coordinate)
                uds.append(ud)
            uds = np.array(uds)
            
            mean_plus_std = np.mean(uds) + np.std(uds)
            
            # check if mean+std of error are lowest and all of error are lower than 5% of tile size
            if mean_plus_std < lowest_mean_plus_std and np.all(uds < ((5*sqSize)/100)):
                lowest_mean_plus_std = mean_plus_std
                best_idx = rand_idx
                best_dlt = dlt
                best_uds = uds
                if not silent:
                    print(" "*100, end="\r")
                    print(f"Mean: {np.mean(uds)}, SD:{np.std(uds)}")
                    
            if not silent:
                percentComplete = ((iterate + 1 + (i * num_iterate)) * 100) / (num_iterate * len(num_calib_points))
                percentRemaining = 100 - percentComplete
                
                elapseTime = time.time() - tStart
                remainningTime = (elapseTime / percentComplete) * percentRemaining
                
                remain_minute = remainningTime // 60
                remain_sec = remainningTime % 60
                
                form = (percentComplete, remain_minute, remain_sec)
                print("%7.3f%% complete, remainning time: %2.0dm %2.0ds"%form, end="\r")
                
    if best_dlt is not None:   
        if not silent:          
            print(" "*100, end="\r")
            print("%7.3f%% complete, time used: %2.0dm %2.0ds" % (percentComplete, elapseTime // 60, elapseTime % 60))
            print("Best index :", best_idx)
            print("Best accaracy :", best_uds)
        return best_dlt
    
    if not silent:
        print("Failed to achieve the desire accuracy, retrying...")
    return findBestDLT(l_file, r_file, theta, phi, sqSize, silent)     

if __name__ == "__main__":
    theta = 60
    phi = 60.0
    sqSize = 22.0
    
    l_file = r"resource\IMG_3881_Moment.jpg"
    r_file = r"resource\MVI_2973_Moment.jpg"
    
    best_dlt = findBestDLT(l_file, r_file, theta, phi, sqSize, False, 2000)
    save_to_file(best_dlt, "param.dlt")
    
    ###########################------ Test ------###########################
    uvRL = []
    for file in [l_file, r_file]:
        points = __findChessCoor(file, theta, phi, sqSize)
        xyz = points.getAllXYZ()
        uv = points.getAllUV()
        uvRL.append(uv)

    uvL = np.array(uvRL[0])
    uvR = np.array(uvRL[1])
    
    p0 = Point(35, theta, phi, sqSize, -1, -1).getXYZ()
    p1 = Point(5, theta, phi, sqSize, -1, -1).getXYZ()
    print(EuclidDistance(p0, p1))
    
    p0 = best_dlt.getXYZ(uvL[35], uvR[35])
    p1 = best_dlt.getXYZ(uvL[5], uvR[5])
    print(EuclidDistance(p0, p1))
    
    save_to_file(best_dlt, "param.dlt")
    ########################################################################