import numpy as np
import pickle

def save_to_file(obj, filename):
    outfile = open(filename,'wb')
    pickle.dump(obj,outfile)
    outfile.close()
    print("save the calibrated parameter to file: ", filename)
        
def load_from_file(filename):
    infile = open(filename,'rb')
    obj = pickle.load(infile)
    infile.close()
    return obj

class DLT():
    '''
    Camera calibration by DLT using known object points and their image points.
    
    Input
    -----
    xyz: array-like object cantain coordinates in the object 3D space.
    uvL: array-like object cantain coordinates in the left-image 2D space.
    uvR: array-like object cantain coordinates in the right-image 2D space.
    
    There must be at least 6 calibration points for the 3D DLT.
    
    Output
    ------
     L: array of 11 parameters of the calibration matrix.
     R: array of 11 parameters of the calibration matrix.
    '''
    
    def __init__(self) -> None:
        self.L = None
        self.R = None

    def calibrate(self):
        if len(self.uvL) != len(self.uvR):
            raise ValueError("uvL and uvR must have the same dimensions")
        
        n = len(self.uvL)
        if (n < 6):
            raise ValueError(f'3D DLT requires at least 6 calibration points. Only {n} points were entered.')

        FL, FR = [], []
        gL, gR = [], []

        for i in range(n):
            x, y, z = self.xyz[i][0], self.xyz[i][1], self.xyz[i][2]
            
            uL, vL = self.uvL[i][0], self.uvL[i][1]
            FL.append([x, y, z, 1, 0, 0, 0, 0, -uL * x, -uL * y, -uL * z])
            FL.append([0, 0, 0, 0, x, y, z, 1, -vL * x, -vL * y, -vL * z])
            gL.extend([[uL], [vL]])

            uR, vR = self.uvR[i][0], self.uvR[i][1]
            FR.append([x, y, z, 1, 0, 0, 0, 0, -uR * x, -uR * y, -uR * z])
            FR.append([0, 0, 0, 0, x, y, z, 1, -vR * x, -vR * y, -vR * z])
            gR.extend([[uR], [vR]])

        FL = np.asarray(FL)
        gL = np.asarray(gL)
        self.L = np.dot(np.dot(np.linalg.pinv(np.dot(FL.T, FL)), FL.T), gL)
        
        FR = np.asarray(FR)
        gR = np.asarray(gR)
        self.R = np.dot(np.dot(np.linalg.pinv(np.dot(FR.T, FR)), FR.T), gR)

    def getXYZ(self, uvL, uvR):
        if self.L is None or self.R is None:
            raise ValueError("You need to calibrate first.")

        uL, vL = uvL
        uR, vR = uvR
        
        L = self.L.T[0]
        R = self.R.T[0]

        Q = [[L[0] - L[8]*uL, L[1] - L[9]*uL, L[2] - L[10]*uL],
             [L[4] - L[8]*vL, L[5] - L[9]*vL, L[6] - L[10]*vL],
             [R[0] - R[8]*uR, R[1] - R[9]*uR, R[2] - R[10]*uR],
             [R[4] - R[8]*vR, R[5] - R[9]*vR, R[6] - R[10]*vR]]

        q = [[uL - L[3]],
             [vL - L[7]],
             [uR - R[3]],
             [vR - R[7]]]

        Q = np.asarray(Q)
        q = np.asarray(q)

        return np.dot(np.dot(np.linalg.pinv(np.dot(Q.T, Q)), Q.T), q).T.tolist()[0]