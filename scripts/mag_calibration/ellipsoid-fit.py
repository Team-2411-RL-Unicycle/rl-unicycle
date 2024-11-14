import sys
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from rluni.icm20948.imu_lib import ICM20948
import importlib.resources as pkg_resources
from scipy import linalg

from threading import Thread
import keyboard  

class Magnetometer(object):
    ''' Magnetometer class with calibration capabilities.

        Parameters
        ----------

        F : float (optional)
            Expected earth magnetic field intensity, default=1.
    '''

    def __init__(self, F=53.):
        with pkg_resources.path('rluni.configs.imu', 'default.yaml') as config_file_path:
            imu_config_path = str(config_file_path)

        self.imu = ICM20948(config_file=imu_config_path)   

        # initialize values
        self.F   = F
        self.b   = np.zeros([3, 1])
        self.A_1 = np.eye(3)

    def read(self):
        ''' Get a sample.

            Returns
            -------
            s : list
                The sample in uT, [x,y,z] (corrected if performed calibration).
        '''
        
        mx, my, mz, flag = self.imu.read_magnetometer()
        
        s = np.array([mx,my,mz]).reshape(3, 1)
        s = np.dot(self.A_1, s - self.b)
        return [s[0,0], s[1,0], s[2,0]]

    def calibrate(self):
        ''' Performs calibration. '''

        print('Collecting samples (Ctrl-C to stop and perform calibration)')

        try:
            s = []
            n = 0
            while True:
                mx, my, mz, flag = self.imu.read_magnetometer()
                s.append([mx, my, mz])
                n += 1
                sys.stdout.write('\rTotal: %d' % n)
                sys.stdout.flush()
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            pass

        # ellipsoid fit
        s = np.array(s).T
        M, n, d = self.__ellipsoid_fit(s)

        # calibration parameters
        # note: some implementations of sqrtm return complex type, taking real
        M_1 = linalg.inv(M)
        self.b = -np.dot(M_1, n)
        self.A_1 = np.real(self.F / np.sqrt(np.dot(n.T, np.dot(M_1, n)) - d) *
                           linalg.sqrtm(M))

    def __ellipsoid_fit(self, s):
        ''' Estimate ellipsoid parameters from a set of points.

            Parameters
            ----------
            s : array_like
              The samples (M,N) where M=3 (x,y,z) and N=number of samples.

            Returns
            -------
            M, n, d : array_like, array_like, float
              The ellipsoid parameters M, n, d.
            
            Fits to the equation h_m^T M h_m + h_m^T n + d = 0, where h_m = [mx, my, mz] magnetic readings
            h_true = A^(h_m - b) is the true magnetic field
            
            Soft Iron: A = [F / sqrt(n^T M^-1 n - d)] * sqrtm(M)
            Hard Iron: b = -M^-1 n

            References
            ----------
            .. [1] Qingde Li; Griffiths, J.G., "Least squares ellipsoid specific
               fitting," in Geometric Modeling and Processing, 2004.
               Proceedings, vol., no., pp.335-340, 2004
        '''

        # D (samples)
        D = np.array([s[0]**2., s[1]**2., s[2]**2.,
                      2.*s[1]*s[2], 2.*s[0]*s[2], 2.*s[0]*s[1],
                      2.*s[0], 2.*s[1], 2.*s[2], np.ones_like(s[0])])

        # S, S_11, S_12, S_21, S_22 (eq. 11)
        S = np.dot(D, D.T)
        S_11 = S[:6,:6]
        S_12 = S[:6,6:]
        S_21 = S[6:,:6]
        S_22 = S[6:,6:]

        # C (Eq. 8, k=4)
        C = np.array([[-1,  1,  1,  0,  0,  0],
                      [ 1, -1,  1,  0,  0,  0],
                      [ 1,  1, -1,  0,  0,  0],
                      [ 0,  0,  0, -4,  0,  0],
                      [ 0,  0,  0,  0, -4,  0],
                      [ 0,  0,  0,  0,  0, -4]])

        # v_1 (eq. 15, solution)
        E = np.dot(linalg.inv(C),
                   S_11 - np.dot(S_12, np.dot(linalg.inv(S_22), S_21)))

        E_w, E_v = np.linalg.eig(E)

        v_1 = E_v[:, np.argmax(E_w)]
        if v_1[0] < 0: v_1 = -v_1

        # v_2 (eq. 13, solution)
        v_2 = np.dot(np.dot(-np.linalg.inv(S_22), S_21), v_1)
        
        a,b,c,f,g,h = v_1
        p,q,r,d = v_2

        # quadric-form parameters
        M = np.array([[a, h, g],
                      [h, b, f],
                      [g, f, c]])
        n = np.array([[p],
                      [q],
                      [r]])

        return M, n, d
    
if __name__ == "__main__":
    try:
        # F in Vancouver, Canada is about 53uT
        mag = Magnetometer(F=53.)
        mag.calibrate()
        while True:
            print(mag.read())
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass