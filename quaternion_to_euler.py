import numpy as np


class Euler():
    def quaternion_to_euler(self, Quaternion):
            w = Quaternion.w()
            x = Quaternion.x()
            y = Quaternion.y()
            z = Quaternion.z()
            
            ysqr = y * y

            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + ysqr)
            X = np.arctan2(t0, t1)

            t2 = +2.0 * (w * y - z * x)

            t2 = np.clip(t2, a_min=-1.0, a_max=1.0)
            Y = np.arcsin(t2)

            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (ysqr + z * z)
            Z = np.arctan2(t3, t4)

            return X, Y, Z

    def save_calib_rpy(self, X, Y, Z):
        pass