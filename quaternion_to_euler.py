import numpy as np

import yaml

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

    def save_calib_quaternion(self, info, Quaternion, calib):
        npzfile = np.load(f'./{info.path}/{info.files[0]}')
        resolution = npzfile['arr_3']
        rostopic = npzfile['arr_4']

        calib_data = {
            'calib' : [
                {
                    'calib_pose' : [f'{calib.t()}']
                },
                {
                    'calib_quaternion' : [f'{Quaternion.x()}, {Quaternion.y()}, {Quaternion.z()}, {Quaternion.w()}']
                },
                {
                    'resolution' : f'{resolution[0][0], resolution[1][0]}'
                },
                {
                    'ros_topic' : f'{rostopic}'
                }
            ]
        }


        file_name = 'calib_quaternion'
        load = yaml.safe_load(f'./unit_test/{file_name}.yaml')

        with open(load, 'w') as outfile:
            yaml.dump(calib_data, outfile)

    def save_calib_rpy(self, info, X, Y, Z, calib):
        npzfile = np.load(f'./{info.path}/{info.files[0]}')
        resolution = npzfile['arr_3']
        rostopic = npzfile['arr_4']

        calib_data = {
            'calib' : [
                {
                    'calib_pose' : [f'{calib.t()}']
                },
                {
                    'calib_rpy' : [f'{X}, {Y}, {Z}']
                },
                {
                    'resolution' : f'{resolution[0][0], resolution[1][0]}'
                },
                {
                    'ros_topic' : f'{rostopic}'
                }
            ]
        }


        file_name = 'calib_rpy'
        load = yaml.safe_load(f'./unit_test/{file_name}.yaml')

        with open(load, 'w') as outfile:
            yaml.dump(calib_data, outfile)