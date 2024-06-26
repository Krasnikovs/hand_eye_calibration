import numpy as np
import g2o
import cv2

import argparse

from gui import Gui, InteractiveGui
from quaternion_to_euler import Euler

import os

import yaml

from tkinter import *

parser = argparse.ArgumentParser()
parser.add_argument(
    "--noise",
    dest="pixel_noise",
    type=float,
    default=1.0,
    help="noise in image pixel space (default: 1.0)",
)
parser.add_argument(
    "--outlier",
    dest="outlier_ratio",
    type=float,
    default=0.0,
    help="probability of spuroius observation  (default: 0.0)",
)
parser.add_argument(
    "--robust", dest="robust_kernel", action="store_true", help="use robust kernel"
)
parser.add_argument("--dense", action="store_true", help="use dense solver")
parser.add_argument("--seed", type=int, help="random seed", default=0)
args = parser.parse_args()

        
class Optimizer():

    def __init__(self, files, path):
        self.path = path
        self.files = files
        

    def show_image(self, current_estimated_pixel, gui):

        for i, file in enumerate(self.files):
            npzfile = (np.load(f'./{self.path}/{file}'))
            image = npzfile['arr_2']
            gui.image_visualization(current_estimated_pixel[i], image)
            

    def save_yaml(self, calib, world_base_pose):
        npzfile = np.load(f'./{self.path}/{self.files[0]}')
        resolution = npzfile['arr_3']
        rostopic = npzfile['arr_4']

        calib_data = {
            'calib' : [
                {
                    'calib_matrix' : [f'{calib._pose[0]}', f'{calib._pose[1]}', f'{calib._pose[2]}', f'{calib._pose[3]}']
                },
                {
                    'resolution' : f'{resolution[0][0], resolution[1][0]}'
                },
                {
                    'ros_topic' : f'{rostopic}'
                }
            ],
            'base_pose' : [
                {
                    'base_matrix' : [f'{world_base_pose._pose[0]}', f'{world_base_pose._pose[1]}', f'{world_base_pose._pose[2]}', f'{world_base_pose._pose[3]}']
                }
            ]
        }


        file_name = 'calib'
        load = yaml.safe_load(f'./unit_test/{file_name}.yaml')

        with open(load, 'w') as outfile:
            yaml.dump(calib_data, outfile)

    def debug(self, camera_robot_vertex, current_estimated_pixel, camera_extrs, measured_tool0_extrs):
        gui = Gui()

        gui.draw_transform(gui.world_id())
        
        calib = gui.transform(camera_robot_vertex.estimate().matrix())
        print(calib)
        
        
        for est, camera_extr, measured_tool0_extr in zip(current_estimated_pixel[:2], camera_extrs[:2], measured_tool0_extrs[:2]):
            measured_tool0_pose = gui.transform(measured_tool0_extr.matrix()).inv()
            optimized_camera_pose = gui.transform(camera_extr.estimate().matrix()).inv()

            world_base_pose = optimized_camera_pose @ calib.inv() @ measured_tool0_pose.inv()
            world_robot_pose = world_base_pose @ measured_tool0_pose
            
            gui.draw_transform(optimized_camera_pose, c_i=0) # red
            
            gui.draw_transform(world_robot_pose, c_i=2) # blue
            
        gui.draw_transform(world_base_pose, c_i=1) # green

        self.show_image(current_estimated_pixel, gui)

        return calib, world_base_pose
    
    def euler_yaml(self, calib, camera_robot_vertex):
        euler = Euler()
        X, Y, Z = euler.quaternion_to_euler(camera_robot_vertex.estimate().orientation())
        euler.save_calib_rpy(self, X, Z, Y, calib)


    def yaml_info(self, camera_robot_vertex, camera_extrs, measured_tool0_extrs):
        gui = Gui()

        calib = gui.transform(camera_robot_vertex.estimate().matrix())
        print(calib)
        
        measured_tool0_pose = gui.transform(measured_tool0_extrs[0].matrix()).inv()
        optimized_camera_pose = gui.transform(camera_extrs[0].estimate().matrix()).inv()

        world_base_pose = optimized_camera_pose @ calib.inv() @ measured_tool0_pose.inv()

        return calib, world_base_pose

    def optimize(self, answer):
        npzfile = []

        for i, file in enumerate(self.files):
            npzfile.append(np.load(f'./{self.path}/{file}'))
            if len(npzfile[i]._files) != 5: 
                print('File doesnt contain needed material')
                exit()


        
        q = []
        t = []
        images = []
        extr_estimate_from_robot = []

        for i, file in enumerate(self.files):
            npzfile.append(np.load(f'./{self.path}/{file}'))
            q.append([npzfile[i]['arr_1']])
            t.append(np.array([npzfile[i]['arr_0']]))
            images.append(npzfile[i]['arr_2'])
            

        for i in range(len(self.files)):
            quaternion = g2o.Quaternion(q[i][0][3], q[i][0][0], q[i][0][1], q[i][0][2])
            extr_estimate_from_robot.append(g2o.SE3Quat(q = quaternion, t = t[i][0]))

        chesboard_point_world_coordinates = (np.mgrid[0:8,0:5,0:1] * 0.02).transpose(1,2,3,0).transpose(1,0,2,3).reshape(-1,3)

        camera_pose_collection = []
        current_estimated_pixel = []
        for i in range(len(self.files)):
            current_estimated_pixel.append([])
        point_id = 40

        optimizer = g2o.SparseOptimizer()
        solver = g2o.BlockSolverSE3(g2o.LinearSolverEigenSE3())
        solver = g2o.OptimizationAlgorithmLevenberg(solver)
        optimizer.set_algorithm(solver)

        camera_projection_model = g2o.CameraParameters(613, (321, 241), 0)
        camera_projection_model.set_id(0)
        optimizer.add_parameter(camera_projection_model)

        chessboard_corner_vertices = []

        for i, point in enumerate(chesboard_point_world_coordinates):
            chessboard_corner_vertex = g2o.VertexPointXYZ()
            chessboard_corner_vertex.set_id(i + len(extr_estimate_from_robot) * 2)
            chessboard_corner_vertex.set_fixed(True)
            chessboard_corner_vertex.set_estimate(point)
            chessboard_corner_vertices.append(chessboard_corner_vertex)
            optimizer.add_vertex(chessboard_corner_vertex)

        for i, pose in enumerate(extr_estimate_from_robot):            

            camera_pose = g2o.VertexSE3Expmap()
            camera_pose.set_id(i)
            camera_pose.set_estimate(pose)
            optimizer.add_vertex(camera_pose)
            camera_pose_collection.append(camera_pose)
            

            ret, chessboard_pixels_image_coordinates = cv2.findChessboardCorners(images[i], (8, 5), None)
            for j, (chessboard_corner_vertex, z) in enumerate(zip(chessboard_corner_vertices, chessboard_pixels_image_coordinates)):
            

                chessboard_corner_edge = g2o.EdgeProjectXYZ2UV()
                chessboard_corner_edge.set_vertex(0, chessboard_corner_vertex)
                chessboard_corner_edge.set_vertex(1, optimizer.vertex(i))

                chessboard_corner_edge.set_measurement(z[0])
                chessboard_corner_edge.set_information(np.identity(2))

                chessboard_corner_edge.set_parameter_id(0, 0)
                optimizer.add_edge(chessboard_corner_edge)
        

        camera_robot_vertex = g2o.VertexSE3()
        camera_robot_vertex.set_id(9999)
        camera_robot_vertex.set_estimate(g2o.Isometry3d())
        optimizer.add_vertex(camera_robot_vertex)

        for i in range(len(camera_pose_collection)):
            for j in range(len(camera_pose_collection)):
                if i >= j:
                    continue

                relative_pose = g2o.EdgeSE3Calib()
                relative_pose.set_vertex(0, camera_pose_collection[i])
                relative_pose.set_vertex(1, camera_pose_collection[j])
                relative_pose.set_vertex(2, camera_robot_vertex)

                v1 = g2o.Isometry3d(g2o.Quaternion(q[i][0][3], q[i][0][0], q[i][0][1], q[i][0][2]), t[i].T)
                v2 = g2o.Isometry3d(g2o.Quaternion(q[j][0][3], q[j][0][0], q[j][0][1], q[j][0][2]), t[j].T)
                measurment = v1 * v2.inverse()

                relative_pose.set_measurement(measurment)
                relative_pose.set_information(np.identity(6))

                optimizer.add_edge(relative_pose)

        optimizer.initialize_optimization()
        if answer:
            optimizer.set_verbose(True)
        else:
            optimizer.set_verbose(False)
        optimizer.optimize(100)
        

        for i in range(len(camera_pose_collection)):
            for j in range(point_id):
                chessboard_corner_vertex = optimizer.vertex(j + len(extr_estimate_from_robot) * 2)

                current_estimated_pixel[i].append(camera_projection_model.cam_map(
                    (camera_pose_collection[i].estimate().matrix() @ np.concatenate([chessboard_corner_vertex.estimate(), [1]]))[:3]
                    ))

        return camera_pose_collection, current_estimated_pixel, camera_robot_vertex, extr_estimate_from_robot


    def main(self):
        interactive_gui = InteractiveGui()
        answer = interactive_gui.debug_answer()

        camera_extrs, current_estimated_pixel, camera_robot_vertex, measured_tool0_extrs = self.optimize(answer)
        
        if answer:
            calib, world_base_pose = self.debug(camera_robot_vertex, current_estimated_pixel, camera_extrs, measured_tool0_extrs)
        else:
            calib, world_base_pose = self.yaml_info(camera_robot_vertex, camera_extrs, measured_tool0_extrs)

        self.save_yaml(calib, world_base_pose)
        self.euler_yaml(calib, camera_robot_vertex)

def main():
    path = './photo/31_5'
    files = os.listdir(path)
    files = [f for f in files if '.npz' in f]
    if files == None:
        print('No files, no optimization')
    else:
        opt = Optimizer(files, path)
        opt.main()
    print('Done')

if __name__ == "__main__":
    main()
