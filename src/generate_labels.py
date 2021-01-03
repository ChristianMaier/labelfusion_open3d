import open3d as o3d
import numpy as np
from pynput import keyboard
import copy
from scipy.spatial.transform import Rotation as R
import os
from pathlib import Path
import cv2
from numpy.linalg import inv

from tkinter import filedialog
from tkinter import *

#root = Tk()
filename =  filedialog.askdirectory(initialdir = "~",title = "Select directory")
print (filename)


#pcd = io.read_point_cloud("/home/christian/Downloads/dyson_lab.klg.ply")
pcd = o3d.io.read_point_cloud(os.path.join(filename,"cad_data", "log.klg.ply"))

# read freiburg file and process it in numpy array, save it in trajectory
freiburg_file = open(os.path.join(filename,"cad_data", "log.klg.freiburg"), "r")
trajectories_lines = freiburg_file.readlines()
trajectories_as_float = []

for line in trajectories_lines[:]:
    line = line[:-1]
    line_as_list = line.split(' ')
    #print(len(line_as_list))
    line_as_float = []
    for i in range(len(line_as_list)):
        line_as_float.append(float(line_as_list[i]))

    trajectories_as_float.append(line_as_float)


trajectory = np.array(trajectories_as_float)

points = np.asarray(pcd.points)
colors = np.asarray(pcd.colors)

#pcd.colors = o3d.utility.Vector3dVector(colors)

vis = o3d.visualization.Visualizer()
vis.create_window(width=int(640), height=int(480))

vis.add_geometry(pcd)
vis.update_geometry(pcd)


ctr = vis.get_view_control()
camera_params = ctr.convert_to_pinhole_camera_parameters()
camera_params.intrinsic.set_intrinsics(int(640), int(480), 613.1024780273438, 611.6202392578125, 319.5, 239.5)
#camera_params.intrinsic.set_intrinsics(int(640), int(480), 611.62, 613.102, 319.5, 239.5)
ctr.convert_from_pinhole_camera_parameters(camera_params)
#rot = np.eye(4)

trajectory_length = trajectory.shape[0]

def move_forward(vis):
    # adjust camera to current trajectory point
    trajectory = move_forward.trajectory
    trajectory_point = trajectory[move_forward.index, :]
    camera_params = ctr.convert_to_pinhole_camera_parameters()
    rot = np.eye(4)

    quaternion = trajectory_point[4:]
    translation = trajectory_point[1:4]
    timestamp = trajectory_point[0]

    #adjust the rotation Matrix
    rotation = R.from_quat(quaternion)
    rot[:3, :3] = inv(rotation.as_matrix())

    # adjust the translation parameters
    vector_world_in_camera = - np.matmul(inv(rotation.as_matrix()), translation)
    rot[:3, 3] = np.array(vector_world_in_camera.transpose())
    #print(rot)
    camera_params.extrinsic = rot
    ctr.convert_from_pinhole_camera_parameters(camera_params)

    camera_params.extrinsic = rot
    camera_params.intrinsic.set_intrinsics(int(640), int(480), 613.1024780273438, 611.6202392578125, 319.5, 239.5)
    #camera_params.intrinsic.set_intrinsics(int(1280), int(720), 648.18, 360.205 , 919.654, 917.43)
    ctr.convert_from_pinhole_camera_parameters(camera_params)

    vis.update_renderer()

    move_forward.index = move_forward.index + 1

    if move_forward.index >= trajectory.shape[0]:
        move_forward.index = move_forward.index - 1

    Path(os.path.join(filename, "labeled_images")).mkdir(parents=True, exist_ok=True)

    vis.capture_screen_image(os.path.join(filename, "labeled_images", str(timestamp) + '.png'))
    # img = cv2.imread(os.path.join(filename_map, "labeled_images", str(timestamp) + '.png'),1)
    #
    # cv2.imshow("Display window", img);
    # cv2.waitKey(500)
    # cv2.imwrite(os.path.join(filename_map, "labeled_images", str(timestamp) + '.png'), img)

    return False

print(np.shape(trajectory))
# use function-object variables to import necessary outer variables
move_forward.index = 0
move_forward.trajectory = trajectory

vis.register_animation_callback(move_forward)
vis.run()
vis.destroy_window()