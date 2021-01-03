import open3d as o3d
from tkinter import filedialog
import os
import multiprocessing
import numpy as np


def run_window(pcd_object):
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name='TopLeft', width=960, height=540, left=0, top=0)
    vis.add_geometry(pcd_object)
    vis.run()
    vis.destroy_window()
    points = np.asarray(pcd_object.points)
    print(points[vis.get_picked_points()])
    return


filename_map = ""
filename_map =  filedialog.askopenfilename(initialdir ="~", title ="Select Map Pointcloud",
                                           filetypes = (("mesh_files","*.ply"),("all files","*.*")))


print (filename_map)
pcd_map = o3d.io.read_point_cloud(filename_map)

filename_object = ""
filename_object =  filedialog.askopenfilename(initialdir ="~", title ="Select Object Pointcloud",
                                              filetypes = (("mesh_files","*.ply"),("all files","*.*")))
pcd_object = o3d.io.read_point_cloud(filename_object)
#pcd = io.read_point_cloud("/home/christian/Downloads/dyson_lab.klg.ply")


window1 = multiprocessing.Process(target=run_window, args=(pcd_map,))
window1.start()

window2 = multiprocessing.Process(target=run_window, args=(pcd_object,))
window2.start()

# joining the process
window1.join()
window2.join()

print("Picking points finished")




# vis2.destroy_window()