import open3d as o3d
from tkinter import filedialog
from tkinter import messagebox
import os
import multiprocessing
import numpy as np


class label_generator():

    def __init__(self):
        """
        Class to generate labeled images to already generated images via a generated point cloud from Elasticfusion and
        the generated freiburg file. This class is a copy to labelfusion (http://labelfusion.csail.mit.edu/) implemented
        in open3d.
        """

        self.map_pointcloud = None
        self.map_filepath = ""
        self.list_object_pointcloud=[]
        self.list_object_filepath=[]

        # Initial dir for input dialogs
        self.__initial_dir = str(os.path.dirname(os.path.abspath(__file__))).split("/src")[0]

        return

    def load_map_file(self):
        """
        Ask user to specify a pointcloud map of the room. Generate input dialog and user has to specify a path. PLY
        format is expected, but all open3d compatible file formats are accepted. Map is stored in object variable and
        filepath is stored.

        :input: None

        :return: returns loaded map file, returns None if user didn't choose a file
        """
        map_pcl = None
        map_filepath = ""
        map_filepath = filedialog.askopenfilename(initialdir =self.__initial_dir, title ="Select Map Pointcloud",
                                           filetypes = (("mesh_files","*.ply"),("all files","*.*")))
        if map_filepath == "":
            print("No Map_File was selected. Trying to continue.")
            return map_pcl
        else:
            print("Following filepath was selected: {}".format(map_filepath))

        self.map_filepath = map_filepath
        map_pcl = o3d.io.read_point_cloud(map_filepath)
        self.map_pointcloud = map_pcl

        return map_pcl

    def load_object_file(self):
        """
        Ask user to specify a pointcloud map of an object to detect. Generate input dialog and user has to specify a path. PLY
        format is expected, but all open3d compatible file formats are accepted. Object is stored in object variable and
        filepath is stored.

        :input: None

        :return: returns loaded object file, returns None if user didn't choose a file
        """

        object_pcl = None
        object_filepath = ""
        object_filepath = filedialog.askopenfilename(initialdir =self.__initial_dir, title ="Select Object Pointcloud",
                                                     filetypes = (("mesh_files","*.ply"),("all files","*.*")))
        if object_filepath == "":
            print("No Object_File was selected. Trying to continue.")
            return object_pcl
        else:
            print("Following filepath was selected: {}".format(object_filepath))

        self.list_object_filepath.append(object_filepath)
        object_pcl = o3d.io.read_point_cloud(object_filepath)
        self.list_object_pointcloud.append(object_pcl)

        return object_pcl

    def __run_pick_points_map_window(self, pcd_object):
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window(window_name='Map_Pointcloud', width=960, height=540, left=0, top=0)
        vis.add_geometry(pcd_object)
        vis.run()
        vis.destroy_window()
        points = np.asarray(pcd_object.points)
        print(points[vis.get_picked_points()])
        return

    def start_picking_points(self):

        text = "please pick three points in the map and the object to detect the initial localisation"
        messagebox.showinfo(title="Object initial localisation", message=text)
        window1 = multiprocessing.Process(target=self.__run_pick_points_map_window, args=(self.map_pointcloud,))
        window1.start()

        window2 = multiprocessing.Process(target=self.__run_pick_points_map_window, args=(self.map_pointcloud,))
        window2.start()

        # joining the processes
        window1.join()
        window2.join()

        print("Picking points finished")
        return

generator = label_generator()
generator.load_map_file()
generator.start_picking_points()
# vis2.destroy_window()