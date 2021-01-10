import open3d as o3d
from tkinter import filedialog
from tkinter import messagebox
import os
import multiprocessing
import numpy as np
import warnings
import pickle
from scipy.spatial.transform import Rotation as R
import cv2
from pathlib import Path


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
        self.list_picked_points= np.empty((0, 2, 3, 3)) # list which stores picked points by user for each specific object, column 0 map, 1 object
        self.list_transform_matrices_objects= np.empty((0, 4, 4)) # list of the transform_matrices where the objects where detected
        self.log_path = ""

        # color coding used for generating labeled pictures. Currently only 6 object are supported
        self.color_coding = np.array([[1, 0 , 0],
                                     [0, 1 , 0],
                                     [0, 0 , 1],
                                     [1, 1 , 0],
                                     [0, 1 , 1],
                                     [1, 0 , 1]])

        # Initial dir for input dialogs
        self.__initial_dir = str(os.path.dirname(os.path.abspath(__file__))).split("/src")[0]
        self.__picture_size = (640, 480)

        return

    def load_log_directory(self):
        """loads a log directory and tries to find the map_file (log.klg.ply) and the objects to detect in object_meshes
        If files cannot be found the load-map_file, or load_object_file are called


        :returns: nothing, files are directly stored in the object variables
        """

        dataset_path = filedialog.askdirectory(initialdir=self.__initial_dir, title="Select directory of log")
        if not dataset_path:
            raise IOError("No directory for saving log was given. Labeling pipeline canceled.")

        self.log_path = dataset_path

        if not os.path.isfile(os.path.join(self.log_path, "cad_data", "log.klg.ply")):
            print("Map file not found. Please select map file manually.")
            self.load_map_file()

        else:
            map_filepath = os.path.join(self.log_path, "cad_data", "log.klg.ply")
            self.map_filepath = map_filepath
            map_pcl = o3d.io.read_point_cloud(map_filepath)
            self.map_pointcloud = map_pcl
            print("Map_file found successfully. Following file was found:")
            print(self.map_filepath)

        def manual_import_handler(self):
            warnings.warn("Warning: Automatic object import failed. PLease pick objects manually")
            self.list_object_pointcloud = []
            self.list_object_filepath = []

            while True:
                MsgBox = messagebox.askquestion('Object import.', 'Automatic Object import failed. Please add objects '
                                                                  'manually. Do you want add another object?',
                                                icon='warning')
                if MsgBox == 'yes':
                    self.load_object_file()
                else:
                    break

            print("Following Files were loaded:")
            print(self.list_object_filepath)
            if self.list_object_filepath == []:
                warnings.warn("Warning: No objects were loaded. Trying to continue")

        if os.path.exists(os.path.join(self.log_path, "cad_data", "object_meshes")):
            DIR = os.path.join(self.log_path, "cad_data", "object_meshes")
            print (len([name for name in os.listdir(DIR) if os.path.isfile(os.path.join(DIR, name))]) )
            print("Following Files were found:")
            print ([name for name in os.listdir(DIR) if os.path.isfile(os.path.join(DIR, name))])

            if os.listdir(DIR) == []:
                manual_import_handler(self)
                return

            # TODO add handling if open3d cannot open file
            for name in os.listdir(DIR):
                if os.path.isfile(os.path.join(DIR, name)):
                    self.list_object_filepath.append(os.path.join(DIR, name))
                    self.list_object_pointcloud.append(o3d.io.read_point_cloud(os.path.join(DIR, name)))

            print("Following Files were loaded:")
            print(self.list_object_filepath)

        else:
            manual_import_handler(self)

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

        map_filepath = filedialog.askopenfilename(initialdir =self.__initial_dir, title ="Select Map Pointcloud",
                                           filetypes = (("mesh_files","*.ply"),("all files","*.*")))
        if not map_filepath:
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
        object_filepath = filedialog.askopenfilename(initialdir =self.__initial_dir, title ="Select Object Pointcloud",
                                                     filetypes = (("mesh_files","*.ply"),("all files","*.*")))
        if not object_filepath:
            print("No Object_File was selected. Trying to continue.")
            return object_pcl
        else:
            print("Following filepath was selected: {}".format(object_filepath))

        self.list_object_filepath.append(object_filepath)
        object_pcl = o3d.io.read_point_cloud(object_filepath)
        self.list_object_pointcloud.append(object_pcl)

        return object_pcl

    def __run_pick_points_map_window(self, pcd_object, return_dict):
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window(window_name='Map_Pointcloud', width=960, height=540, left=0, top=0)
        vis.add_geometry(pcd_object)
        vis.run()
        vis.destroy_window()
        points = np.asarray(pcd_object.points)
        picked_points=[]

        if len(vis.get_picked_points()) < 3 :
            raise IOError("Not enough points were picked, three points are necessary.")

        elif len(vis.get_picked_points()) == 3:
            print("Following points of map were picked:")
            picked_points = points[vis.get_picked_points()]

        else:
            warnings.warn("Warning: Too many points picked. Only first three are used")
            picked_points = points[vis.get_picked_points()]
            print("Following points are used:")
            picked_points = picked_points[:3]
        print(picked_points)
        return_dict[0] = picked_points
        return

    def __run_pick_points_object_window(self, pcd_object, return_dict):
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window(window_name='Object_Pointcloud', width=960, height=540, left=960, top=0)
        vis.add_geometry(pcd_object)
        vis.run()
        vis.destroy_window()
        points = np.asarray(pcd_object.points)
        picked_points=[]

        if len(vis.get_picked_points()) < 3 :
            raise IOError("Not enough points were picked, three points are necessary.")

        elif len(vis.get_picked_points()) == 3:
            print("Following points of object were picked:")
            picked_points = points[vis.get_picked_points()]

        else:
            warnings.warn("Warning: Too many points picked. Only first three are used")
            picked_points = points[vis.get_picked_points()]
            print("Following points are used:")
            picked_points = picked_points[:3]
        print(picked_points)
        return_dict[1] = picked_points
        return

    def start_picking_points(self, object_identifier):

        # initialize lists. they are filled later
        self.list_picked_points = np.append(self.list_picked_points, np.zeros((1, 2, 3, 3)), axis=0)
        self.list_transform_matrices_objects = np.append(self.list_transform_matrices_objects, np.zeros(((1, 4, 4))), axis=0)
        self.list_transform_matrices_objects[-1, : , :] = np.eye(4)

        # text = "please pick three points in the map and the object to detect the initial localisation"
        # messagebox.showinfo(title="Object initial localisation", message=text)

        manager = multiprocessing.Manager()
        return_dict = manager.dict()

        window1 = multiprocessing.Process(target=self.__run_pick_points_map_window, args=(self.map_pointcloud,return_dict))
        window1.start()

        window2 = multiprocessing.Process(target=self.__run_pick_points_object_window, args=(self.list_object_pointcloud[object_identifier],return_dict))
        window2.start()

        # joining the processes
        window1.join()
        window2.join()

        self.list_picked_points[-1, 0 , :, :] = return_dict[0]
        self.list_picked_points[-1, 1, :, :] = return_dict[1]
        #print(self.list_picked_points)
        print("Picking points finished")
        return

    def align_object(self,object_identifier, b_show_alignment, i_icp_iterations):
        """ this function function aligns the object in the map from the input of the user. The initial alignment can
        then be used by icp to get a better alignment. Alignment is done by using the points in list_picked_point calculate
        the planes (by cross product) and align the planes. Alignment is done at the first point picked at the map and object

        :input object identifier as int

        :returns the aligned pointcloud of the object, pcl object in list_object_pcl is also overritten"""

        self.start_picking_points(object_identifier)

        aligned_pointcloud = None
        pcd_object = self.list_object_pointcloud[object_identifier]

        # TODO: Check if points were picked

        # First calculate the vectors a and b from points --> coor frame of map
        map_points = self.list_picked_points[object_identifier, 0, :, :]
        map_vector_a = map_points[1,:] - map_points[0, :]
        map_vector_b = map_points[2, :] - map_points[0, :]
        map_cross_vector_a_b = np.cross(map_vector_a, map_vector_b)

        map_coor_x = map_vector_a/np.linalg.norm(map_vector_a)
        map_coor_z = map_cross_vector_a_b/np.linalg.norm(map_cross_vector_a_b)
        map_coor_y = np.cross(map_coor_x, map_coor_z)/np.linalg.norm(np.cross(map_coor_x, map_coor_z))

        object_points = self.list_picked_points[object_identifier, 1, :, :]
        object_vector_a = object_points[1, :] - object_points[0, :]
        object_vector_b = object_points[2, :] - object_points[0, :]
        object_points_cross_vector_a_b = np.cross(object_vector_a, object_vector_b)

        object_coor_x = object_vector_a/np.linalg.norm(object_vector_a)
        object_coor_z = object_points_cross_vector_a_b/np.linalg.norm(object_points_cross_vector_a_b)
        object_coor_y = np.cross(object_coor_x, object_coor_z)/np.linalg.norm(np.cross(object_coor_x, object_coor_z))

        #calc rot matrix from coor frame.
        rotation_matrix_unit_to_object = np.transpose(np.array([object_coor_x, object_coor_y, object_coor_z]))
        rotation_matrix_unit_to_map = np.transpose(np.array([map_coor_x, map_coor_y , map_coor_z]))

        rotation_matrix_object_to_unit = np.linalg.inv(rotation_matrix_unit_to_object)
        rotation_matrix = np.matmul(rotation_matrix_unit_to_map, rotation_matrix_object_to_unit)

        # shift object to center of its picked coor_frame, then rotate it, then shift it to map position. Combine it
        # together to one transform
        translation_1 = np.eye(4)
        translation_1[:3,3] = -object_points[0, :]
        #print(translation_1)

        rotation_transform_matrix = np.eye(4)
        rotation_transform_matrix[:3,:3] = rotation_matrix
        #print(rotation_transform_matrix)

        translation_2 = np.eye(4)
        translation_2[:3,3] = map_points[0, :]
        #print(translation_2)

        transform_matrix_comb = np.matmul(translation_2, np.matmul(rotation_transform_matrix,translation_1))
        pcd_object.transform(transform_matrix_comb)

        if (b_show_alignment):
            print("Alignment from picked points")
            vis = o3d.visualization.Visualizer()
            vis.create_window(window_name='Initial_Object_Alignment', width=960, height=540, left=960, top=0)
            vis.add_geometry(pcd_object)
            vis.update_geometry(pcd_object)
            vis.add_geometry(self.map_pointcloud)
            vis.update_geometry(self.map_pointcloud)
            vis.run()
            vis.destroy_window()

        # TODO add Input dialog if realignment is needed
        print("Starting ICP:")
        threshold = 0.02
        print(o3d.pipelines.registration.evaluate_registration(pcd_object, self.map_pointcloud, threshold))
        reg_p2p = o3d.pipelines.registration.registration_icp(pcd_object, self.map_pointcloud, threshold,
                                                              np.array(
                                                                  [[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.],
                                                                   [0., 0., 0., 1.]]),
                                                              o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                                                              o3d.pipelines.registration.ICPConvergenceCriteria(
                                                                  max_iteration=i_icp_iterations))

        pcd_object.transform(reg_p2p.transformation)


        self.list_transform_matrices_objects[object_identifier] = np.matmul(reg_p2p.transformation, transform_matrix_comb)

        mesh_object = o3d.io.read_triangle_mesh(self.list_object_filepath[object_identifier])
        mesh_object.transform(self.list_transform_matrices_objects[object_identifier, : , :])

        if (b_show_alignment):
            vis = o3d.visualization.Visualizer()
            vis.create_window(window_name='Final Object_Alignment', width=960, height=540, left=960, top=0)
            vis.add_geometry(mesh_object)
            vis.update_geometry(mesh_object)

            vis.add_geometry(pcd_object)
            vis.update_geometry(pcd_object)

            vis.add_geometry(self.map_pointcloud)
            vis.update_geometry(self.map_pointcloud)
            vis.run()
            vis.destroy_window()

        print("Info: Alignment finished successfully")
        return aligned_pointcloud

    def save_alignment(self):
        """Saves the alignment. Pickle library is used."""

        # PCL object can't be saved --> have to be cleared
        self.map_pointcloud = None
        self.list_object_pointcloud = None
        with open(os.path.join(self.log_path, "cad_data", "label_generator_object.pkl"), 'wb') as output:
            pickle.dump(self, output, pickle.HIGHEST_PROTOCOL)
        return

    def load_alignment(self):
        """loads an already saved alignment.

        :returns: the loaded object"""
        try:
            with open(os.path.join(self.log_path, "cad_data", "label_generator_object.pkl"), 'rb') as input:
                generator_object = pickle.load(input)

        except:
            raise IOError("Could not open label_generator object. Does it already exist?")

        return generator_object

    def display_trajectory(self):

        # read freiburg file and process it in numpy array, save it in trajectory
        freiburg_file = open(os.path.join(self.log_path, "cad_data", "log.klg.freiburg"), "r")
        trajectories_lines = freiburg_file.readlines()
        trajectories_as_float = []

        for line in trajectories_lines[:]:
            line = line[:-1]
            line_as_list = line.split(' ')
            line_as_float = []
            for i in range(len(line_as_list)):
                line_as_float.append(float(line_as_list[i]))

            trajectories_as_float.append(line_as_float)

        trajectory = np.array(trajectories_as_float)

        # load cal file for camera params an process it
        if os.path.isfile(os.path.join(self.log_path, "cal_640.txt")):
            cal_file = open(os.path.join(self.log_path, "cal_640.txt"), "r")
            line_cal = str(cal_file.readline())
            line_cal_as_list = line_cal.split(' ')
            line_cal_as_float = []
            print(line_cal_as_list)
            for number in line_cal_as_list:
                if not number == '':
                    line_cal_as_float.append(float(number))
                    
            calibration_intrinsic = [self.__picture_size[0], self.__picture_size[1],line_cal_as_float[0],
                                     line_cal_as_float[1],
                                     line_cal_as_float[2],
                                     line_cal_as_float[3]]

        else:
            # standard calibration if no file is available
            calibration_intrinsic = [self.__picture_size[0], self.__picture_size[1], 613.1024780273438, 611.6202392578125, 319.5, 239.5]

        vis = o3d.visualization.Visualizer()
        vis.create_window(width=self.__picture_size[0], height=self.__picture_size[1])

        # load as mesh, not as point cloud
        for i in range(len(self.list_object_filepath)):
            mesh_object = o3d.io.read_triangle_mesh(self.list_object_filepath[i])
            mesh_object.transform(self.list_transform_matrices_objects[i, : , :])
            print(self.color_coding[i, :])
            mesh_object.paint_uniform_color(self.color_coding[i, :])
            vis.add_geometry(mesh_object)
            vis.update_geometry(mesh_object)

        ctr = vis.get_view_control()
        camera_params = ctr.convert_to_pinhole_camera_parameters()
        camera_params.intrinsic.set_intrinsics(calibration_intrinsic[0],
                                               calibration_intrinsic[1],
                                               calibration_intrinsic[2],
                                               calibration_intrinsic[3],
                                               calibration_intrinsic[4],
                                               calibration_intrinsic[5])

        ctr.convert_from_pinhole_camera_parameters(camera_params)

        render_option = vis.get_render_option()
        render_option.background_color = np.array([0, 0, 0])

        try:
            pictures_file = open(os.path.join(self.log_path, "associations.txt"), "r")
        except:
            raise IOError("Could not open associations.txt")

        pictures_file_list = pictures_file.readlines()
        pictures_names = []
        for line in pictures_file_list:
            pictures_names.append(line.split(" ")[0])

        #print(pictures_names)
        Path(os.path.join(self.log_path, "labeled_images")).mkdir(parents=True, exist_ok=True)

        for i in range(np.shape(trajectory)[0]):
            trajectory_point = trajectory[i, :]

            camera_params = ctr.convert_to_pinhole_camera_parameters()
            rot = np.eye(4)

            quaternion = trajectory_point[4:]
            translation = trajectory_point[1:4]
            timestamp = trajectory_point[0]

            # adjust the rotation Matrix
            rotation = R.from_quat(quaternion)
            rot[:3, :3] = np.linalg.inv(rotation.as_matrix())
            vector_world_in_camera = - np.matmul(np.linalg.inv(rotation.as_matrix()), translation)
            rot[:3, 3] = np.array(vector_world_in_camera.transpose())
            camera_params.extrinsic = rot

            ctr.convert_from_pinhole_camera_parameters(camera_params)

            camera_params.extrinsic = rot
            camera_params.intrinsic.set_intrinsics(calibration_intrinsic[0],
                                                   calibration_intrinsic[1],
                                                   calibration_intrinsic[2],
                                                   calibration_intrinsic[3],
                                                   calibration_intrinsic[4],
                                                   calibration_intrinsic[5])

            ctr.convert_from_pinhole_camera_parameters(camera_params)

            vis.update_geometry(mesh_object)
            vis.poll_events()
            vis.update_renderer()

            img = cv2.cvtColor(np.asarray(vis.capture_screen_float_buffer(False)), cv2.COLOR_BGR2RGB)

            ones = np.ones_like(img[:, :, 0])
            object_images = np.zeros([len(self.list_object_filepath), np.shape(img)[0], np.shape(img)[1], np.shape(img)[2] ] )
            img_changed = img

            for j in range(len(self.list_object_filepath)):
                color = j
                object_images[j, :, :, 2] = np.multiply((np.equal(ones * self.color_coding[color, 0],
                                                                  img[:, :, 2])).astype(int), img[:, :, 2])
                object_images[j, :, :, 1] = np.multiply((np.equal(ones * self.color_coding[color, 1],
                                                                  img[:, :, 1])).astype(int), img[:, :, 1])
                object_images[j, :, :, 0] = np.multiply((np.equal(ones * self.color_coding[color, 2],
                                                                  img[:, :, 0])).astype(int), img[:, :, 0])
                width_object = []
                height_object = []
                width_object = np.unique((np.nonzero(object_images[j, :, :, :])[0]))
                height_object = np.unique((np.nonzero(object_images[j, :, :, :])[1]))
                cv2.rectangle(img_changed, (height_object.min(), width_object.min()), (height_object.max(), width_object.max()),
                              (1, 1, 1), 3)
                cv2.imshow("Display window", object_images[j, :, :, :])
                cv2.waitKey(500)

            img_changed = img_changed * 255
            cv2.imwrite(os.path.join(self.log_path, "labeled_images", str(pictures_names[i]) + ".png"),img_changed)
            # cv2.imshow("Display window", img)
            # cv2.waitKey(1)
            # cv2.imwrite(os.path.join(filename_map, "labeled_images", str(timestamp) + '.png'), img)

        vis.destroy_window()

        return


generator = label_generator()
generator.load_log_directory()
#generator.load_map_file()
#generator.load_object_file()
#generator.start_picking_points()
#generator.align_object(0, True, 5)
#generator.align_object(1, True, 5)
#generator.save_alignment()
generator = generator.load_alignment()
generator.display_trajectory()
