## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
import os
from datetime import datetime
from pathlib import Path
import subprocess

from pynput import keyboard
import copy

# define source library path, TODO update way to implement this
#SOURCE_LIB_PATH = "/home/christian/python_projects/image_data_pipeline"

class image_collecter:
    # class for collecting images from rgb and depth stream and processing them in the right format
    def __init__(self, save_images):
        self.dataset_name = (str(datetime.now()).split('.')[0]).replace(" ", "_")
        self.source_lib_path = str(os.path.dirname(os.path.abspath(__file__))).split("/src")[0]
        self.dataset_source_path = os.path.join(self.source_lib_path,"image_data", self.dataset_name)
        self.list_depth_images = []
        self.list_color_images = []
        self.stream_resolution = (1280, 720)
        self.image_dimension = (640, 480)
        self.save_images = save_images
        self.b_run_image_loop = False
        self.camera_intrinsics = []

        # keyboard_listener for controlling image_processing
        self.keyboard_listener = None

    def start_image_collection(self):
        pipeline = rs.pipeline()

        # set correct depth sensor setting --> depth scale

        # if depth_sensor.supports(rs.option.depth_units):
        #     depth_sensor.set_option(rs.option.depth_units, 1/5000)

        config = rs.config()
        config.enable_stream(rs.stream.depth, self.stream_resolution[0], self.stream_resolution[1], rs.format.z16, 30)
        config.enable_stream(rs.stream.color, self.stream_resolution[0], self.stream_resolution[1], rs.format.bgr8, 30)

        pipe_profile = pipeline.start(config)
        depth_sensor = pipe_profile.get_device().first_depth_sensor()
        print(depth_sensor.get_depth_scale())
        depth_sensor.set_option(rs.option.depth_units, 1/5000)
        print("Depth Scale is: ", depth_sensor.get_depth_scale())

        # save camera parameters
        color_profile = rs.video_stream_profile(pipe_profile.get_stream(rs.stream.color))
        color_intrinsics = color_profile.get_intrinsics()

        self.camera_intrinsics = color_intrinsics
        # print(color_intrinsics)
        # print(color_intrinsics.fx , color_intrinsics.fy)

        # align color and depth frame
        align_to = rs.stream.color
        align = rs.align(align_to)

        self.start_keyboard_listener()
        self.b_run_image_loop = True

        try:
            while self.b_run_image_loop:
                # Wait for a coherent pair of frames: depth and color
                frames = pipeline.wait_for_frames()

                aligned_frames = align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                if not depth_frame or not color_frame:
                    continue

                # Convert images to numpy arrays
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                #depth_image = cv2.convertScaleAbs(depth_image, alpha=0.04)

                #depth_colormap = depth_image
                color_image = self.crop_and_resize_image(color_image)
                depth_image = self.crop_and_resize_image(depth_image)

                # Convert Greyscale for desplaying during recording
                depth_image_3_channel = cv2.cvtColor((depth_image / 256).astype('uint8'), cv2.COLOR_GRAY2BGR)

                # convert to 16bit
                #depth_image = (depth_image * 256).astype('uint16')
                # Stack both images horizontally
                images = np.concatenate((color_image, depth_image_3_channel), axis=1)

                # Show images
                cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('RealSense', images)
                cv2.waitKey(1)

                if self.save_images:

                    time = datetime.now().timestamp()
                    self.save_color_image(color_image, str(time))
                    self.save_depth_image(depth_image, str(time))

        finally:
            # Stop streaming
            pipeline.stop()

        if self.save_images:
            self.save_txt_files()
            self.generate_associate_txt_file()
        return

    def crop_and_resize_image(self, image):
        max_loc = np.argmax(np.shape(image))
        if max_loc != 1:
            print("Warning: image is of different shape then expected. Resize passed")
            return image

        # crop every pixel outside the center
        center_position_pixels = int(np.shape(image)[1] / 2 )
        # Adjust image ratio
        desired_image_ratio = self.image_dimension[1] / self.image_dimension[0]

        image = image[:, center_position_pixels - int(self.stream_resolution[1]*desired_image_ratio) :
                         center_position_pixels + int(self.stream_resolution[1]*desired_image_ratio)]
        # resize to final shape
        image = cv2.resize(image, self.image_dimension, interpolation=cv2.INTER_AREA)
        return image

    def save_depth_image(self, image, image_name):
        image_path = os.path.join(self.dataset_source_path, "depth", str(image_name) + ".png")
        Path(os.path.join(self.dataset_source_path, "depth")).mkdir(parents=True, exist_ok=True)
        cv2.imwrite(image_path, image)
        self.list_depth_images.append([str(image_name),os.path.join("depth", str(image_name) + ".png")])
        return
    def save_color_image(self, image, image_name):
        image_path = os.path.join(self.dataset_source_path, "rgb", str(image_name) + ".png")
        Path(os.path.join(self.dataset_source_path, "rgb")).mkdir(parents=True, exist_ok=True)
        cv2.imwrite(image_path,image)
        self.list_color_images.append([str(image_name),os.path.join("rgb", str(image_name) + ".png")])
        return
    def save_txt_files(self):
        """"saves the depth and rgb text file with the linking information"""
        depth_file = open(os.path.join(self.dataset_source_path, "depth.txt"), "w+")
        for row in self.list_depth_images:
            depth_file.write(row[0] + " " + row[1] + "\n")
        depth_file.close()
        color_file = open(os.path.join(self.dataset_source_path, "rgb.txt"), "w+")
        for row in self.list_color_images:
            color_file.write(row[0] + " " + row[1] + "\n")
        color_file.close()

    def on_press(self, key):
        print('{0} pressed'.format(
            key))

    def on_release(self, key):
        print('{0} release'.format(
            key))
        if key == keyboard.Key.esc:
            self.b_run_image_loop = False

    def start_keyboard_listener(self):
        self.keyboard_listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.keyboard_listener.start()
        return

    def generate_associate_txt_file(self):
        associate_file_path = Path(os.path.join(self.source_lib_path, "src", "associate.py"))

        try:
            associate_file_path.resolve(strict=True)
        except FileNotFoundError as err:
            print("Error. Associate.py not found.")
            print(err)

        try:
            subprocess.check_call(['python2.7 ' + str(associate_file_path) + " " +
                                   os.path.join(self.dataset_source_path, "depth.txt ") +
                                   os.path.join(self.dataset_source_path, "rgb.txt ") +
                                   "> " + os.path.join(self.dataset_source_path, "associations.txt")], shell=True)
        except subprocess.CalledProcessError as error:
            print(error)

        print("Generation associations.txt finished.")

        return

    def generate_klg(self):
        """
        generate the the klg file for elasticfusion from the png pictures. Docker for png_to_klg-docker is necessary

        input: nothing

        :return:
        """
        try:
            subprocess.check_call(["docker run --rm --volume "+'"'+ self.dataset_source_path.split(self.dataset_name)[0] +
            """:/input-output:rw" jjkka132/pngtoklg /bin/bash -c "/pngtoklg/png_to_klg/build/pngtoklg -w '/input-output/"""
                                   + self.dataset_name + """/' -o '/input-output/log.klg'" """],
                                  shell=True)
            Path(os.path.join(self.dataset_source_path,"cad_data")).mkdir(parents=True, exist_ok=True)

            # change location of klg file
            os.rename((self.dataset_source_path.split(self.dataset_name)[0])+ "log.klg",
                      os.path.join(self.dataset_source_path, "cad_data","log.klg"))

        except subprocess.CalledProcessError as error:
            print(error)

        camera_params_file = open(os.path.join(self.dataset_source_path, "cal.txt"), "w+")
        camera_params_file.write(str(self.camera_intrinsics.fx)+ " " +
                                 str(self.camera_intrinsics.fy)+ " " +
                                 str(self.camera_intrinsics.ppx)+ " "+
                                 str(self.camera_intrinsics.ppy)+ " ")

        camera_params_file = open(os.path.join(self.dataset_source_path, "cal_640.txt"), "w+")
        camera_params_file.write("613.1024780273438 611.6202392578125 319.5 239.5 ")


        return

    def start_elasticfusion(self):
        """
        starts elasticfusion, map has to be saved manually
        """
        subprocess.check_call(["/home/christian/programms/ElasticFusion/GUI/build/ElasticFusion -f -l "+
                               os.path.join(self.dataset_source_path,"cad_data", "log.klg")+ " -cal " +
                               os.path.join(self.dataset_source_path, "cal_640.txt")], shell=True)
        try:
            pass
        except subprocess.CalledProcessError as error:
            print(error)

        return

img_coll = image_collecter(True)
img_coll.start_image_collection()
img_coll.generate_klg()
img_coll.start_elasticfusion()

