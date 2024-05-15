#!/usr/bin/env python3

# Source: https://www.youtube.com/watch?v=Ykyqkc3fKnw

import rospy
import time
import yaml
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from threading import Thread

class VideoStream:
    def __init__(self, src) -> None:
        rospy.loginfo('Initializing video stream...')
        self.capture = cv.VideoCapture(src, cv.CAP_GSTREAMER)

        rospy.loginfo('Starting video stream thread...')
        self.thread = Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()

    def update(self): 
        while True:
            _, self.frame = self.capture.read()

    def get_frame(self):
        return self.frame

class CameraInfoMsgGenerator:
    def __init__(self, config_path) -> None:
        self.calib_data = self.read_config_file(config_path)

        self.frame_id = self.calib_data['camera_name']

        self.camera_matrix = np.array(self.calib_data['camera_matrix']['data']).reshape((self.calib_data['camera_matrix']['rows'],self.calib_data['camera_matrix']['cols']))
        self.rect_matrix = np.array(self.calib_data['rectification_matrix']['data']).reshape((self.calib_data['rectification_matrix']['rows'],self.calib_data['rectification_matrix']['cols']))
        self.proj_matrix = np.array(self.calib_data['projection_matrix']['data']).reshape((self.calib_data['projection_matrix']['rows'],self.calib_data['projection_matrix']['cols']))

        self.dist_model = self.calib_data['distortion_model']
        self.dist_coefficients = np.array(self.calib_data['distortion_coefficients']['data'])
        self.height = self.calib_data['image_height']
        self.width = self.calib_data['image_width']

        self.cam_info = self.generate_cam_info()


    def read_config_file(self, config_path):
        rospy.loginfo('Reading camera configuration file...')
        with open(config_path, 'r') as file:
            calib_data = yaml.safe_load(file)

        rospy.loginfo("Reading camera configuration file - FINISHED")
        return calib_data
    
    def generate_cam_info(self):
        rospy.loginfo("Generating camera info message...")
        cam_info = CameraInfo()
        cam_info.header.frame_id = self.frame_id

        cam_info.width = self.width
        cam_info.height = self.height

        cam_info.distortion_model = self.dist_model
        cam_info.D = list(self.dist_coefficients)
        cam_info.K = list(self.camera_matrix.flatten())
        cam_info.R = list(self.rect_matrix.flatten())
        cam_info.P = list(self.proj_matrix.flatten())

        cam_info.binning_x = 0
        cam_info.binning_y = 0

        cam_info.roi.x_offset = 0
        cam_info.roi.y_offset = 0
        cam_info.roi.height = 0
        cam_info.roi.width = 0
        cam_info.roi.do_rectify = True

        rospy.loginfo("Generating camera info message - FINISHED")
        return cam_info
    
    def get_cam_info(self, seq, timestamp):
        self.cam_info.header.stamp = timestamp
        self.cam_info.header.seq = seq
        return self.cam_info
    
def get_pipeline_str(sensor_id: int=0, sensor_mode: int=3, width: int=640, height: int=480, fps: int=21, flip: int=0):
    pipeline_str = (
        'gst-launch-1.0 -e nvarguscamerasrc sensor-id=%d sensor-mode=%d ! '
        'video/x-raw(memory:NVMM), '
        'width=(int)%d, height=(int)%d, '
        'format=(string)NV12, framerate=(fraction)%d/1 ! '
        'nvvidconv flip-method=%d ! '
        'video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! '
        'videoconvert ! '
        'video/x-raw, format=(string)BGR ! appsink' % 
        (sensor_id, sensor_mode, width, height, fps, flip, width, height))
    
    return pipeline_str

def _load_yaml(path: str):
    """
    Function to load camera data:
        
    Parameters:
    - path (str): Path of yaml file
    """
    with open(path, "r") as file:
        data = yaml.safe_load(file)
    return data

def main():
    rospy.init_node('camera')
    freq = 12
    rate = rospy.Rate(freq)

    # Publisher
    pub_img = rospy.Publisher('/mono_cam/image_raw', Image, queue_size=1)
    pub_cam_info = rospy.Publisher('/mono_cam/camera_info', CameraInfo, queue_size=1)

    # Create OpenCV - ROS Bridge
    bridge = CvBridge()

    # Create ImageConverter to undistort the frames
    rospy.loginfo('Setting up image converter...')
    cam_info_gen = CameraInfoMsgGenerator("/home/jetson/workspace/catkin_ws/config/camera/ost_mono.yaml")


    # Create VideoStream instances
    rospy.loginfo('Opening camera devices...')
    cam = VideoStream(get_pipeline_str(sensor_id=0, width=cam_info_gen.width, height=cam_info_gen.height, fps=freq))
    
    # Wait for valid frames
    try:
        while True:
            time.sleep(1)
            rospy.loginfo(f'Frames available?: {cam.frame is not None}')
            
            if cam.frame is not None:
                rospy.loginfo('Camera Device is operating!')
                break
            
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down...")
        cam.capture.release()
        exit(1)

    # Create messages
    seq = 0


    try:
        while not rospy.is_shutdown():

            clock = rospy.Time.now()

            # Get frames
            frame = cam.get_frame()

            # CameraInfo message
            cam_info_msg = cam_info_gen.get_cam_info(seq=seq, timestamp=clock)


            # Camera message
            img_msg = bridge.cv2_to_imgmsg(frame, 'bgr8')
            img_msg.header.seq = seq
            img_msg.header.frame_id = cam_info_msg.header.frame_id
            img_msg.header.stamp = cam_info_msg.header.stamp

            # increase sequence
            seq += 1

            # Publish topics
            pub_img.publish(img_msg)
            pub_cam_info.publish(cam_info_msg)

            rate.sleep()

    except Exception as e:
        rospy.logerr(e)
        rospy.loginfo("Shutting down...")
    finally:
        cam.capture.release()
        exit(1)


if __name__ == '__main__':
    main()
