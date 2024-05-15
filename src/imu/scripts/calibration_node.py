#!/usr/bin/env python3

"""
Module/Script Name: calibration_node.py
Author: Florentin Hau
Date: 30.01.2024

Description:
    IMU Calibration Node

    This Python script serves as a ROS node for calibrating an IMU (Inertial Measurement Unit). 
    It subscribes to "/imu/data_raw" topic for raw IMU data, applies gyroscope and accelerometer calibration, 
    and publishes the calibrated data to the "/imu/data_calib" topic.

Usage:
    Run the script with the following command:

        "rosrun package_name calibration_node.py"

    Replace "package_name" with the actual name of your ROS package.

"""


import rospy
from sensor_msgs.msg import Imu
import numpy as np
from auto_calibration_gyroscope import AutoCalibration
import yaml

class IMUCalibrationNode:
    def __init__(self) -> None:
        """
        Initialize the IMU Calibration Node.
        """
        rospy.init_node("imu_calibration_node")

        # Load constants from yaml file
        rospy.loginfo("Load calibration data for IMU...")
        self.calib_data = self._load_yaml(rospy.get_param("~imu_config_path"))
        
        self.ACCELEROMETER_A_INV = np.array(self.calib_data["accelerometer_matrix"]["data"]).reshape((self.calib_data['accelerometer_matrix']['rows'],self.calib_data['accelerometer_matrix']['cols']))

        self.ACCELEROMETER_BIAS = np.array(self.calib_data["accelerometer_bias"]["data"])
        self.GYROSCOPE_BIAS = AutoCalibration(recording_time=10.0).get_bias()
        rospy.loginfo(f"Gyroscope Bias: {self.GYROSCOPE_BIAS}")

        # Publisher and Subscriber
        self.publisher = rospy.Publisher("/imu/data_calib", Imu, queue_size=1)
        self.subscriber = rospy.Subscriber("/imu/data_raw", Imu, self._callback)


    # ----- Public Methods -----

    def run(self) -> None:
        """
        Run the ROS node.
        """
        rospy.spin()


    # ----- Private Methods -----

    def _callback(self, msg: Imu) -> None:
        """
        Callback function for raw IMU data.
        Adjusts gyroscope and accelerometer readings and publishes calibrated data.

        Parameters:
        - msg (Imu): IMU message.
        """
        
        # Adjust gyroscope readings by subtracting the bias values
        msg.angular_velocity.x = np.radians(msg.angular_velocity.x - self.GYROSCOPE_BIAS[0])
        msg.angular_velocity.y = np.radians(msg.angular_velocity.y - self.GYROSCOPE_BIAS[1])
        msg.angular_velocity.z = np.radians(msg.angular_velocity.z - self.GYROSCOPE_BIAS[2])

        # Extract linear acceleration readings from the message
        linear_acceleration_np = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

        # Subtract accelerometer bias and apply inverse of the calibration matrix
        linear_acceleration_np = np.dot(self.ACCELEROMETER_A_INV, linear_acceleration_np - self.ACCELEROMETER_BIAS)

        # Update message with calibrated linear acceleration values
        msg.linear_acceleration.x = linear_acceleration_np[0]
        msg.linear_acceleration.y = linear_acceleration_np[1]
        msg.linear_acceleration.z = linear_acceleration_np[2]

        # Publish the calibrated IMU data
        self.publisher.publish(msg)
        
    def _load_yaml(self, config_path: str):
        """
        Function to load calibration data:
            -> Calibration matrix and
            -> Calibration bias
            for accelerometer data
            
        Parameters:
        - config_path (str): Path of configuration file
        """
        with open(config_path, "r") as file:
            calib_data = yaml.safe_load(file)
        return calib_data
            

def main() -> None:
    """
    Main function to instantiate and run the IMU Calibration Node.
    """
    imu_calibrator = IMUCalibrationNode()
    imu_calibrator.run()

if __name__ == "__main__":
    main()
