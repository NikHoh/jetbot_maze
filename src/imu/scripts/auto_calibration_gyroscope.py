#!/usr/bin/env python3

"""
Module/Script Name: auto_calibration_gyroscope.py
Author: Florentin Hau
Date: 30.01.2024

Description:
    Auto Calibration for Gyroscope

    This Python script defines a class 'AutoCalibration' for automatically calibrating a gyroscope. 
    The class subscribes to raw IMU data, records gyroscope readings for a specified duration, and computes the mean bias as the calibration result.

Usage:
    To use the AutoCalibration class, create an instance and call the 'run' method. The computed bias can be obtained using the 'get_bias' method.

	Example:
	    ```
	    auto_calibrator = AutoCalibration()
	    auto_calibrator.run()
	    gyro_bias = auto_calibrator.get_bias()
	    print("Gyroscope Bias:", gyro_bias)
	    ```


"""

import rospy
import numpy as np
import threading

from queue import Queue
from sensor_msgs.msg import Imu


class AutoCalibration:
    def __init__(self, topic: str="/imu/data_raw", recording_time: float=20.0):
        """
        Initialize the AutoCalibration class.

        Parameters:
        - topic (str): ROS topic for raw IMU data.
        - recording_time (float): Time duration for recording data.
        """
        self.topic = topic
        self.recording_time = recording_time
        self.data = None
        self.bias = np.zeros(3)
        self.cache = Queue()

        self.lock = threading.Lock()
        subscriber_thread_handle = threading.Thread(target=self._subscriber_thread, daemon=True)
        subscriber_thread_handle.start()
        

        self.run()


    # ----- Public Methods -----

    def run(self):
        """
        Run the calibration process: record data and compute bias.
        """
        self._record_data()
        self._compute_bias(self.data)

    def get_bias(self) -> np.ndarray:
        """
        Get the computed bias.

        Returns:
        - np.ndarray: Computed bias values as a NumPy array.
        """
        return self.bias


    # ----- Private Methods -----

    def _imu_callback(self, msg):
        """
        Callback function for IMU data. Collects angular velocity data in the cache.

        Parameters:
        - msg (Imu): IMU message.
        """
        x, y, z = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        with self.lock:
            self.cache.put([x, y, z])

    def _subscriber_thread(self):
        """
        Subscriber thread to listen to raw IMU data.
        """
        sub_imu = rospy.Subscriber(self.topic, Imu, callback=self._imu_callback)
        rospy.spin()

    def _record_data(self):
        """
        Record IMU data for a specified duration and store it in the cache.

        This method performs the following steps:
        1. Waits for 2 seconds to ensure the gyroscope is stable.
        2. Clears the cache to start with an empty buffer.
        3. Records IMU data for the specified duration.
        4. Uses a high-frequency rate to ensure accurate time measurements.
        5. Retrieves data from the cache and stores it in the class variable.

        Note: The method relies on the subscriber thread to populate the cache with IMU data.

        """
        rospy.loginfo("[INFO]: Do not move the gyroscope... Measurement is running!")
        rospy.sleep(2)  # Wait for 2 seconds to ensure the gyroscope is stable before recording

        rospy.loginfo("[INFO]: Clear cache...")
        with self.lock:
            # Clear the cache by retrieving all existing items
            while not self.cache.empty():
                self.cache.get()

        rospy.loginfo("[INFO]: Record data for {} seconds...".format(self.recording_time))

        # Use a high-frequency rate to ensure accurate time measurements
        rate = rospy.Rate(1000)  # 1000 Hz rate for accurate time measurements
        start_time = rospy.Time.now()
        curr_time = start_time

        # Record data for the specified duration
        while (curr_time - start_time).to_sec() < self.recording_time:
            rate.sleep()  # Sleep to meet the specified rate
            curr_time = rospy.Time.now()

        with self.lock:
            # Retrieve data from the cache and store it in the class variable
            self.data = [self.cache.get() for _ in range(self.cache.qsize())]

        rospy.loginfo(" Done.")


    def _compute_bias(self, data):
        """
        Compute the mean bias from recorded data.

        Parameters:
        - data (list): List of recorded data samples.
        """
        self.bias = np.mean(data, axis=0)

