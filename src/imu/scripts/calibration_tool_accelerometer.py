#!/usr/bin/env python3

"""
Module/Script Name: accelerometer_calibration.py
Author: Florentin Hau
Date: 30.01.2024

Description:
    Accelerometer Calibration Tool
    
    This Python script servers as a ROS node for recording accelerometer data.
    It allows the user to collect and save samples to a .txt file.
    
    This Python script serves as a ROS node for calibrating an accelerometer. It collects IMU data samples, processes them, and allows the user to save the calibrated data to a file. The tool runs until the specified number of samples is collected or until the user decides to quit.

Classes:
    - AccelerometerCalibration: Class representing the accelerometer calibration tool.

Usage:
    Run the script with the following command:

        "rosrun package_name calibration_tool_accelerometer.py"

    Replace "package_name" with the actual name of your ROS package.

Instructions:
    - Press 'm' to start a measurement.
    - Press 's' to save all previous measurements.
    - Press 'q' to save and quit the tool.
    - Press 'r' to record additional data permanently.
    
    - For each new sample / measurement you have to position the IMU in a different angle. You can use the calibration tool for help.

Note:
    The script initializes a ROS node and a separate thread for subscribing to IMU data messages. The user interacts with the tool through the terminal.

"""

import rospy
import numpy as np
import pandas as pd
import threading
from queue import Queue

from enum import Enum
from sensor_msgs.msg import Imu

class State(Enum):
    """
    Enumeration class representing the possible states of the accelerometer calibration tool.
    """
    WAIT_FOR_INPUT = 0
    MEASUREMENT = 1
    SAVE = 2
    RECORD = 3
    QUIT = -1

class AccelerometerCalibration:
    def __init__(self, filename, max_samples=500, measurements=50):
        """
        Initializes the AccelerometerCalibration object.

        Parameters:
        - max_samples (int): Maximum number of samples to collect.
        - measurements (int): Number of measurements to record for each sample.
        - filename (str): Path to the file where samples will be saved.
        """

        self.MAX_SAMPLES = max_samples
        self.MEASUREMENTS = measurements
        self.FILENAME = filename

        self.samples = []  # List to store collected samples
        self.cache = Queue()  # Queue to temporarily store IMU data
        self.lock = threading.Lock()  # Lock to ensure thread safety
        
        self.subscriber_thread_handle = threading.Thread(target=self.subscriber_thread, daemon=True)
        self.subscriber_thread_handle.start()


    # ----- Public Methods -----

    def run(self) -> None:
        """
        Main entry point for the accelerometer calibration tool.
        """
        self._print_intrudoctions()
        self.state = State.WAIT_FOR_INPUT

        while not rospy.is_shutdown() and len(self.samples) < self.MAX_SAMPLES:
            if self.state is State.WAIT_FOR_INPUT:
                self._handle_waiting_input_state()
            elif self.state is State.MEASUREMENT:
                self._handle_measurement_state()
                self.state = State.WAIT_FOR_INPUT
            elif self.state is State.SAVE:
                self._handle_save_state()
                self.state = State.WAIT_FOR_INPUT
            elif self.state is State.QUIT:
                break
        
        self._handle_quit_state()
        rospy.loginfo("Shutting down...")
        rospy.signal_shutdown(0)


    # ----- Private Methods -----
    
    def callback(self, imu_raw):
        """
        Callback function for handling raw IMU data.
        Data is stored temporarly in cache.
        """
        x, y, z = imu_raw.linear_acceleration.x, imu_raw.linear_acceleration.y, imu_raw.linear_acceleration.z
        with self.lock:
            self.cache.put([x, y, z])  

    def subscriber_thread(self):
        """
        Subscriber thread for listening to raw IMU data.
        """
        sub_imu = rospy.Subscriber("/imu/data_raw", Imu, self.callback)
        rospy.spin()

    def _print_intrudoctions(self) -> None:
        """
        Displays introductory messages for the user.
        """
        rospy.loginfo("[WELCOME]: ACCELEROMETER CALIBRATION TOOL\n")
        rospy.loginfo("Explanation:")
        rospy.loginfo("Press 'm' to start measurement")
        rospy.loginfo("Press 's' to save all previous measurements")
        rospy.loginfo("Press 'q' to save and quit the tool\n")

    def _handle_waiting_input_state(self) -> None:
        """
        Handles the WAIT_FOR_INPUT state.
        """
        print()
        rospy.loginfo(f"Current sample count: {len(self.samples)} (max. 500)")
        user = input("[INPUT]: ")

        if user == 'm':
            self.state = State.MEASUREMENT
        elif user == 's':
            self.state = State.SAVE
        elif user == 'q':
            self.state = State.QUIT

    def _handle_measurement_state(self) -> None:
        """
        Handles the MEASUREMENT state.
        """
        self._record_data()

    def _handle_save_state(self) -> None:
        """
        Handles the SAVE state.
        """
        self._save_samples()

    def _handle_quit_state(self) -> None:
        """
        Handles the QUIT state.
        """
        self._save_samples()

    def _record_data(self) -> None:
        """
        Records IMU data and processes the measurement.
        """
        rospy.loginfo("Do not move the IMU... Measurement is running!")
        rospy.sleep(2)
        rospy.loginfo("Clear cache...")
        with self.lock:
            while not self.cache.empty():
                self.cache.get()

        rospy.loginfo("Record data...")

        while self.cache.qsize() < self.MEASUREMENTS:
            rospy.sleep(0.1)

        with self.lock:
            data = [self.cache.get() for _ in range(self.MEASUREMENTS)]

        rospy.loginfo("Measurement done.")
        sample = self._process_measurement(data)
        with self.lock:
            self.samples.append(sample)
        rospy.loginfo(f"Sample: {sample}")


    def _save_samples(self) -> None:
        """
        Saves the recorded samples to a file.
        """
        rospy.loginfo("Saving samples...")
        self._data_2_txt(self.samples, delimiter='\t')
        rospy.loginfo("Done.")
        rospy.loginfo(f"Samples saved to {self.FILENAME}")

    def _process_measurement(self, data: list) -> list:
        """
        Processes raw IMU data to compute a average measurement sample.

        Parameters:
        - data (list): Raw data from IMU.

        Returns:
        - list: Processed measurement sample.
        """
        ax = ay = az = 0.0

        count = len(data)
        for xyz in data:
            ax += xyz[0]
            ay += xyz[1]
            az += xyz[2]

        ax /= count
        ay /= count
        az /= count

        sample = [ax, ay, az]
        return sample


    def _data_2_txt(self, mylist: list, delimiter: str = ',', f_mode: str = 'w') -> None:
        """
        Converts a list to a Pandas dataframe and saves it as a text file.

        Parameters:
        - mylist (list): List of data.
        - delimiter (str): Delimiter for the text file.
        - f_mode (str): File mode ('w' for write, 'a' for append).
        """
        df = pd.DataFrame(mylist)
        df.to_csv(
            self.FILENAME,  # path and filename
            sep=delimiter,
            mode=f_mode,
            header=False,  # no col. labels
            index=False  # no row numbers
        )


def main() -> None:
    """
    Main function to run the accelerometer calibration tool.
    """
    rospy.init_node("imu_calib_tool")


    calibration_tool = AccelerometerCalibration(filename=rospy.get_param("~imu_calib_path"))
    calibration_tool.run()

if __name__ == "__main__":
    main()
