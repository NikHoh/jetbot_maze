#!/usr/bin/env python3

"""
Module/Skript Name: motor_control.py
Author: Florentin Hau
Date: 15.02.2024


Description:
    Motor Control: RPM to PWM Command Converter

    This script serves as a motor control interface, acting as a bridge between motor RPM values and the corresponding PWM commands sent to the motors.
    It utilizes motor characteristics, including polynomial functions representing the motor's RPM-to-PWM relationship.

    
Classes:
    - PWMCMDPublisher: Handles the publishing of scaled PWM commands for left and right motors.
    - RPM2PWMCMDConverter: Converts RPM values to scaled PWM commands using motor characteristics.
    - MotorControlNode: ROS node that subscribes to desired RPM messages, converts them to PWM commands using motor characteristics, and publishes the commands.

    
Usage:
    This script is intended to be used as a ROS node. Run it as follows:
    
        "rosrun package_name motor_control.py"
    
    Make sure to replace "package_name" with the actual name of your ROS package.

"""

import rospy
import math
import sys
import ast
import yaml
import numpy as np
from motor.msg import MotorPWM, MotorRPM

class PWMCMDPublisher:
    def __init__(self):
        """
        Initialize the PWMCMDPublisher class.

        This class is responsible for publishing scaled PWM commands for motors.

        Attributes:
        - pub_pwmcmd (rospy.Publisher): ROS publisher for MotorPWM messages.
        - msg_pwmcmd (MotorPWM): Instance of MotorPWM message for publishing PWM commands.
        """
        self.pub_pwmcmd = rospy.Publisher("/motor/pwm_cmd", MotorPWM, queue_size=1)
        self.msg_pwmcmd = MotorPWM()

    def publish_pwm_cmd(self, pwmcmd_left, pwmcmd_right) -> None:
        """
        Publish PWM commands for left and right motors.

        Parameters:
        - pwmcmd_left (float): PWM command for the left motor.
        - pwmcmd_right (float): PWM command for the right motor.
        """
        self.msg_pwmcmd.pwm_left = pwmcmd_left
        self.msg_pwmcmd.pwm_right = pwmcmd_right
        self.pub_pwmcmd.publish(self.msg_pwmcmd)
        
        
class RPM2PWMCMDConverter:
    MAX_VOLTAGE = 6.0
    DRIVER_VOLTAGE = 12.0

    def __init__(self, left_ideal_coef, right_ideal_coef, left_coef, right_coef):
        """
        Initialize the RPM2PWMCMDConverter class.

        This class is responsible for converting RPM values to scaled PWM commands.

        Parameters:
        - left_coef (list): Coefficients for the left motor RPM to PWM command polynomial.
        - right_coef (list): Coefficients for the right motor RPM to PWM command polynomial.

        Attributes:
        - left_rpm2pwmcmd (numpy.poly1d): Polynomial function for left RPM to PWM command.
        - right_rpm2pwmcmd (numpy.poly1d): Polynomial function for right RPM to PWM command.
        - left_pwmcmd2rpm (numpy.poly1d): Inverse polynomial function for left PWM command to RPM.
        - right_pwmcmd2rpm (numpy.poly1d): Inverse polynomial function for right PWM command to RPM.
        - max_rpm (float): Maximum RPM allowed for the motor.
        - scale_ratio (float): Ratio for scaling PWM commands based on voltage limits.
        """
        if len(left_coef) == 2:
            self.left_rpm2pwmcmd = lambda y: 1 / left_coef[0] * y
            self.left_pwmcmd2rpm = lambda x: left_coef[0] * x
            
        else:
            rospy.logerr(f"Left motor characteristic is faulty: {left_coef}")
            rospy.loginfo("Shutting down...")
            exit(1)
            
        if len(right_coef) == 2:
            self.right_rpm2pwmcmd = lambda y: 1 / right_coef[0] * y
            self.right_pwmcmd2rpm = lambda x: right_coef[0] * x
            
        else:
            rospy.logerr(f"Right motor characteristic is faulty: {right_coef}")
            rospy.loginfo("Shutting down...")
            exit(1)
       
       
        self.left_ideal_pwmcmd2rpm = lambda x: left_ideal_coef[0] * x
        self.right_ideal_pwmcmd2rpm = lambda x: right_ideal_coef[0] * x
       
        self.max_rpm = min(self.left_pwmcmd2rpm(1.0), self.right_pwmcmd2rpm(1.0))
        
        self.scale_ratio = self.MAX_VOLTAGE / self.DRIVER_VOLTAGE

    def convert(self, pwm_left, pwm_right) -> tuple:
        """
        convert PWM values to ideal RPM values.

        Parameters:
        - pwm_left (float): PWM value for the left motor.
        - pwm_right (float): PWM value for the right motor.

        Returns:
        - tuple: ideal RPM values for left and right motors.
        """
        return self._left_ideal_pwmcmd2rpm(pwm_left), self.right_ideal_pwmcmd2rpm(pwm_right)
        
        


    def convert_scaled(self, rpm_left, rpm_right) -> tuple:
        """
        convert RPM values to scaled PWM commands.

        Parameters:
        - rpm_left (float): RPM value for the left motor.
        - rpm_right (float): RPM value for the right motor.

        Returns:
        - tuple: Scaled PWM commands for left and right motors.
        """
        left_is_neg = rpm_left < 0
        right_is_neg = rpm_right < 0
        
        rpm_left = abs(rpm_left)
        rpm_right = abs(rpm_right)
        
        if rpm_left > self.max_rpm or rpm_right > self.max_rpm:
            left_pwmcmd = self.left_rpm2pwmcmd(self.max_rpm)
            right_pwmcmd = self.right_rpm2pwmcmd(self.max_rpm)
        else:
            left_pwmcmd = self.left_rpm2pwmcmd(rpm_left) if rpm_left != 0.0 else 0.0
            right_pwmcmd = self.right_rpm2pwmcmd(rpm_right) if rpm_right != 0.0 else 0.0
            
        if left_is_neg:
            left_pwmcmd = -left_pwmcmd
        if right_is_neg:
            right_pwmcmd = -right_pwmcmd
            
        return left_pwmcmd, right_pwmcmd
        

        
class MotorControlNode:
    def __init__(self):
        """
        Initialize the MotorControlNode class.

        This class represents the ROS node for converting RPM to PWM commands.

        Attributes:
        - publisher (PWMCMDPublisher): Instance of PWMCMDPublisher for publishing PWM commands.
        - converter (RPM2PWMCMDConverter): Instance of RPM2PWMCMDConverter for converting RPM to PWM.
        - timeout_duration (rospy.Duration): Duration for checking message timeout.
        - last_msg_time (rospy.Time): Time of the last received RPM message.
        """
        rospy.init_node("motor_control", anonymous=True)

        self.config_data = self._load_yaml(rospy.get_param("~motor_config_path"))

        self.publisher = PWMCMDPublisher()
        self.converter = RPM2PWMCMDConverter(
                left_ideal_coef=self.config_data["ideal_motor_characteristic"]["left"]["data"],
                right_ideal_coef=self.config_data["ideal_motor_characteristic"]["right"]["data"],
                left_coef=self.config_data["motor_characteristic"]["left"]["data"],
                right_coef=self.config_data["motor_characteristic"]["right"]["data"]
        )


    # ----- Public Methods -----

    def run(self) -> None:
        """
        Run the MotorControlNode ROS node.

        This method initializes the subscriber callbacks, then spins the ROS node.
        """
        subscriber = rospy.Subscriber("/motor/pwm_soll", MotorPWM, callback=self._callback)
        rospy.loginfo("Waiting for PWM messages...")
        rospy.spin()


    # ----- Private Methods -----

    def _load_yaml(self, path: str):
        """
        Function to load camera data:
            
        Parameters:
        - path (str): Path of yaml file
        """
        with open(path, "r") as file:
            data = yaml.safe_load(file)
        return data

    def _callback(self, msg) -> None:
        """
        Callback function for handling received PWM messages and convert it to ideal RPM Values.

        Parameters:
        - msg (MotorRPM): Received MotorRPM message.
        """
        self.last_msg_time = rospy.Time.now()

        pwm_left = msg.pwm_left
        pwm_right = msg.pwm_right
        
        rpm_left, rpm_right = self.converter.convert(pwm_left, pwm_right)

        pwmcmd_conv_left, pwmcmd_conv_right = self.converter.convert_scaled(rpm_left, rpm_right)
        
        rospy.loginfo(f"Left: PWM {pwmcmd_conv_left}, Right: PWM {pwmcmd_conv_right}")
        self.publisher.publish_pwm_cmd(pwmcmd_conv_left, pwmcmd_conv_right)


def main() -> None:
    """
    Main function to run the MotorControlNode node.
    """
    motor_control = MotorControlNode()
    motor_control.run()

if __name__ == "__main__":
    main()
