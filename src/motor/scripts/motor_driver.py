#!/usr/bin/env python3

"""
Module/Skript Name: motor_driver.py
Author: Florentin Hau
Date: 26.01.2024


Description:
    Motor Driver: PWM to Motor Voltage Converter

    This script functions as a ROS node for controlling motors based on received PWM (Pulse Width Modulation) messages. It uses the Adafruit MotorHAT library to adjust motor voltages accordingly.

    
Classes:
    - MotorDriver: ROS node for controlling motors based on PWM messages.

    
Usage:
    This script is intended to be used as a ROS node. Run it as follows:

        "rosrun package_name motor_controller.py"    

    Replace "package_name" with the actual name of your ROS package.

    
"""

import rospy
import rosnode

from Adafruit_MotorHAT import Adafruit_MotorHAT
from motor.msg import MotorPWM

class MotorDriver:
    MOTOR_LEFT = 1
    MOTOR_RIGHT = 2
    
    MAX_VOLTAGE = 6.0
    DRIVER_VOLTAGE = 12.0
    
    def __init__(self):
        """
        Initializes the MotorDriver node.

        This class is responsible for controlling the motor voltages based on
        received PWM (Pulse Width Modulation) messages.

        Attributes:
        - MOTOR_LEFT (int): Identifier for the left motor.
        - MOTOR_RIGHT (int): Identifier for the right motor.
        - max_pwm (int): Maximum PWM value for 8-bit PWM control.
        - driver (Adafruit_MotorHAT): Adafruit MotorHAT driver object.
        - motors (dict): Dictionary containing motor objects for left and right motors.
        - pwm_channels (dict): Dictionary mapping motor IDs to PWM channels.
        - last_pwm_l (float): Previous left motor voltage value.
        - last_pwm_r (float): Previous right motor voltage value.
        - timeout_duration (rospy.Duration): Duration for checking message timeout.
        - last_msg_time (rospy.Time): Time of the last received RPM message.
        """
        rospy.init_node("motor_driver", anonymous=True)
        
        self.max_pwm = 255
        self.scale_ratio = self.MAX_VOLTAGE / self.DRIVER_VOLTAGE
        
        # Initialize Adafruit MotorHAT driver
        self.driver = Adafruit_MotorHAT(i2c_bus=1)
        
        # Get motor objects from the driver
        self.motors = {
            self.MOTOR_LEFT: self.driver.getMotor(self.MOTOR_LEFT),
            self.MOTOR_RIGHT: self.driver.getMotor(self.MOTOR_RIGHT)
        }
        
        # Map motor IDs to PWM channels
        self.pwm_channels = {
            self.MOTOR_LEFT: (1, 0),
            self.MOTOR_RIGHT: (2, 3)
        }
        
        self.last_pwm_l = -999
        self.last_pwm_r = -999
        
        self.timeout_duration = rospy.Duration(1.0)
        self.last_msg_time = rospy.Time.now()
        rospy.Timer(self.timeout_duration, callback=self._check_timeout)
        self.sub = rospy.Subscriber("/motor/pwm_cmd", MotorPWM, self._pwm_listener, queue_size=1)
        


    # ----- Public Methods -----
        
    def destroy_node(self) -> None:
        """
        Stops the ROS node and performs necessary cleanup.
        """
        rospy.loginfo("Stopping robot...")
        self.stop()
        
    def stop(self) -> None:
        """
        Stops both motors by setting their speeds to 0.
        """
        self.set_speed(0.0, 0.0)
        
    def set_speed(self, left: float, right: float) -> None:
        """
        Sets the motor speeds based on received voltage values.

        Args:
        - right (float): Voltage value for the right motor in the range [-1.0, 1.0].
        - left (float): Voltage value for the left motor in the range [-1.0, 1.0].
        """
        scaled_left = self._scale(left)
        scaled_right = self._scale(right    )
        
        rospy.loginfo(f"Left Motor Voltage: {scaled_left * self.DRIVER_VOLTAGE:.03f}, Right Motor Voltage: {scaled_right  * self.DRIVER_VOLTAGE:.03f}")
        
        self._set_pwm(self.MOTOR_LEFT, -scaled_left)
        self._set_pwm(self.MOTOR_RIGHT, -scaled_right)
        

    # ----- Private Methods -----
    
    def _scale(self, value) -> float:
        """
        Scale the PWM command value.

        Parameters:
        - value (float): PWM command value.

        Returns:
        - float: Scaled PWM command value.
        """
        scaled_value = min(abs(value) * self.scale_ratio, self.scale_ratio)

        if value < 0:
            scaled_value = -scaled_value

        return scaled_value
    
    
    def _check_timeout(self, event) -> None:
        """
        Check for message timeout and publish zero PWM commands if timeout occurs.

        Parameters:
        - event: Timer event triggering the check.
        """
        if rospy.Time.now() - self.last_msg_time > self.timeout_duration:
            rospy.loginfo("No PWM commands received. Motor state: OFF")
            self.stop()
    
    def _set_pwm(self, motor: int, value: float) -> None:
        """
        Sets the PWM value for a given motor based on the received pwm command value.

        Args:
        - motor (int): Motor identifier (MOTOR_LEFT or MOTOR_RIGHT).
        - value (float): Voltage value in the range [-1.0, 1.0].
        """
        pwm_value = int(min(max(abs(value) * self.max_pwm, 0.0), self.max_pwm))
        
        self.motors[motor].setSpeed(pwm_value)
        
        if value > 0:
            self.motors[motor].run(Adafruit_MotorHAT.BACKWARD)
        elif value < 0:
            self.motors[motor].run(Adafruit_MotorHAT.FORWARD)
        else:
            self.motors[motor].run(Adafruit_MotorHAT.RELEASE)
            
    def _pwm_listener(self, msg: MotorPWM) -> None:
        """
        Callback function for receiving PWM messages.

        Adjusts motor speeds based on received PWM values.

        Args:
        - msg (MotorPWM): ROS message containing PWM values for left and right motors.
        """
        self.last_msg_time = rospy.Time.now()
        
        pwm_l = msg.pwm_left
        pwm_r = msg.pwm_right
        
        if pwm_l == self.last_pwm_l and pwm_r == self.last_pwm_r:
            return
        
        self.last_pwm_l = pwm_l
        self.last_pwm_r = pwm_r  
        
        self.set_speed(pwm_l, pwm_r)


def main() -> None:
    """
    Main function to instantiate and run the MotorDriver node.
    """
    motor_controller = MotorDriver()
    rospy.loginfo("Listening for PWM commands...")
    
    rospy.spin()
    motor_controller.destroy_node()
    
    

if __name__ == "__main__":
    main()
