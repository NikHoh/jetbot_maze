#!/usr/bin/env python3

import rospy
import sys
import os
import select

from motor.msg import MotorPWM

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

JETBOT_MAX_LIN_VEL = 0.4
JETBOT_MAX_ANG_VEL = 2.0

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your JetBot!
------------------------------

Moving around

    f   j
    v   n 

f/v : increase/decrease PWM command left motor
j/n : increase/decrease PWM command right motor

space-key: force stop

CTRL-C to Quit
"""

def get_key(settings):
    if os.name == 'nt':
        return msrcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin],[],[],0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin,termios.TCSADRAIN, settings)
    return key

def make_simple_profile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input

    return output

def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel

def check_linear_limit_velocity(velocity):
    return constrain(velocity, -JETBOT_MAX_LIN_VEL, JETBOT_MAX_LIN_VEL)


def main():
    global JETBOT_MAX_LIN_VEL
    global JETBOT_MAX_ANG_VEL

    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node("teleop_keyboard")
    pub = rospy.Publisher("/motor/pwm_cmd", MotorPWM, queue_size=1)
    
    status = 0
    left_motor_vel = 0.0
    right_motor_vel = 0.0

    rate = rospy.Rate(10)  # 10Hz

    try:
        print(msg)
        while not rospy.is_shutdown():
            key = get_key(settings)

            if key == "f":
                left_motor_vel = check_linear_limit_velocity(left_motor_vel + LIN_VEL_STEP_SIZE)
            elif key == "v":
                left_motor_vel = check_linear_limit_velocity(left_motor_vel - LIN_VEL_STEP_SIZE)
            elif key == "j":
                right_motor_vel = check_linear_limit_velocity(right_motor_vel + LIN_VEL_STEP_SIZE)
            elif key == "n":
                right_motor_vel = check_linear_limit_velocity(right_motor_vel - LIN_VEL_STEP_SIZE)
            elif key == ' ':
                left_motor_vel = 0.0
                right_motor_vel = 0.0
            else:
                if key == "\x03":
                    break
            status += 1

            if status == 20:
                print(msg)

            pwm_cmd_msg = MotorPWM()
            pwm_cmd_msg.pwm_left = left_motor_vel
            pwm_cmd_msg.pwm_right = right_motor_vel

            print(f"Left Motor: {left_motor_vel}, Right Motor: {right_motor_vel}")
            pub.publish(pwm_cmd_msg)

            rate.sleep()

    except Exception as e:
        print(e)

    finally:
        pwm_cmd_msg = MotorPWM()
        pwm_cmd_msg.pwm_left = 0.0
        pwm_cmd_msg.pwm_right = 0.0

        print(f"Left Motor: {left_motor_vel}, Right Motor: {right_motor_vel}")
        pub.publish(pwm_cmd_msg)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
            