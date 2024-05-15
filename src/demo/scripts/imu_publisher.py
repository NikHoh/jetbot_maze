#!/usr/bin/env python3
import rospy
import tf2_ros
from sensor_msgs.msg import (
    Imu,
    MagneticField
)
from geometry_msgs.msg import TransformStamped

import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER, 
    BNO_REPORT_GYROSCOPE, 
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR
)
from adafruit_bno08x.i2c import BNO08X_I2C

G_ACCEL = 9.810576  # g in Frankfurt

# Set up communication with the BNO08X IMU via the I2C bus
i2c = busio.I2C(board.SCL_1, board.SDA_1)
bno = BNO08X_I2C(i2c)

bno.initialize()

# Activate sensores and reports
bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_MAGNETOMETER)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

# Start ROS node
rospy.init_node("imu_reader")

# Initialise publisher and message
imu_pub = rospy.Publisher("/imu/data_raw", Imu, queue_size=1)
mag_pub = rospy.Publisher("/imu/mag", MagneticField, queue_size=1)
tf_broadcaster = tf2_ros.TransformBroadcaster()
rate = rospy.Rate(20)   # 20Hz

imu_msg = Imu()
imu_msg.header.frame_id = "imu_sensor"
imu_seq = 0

mag_msg = MagneticField()
mag_msg.header.frame_id = "imu_sensor"
mag_seq = 0

tf_msg = TransformStamped()
tf_msg.header.frame_id = "odom"
tf_msg.child_frame_id = "demo_imu_frame"
tf_seq = 0



rospy.loginfo("Sending raw IMU data...")

# Main loop
while not rospy.is_shutdown():
    # Read data
    accel_x, accel_y, accel_z = bno.acceleration
    gyro_x, gyro_y, gyro_z = bno.gyro
    mag_x, mag_y, mag_z = bno.magnetic
    quat_x, quat_y, quat_z, quat_w = bno.quaternion
    
    stamp = rospy.Time.now()
    
    # Message Imu
    # Update header
    imu_msg.header.stamp = stamp
    imu_msg.header.seq = imu_seq
    imu_seq += 1
    
    # Linear acceleration
    imu_msg.linear_acceleration.x = accel_x / G_ACCEL
    imu_msg.linear_acceleration.y = accel_y / G_ACCEL
    imu_msg.linear_acceleration.z = accel_z / G_ACCEL
    imu_msg.linear_acceleration_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    
    # Angular velocity
    imu_msg.angular_velocity.x = gyro_x
    imu_msg.angular_velocity.y = gyro_y
    imu_msg.angular_velocity.z = gyro_z
    imu_msg.angular_velocity_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    # Orientation - Quaternion
    imu_msg.orientation.x = quat_x
    imu_msg.orientation.y = quat_y
    imu_msg.orientation.z = quat_z
    imu_msg.orientation.w = quat_w
    imu_msg.orientation_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Message MagneticField
    # Update header
    mag_msg.header.stamp = stamp
    mag_msg.header.seq = mag_seq
    mag_seq += 1

    # Magnetic field
    mag_msg.magnetic_field.x = mag_x
    mag_msg.magnetic_field.y = mag_y
    mag_msg.magnetic_field.z = mag_z
    mag_msg.magnetic_field_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    ## TF Message
    # Update header
    tf_msg.header.stamp = stamp
    tf_msg.header.seq = tf_seq
    tf_seq += 1

    # TF
    tf_msg.transform.translation.x = 0.0
    tf_msg.transform.translation.y = 0.0
    tf_msg.transform.translation.z = 0.0
    tf_msg.transform.rotation = imu_msg.orientation

    # Publish messages
    imu_pub.publish(imu_msg)
    mag_pub.publish(mag_msg)
    tf_broadcaster.sendTransform(tf_msg)
    
    rate.sleep()



