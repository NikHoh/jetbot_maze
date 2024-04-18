# Project Seminar 'Robotics and Computational Intelligence' 2024 presented by RIS | Technical University of Darmstadt

More information on the course can be found here: https://www.etit.tu-darmstadt.de/ris/lehre_ris/lehrveranstaltungen_ris/pro_robo_ris/index.de.jsp

This repository serves as a starting base for the students participating in this year's course.

The task will be to program a JetBot that is set in a maze environment. Here, the JetBot is supposed to handle different tasks separately.

The tasks include but are not limited

1) autonomously navigating from a given point A to a given point B in a known maze, and
2) exploring and mapping an unknown maze of unknown size.

Detailed information on the tasks is given internally via Moodle.

![JetBot on its way through a maze](https://github.com/NikHoh/jetbot_maze/blob/main/images/jetbot_in_the_maze.jpg)

The JetBot (to be precise the Jetson Chip on the JetBot) is supposed to run a Linux (Ubuntu) operating system. Thus, it can be connected with a keyboard, mouse, and monitor and behaves like a normal computer.

The Jetson will also run a program called ROS (Robot Operating System).

ROS enables 
- the handling of the complete Sense-Plan-Act pipeline in different sub-programs called nodes,
    - e.g. acquiring measurements of different sensors
    - doing calculations (e.g. filtering) with the measurements
    - implementing some behavior (e.g. as a finite automaton) depending on the given tasks
    - calculating output signals
    - controlling the motors to drive
- as well as the communication (e.g. of ROS node's messages) with other computers in the same network, that may also run (other) ROS nodes, e.g. to save computation resources on the JetBot

In this course, you will use the ROS Version Noetic.

ROS nodes, (i.e. the functionality of the JetBot to solve the tasks) can either be programmed in C or Python, it's up to you. 

This repository will only provide you with the instructions how to set up your JetBot including
- flashing the SD Card with Ubuntu
- installing the necessary software
- installing ROS and needed ROS packages (either provided within this repository or from 3rd party) to
    - acquire raw images of the JetBots mono camera (not calibrated)
    - acquire raw sensor measurements (3DOF acceleration and 3DOF angular velocity) of the attached IMU sensor (not calibrated) (no standard configuration)
    - calculate a transformation (tf) between the camera coordinate frame and the coordinate frame of an AprilTag via the apriltag_ros package
    - calculate a `tags.yaml` file for your specific maze configuration so that the apriltag_ros package is capable of calculating a transformation (tf) between the camera coordinate frame and the maze (world) frame
    - let the JetBot's left and right wheels move at different velocities.
    - acquire some additional (ground truth) localization information from the OptiTrack camera system that our robot lab is equipped with
 
  All other nodes and functionality have to be programmed by yourself. This potentially includes but is not limited to
  - a ROS master-slave setup so that 1) some calculations can be handled on a separate PC, and 2) the JetBot can be controlled via a separate PC (more information below)
  - calibration of the camera
  - calibration of the IMU
  - (optimization of the AprilTag detection results via the `settings.yaml` file in the apriltag_ros package)
  - sensor fusion
  - evaluation and optimization of the JetBot's localization quality within the world frame (x, y, yaw) (e.g. by comparing to some ground truth acquired by the external OptiTrack camera system).
  - trajectory (path) planning
  - trajectory tracking control
  - maze exploration and mapping
  - logic that combines different of the named functionalities (e.g. in ROS `*.launch` files) to solve the different given tasks

Before starting with the instructions, some hints and links are given to provide you bis (basic) knowledge that you should be familiar with before starting to set up the JetBot and solving the tasks.

# Basic Knowledge (all of you should know)

Before starting, some basic knowledge should be available. In case you are not familiar with the following, read or watch some tutorials:
- some basics in Linux (you will use Ubuntu 18.04)
    - basic console commands `cd`, `ls`, `mkdir`, `source`, `cp`, `mv`, `rm`, `chmod`, ...
    - the purpose of `sudo`
    - the purpose of `apt-get`
- some basics in C/C++
    - C/C++ compiling procedure including the purpose of `cmake`, `make` and `CmakeLists.txt`
- some basics in Python
    - the purpose of `pip`
- the purpose of Git as well as basic commands `commit`, `push`, `pull`, `clone`, `fork`, ...
- ROS (you will use the ROS1 version Noetic)
    - *Note: How ROS is installed on the JetBot will be explained further below*
    - do the tutorials at http://wiki.ros.org/ROS/Tutorials
    - RVIZ
    - rqt (e.g. `rqt_graph` )

# Distributed Knowledge (at least one person per group should know)

At least one group participant should be familiar with:
- basic image processing routines
    - pinhole model: http://wiki.ros.org/image_pipeline/CameraInfo
    - camera calibration: http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration
    - rectification: http://wiki.ros.org/image_proc

- coordinate systems and transforms in ROS
    - the helpful tool rqt_tf_tree

- the purpose of AprilTags
    - https://github.com/AprilRobotics/apriltag
    - *there are also some papers ...*


# Getting started

In the following, you are going to set up the basic ROS environment on the JetBot so that the robot will be able to localize itself visually (only with the help of its camera) within a given arena. 
*Note that relying only on the camera as solely localization technique will probably lead to large localization errors due to motion blur and resulting detection errors in the apriltag_ros package, which is why we suggest to also make use of the IMU data.* 

## The maze

The maze can be set up modularly in size and configuration and consits of base plates and wall plates. The wall plates of the maze are laser-engraved with recursive AprilTags that are used by the JetBot to localize itself.
The global coordinate system's origin is set in one corner of the maze.

More information on the arena, e.g. on size of the base and wall plates and how to acquire the `tags.yaml` needed in the apriltag_ros package for localization purposes of the JetBot can be found in a separate repository:

https://github.com/NikHoh/apriltag-maze

*Note: Only the "Set up a maze" part is relevant for you. You do not have to "Craft a maze". Three configurable mazes are located in different rooms of our institute.*

# More

More information on set up, installation and testing of the JetBot will be available after the Hardware handover two weeks after the kickoff of the project seminar.


