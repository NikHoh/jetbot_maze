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
- some basics in Linux (you will use Ubuntu 20.04)
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
	
- the basic functionality of an IMU

- the kinematic model of a differential drive robot 


# Getting started

In the following, you are going to set up the basic ROS environment on the JetBot so that the robot will be able to localize itself visually (only with the help of its camera) within a given arena. 
*Note that relying only on the camera as solely localization technique will probably lead to large localization errors due to motion blur and resulting detection errors in the apriltag_ros package, which is why we suggest to also make use of the IMU data.* 

## The maze

The maze can be set up modularly in size and configuration and consits of base plates and wall plates. The wall plates of the maze are laser-engraved with recursive AprilTags that are used by the JetBot to localize itself.
The global coordinate system's origin is set in one corner of the maze.

More information on the arena, e.g. on size of the base and wall plates and how to acquire the `tags.yaml` needed in the apriltag_ros package for localization purposes of the JetBot can be found in a separate repository:

https://github.com/NikHoh/apriltag-maze

*Note: Only the "Set up a maze" part is relevant for you. You do not have to "Craft a maze". Three configurable mazes are located in different rooms of our institute. If you want to localize the cube, you have to add 6 more standalone-tags to the 'tags.yaml' - see below*

## General coding advice

Carefully follow the the following instructions. **IMPORTANT: Don't copy paste commands blindfold. Try to understand what's the purpose of the command and also read what happens in the console output.** In case of some errors:
```python
def in_case_of_error():
    if an_error_occured_before_that_you_missed:
        in_case_of_error()
    
    read_the_error_message # very important!!!
    
    if you_can_find_out_where_the_error_originates_from:
        solve_the_error()
        return

    for _ in range(5):
        if you_can_google_the_error_and_find_some_help_in_an_online_forum:
            solve_the_error()
            return 

    post_a_message_in_the_moodle_forum()

    if get_help_from_moodle_forum:
        solve_the_error()
        return
    else: 
        solve_the_error()
        write_answer_to_moodle_forum()
        return
```

## JetBot installation

Follow these steps to setup your JetBot:

1. **Download Jetson Nano Ubuntu 20 Image**:
   - Download the image [here](https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image?tab=readme-ov-file).
   - Password: jetson

2. **Install the Image**:
   - Use a tool like [Etcher](https://etcher.balena.io/) to install the downloaded image onto your SD-Card.

3. **Change Keyboard Layout**:
   - Navigate to: Start >> Settings >> Region & Language >> + >> German (T3).

4. **Resize the Partition**:
   - Install GParted: `sudo apt-get install gparted`.
   - Launch GParted: `gparted`.
     - Navigate to Partition >> Resize/Move.
     - Set APP to new size (MiB) to 58000.
     - Click Resize and then Apply all operations with ✅.

5. **Install ROS Noetic**:
   - Follow the instructions on the [ROS Noetic Installation page](http://wiki.ros.org/noetic/Installation/Ubuntu) and install the Desktop-Full Version.

6. **Source ROS Noetic Setup Script**
   - Set setup script to the Shell configuration file:
     - `gedit ~/.bashrc`
     - add: `source /opt/ros/noetic/setup.bash`

7. **Install Catkin Tools**:
   - ```sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'```
   - `wget http://packages.ros.org/ros.key -O - | sudo apt-key add -`
   - `sudo apt-get update`
   - `sudo apt-get install python3-catkin-tools`
   - For more information click [here](https://catkin-tools.readthedocs.io/en/latest/installing.html).

8. **Create Workspace**:
   - Create a workspace directory: `mkdir workspace`.
   - Navigate to the workspace: `cd workspace`.
   - Create a Catkin workspace: `mkdir catkin_ws`.
   - Navigate to the Catkin workspace: `cd catkin_ws`.
   - Create a Source directory: `mkdir src`.
     - Initiate Catkin Workspace with: `catkin init`.
     - Build your workspace with: `catkin build`.

9. **Source your Workspace Setup Script**:
   - Add setup script to the Shell configuration file:
     - Open the configuration file: `gedit ~/.bashrc`
     - Add the path to your `setup.bash` file from your workspace, e.g.:
       - `source ~/workspace/catkin_ws/devel/setup.bash`

10. **Install Additional Libraries**:
   - Install jetson-inference:
     - Refer to the installation guide for jetson-inference [here](https://github.com/dusty-nv/jetson-inference/blob/master/docs/building-repo-2.md).
     - without PyTorch!
   - Install 2JetsonGPIO:
     - Follow the installation guide for 2JetsonGPIO [here](https://github.com/pjueon/JetsonGPIO/blob/master/docs/installation_guide.md).
   - Install CircuitePython:
     - `sudo pip3 install adafruit-circuitpython-bno08x`.
   - Install Adafruit_MotorHAT:
     - sudo pip install Adafruit_MotorHAT

11. **Install Additional ROS Packages**:
	- `sudo apt-get install ros-noetic-apriltag-ros`.
    - `sudo apt-get install ros-noetic-imu-filter-madgwick`.
    - `sudo apt-get install ros-noetic-image-pipeline`.
	
12. Zip download or clone this repository and put the files from the `src` and the `config` folder into the correct positions of your workspace according to the following file structure. Then build the workspace again with `catkin build`.
	
## File structure

The file structure that you will create will look like: 

```
|-- workspace
  |-- jetson-inference
  |   |-- ```
  |-- catkin_ws
  |   |-- build
  |   |   |-- ```
  |   |-- devel
  |   |   |-- ```
  |   |-- logs
  |   |   |-- ```
  |   |-- src
  |   |   |-- camera (*inserted from this repo*)
  |   |   |-- demo (*inserted from this repo*)
  |   |   |-- imu (*inserted from this repo*)
  |   |   |-- motor (*inserted from this repo*)
  |   |   |-- ...
  |   |-- config
  |   |   |-- camera (*inserted from this repo*)
  |   |   |-- imu (*inserted from this repo*)
  |   |   |-- motor  (*inserted from this repo*)
  
```

## Testing JetBot
Let's check that the different components of the robot are working under ROS.
First open a new terminal, and start `roscore`.

Important: There might be some problem when trying to rosrun python files saying that the file is not executable. You then have to change the Linux file permissions to executable (see [here](https://askubuntu.com/questions/229589/how-to-make-a-file-e-g-a-sh-script-executable-so-it-can-be-run-from-a-termi), there is also a GUI version). 

### Running the motors
Open a new terminal, and start the *motor_driver* node:

`rosrun demo motor_driver.py`

This Node will listen to the following topic:
- */motor/pwm_cmd*: 
   - Type: *motor/MotorPWM* 
   - Values: -1.0, ..., 1.0

Open a new terminal and run a some test commands:(it is recommended to initially test with JetBot up on blocks, wheels not touching the ground)
- `rostopic pub /motor/pwm_cmd motor/MotorPWM "pwm_left: 0.4 pwm_right: 0.4"`
   - Note: press *TAB* after the topic "/motor/pwm_cmd" to autocomplete the topic type and get the right message format. Change `0.0` to a value between `-1.0` and `1.0`.
   - The wheels will spin for only one second. The motor driver has an integrated safety feature that causes the motors to stop if it doesn't receive any new PWM commands for one second.

- `rostopic pub -r 10 /motor/pwm_cmd motor/MotorPWM "pwm_left: 0.4 pwm_right: 0.4"`
   - Note: press *TAB* after the topic "/motor/pwm_cmd" to autocomplete the topic type and get the right message format. Change `0.0` to a value between `-1.0` and `1.0`
   - The message will be published at a rate of `10Hz`.
   - The wheels will keep spinning until CTRL-C is pressed.

### Using the Keyboard control
Make sure the motor node is still running.

Open a new terminal and start the teleoperation node:

`rosrun demo teleop_keyboard.py`

This node allows you to control the left and right motor with your keyboard.
- Press `f/v` to increase/decrease the PWM command for the left motor.
- Press `j/n` to increase/decrease the PWM command for the right motor.
- Press `SPACE` to stop both motors.
- Press `CTRL-C` to quit.

### Using the camera
To begin streaming the JetBot camera, start the jetbot_camera node:

`rosrun demo camera.py`

The video frames will be published to the `/mono_cam/image_raw` topic as sensor_msgs::Image messages with BGR8 encoding. To test the camera feed open `rqt` in a new consol tab and select *Plugins >> Visualization >> Image View*. Next select the camera topic in the drop down menu.


### Using the IMU
To check if the IMU on the robot is functioning, start the node *imu_publisher* with 

`rosrun demo imu_publisher.py`.

If data is being read successfully, it will be published on the topics
- `/imu/data_raw` (Orientation, angular velocity, linear acceleration) and
- `/imu/mag` (Magnetic field)

You can show all existing topics using `rostopic list` and inspect individual ones with `rostopic echo [TOPIC_NAME]`

In addition you can visualize the orientation in *RVIZ*:
1. Open `rviz` in a new consol tab.
2. Change *Fixed Frame* to `odom`.
3. *Add* a new viszualization: *TF*.
4. Enable frames you want to see. (TF >> Frames >> `demo_imu_frame`). You can also disable `odom`, because it is the fixed world frame.
5. Feel free to move the IMU and observe how the coordinate system moves along with it.


### Build apriltag

Follow the instructions from:

https://github.com/AprilRobotics/apriltag_ros

Important: Since, you have already installed the apriltag_ros package globally on your JetBot with the command `sudo apt-get install ros-noetic-apriltag-ros`, you can skip the instructions to install it in your catkin workspace. It is already installed! The path to the package is `/opt/ros/noetic/share/apriltag_ros/`.

Make sure that you don't copy/paste comments from the section "Quickstart" blindfold. Use the correct `src` folder (see the file structure above).

#### Test apriltag

More information about apriltag_ros: http://wiki.ros.org/apriltag_ros

Have a look at the tutorial to get an idea of what is happening: http://wiki.ros.org/apriltag_ros/Tutorials/Detection%20in%20a%20video%20stream

To work properly, apriltag_ros needs:
- A **calibrated camera**
    - a default camera calibration is already available in the repo (ost.yaml)
    - **BUT**: every camera behaves differently, so you should calibrate your camera yourselves:
    ```bash
    # Run camera node:
    rosrun demo camera.py

    # make sure the camera node is running and publishing /mono_cam/image_raw and /mono_cam/camera_info topics
    rostopic list

    # Run calibration node: (You have to adjust board size and square size!)
    rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/mono_cam/image_raw camera_name:=/mono_cam
    ```
    - Move the checkerboard as shown [here](https://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration#Moving_the_Checkerboard) to gather samples.
      - Hint: large checkerboards for calibration are available at the institute, you can use them there (write an e-mail to ask or come into the office of a WiMi)
    - Replace the values in `~/workspace/catkin_ws/config/camera/ost_mono.yaml`.

- A **rectified image**
    - The image `image_raw` can be rectified with the help of the [ROS image_proc library](http://wiki.ros.org/image_proc).
    - The respective node is already included in the launch file, which is described next.
    - You don't have to do something here but should understand how image_proc is integrated into the launch file.
    ```launch
    <launch>
      <!--Run camera node-->
      <include file="$(find camera)/launch/mono_cam.launch"/>

      <!--Run image rectification node-->
      <arg name="ROS_NAMESPACE" default="mono_cam"/>
      <node pkg="image_proc" type="image_proc" name="image_proc" ns="$(arg ROS_NAMESPACE)"/>
    </launch>
    ```
    - To run the launch file enter `roslaunch camera mono_cam_rect.launch`.
    - As before you can see the camera feed via `rqt`. Now you can choose between the raw image and differently processed images. The most relevant one is `/mono_cam/image_rect`.

- A settings.yaml and tags.yaml configuration set
    - you can find them both in the folder `config/apriltag` in this repo
    - **Important**: copy them to the correct position at `/opt/ros/noetic/share/apriltag_ros/config`. To be able to do this you have to open the folder "as Root" (Right click >> Open as Root >> `jetson`)

#### Test the cube

One task is to detect a cube in the maze. 
A 3x3x3 cm cube is placed in the maze. The cube is marked on each side with an apriltag (27 mm wide, size = 16.2 mm). The six apriltags of the cube are stored in the `tags.yaml` as standalone tags. 

Important: if you build your `tags.yaml` file with the `maze_builder`, the six standalone tags must be added manually:

```python
standalone_tags:
  [
    {id: 992, size: 0.0162},
    {id: 993, size: 0.0162}, 
    {id: 994, size: 0.0162}, 
    {id: 995, size: 0.0162}, 
    {id: 996, size: 0.0162}, 
    {id: 997, size: 0.0162} 
  ]
```

To build the cube with the April tags yourself, you can use the PDF `cube_layout_16,2mm.pdf` in the `images` folder:

1. print the PDF (make sure that no scaling is done during the printing process - 100% scaling, the boxes should be exactly 3mm wide)
2. cut open the rolled-up mesh of the cube
3. attach double-sided tape to the tags 
4. place the cube on the tags and roll it over all the other tags. Make sure there are as few edges and creases in the paper as possible.


## How to...
### IMU Calibration - Accelerometer
To obtain high-quality data from the IMU, and specifically from the accelerometer, it is essential to calibrate it, as there are no ideal sensors in practice. In our case, only the stationary behavior is considered. In the absence of motion, the only force acting is the Earth's gravitational field, leading to an acceleration vector magnitude of `1g`. In three-dimensional space, this means that the measured values should align with the surface of the unit sphere.

To collect data for the calibration process, hold the IMU still in various orientations and read the current values. 
There is a 3D-printed IMU calibration tool available for this at the institute that you can use (write an e-mail to ask or come into the office of a WiMi). 
Next run the `imu_calibration.launch` file with
```bash
roslaunch imu imu_calibration.launch
```
and follow the instructions. Samples will be saved in `~/workspace/catkin_ws/config/imu/acceleration_data_raw.txt`.

To process this data use [*Magneto v1.2*](https://sites.google.com/view/sailboatinstruments1/a-download-magneto-v1-2).
- Open the `acceleration_data_raw.txt` file with *Magneto v1.2* and click *Calibrate*.
- Open `~/workspace/catkin_ws/config/imu/imu_config.yaml` and adjust the `accelerometer_matrix A^(-1)` and the `accelerometer_bias b` accordingly.

You should then be able to listen to the calibrated IMU data: 
`rosrun imu calibration_node.py` 
(have a look into this python file for further explanation)

## ROS Master Slave Setup
- install Ubuntu 20.04 and ROS Noetic on another computer (virtual maschine possible) and try to connect the robot the the computer via ROS:
    - http://wiki.ros.org/ROS/Tutorials/MultipleMachines
    - Hints:
        - both machines must run Ubuntu 20.04 
        - check that openssh-client and openssh-server are installed on both devices (https://www.cyberciti.biz/faq/how-to-install-ssh-on-ubuntu-linux-using-apt-get/)
        - check with `sudo service ssh status` on both devices (https://kinsta.com/knowledgebase/ssh-connection-refused/)
        - if JetBot and your PC are connectd to the same network and you have retrieved the IP adresses of both devices for example as:
            - JetBot: `172.25.1.217` and
            - PC: `172.25.1.108`
        - you should then be able to ssh from your PC to the JetBot by `ssh jetbot@172.25.1.217`

        - for more troubleshotting this may be helpful: 
            - http://wiki.ros.org/ROS/NetworkSetup
            - http://wiki.ros.org/ROS/Troubleshooting
    - then you can test the JetBot without using its GUI




