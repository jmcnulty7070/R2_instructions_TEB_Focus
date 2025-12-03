Rosmaster_R2_Master_Manual.md

------------------------------------------------------------
📘 ROSMaster R2 — Complete Robotics Manual
------------------------------------------------------------

Version: 1.0
Platform: ROS1 Noetic / Melodic
Robot: Yahboom Rosmaster R2
Authoring: Combined and rewritten from 25 original PDFs
Format: A1 — Clean professional Markdown

Table of Contents

Introduction

Safety & Power

Mechanical Assembly

Expansion Board

Motor Drivers

Teleoperation Modes

Joystick Control

STM32 Tutorials

FreeRTOS

PID & Kinematic Theory

ROS Installation

ROS Workspace Setup

URDF & TF

Sensors (Camera, LiDAR, IMU)

Mapping (Cartographer)

Localization

Navigation (TEB + Costmaps)

Autopilot Using Maps

AI Vision (Segmentation)

YOLO Object Detection

TensorRT Acceleration

Data Collection for Training

Model Training & Deployment

Voice Control

Troubleshooting

Glossary

Command Cheatsheets

Index

------------------------------------------------------------
CHAPTERS 1–3
------------------------------------------------------------
1. Introduction

The Yahboom Rosmaster R2 is an educational and research-grade Ackermann-steering robotic platform designed for:

ROS1-based navigation

Real-time perception

Sensor fusion using LiDAR + IMU + Vision

Autonomous waypoint racing

AI-powered driving (YOLO + Segmentation)

Open-source development

This manual consolidates and rewrites all original tutorial PDFs into a single structured technical document.

2. Safety & Power
Power Domains
Component	Voltage	Notes
LiPo Battery	7.4–12.6V	Main motor power
Motor Driver	12V	Drives steering/drive motors
STM32 Mainboard	5V	Logic power
Jetson Board	5V @ 4–5A	MUST be stable
Sensors	5V	Camera, LiDAR, IMU
Safety Guidelines

Never connect/disconnect power while the robot is active.

Keep LiPo batteries balanced and monitored.

Prevent wires from touching steering linkages.

Secure the Jetson cooling fan and use MAXN mode for performance.

Avoid operating the robot on elevated surfaces.

3. Mechanical Assembly
3.1 Chassis

Attach battery tray

Route wires cleanly

Bolt lower carbon plates securely

3.2 Steering System (Ackermann)

Center the steering servo before tightening linkages

Ensure no mechanical binding

Verify full left/right symmetric lock

3.3 Motor Assembly

Mount rear drive motor

Ensure coupler is aligned

Connect to motor driver OUT terminals

3.4 Upper Deck Install

Order of install recommended:

Jetson board

STM32 control board

IMU (centered)

LiDAR (front, level)

Camera (front, horizontal)

3.5 Wheels & Suspension

Ensure wheels spin freely

Tighten but do not over-tighten wheel nuts

Check steering toe alignment

3.6 Final Pre-Power Checklist

All screws tightened

All wires secured

Servo calibration done

LiDAR spins freely

Jetson power supply stable

------------------------------------------------------------
CHAPTERS 4–6
------------------------------------------------------------
4. Expansion Board

The Rosmaster R2 includes a custom STM32-based expansion board that handles real-time control and bridges sensors and actuators to ROS.

Its responsibilities:

Motor PWM output

Steering servo PWM

UART communication (voice module, debugging)

I²C (IMU and other sensors)

Power distribution

Button input reading

Safety watchdog control

4.1 Expansion Board Layout

Typical connectors include:

5V logic power input

12V motor driver power input

Servo output pins (PWM 50Hz)

Motor driver DIR/PWM pins

IMU I²C header

UART header for voice module

USB port (STM32 → Jetson)

Boot and reset buttons

The board is connected to the Jetson via USB and appears as /dev/ttyACM0.

4.2 ROS Integration

The STM32 firmware communicates using ROS through:

rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0


Provides topics such as:

/rosmaster/servo_cmd
/rosmaster/motor_cmd
/imu/data
/buttons

4.3 IMU Interface (I²C)

The IMU (ICM20948 or MPU9250) connects to the expansion board over I²C.

Orientation must be correctly defined in imu_link

Data fed through STM32 → ROS serial → EKF or IMU filter

4.4 Button Inputs

User buttons feed into ROS via:

/buttons


Used for:

Teleop mode switching

Reset

Start/stop functions

Custom automation behaviors

4.5 Power Handling

The expansion board regulates:

5V logic rails

Servo power

Motor driver signal voltage

Do not power motors directly through the expansion board; always use the dedicated motor driver terminals.

4.6 Firmware Flashing

To re-flash:

Connect USB

Press BOOT0 if required

Build inside STM32CubeIDE

Flash firmware

After reboot, the board reconnects automatically.

5. Motor Drivers

The R2 uses a DC motor driver module for:

Rear wheel propulsion

Steering servo (via PWM from STM32)

Two key components:

DC motor driver (for throttle)

Servo motor (for steering)

5.1 Motor Driver Connections

Typical wiring:

12V battery → Motor Driver VIN
Motor Driver OUT → DC Motor Leads
PWM from STM32 → Motor Driver PWM
DIR from STM32 → Motor Driver DIR
GND → Shared with STM32 + Jetson

5.2 ROS Control Path

The control pipeline:

ROS /cmd_vel → STM32 (via rosserial) → Motor Driver → Motor Output


This separates high-level motion planning (ROS) from low-latency control (STM32).

5.3 Motor Driver Modes

The driver supports:

Forward

Reverse

Brake

PWM-based speed control

Correct polarity is essential.
If the robot drives backward when commanded forward, invert motor wiring or adjust direction logic in STM32 firmware.

5.4 Steering Servo Operation

Standard 50 Hz PWM

Typical range: 500–2500 µs

Center usually near 1500 µs

Steering limits must be configured to avoid linkage strain

Servo jitter usually indicates:

Underpowered 5V rail

Mechanical binding

Incorrect PWM timing

EMI from motor wiring

5.5 Motor/Servo Calibration Checklist

Wheels off the ground

Power on robot

Send neutral command

Verify steering is centered

Test small PWM increments

Verify full left/right range without binding

Test forward/reverse with no load

6. Teleoperation Modes

The Rosmaster R2 supports several teleoperation (manual driving) modes:

Keyboard control

Joystick / gamepad control

Button-based control

Web UI control (optional)

Direct STM32 button modes

Emergency stop (E-stop)

6.1 Keyboard Teleop

Start the keyboard teleop node:

rosrun teleop_twist_keyboard teleop_twist_keyboard.py


Controls:

W — forward

S — backward

A — steer left

D — steer right

Space — stop

Q/E — speed scaling

6.2 Teleop Twist Joy (Gamepad)

Start joystick teleoperation:

roslaunch teleop_twist_joy teleop.launch


Supported controllers:

Logitech F710

PS2-style controller

PS4 / Xbox

Generic HID gamepads

6.3 ROS Topic Interface

All teleop modes ultimately publish to:

/cmd_vel


Then:

/cmd_vel → move_base (if active) → STM32 → Motor Driver + Servo


If navigation/autopilot is running, teleop may be overridden or blended depending on configuration.

6.4 Safety Features

If teleop node stops publishing, the robot halts

STM32 watchdog kills motors on communication loss

Pressing the physical button on STM32 can trigger STOP

Some builds include E-stop remotes

6.5 Teleop Troubleshooting
Problem	Likely Cause	Fix
Robot slow	Low speed scaling	Press E/Q or adjust config
Steering reversed	Servo direction wrong	Invert PWM or swap linkage
Robot drifts	Bad center point	Recalibrate steering servo
No response	Wrong /dev/input/js0	Change launch file mapping

Chapters 7–9 (Clean Markdown Format)
7. Joystick Control

The Rosmaster R2 supports multiple joystick/gamepad types through ROS:

Logitech F710

PS2-style controllers

Xbox 360/Xbox One

PS4 DualShock

Generic USB HID devices

Communication is handled by two ROS packages:

joy (hardware input driver)

teleop_twist_joy (mapping buttons/axes to /cmd_vel)

7.1 Running the Joystick Node

Start the package:

rosrun joy joy_node


Check joystick output:

rostopic echo /joy

7.2 Teleop Twist Joy

Launch standard teleoperation:

roslaunch teleop_twist_joy teleop.launch


Example config (teleop.yaml):

axes:
  linear: 1
  angular: 0

scales:
  linear: 0.8
  angular: 1.2

buttons:
  enable_button: 4
  enable_turbo_button: 5


Start with:

roslaunch teleop_twist_joy teleop.launch config:=teleop.yaml

7.3 PS2 Controller Mapping (Typical)
Button	Function
Left stick	Steering
Right stick	Throttle
L1 / R1	Mode change
Start	Engage
Select	Disengage
7.4 Advanced Teleop Uses

Mapping + SLAM (Cartographer):

roslaunch teleop_twist_joy teleop.launch
roslaunch r2_cartographer r2_cartographer_online.launch


Mapping + Navigation:

roslaunch teleop_twist_joy teleop.launch
roslaunch r2_navigation bringup.launch

7.5 Joystick Troubleshooting

No /joy → permissions or wrong device index

Controls reversed → adjust axis indices in config

Lag → avoid Bluetooth, use USB

Robot veers → recalibrate servo center position

8. STM32 Tutorials

The STM32 microcontroller handles low-level robot operations:

Motor control

Servo control

IMU reading

UART/I²C communication

Button monitoring

Safety watchdog

This chapter summarizes the STM32 development instructions.

8.1 Development Tools

Install:

STM32CubeIDE

(Optional) STM32CubeMX

USB cable for flashing

Typical setup:

Open STM32CubeIDE → Import Project → Build → Flash

8.2 Project Structure
Core/
  Inc/     # Header files
  Src/     # C source files
Drivers/
  CMSIS/   # ARM Cortex abstraction
  HAL/     # Hardware Abstraction Layer
Middlewares/
  IMU drivers, sensor utilities

8.3 GPIO Example (Button Input)
GPIO_InitTypeDef GPIO_InitStruct = {0};
GPIO_InitStruct.Pin = GPIO_PIN_0;
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
    // Button pressed
}

8.4 Servo PWM Example
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_width_us);


Typical values:

500 µs → full left

1500 µs → center

2500 µs → full right

8.5 Motor PWM + DIR Example
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET); // DIR
__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, duty_cycle);

8.6 I²C IMU Read Example
HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, REG_ACCEL_X, 1, buffer, 6, 100);

8.7 UART Example
HAL_UART_Transmit(&huart2, data, size, 100);
HAL_UART_Receive(&huart2, buffer, len, 100);

8.8 ROS Integration (rosserial)
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200


Publishes/subscribes to:

/rosmaster/servo_cmd
/rosmaster/motor_cmd
/sensor/imu_raw
/buttons

8.9 Flashing STM32 Firmware

Steps:

Connect USB

Open STM32CubeIDE

Build project

Click “Flash”

9. FreeRTOS

The STM32 firmware uses FreeRTOS for concurrent tasks with precise timing.

9.1 Common Tasks
Task	Purpose
motor_task	Motor PWM updates
servo_task	Steering PWM updates
imu_task	IMU polling
uart_task	Voice/debug UART
button_task	Physical buttons
watchdog_task	Safety checks
9.2 Task Structure Example
void motor_task(void *argument) {
    for(;;) {
        update_motor_pwm();
        vTaskDelay(5); // every 5 ms
    }
}

9.3 Priority Recommendations
watchdog_task      Highest
motor_task         High
servo_task         High
imu_task           Medium
uart_task          Medium
button_task        Low

9.4 Queue Example
MotorCommand cmd;
xQueueReceive(motor_queue, &cmd, portMAX_DELAY);


Used for:

Motor commands

Servo angles

IMU packets

9.5 Troubleshooting FreeRTOS

Servo jitter → reduce task delay, increase power

Missed IMU reads → adjust priority

Freezes → watchdog task errors

Slow performance → excessive debug printing

10. PID & Kinematic Theory

The Rosmaster R2 uses Ackermann steering, meaning the robot must obey vehicle-like kinematics rather than differential-drive kinematics.
This chapter explains how the robot turns, how velocity is interpreted, and how PID smoothing is applied.

10.1 Ackermann Geometry Basics

Ackermann steering ensures each front wheel turns at a different angle so the robot follows a circular arc without slipping.

Geometry relationships:

tan(δ_inner) = L / (R - W/2)
tan(δ_outer) = L / (R + W/2)


Where:

L = wheelbase

W = track width

R = turning radius

δ_inner/outer = wheel angles

On Rosmaster R2, both wheels are linked mechanically and controlled by one servo, so the system approximates the correct inner/outer angle relationship.

10.2 ROS Velocity Model

ROS navigation expects twist commands:

/cmd_vel.linear.x     # speed (m/s)
/cmd_vel.angular.z    # turn rate (rad/s)


These must be converted to:

Steering servo PWM

Motor PWM

The STM32 firmware handles that conversion.

10.3 Forward Kinematic Equation

Ackermann forward kinematics:

ω = v / L * tan(δ)


Where:

ω = angular velocity

v = linear velocity

δ = steering angle

This is used by TEB local planner and EKF.

10.4 Steering PID Control

Steering uses a PD or PID loop to smooth movement:

error = target_angle - measured_angle;
derivative = (error - last_error) / dt;
output = Kp*error + Kd*derivative;
last_error = error;


P handles immediate difference

D dampens oscillations

I is often unnecessary for steering

10.5 Motor PID Control

The drive motor uses PID to keep the speed stable:

error = target_speed - current_speed;

integral += error * dt;
derivative = (error - prev_error) / dt;

output = Kp*error + Ki*integral + Kd*derivative;
prev_error = error;


Symptoms of incorrect PID:

Problem	Fix
Oscillation	Lower Kp or Kd
Slow response	Increase Kp
Drifts over time	Increase Ki (lightly)
10.6 Practical Tuning Tips

Tune Kp first

Add D if oscillating

Add tiny Ki for long, straight-line speed holding

Do not over-tune — small changes matter

Keep testing at low speeds first

11. ROS Installation

This chapter covers ROS1 Melodic (Ubuntu 18.04) and ROS1 Noetic (Ubuntu 20.04), which are the supported systems for Rosmaster R2.

11.1 Add the ROS Repository
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
> /etc/apt/sources.list.d/ros-latest.list'


Add key:

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' \
--recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654


Update:

sudo apt update

11.2 Install ROS Desktop
Melodic (Ubuntu 18.04)
sudo apt install ros-melodic-desktop-full

Noetic (Ubuntu 20.04)
sudo apt install ros-noetic-desktop-full

11.3 Setup ROS Environment
echo "source /opt/ros/$(rosversion -d)/setup.bash" >> ~/.bashrc
source ~/.bashrc

11.4 Initialize rosdep
sudo rosdep init
rosdep update

11.5 Install Build Tools

Melodic:

sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential


Noetic:

sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

11.6 Verify ROS Installation
roscore


If the ROS master starts, installation succeeded.

11.7 Install Common R2 Packages
sudo apt install ros-$(rosversion -d)-joy
sudo apt install ros-$(rosversion -d)-teleop-twist-joy
sudo apt install ros-$(rosversion -d)-navigation
sudo apt install ros-$(rosversion -d)-cartographer-ros
sudo apt install ros-$(rosversion -d)-map-server

12. ROS Workspace Setup

All robot packages live inside a catkin workspace.

12.1 Create the Workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make


Add to environment:

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

12.2 Add the Rosmaster R2 Packages

Clone:

cd ~/catkin_ws/src
git clone https://github.com/YahboomTechnology/Rosmaster-R2-ROS.git


or your customized repo.

12.3 Build the Workspace
cd ~/catkin_ws
catkin_make


On Noetic (Python 3):

catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3

12.4 Workspace Layout
catkin_ws/
  src/
    r2_bringup/
    r2_navigation/
    r2_cartographer/
    r2_description/
    teleop_twist_joy/
  build/
  devel/

12.5 Typical Launch Files
Start mapping:
roslaunch r2_cartographer r2_cartographer_online.launch

Start navigation:
roslaunch r2_navigation bringup.launch

Start rosserial (STM32):
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0

12.6 Troubleshooting Workspace
Problem	Fix
Workspace won’t build	Run rosdep install --from-paths src --ignore-src -r -y
Command not found	You forgot to source devel/setup.bash
Python errors	Noetic requires Python3 only
Missing dependencies	Re-run catkin_make

Here are Chapters 13–15, fully cleaned and Markdown-perfect for your .md manual.

👉 Paste these directly under Chapter 12.

Chapters 13–15 (Clean Markdown Format)
13. URDF & TF

URDF (Unified Robot Description Format) defines the robot’s physical model, links, joints, and sensor positions.
TF provides the transform tree used by navigation, mapping, EKF, and visualization.

13.1 URDF Purpose

URDF is used to:

Visualize the robot in RViz

Define sensor mounting positions

Configure collision geometry

Generate the TF tree

Provide kinematic layout for navigation

13.2 Typical Rosmaster R2 TF Tree
map
 └── odom
      └── base_link
           ├── base_footprint
           ├── imu_link
           ├── camera_link
           └── lidar_link


Key frames:

map: Global fixed frame

odom: Integrated odometry frame

base_link: Robot body center

camera_link / lidar_link: Sensor frames

13.3 Example URDF Snippet
<robot name="rosmaster_r2">

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.4 0.25 0.1"/>
      </geometry>
    </visual>
  </link>

  <joint name="base_to_lidar" type="fixed">
    <origin xyz="0.18 0 0.16" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="lidar_link"/>
  </joint>

  <link name="lidar_link"/>

</robot>

13.4 Launching the URDF Visualization
roslaunch r2_description display.launch


Add in RViz:

RobotModel

TF

LaserScan

Camera view

13.5 Common URDF Fixes
Symptom	Cause	Fix
Laser rotated 90°	Wrong RPY	Adjust <origin rpy="...">
Robot floating	Z offset too high	Reduce Z in base_link
Map drifting	Wrong IMU orientation	Correct imu_link transform
Navigation curve incorrect	Footprint wrong	Fix footprint or base size
14. Sensors (Camera, LiDAR, IMU)

This chapter covers the three primary sensors on Rosmaster R2:
Camera, LiDAR, and IMU — all essential for mapping, navigation, and AI.

14.1 Camera Sensors (Astra / RealSense / USB Cameras)

Common cameras:

Astra Pro Plus

Intel RealSense D435/D455

Logitech C920

14.1.1 Astra Camera Launch
roslaunch astra_camera astra.launch


Topics:

/camera/rgb/image_raw
/camera/depth/image_raw
/camera/depth/points

14.1.2 RealSense Camera Launch
roslaunch realsense2_camera rs_camera.launch


Recommended parameters:

align_depth:=true
enable_pointcloud:=true

14.1.3 Camera Troubleshooting
Problem	Fix
No video	Try USB2 port, not USB3
Depth noise	Enable align_depth
Low FPS	Reduce resolution
Jetson freezes	Use powered USB hub
14.2 LiDAR

Supported LiDARs:

RPLIDAR A1/A2/A3/S2

LDROBOT LD19

YDLIDAR TG series

Primary topic:

/scan

14.2.1 RPLIDAR Launch
roslaunch rplidar_ros rplidar.launch

14.2.2 LD19 Launch
roslaunch ld19_driver ld19.launch


Set correct port:

/dev/ttyUSB0


Baud: 230400

14.2.3 LiDAR Troubleshooting

Incorrect URDF RPY → rotated map

LiDAR too low → blind spots

No data → wrong device index or permissions

Noise → add scan filtering or remount sensor

14.3 IMU (ICM20948 / MPU9250)

The IMU provides:

Acceleration

Angular velocity

Optional fused orientation

Topic:

/imu/data

14.3.1 IMU Launch
roslaunch r2_bringup imu.launch

14.3.2 IMU Orientation Rules

IMU must be mounted:

Z-axis up

X forward

Y left

Fix inconsistencies in imu_link transform.

14.3.3 Calibration Tips

Calibrate on stable surface

Avoid magnetic sources

Run IMU filter:

roslaunch imu_filter_madgwick imu_filter.launch

15. Mapping (Cartographer)

Cartographer performs 2D SLAM, building a map in real time using LiDAR and optionally IMU.

15.1 Launching Cartographer

Start mapping:

roslaunch r2_cartographer r2_cartographer_online.launch


Required inputs:

/scan
/tf
/tf_static
/imu/data   (optional but recommended)

15.2 Viewing the Map in RViz

Set Fixed Frame: map

Add displays:

Map

LaserScan

TF

RobotModel

15.3 Saving Maps
rosrun map_server map_saver -f my_map


Outputs:

my_map.yaml
my_map.pgm

15.4 Cartographer Config (Lua)

Example:

TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40

15.5 Mapping Best Practices

Move slowly and smoothly

Keep LiDAR level

Avoid shiny floors

Avoid pointing camera or LiDAR at windows

15.6 Common Mapping Issues
Symptom	Cause	Fix
Map rotates	Wrong IMU orientation	Fix imu_link frame
Map drifts	No IMU + bad odom	Slow down or add IMU
Holes in wall	Noisy scan	Adjust LiDAR mount
Map too small	Wrong resolution	Change map_resolution

16. Localization

Localization determines the robot’s position inside a previously created map.
The Rosmaster R2 supports multiple localization methods:

AMCL (Adaptive Monte Carlo Localization)

Cartographer in Localization Mode

EKF-based odometry fusion using IMU + wheel odom

Static map-based pose estimation

AMCL is the standard for map-based navigation on the R2.

16.1 What Localization Does

Localization answers one question:

“Where am I on this map?”

It provides:

map → odom transform

Correct robot orientation

Stable navigation inputs

Confidence values for global planners

16.2 AMCL Launch Example
roslaunch r2_navigation amcl.launch map_file:=/home/robot/maps/home1.yaml


Required topics:

/scan
/map
/odom
/tf
/initialpose

16.3 Setting Initial Pose

In RViz:

Select 2D Pose Estimate

Click on the robot’s starting position

Drag arrow to set orientation

Or publish manually:

rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped ...

16.4 AMCL Key Parameters
min_particles: 500
max_particles: 2000
update_min_d: 0.1
update_min_a: 0.1
laser_z_hit: 0.95
laser_z_rand: 0.05
odom_alpha1: 0.1
odom_alpha2: 0.1
odom_alpha3: 0.1
odom_alpha4: 0.1

16.5 Cartographer Localization Mode

Instead of AMCL:

roslaunch r2_cartographer localization.launch load_state_filename:=map.pbstream


Advantages:

Smooth IMU integration

Good for dynamic environments

Disadvantages:

Higher CPU load

More sensitive to LiDAR noise

16.6 EKF Odometry Fusion

Fuses:

Wheel odometry

IMU angular velocity

IMU linear acceleration

Launch:

roslaunch r2_bringup ekf.launch


Produces:

/odom
/base_link → odom transform

16.7 Localization Troubleshooting
Problem	Cause	Fix
Robot drifts over time	Bad wheel odom or IMU	Recalibrate IMU, fix wheel radius
Robot teleports	Incorrect TF	Verify URDF and transforms
Robot rotates incorrectly	IMU axes wrong	Fix IMU orientation
AMCL never converges	Noisy LiDAR	Remount sensor, adjust parameters
Robot starts aligned wrong	Incorrect initial pose	Reset in RViz
17. Navigation (TEB + Costmaps)

The Rosmaster R2 uses the TEB Local Planner, which is ideal for Ackermann steering robots. Navigation relies on:

A global planner to create long paths

A local planner (TEB) to compute smooth curvature-constrained motion

Costmaps to represent obstacles

17.1 Navigation Launch
roslaunch r2_navigation bringup.launch map_file:=/home/robot/maps/home1.yaml


Includes:

AMCL

Move base

Costmaps

TEB local planner

17.2 Required Topics
/scan
/odom
/map
/tf
/cmd_vel
/move_base_simple/goal

17.3 Costmaps Overview
Global Costmap

Uses static map

Provides long path

Slow updates

Local Costmap

Rolling window (~3–6m)

Uses real-time LiDAR

Avoids dynamic obstacles

17.4 TEB Planner Basics

TEB = Timed Elastic Band.

It computes:

Steering curvature

Velocity profile

Obstacle avoidance

Ackermann kinematics constraints

17.5 Important TEB Parameters
min_turning_radius: 0.3
max_vel_x: 1.0
acc_lim_x: 1.5
footprint: [[-0.18, -0.13], [-0.18, 0.13], [0.18, 0.13], [0.18, -0.13]]
inflation_radius: 0.25
weight_obstacle: 50
weight_optimaltime: 1.0
weight_kinematics_nh: 2000

17.6 Sending Navigation Goals

In RViz:

Use 2D Nav Goal

Or manually:

rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped ...

17.7 Multi-Waypoint Navigation

The robot can follow a sequence of waypoints:

rosrun r2_navigation waypoint_follower.py waypoints.yaml


Example file:

waypoints:
  - { x: 1.0, y: 2.0, yaw: 0.0 }
  - { x: 2.0, y: 3.0, yaw: 1.57 }
  - { x: 3.0, y: 2.0, yaw: 3.14 }


Set looping:

loop: true

17.8 Navigation Troubleshooting
Problem	Cause	Fix
Oscillation	High TEB gains	Reduce weight_obstacle
Cutting corners	Wrong turn radius	Increase min_turning_radius
Robot slows/stops	Bad obstacle inflation	Adjust inflation radius
Gets stuck	Bad map or dynamic obstacles	Tune recovery behaviors
Sharp jerks	Speed too high	Lower max_vel_x
18. Autopilot Using Maps

Autopilot uses mapping + localization + navigation to drive autonomously through a sequence of waypoints or mission paths.

18.1 Full Autopilot Launch
roslaunch r2_autopilot autopilot.launch map_file:=/home/robot/maps/home1.yaml


Includes:

Localization

Move base

TEB planner

Waypoint loader

Velocity controller

18.2 Mission Loading (Waypoints)

Mission YAML file example:

waypoints:
  - {x: 0.5, y: 1.0, yaw: 0.0}
  - {x: 2.0, y: 1.0, yaw: 0.5}
  - {x: 3.0, y: -1.0, yaw: 3.14}
loop: true


Launch:

roslaunch r2_autopilot mission.launch mission_file:=track.yaml

18.3 Autopilot Flow

Load map

Load mission

Localize robot (initial pose)

Begin following first waypoint

TEB adjusts motion

LiDAR avoids obstacles

Loop if enabled

18.4 Dynamic Obstacle Handling

TEB handles:

People walking in front

Chairs/furniture

Random objects

LiDAR + costmap create temporary obstacles.
TEB replans continuously around them.

18.5 Autopilot Safety Notes

Indoors: keep max_vel_x ≤ 0.8 m/s

Ensure clear LiDAR line-of-sight

Start with conservative TEB tuning

Always set initial pose in RViz

Keep emergency stop accessible

18.6 Autopilot Troubleshooting
Problem	Cause	Fix
Drives in circles	Wrong IMU orientation	Fix imu_link
Jerky movement	High speed	Lower max_vel_x
Stops randomly	AMCL lost	Re-localize
Crashes into objects	LiDAR blocked	Adjust mount / raise sensor
Stuck in corners	Poor waypoint geometry	Add intermediate points

19. Voice Control

The Rosmaster R2 supports optional voice-command modules using:

Offline voice recognition (e.g., I2S microphone module + STM32 parsing)

Online cloud engines (Google, Azure, etc.) — optional

Simple serial/UART keyword recognition chips

Voice commands trigger:

Start/stop drive

Change modes

Trigger actions (lights, horn, recording)

19.1 Built-In Voice Module (UART)

Typical UART commands:

CMD_FORWARD
CMD_BACKWARD
CMD_LEFT
CMD_RIGHT
CMD_STOP


STM32 receives UART bytes and converts them to:

Motor PWM

Servo commands

Mode triggers

19.2 ROS Voice Topic

If using cloud/PC-based voice recognition:

rostopic pub /voice_cmd std_msgs/String "forward"


Examples:

rosrun voice_control voice_node.py

19.3 Example Voice Node
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def callback(msg):
    cmd = msg.data.lower()
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    t = Twist()

    if cmd == "forward":
        t.linear.x = 0.4
    elif cmd == "stop":
        t.linear.x = 0
    elif cmd == "left":
        t.angular.z = 0.5
    elif cmd == "right":
        t.angular.z = -0.5

    pub.publish(t)

rospy.init_node("voice_control")
rospy.Subscriber("/voice_cmd", String, callback)
rospy.spin()

19.4 Voice Control Safety Tips

Always include a “STOP” keyword

Limit speed for voice control

Use clear, distinct command words

Avoid noisy environments

19.5 Voice Troubleshooting
Problem	Cause	Fix
Not recognizing words	Noise, poor mic	Move mic or use noise filter
Robot moves late	Delay in pipeline	Reduce buffer size
Wrong movement	Mapping mismatch	Update voice command table
No reaction	UART or ROS topic wrong	Check /voice_cmd
20. App & Bluetooth Control

The Rosmaster R2 supports mobile control via Bluetooth or Wi-Fi.
This includes:

Yahboom app

Custom Android/iOS apps

Simple BLE gamepad emulation

The STM32 receives commands over UART or BLE and translates them into motor and servo signals.

20.1 Bluetooth Setup

Typical steps:

Power robot

Pair phone with "Yahboom_R2_BT"

Open control app

Connect

Send commands to STM32

20.2 Supported Control Modes

Virtual joystick

Button-based directional control

Speed sliders

Mode toggles

Servo angle controls

20.3 Bluetooth UART Format

Commands typically look like:

0xAA 0x55 <speed> <steer> 0x0D


The STM32 decodes these bytes and applies:

Motor PWM output

Steering servo pulse

20.4 Bluetooth ROS Bridge

You can bridge the app to ROS:

rosrun r2_bluetooth bt_bridge.py


Example outputs:

/bt_cmd_vel
/bt_buttons

20.5 Custom App (Optional)

You can design a custom Android or iOS controller that sends:

JSON → BLE UART → STM32 → /cmd_vel → robot


Example JSON:

{"linear":0.4, "angular":0.2}

20.6 Wi-Fi Control (Advanced)

Using a socket server:

python3 wifi_server.py


Phone sends UDP packets like:

v=0.4,a=-0.3

20.7 Bluetooth Troubleshooting
Problem	Cause	Fix
Won’t pair	Module not in pairing mode	Reset module
Lag	Poor signal	Bring phone closer
No servo movement	Wrong command format	Re-check UART bytes
App freezes	Old version	Update app
21. AI Vision

AI vision adds intelligent perception to the Rosmaster R2:

Lane detection

Object detection

Semantic segmentation

Face or hand detection

AprilTag / QR code recognition

Supported frameworks:

ROS image pipelines

OpenCV

TensorRT

Jetson Nano/Xavier/Orin acceleration

21.1 ROS Camera Pipeline

Make sure camera is publishing:

/camera/rgb/image_raw
/camera/depth/image_raw (if available)


Convert to OpenCV format:

cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

21.2 Lane Detection Example (OpenCV)
gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
edges = cv2.Canny(gray, 100, 200)
lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, minLineLength=40, maxLineGap=80)

21.3 Object Detection (TensorRT)

Run a YOLO engine:

roslaunch r2_ai yolo.launch


Producing:

/detections


Each detection includes:

Label

Confidence

Bounding box

21.4 Semantic Segmentation (Jetson)

Example launch:

roslaunch r2_ai segmentation.launch model:=fcn-resnet18-cityscapes


Outputs:

/segmentation_mask


Use mask to:

Follow sidewalks

Detect drivable area

Avoid grass or walls

21.5 Depth Mapping

If using RealSense:

/camera/depth/image_raw
/camera/depth/points


Applications:

Obstacle distance estimation

Height detection

Hole/ledge detection

21.6 AprilTag Recognition
roslaunch apriltag_ros apriltag.launch


Used for:

Precise docking

Indoor navigation

Location anchors

21.7 AI → Steering Conversion

Once AI estimates a desired direction, convert to steering commands:

t.angular.z = steer_output
t.linear.x = speed
pub.publish(t)

21.8 AI Troubleshooting
Problem	Cause	Fix
No GPU acceleration	Wrong backend	Use TensorRT engine
Slow FPS	High resolution	Reduce image size
Robot wobbles	No low-pass filter	Smooth steering
Wrong colors	BGR/RGB mismatch	Fix conversion
Segmentation edges fuzzy	Poor lighting	Increase exposure



