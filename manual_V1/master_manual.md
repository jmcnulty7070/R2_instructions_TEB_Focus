# Master ROS1-R2 Robot Manual

## Table of Contents
- 1. Introduction
- 2. Hardware & Basics
- 3. ROS Control
- 4. Mapping Systems
- 5. Race Loop & TEB Navigation
- 6. Commands Cheatsheet
- 7. Glossary

---

## 1. Introduction
This manual summarizes all robot tutorials in simple 8th‑grade language.

## 2. Hardware & Basics
# Basics Manual

## 1、Docker overview and docker installation.pdf

1. Docker overview and docker  
installation 
Raspberry Pi PI5 Master Control ’s ROS1 courses are all in docker containers.  
1. Docker overview  and docker installation
1.1, docker overview
1.1.1. Why does docker appear?
1.1.2. The core idea of Docker
1.1.3. Compa ring virtual machines and Docker
1.1.4, docker architecture
1.1.5, Docker core objects
1.1.6, Mirror, container, warehouse
1.1.7, Docker operating mechanism
1.2, docker installation
Docker official website: http://www.docker.com
Docker Chinese website: https://www.docker-cn.com
Docker Hub (warehouse) official website: https://hub.docker.com
 
The operating environment and software and hardware reference configuration are as follows:
Reference model: ROSMASTER X3
Robot hardware configuration: Arm series main control, Silan A1 lidar, AstraPro Plus depth  
camera
Robot system: Ubuntu (no version required) + docker (version 20.10.21 and above)
PC virtual machine: Ubuntu (18.04) + ROS (Melodic)
Usage scenario: Use on a relatively clean 2D plane
 
1.1, docker overview  
Docker is an application container engine project, developed based on Go language and open  
source.
 
1.1.1. Why does docker appear?  
Let me mention a few scenarios first:
1. The operation and maintenance team will deploy the project developed for you to the server  
and tell you that there is a problem and it cannot be started. You ran it locally and found that  
there is no problem...
2. The project to be launched is unavailable due to some software version updates...
3. Some projects involve a lot of environmental content, including various middlewares, various  
configurations, and the deployment of many servers...
These problems are actually related to the environment.
To avoid various problems caused by different environments, it is best to deploy the project  
together with the various environments required by the project.
For example, if the project involves redis, mysql, jdk, es and other environments, bring the entire  
environment with you when deploying the jar package. So the question is, how can we bring the  
environment with the project?
Docker is here to solve this problem!
 
1.1.2. The core idea of Docker  
 
This is the logo of Docker, a whale full of containers. On the back of the whale, the containers are  
isolated from each other. This is the core idea of Docker.
For example, if multiple applications were running on the same server before, there may be  
conflicts in the software's port occupancy. Now, after isolation, they can run independently. In  
addition, docker can maximize the use of server capabilities.
 
1.1.3. Comparing virtual machines and Docker  
 

The docker daemon can communicate directly with the main operating system to allocate  
resources to each docker container; it can also isolate the container from the main operating  
system and isolate each container from each other. A virtual machine takes minutes to start,  
while a docker container can start in milliseconds. Since there is no bloated slave operating  
system, docker can save a lot of disk space and other system resources.
Virtual machines are better at completely isolating the entire operating environment. For  
example, cloud service providers often use virtual machine technology to isolate different  
users. Docker is usually used to isolate different applications, such as front-end, back-end  
and database.
Docker containers save resources and are faster than virtual machines (start, shut down,  
create, delete)
 
1.1.4, docker architecture  
Docker uses a client-server architecture. The docker client communicates with the docker  
daemon, which is responsible for the heavy lifting of building, running, and distributing docker  
containers. The docker client and daemon can run on the same system, or the docker client can be  
connected to a remote docker daemon. The docker client and daemon communicate using REST  
API through UNIX sockets or network interfaces. Another docker client is docker compose, which  
allows you to work with applications composed of a set of containers.
 
docker client is the docker command used directly after installing docker.
docker host is our docker host (that is, the operating system with docker installed)
Docker daemon is the background daemon process of docker, which listens and processes  
docker client commands and manages docker objects such as images, containers, networks  
and volumes.
Registry is a remote warehouse where Docker pulls images, providing a large number of  
images for download. After the download is completed, they are saved in images (local image  
warehouse).
images is docker's local image warehouse. You can view image files through docker images.
 
1.1.5, Docker core objects  
 
1.1.6, Mirror, container, warehouse  
Image:
Container:
Repository:The docker image (Image) is a read-only template. Images can be used to create 
docker containers, and one image can create many containers. Just like classes 
and objects in Java, classes are images and containers are objects.
Docker uses a container to run an application or a group of applications 
independently. Containers are running instances created from images. It can be 
started, started, stopped, deleted. Each container is isolated from each other to 
ensure a secure platform. A container can be thought of as a simplified version 
of the Linux environment (including root user permissions, process space, user 
space, network space, etc.) and the applications running in it. The definition of 
a container is almost exactly the same as that of an image. It is also a unified 
perspective of a bunch of layers. The only difference is that the top layer of 
the container is readable and writable.
It is necessary to correctly understand the concepts of warehousing/mirror/container:
Docker itself is a container running carrier or management engine. We package the  
application and configuration dependencies to form a deliverable running environment. This  
packaged running environment is like an image file. Only through this image file can a docker  
container be generated. The image file can be thought of as a template for the container.  
docker generates an instance of the container based on the image file. The same image file  
can generate multiple container instances running simultaneously.
The container instance generated by the image file is itself a file, called an image file.
A container runs a service. When we need it, we can create a corresponding running instance  
through the docker client, which is our container.
As for the warehouse, it is a place where a bunch of images are placed. We can publish the  
images to the warehouse and pull them out of the warehouse when needed.
 
 
1.1.7, Docker operating mechanism  
docker pull execution process:
1. The client sends instructions to the docker daemon
2. The docker daemon first checks whether there are related images in the local images.
3. If there is no relevant mirror locally, request the mirror server to download the remote mirror  
to the local
docker run execution process:
1. Check whether the specified image exists locally. If it does not exist, download it from the  
public warehouse.
2. Create and start a container using the image
3. Allocate a file system (simplified Linux system) and mount a read-write layer outside the  
read-only mirror layer
4. Bridge a virtual interface from the bridge interface configured on the host to the container.
5. Configure an IP address from the address pool to the container
6. Execute user-specified applications
 
1.2, docker installation  
1. Official website installation reference manual: https://docs.docker.com/engine/install/ubunt
u/A repository is a place where image files are stored centrally. Warehouses are 
divided into two forms: public warehouses (public) and private warehouses 
(private).
The largest public repository is docker hub (https://hub.docker.com/), which 
stores a large number of images for users to download. Domestic public warehouses 
include Alibaba Cloud, NetEase Cloud, etc.
2. You can use the following command to install it with one click:
3. Check docker version
4. Test command
The following output indicates that the Docker installation is successful.curl -fsSL https://get.docker.com | bash -s docker --mirror Aliyun
sudo docker version
sudo docker run hello-world

---

## 1. How to use the Astra camera.pdf

Launch file Start the camera model
astra.launch Astra, Astra S, Astra mini, Astra mini S
astraplus.launch more
astrapro.launch Astra pro
embedded_s.launch Deeyea
dabai_u3.launch Nature
gemini.launch Gemini1 How to use the Astra camera  
1 How to use the Astra camera 
1.1 SDK usage - Linux 
1.1.1 dependent environment 
1.1.2 Camera SDK&Samples 
1.1.3 Ope nNI camera test tool 
1.2 AstraSDK-win 
1.2.1 Install the driver 
1.2.2 Download SDK 
1.3 OrbbecViewer-win 
1.4 Web monitoring 
Official website link: http://www.orbbec.com.cn/ 
Developer Community: https://developer.orbbec.com.cn/ 
Astra Camera: https://github.com/orbbec/ros_astra_camera 
Normal camera: https://github.com/bosch-ros-pkg/usb_cam.git 
Astra SDK ：  https://developer.orbbec.com.cn/download.html?id=53 
Basic use of Astra SDK for Windows: https://developer.orbbec.com.cn/course_details.html?id=53 
Astra SDK environment setup: https://developer.orbbec.com.cn/course_details.html?id=16 
Create astra udev rule 
In the new environment, you need to execute the [create_udev_rules] file in the [scripts] folder of  
the [astra_camera] function package, enter the directory where the file is located, and execute the  
command 
launch start command 
Camera hardware structure diagram: ./create_udev_rules 
1.1 SDK usage - Linux  
Operating environment: virtual machine or dual system  
Developer Community: https://developer.orbbec.com.cn/download.html?id=53 
1.1.1 dependent environment  
Go to the developer community to download the SDK file, that is(Astra SDK and OpenNI2 SDK, the  
version and system architecture must match). 
sudo apt-get install ros-melodic-serial ros-melodic-bfl ros-melodic-mbf-msgs ros-
melodic-pointcloud-to-laserscan ros-melodic-rgbd-launch ros-melodic-libuvc-* ros-
melodic-uvc-camera ros-melodic-usb-cam ros-melodic-ar-track-alvar ros-melodic-
camera-calibration build-essential freeglut3 freeglut3-dev libsfml-dev
Note: All searches on the Internet are for the latest version. The versions in our supporting  
materials include [v2.1.2], [v2.1.3], etc. The following takes [v2.1.2] as an example, other versions  
are similar. 
1.1.2 Camera SDK&Samples  
The folder name and file path may not be the same, change them according to your needs. 
The output contains the following two lines, pay attention to delete the install in the penultimate  
path: 
After deleting install: 
Copy the output to the end of ~/.bashrc 
The amples directory is the sample program, which needs to depend on the include and lib  
directories 
SFML effect demo 
The bin folder is as follows: tar -zxvf AstraSDK-v2.1.2-Ubuntu18.04-x86_64.tar.gz
cd AstraSDK-v2.1.2-Ubuntu18.04-x86_64/install  # Go to the install folder
sudo sh ./install.sh
export ASTRA_SDK_INCLUDE=/home/yahboom/software/AstraSDK-v2.1.2-Ubuntu18.04-
x86_64/install/include  
export ASTRA_SDK_LIB=/home/yahboom/software/AstraSDK-v2.1.2-Ubuntu18.04-
x86_64/install/lib 
export ASTRA_SDK_INCLUDE=/home/yahboom/software/AstraSDK-v2.1.2-Ubuntu18.04-
x86_64/include 
export ASTRA_SDK_LIB=/home/yahboom/software/AstraSDK-v2.1.2-Ubuntu18.04-
x86_64/lib 
gedit ~/.bashrc 
source ~/.bashrc 
Note: sudo ./ or ./ can be used to start the bin folder, and the files with the suffix -SFML will  
be displayed on the screen; the methods are similar, and other effects can be tested. If the  
virtual machine fails to start, please try several times, it is easier to start under the dual  
system.  
cd ~/AstraSDK-v2.1.2-Ubuntu18.04-x86_64/bin/
./SimpleBodyViewer-SFML    # Skeleton detection
./SimpleHandViewer-SFML    # finger following 
1.1.3 OpenNI camera test tool  
Install OpenNI 
the device to  initialize the OpenNI environment 
compile and run unzip OpenNI_2.3.0.55.zip 
cd OpenNI_2.3.0.55/Linux/OpenNI-Linux-x64-2.3.0.55 
chmod +x install.sh 
sudo ./install.sh 
source OpenNIDevEnvironment 
cd Samples/SimpleViewer 
make 
cd Bin/x64-Release 
./SimpleViewer 
1.2 AstraSDK-win  
https://developer.orbbec.com.cn/download.html?id=32 
1.2.1 Install the driver  
After the download is complete, double-click to install it. The signs of success are as follows 
1.2.2 Download SDK  
After the download is complete, unzip the folder, 
Enter the bin folder and double-click any file with the suffix exe to test it. 
1.3 OrbbecViewer-win  
https://developer.orbbec.com.cn/download.html?id=77 
Unzip, enter the OrbbecViewer_v1.1.1 folder, double-click 
 That's it. 
1.4 Web monitoring  
Environment construction 
Start the camera 
<PI5 needs to open another terminal and enter the same docker container#Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS2/07, Docker 
tutorial
~/run_docker.sh
sudo apt-get install ros-melodic-async-web-server-cpp ros-melodic-web-video-
server ros-melodic-usb-cam 
roslaunch astra_camera astraproplus.launch     # Astra 
roslaunch usb_cam usb_cam-test.launch      # USB 
start web_video_server 
Check 
 rosrun  web_video_server  web_video_server 
View in local web browser 
http://localhost:8080/ 
It must be under the same local area network, and other devices can view it 
http://192.168.2.103:8080/ 
(192.168.2.103 is the IP address of the master) 
Note: It is recommended to use Google Chrome or mobile QQ browser, other browsers 
may not be able to open the image 

---

## 1. Introduction of voice module.pdf

1. Introduction of voice module  
1.1. Voice Interaction Module Introduction  
      The CI1302 is a new-generation, high-performance neural network intelligent voice chip  
developed by Qiyingtailun. It integrates Qiyingtailun's proprietary brain neural network processor,  
the BNPU V3, and a CPU core. It boasts a system clock speed of up to 220MHz, built-in SRAM of up  
to 640KB, a power management unit (PMU) and an RC oscillator, a dual-channel, high-
performance, low-power audio codec, and multiple peripheral control interfaces such as UART,  
IIC, IIS, PWM, GPIO, and PDM. Requiring only a small number of peripheral components such as  
resistors and capacitors, the chip can implement hardware solutions for various intelligent voice  
products, offering an extremely high cost-effectiveness.
       Utilizing third-generation hardware BNPU technology, it supports neural networks such as  
DNN, TDNN, RNN, and CNN, as well as parallel vector operations. It enables voice recognition,  
voiceprint recognition, command word self-learning, voice detection, and deep learning noise  
reduction. This chip solution also supports multiple global languages, including Chinese, English,  
and Japanese. It can be widely used in home appliances, lighting, toys, wearable devices,  
industrial, and automotive products, enabling voice interaction and control, as well as various  
intelligent voice solutions.
      The CI1302 chip features a Brain Neural Network Processing Unit (BNPU) core, supporting  
offline NN acceleration and hardware acceleration for voice signal processing. The CPU clock  
speed can reach 220MHz, enabling offline far-field voice recognition. It also has 2MB of built-in  
FLASH memory and supports 300 command words.
1.2 Operating Principle  
      This module uses a command-based wakeup mode. The user must speak the preset wakeup  
word to activate the voice interaction module. Once activated, voice recognition will begin. The  
default wakeup keyword in the factory firmware is "Xiao Ya Xiao Ya." If no voice is recognized after  
20 seconds, the module enters sleep mode and must be reactivated upon further use.
      When the CI1302 chip recognizes a corresponding voice entry, it transmits it through the serial  
port and IIC interface and reports it back. The IIC chip stores the received voice command and  
transmits it via the IIC slave protocol.
      The module supports modifying wake-up words, command words, and customizing entries.  
Learn how to do this in the tutorials "2. Modifying Wake-Up and Command Words" and "3.  
Creating Custom Protocol Entry."
1.3. Notes  
Use a 5V power supply. Voltages exceeding 5V will damage the module.
Use in a quiet environment; noisy environments will affect recognition performance.
When speaking a term, speak loudly and slowly. It is recommended to stay within 5 meters of  
the module.
1.4. Hardware Interface Description  

Serial
NumberHardware Name Description
1 Speaker Converts analog signals into sound
2 Slide Switch Switches the serial port for firmware burning
3 Microphone Converts sound into analog signals
4 RST Button Reset Button
5Power Indicator
(Red)Steady on when power is normal
6 Type-C PortUsed for power supply and downloading CI1302 chip
and SCT8 firmware updates
7 Amplifier ChipConverts digital signals into analog signals to drive the
speaker
8 CI1302 ChipHigh-performance voice recognition chip that
recognizes voice and outputs signals
9Reverse
Connection
Protection5V and GND reverse connection protection
10 IIC PortActs as a slave device, used for power supply and
communication with the host device
11 Serial PortProvides an external serial port for controlling
announcements via protocol
12 STC8H chipConverts voice chip commands into IIC protocol
commands and serial port commands
1.5 Using the Voice Module  
1.2.1 Wiring  
The module connects to the ROSMASTER main control unit (or hub board) via a universal Type-C  
data cable.
1.2.2 Wake-up Word  
The wake-up word is "Hi,yahboom." When waking up, speak slowly; speaking too quickly will  
prevent the module from recognizing you. Only after waking up the module can it recognize other  
command words. Within 20 seconds of waking up, there's no need to wake up again; simply speak  
the command word.
Voice Recognition
ContentVoice Module
Sends to HostHost Sends to
Voice ModuleVoice Broadcast
Content
Car Stop AA 55 00 01 FB AA 55 00 01 FB OK, Stopped
Car Forward AA 55 00 04 FB AA 55 00 04 FBOK, Moving
Forward
Car Backward AA 55 00 05 FB AA 55 00 05 FBOK, Moving
Backward
Car Turn Left AA 55 00 06 FB AA 55 00 06 FB OK, Turning Left
Car Turn Right AA 55 00 07 FB AA 55 00 07 FB OK, turning right
Car turns left AA 55 00 08 FB AA 55 00 08 FB OK, turning left
Car turns right AA 55 00 09 FB AA 55 00 09 FB OK, turning right
Voice
Recognition
ContentVoice Module
Sends to HostHost Sends to
Voice ModuleVoice Broadcast
Content
Turn off lights AA 55 00 0A FB AA 55 00 0A FB OK, lights turned off
Turn on red light AA 55 00 0B FB AA 55 00 0B FB OK, red light is on
Green light on AA 55 00 0C FB AA 55 00 0C FB OK, green light is on
Blue light on AA 55 00 0D FB AA 55 00 0D FB OK, blue light is on
Yellow light on AA 55 00 0E FB AA 55 00 0E FB OK, yellow light is on
Turn on the
running lightAA 55 00 0F FB AA 55 00 0F FBOK, running light is
on
Turn on the
gradient lightAA 55 00 10 FB AA 55 00 10 FBOK, gradient light is
on
Turn on the
breathing lightAA 55 00 11 FB AA 55 00 11 FBOK, the breathing
light is on
Battery level
displayAA 55 00 12 FB AA 55 00 12 FBOK, the battery level
is displayed1.2.3 Command Words  
1) Voice Control of Car Movement
2) Voice Control RGB Light Strip Effect
3) Voice Control Color Recognition
Voice Recognition
ContentVoice Module
Sends to HostHost Sends to
Voice ModuleVoice Broadcast
Content
What color is this? AA 55 00 3C FB AA 55 FF 3D FB This is red
What color is this? AA 55 00 3C FB AA 55 FF 3E FB This is blue
What color is this? AA 55 00 3C FB AA 55 FF 3F FB This is green
What color is this? AA 55 00 3C FB AA 55 FF 40 FB This is yellow
Voice Recognition
ContentVoice Module
Sends to HostHost Sends to
Voice ModuleVoice Broadcast
Content
Start Tracking
YellowAA 55 00 48 FB AA 55 00 48 FBOK, start tracking
yellow
Start Tracking Red AA 55 00 49 FB AA 55 00 49 FBOK, start tracking
red
Start Tracking
GreenAA 55 00 4A FB AA 55 00 4A FBOK, start tracking
green
Start Tracking Blue AA 55 00 4B FB AA 55 00 4B FBOK, start tracking
blue
Cancel Tracking AA 55 00 4C FB   AA 55 00 4C FB   OK, tracking
canceled
Voice
recognition
contentVoice module
sends to hostHost sends to
voice moduleVoice broadcast content
Turn off line
patrolAA 55 00 16 FB AA 55 00 16 FBOK, line patrol function turned
off
Red line patrol AA 55 00 17 FB AA 55 00 17 FBOK, red line patrol function
turned on
Green line
patrolAA 55 00 18 FB AA 55 00 18 FBOK, green line patrol function
turned on
Blue line patrol AA 55 00 19 FB AA 55 00 19 FBOK, the Blue Line patrol
function has been activated
Yellow Line
patrolAA 55 00 1A FB AA 55 00 1A FBOK, the Yellow Line patrol
function has been activated4) Voice Control Color Tracking
5) Voice-controlled autonomous driving (line patrol)
6) Voice-controlled multi-point navigation
Voice recognition
contentVoice module
sends to hostHost sends to voice
moduleVoice broadcast
content
Navigate to
location 1AA 55 00 13 FB AA 55 00 13 FBOK, heading to
location 1
Navigate to
location 2AA 55 00 14 FB AA 55 00 14 FBOK, heading to
location 2
Navigate to
location 3AA 55 00 15 FB AA 55 00 15 FBOK, heading to
position 3
Navigate to
position 4AA 55 00 20 FB AA 55 00 20 FBOK, heading to
position 4
Return to origin AA 55 00 21 FB AA 55 00 21 FBOK, returning to
origin

---

## 1. Mobile APP remote control tutorial.pdf

1. Mobile APP remote control tutorial  
1. Mobile APP remote control tutorial 
1.1. Mobile phone scan code to install APP 
1.2. APP connection to Rosmaster 
1.2.1. Select equipme nt 
1.2.2. Establish a network connection 
1.3, APP function introduction 
1.3.1. Remote control 
1.3.3. colorful lights 
1.3.4. Instructions for use 
1.1. Mobile phone scan code to install APP  
For Android system users, open your mobile browser, scan the QR code below to download and  
install the [MakerControl] APP.  
IOS system users, please open the App store and search for [MakerControl], or open the code  
scanner, scan the QR code below, and download and install the [MakerControl] APP.  
If the latest version of [MakerControl] APP is already installed on your phone, you do not need to  
install it again.  
 
1.2. APP connection to Rosmaster  
Rosmaster's factory-built system comes with a hotspot signal [ROSMASTER] and password  
[12345678]. You can first use your mobile phone to connect to Rosmaster's hotspot signal to form  
a local area network. Or connect both Rosmaster and mobile phone to the same router to form a  
local area network.  
1.2.1. Select equipment  
Open the [MakerControl] APP for the first time, according to the purchased robot model, you  
need to select the [Rosmaster X3] device in the [ROS Robot]  
1.2.2. Establish a network connection  
Click the WiFi icon in the upper right corner to connect the ROSMASTER robot.  
Fill in the IP address displayed by the OLED in the ROSMASTER robot in the IP column. Use the  
default parameters in the Port and Video columns. Click [Connect]. After the connection is  
successful, it will automatically jump to the main control interface, and the WiFi icon in the upper  
right corner is no longer there. A prohibition sign appears.  
Note: Before connecting the device, please confirm that the mobile phone is connected to the  
ROSMASTER hotspot signal, or the mobile phone and the ROSMASTER car are connected to the  
same router. And the app program has been started (the factory system defaults to start the app  
program).  
 
1.3, APP function introduction  
The main interface of Rosmaster APP is divided into three modules, each of which corresponds to  
different functions.  
1.3.1. Remote control  
Click the [remote control] icon on the main interface, and the following interface will appear.  
Part 1. Adjust the speed: control the running speed of robot.
Part 2. Adjust the angle of the front wheel servo: control the rotation angle of the front wheel  
servo.
Part 3. Control the car to move forward and backward, turn left, turn right and stop.
Part 4. Switching the full screen mode: Displays the full screen of the camera, which can be  
matched with
USB wireless handle and stand to use.
Part 5. Camera display screen.
Part 6. Switch control mode: gravity induction, button control, rokcer control.
Part 7. Whistle: Control the buzzer, press the buzzer to turn on, release the buzzer to turn off.
Part 8. Auto-stabilization mode: When auto-stabilization mode is turned on, the car will  
immediately receive a stop command; When the car receives a stop command, it will coast for a  
period of time and stop.
Part 9. Control spin left and right: control the maximum angle of the front wheel servo of robot to  
rotate left and right.
 
1.3.3. colorful lights  
Click the [Dazzling Lights] icon on the main interface, and the following interface will appear.  
The colorful lights are divided into three parts.  
Part 1. [Color Switch] in the upper left part: This function can modify the RGB color of the light bar  
in real time, directly drag the [R] [G] [B] drag bar, you can see the real-time change of the RGB  
light bar at the rear of the Rosmaster Cool lighting effects.  
Part 2. Lower left part [fixed color switching]: This function can make the RGB light bar display red,  
green, blue, yellow, purple, cyan, white, off. At the same time, the color of the monochrome  
breathing light can also be adjusted.  
Part 3. [Cool special effects] on the right: Each time you click a button, specific special effects will  
be displayed, including running water lights, marquee lights, star lights, monochrome breathing  
lights, and gradient lights. Click the button again to exit the special effects; Drag the bar to change  
the speed of the lighting effect, the default is 5, the fastest is 1, and the slowest is 10.  
1.3.4. Instructions for use  
Click the [Configuration] icon on the main interface, and the following interface will appear.
Adjust the relative zero value (default angle value) of the front wheel servo on this interface.
Steps:
1. Lift the front wheel of robot away from the ground.
2. Then adjust the sliding bar and observe the two front wheels of robot. The best effect is that  
keep the two front wheels parallel to the front of the robot car.
3. Click "comfirm" to save the data.

---

## 1. Multi-machine handle control.pdf

1. Multi-machine handle control  
1. Multi-machine handle control
1.1. Multi-machine configuration
1.1.1. Multi-machine commu nication settings
1.1.2. Multi-machine time synchronization
1.2. Use
1.2.1. Start the robot
1.2.2. Turn on handle control
1.3. Handle control analysis
1.3.1. Control multiple robots
1.3.2. Control a robot
1.1. Multi-machine configuration  
When using multi-machine handle control, you first need to ensure that the robots are under the  
same LAN and configured with the same [ROS_MASTER_URI]; multiple robots can only have one  
host to control their movements. In the case of this section, the virtual machine is set as the host  
machine, and other robots are slave machines. There are several slave machines. Of course, you  
can also set a certain robot as the master machine and the others as slave machines.
1.1.1. Multi-machine communication settings  
Check the IP of the virtual machine
Next, you only need to modify the .bashrc file of the slave machine (Robot side). There are several  
slave machines and several configurations.ifconfig
#Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
sudo vim ~/.bashrc
Find the following line of content
The [IP] here uses the IP (virtual machine side).
After setting the IP, it is best to refresh the environment variables.
1.1.2. Multi-machine time synchronization  
Install
time view command
When there is a network, the system will automatically synchronize the network system time  
without setting. If there is no network, the time needs to be set when the time between robots is  
different.
Internet available
Automatically synchronize network time command
No network
The test case is using two robots.
Command to manually set time
1), Server-side (host-side) configuration
Open the [ntp.conf] file
Add at the end of the fileexport ROS_MASTER_URI=http://IP:11311
export ROS_MASTER_URI=http://192.168.2.106:11311
source ~/.bashrc
sudo apt install ntp ntpdate
date -R
ntpd pool ntp.ubuntu.com
sudo date -s "2020-01-1 01:01:01"
sudo vim /etc/ntp.conf
restrict 192.168.2.0 mask 255.255.255.0 nomodify notrap
server 127.127.1.0 # local clock
fudge 127.127.1.0 stratum 10
The first line is to enable the machine on the 192.168.2.xxx network segment to synchronize time  
with this machine (specifically, it depends on whether your IP is 192.168.2.xxx. If it is different,  
change it to your actual format)
The second and third lines are for time synchronization between the local hardware time and the  
local ntp service.
After making the changes, restart the ntp service
The server-side settings are completed, let’s start the client-side settings.
2. Client (slave side) configuration
Open the [ntp.conf] file
Add [server]+[IP] at the end of the file
3), time synchronization
On the [server side], execute the following code to check whether the ntp service of the [server  
side] is running.
If it is not started, restart the ntp service.
Network updates (required for all devices)
Time synchronizationsudo /etc/init.d/ntp restart
sudo vim /etc/ntp.conf
server 192.168.2.106
service ntp status
sudo /etc/init.d/networking restart
If the client's NTP is also enabled, you need to close the NTP service first and then perform time  
synchronization.
1.2. Use  
Take the virtual machine as the host machine and the two robots as slave machines as an  
example.
1.2.1. Start the robot  
Virtual machine side
Start the command (robot1 side). For the convenience of operation, this section takes [mono +  
laser + yahboomcar] as an example.
Start the command (robot2 side). For the convenience of operation, this section takes [mono +  
laser + yahboomcar] as an example.
More robots and so on.
1.2.2. Turn on handle control  
Method 1: One controller controls multiple robots at the same time
Connect the controller receiver to the USB port of the virtual machine and start the command
Note: The [launch] file defaults to three robots. 
joy_multi.launchsudo ntpdate 192.168.2.106
sudo /etc/init.d/ntp stop
roscore
roslaunch yahboomcar_multi laser_bringup_multi.launch ns:=robot1 # laser + 
yahboomcar
roslaunch yahboomcar_multi laser_usb_bringup_multi.launch ns:=robot1 # mono + 
laser + yahboomcar
roslaunch yahboomcar_multi laser_astrapro_bringup_multi.launch ns:=robot1 # Astra 
+ laser + yahboomcar
roslaunch yahboomcar_multi laser_bringup_multi.launch ns:=robot2 # laser + 
yahboomcar
roslaunch yahboomcar_multi laser_usb_bringup_multi.launch ns:=robot2 # mono + 
laser + yahboomcar
roslaunch yahboomcar_multi laser_astrapro_bringup_multi.launch ns:=robot2 # Astra 
+ laser + yahboomcar
roslaunch yahboomcar_multi joy_multi.launch
If there are 2 robots, just comment out the part of the third robot. If you don't comment it out, it  
will not have any effect. If there are 4 robots or more, you can add it according to the samples of  
the first 3 robots.
Method 2: One handle controls one robot alone
<PI5 needs to open another terminal to enter the same docker container
Connect the handle receiver to the corresponding robot, and start the command on the  
corresponding robot side (take robot1 as an example)<launch>
     <arg name="first_robot1" default="robot1"/>
     <arg name="second_robot2" default="robot2"/>
     <arg name="third_robot3" default="robot3"/>
     <param name="use_sim_time" value="false"/>
     <node name="joy_node" pkg="joy" type="joy_node" output="screen" 
respawn="false"/>
     <!-- ############################ first_robot1 ################ 
############# -->
     <include file="$(find yahboomcar_multi)/launch/library/joy_base.launch">
         <arg name="ns" default="$(arg first_robot1)"/>
     </include>
     <!-- ############################ second_robot2 ################ 
############# -->
     <include file="$(find yahboomcar_multi)/launch/library/joy_base.launch">
         <arg name="ns" default="$(arg second_robot2)"/>
     </include>
     <!-- ############################ third_robot3 ################ 
############# -->
     <include file="$(find yahboomcar_multi)/launch/library/joy_base.launch">
         <arg name="ns" default="$(arg third_robot3)"/>
     </include>
</launch>
roslaunch yahboomcar_multi joy_each.launch ns:=robot1
1.3. Handle control analysis  
Node view
1.3.1. Control multiple robots  
It can be seen from the figure that when one controller controls multiple robots at the same time,  
only one [joy_node] node is needed to receive the controller signal, and different [yahboom_joy]  
nodes are set for different robots to control the corresponding robots.
1.3.2. Control a robot  
It can be seen from the figure that when a handle controls a robot, each robot must be set and  
controlled separately.rqt_graph

---

## 1. Update the firmware.pdf

1. Update the firmware of the expansion board  
1. Update the firmware of the expansion board 
1.1. Update firmware statement 
1.2. Download burning software and firmware 
1.3. Install the CH340 driver 
1.4. Compu ter connection to Rosmaster 
1.5. configure the burning software 
1.6. Start burning firmware 
1.1. Update firmware statement  
The MCU integrated in the Rosmaster expansion board has already programmed the firmware  
when it leaves the factory. If it is not necessary, please do not update the firmware.  
1.2. Download burning software and firmware  
Download burning software  
This time burning Rosmaster expansion board microcontroller firmware requires mcuisp (or  
flymcu) burning software, please go to http://www.mcuisp.com website to download mcuisp (or  
flymcu) burning software; you can also use the data provided directly mcuisp software.  
The mcuisp software is a green version software, which does not need to be installed. Double-
click to open it to use.  
Download expansion board MCU firmware  
In this course material, the latest firmware file of the Rosmaster expansion board with the latest  
version is provided. The name is Rosmaster_XXX.hex, where XXX is the version number.  
Also available for download at: Yahboom Rosmaster  
 
1.3. Install the CH340 driver  
Since the USB communication of the Rosmaster expansion board uses the CH340 chip, the driver  
of the CH340 chip needs to be installed. If the computer has already installed the CH340 driver,  
there is no need to install it again.  
Unzip the [Uart drive (CH340).zip] in the course materials, double-click to open the CH341SER.EXE  
program  
 
Click Install. After the installation is complete, you will be prompted that the installation was  
successful.  
1.4. Computer connection to Rosmaster  
Note: Before connecting the Rosmaster to the computer, please unplug the Micro USB data cable  
and power cable connecting the expansion board to the Jetson Nano.  
Insert one end of the USB data cable into the USB port of the computer, and the other end into  
the Micro USB port of the Rosmaster expansion board.  
1.5. configure the burning software  
When searching for a serial port, if there are multiple serial port numbers, it is not confirmed  
which one is the Rosmaster. Solution 1: Unplug other USB ports and search again; Solution 2: First  
unplug the Rosmaster USB data cable, click Search Serial Port, write down the searched serial port  
number, insert the Rosmaster USB data cable, search the serial port again, and compare before  
and after Twice, the newly added serial port number is the serial port number of the Rosmaster.  
When choosing the firmware, you need to select the firmware path downloaded in the first step. It  
is best not to have Chinese or special symbols in the path. As an example, the above picture is to  
put the downloaded Rosmaster_V3.1.hex on the desktop.  
The last is the configuration selection at the bottom. Be sure to select the option of [DTR low-level  
reset, RTS high-level into BootLoader] option, otherwise the download may fail.  
1.6. Start burning firmware  
Please put the microcontroller on the expansion board into the programming mode first:  
First press and hold the BOOT0 key on the expansion board, then press the RESET key, and finally  
release the BOOT0 key.  
Click [Start Programming], and the mcuisp burning software will burn the firmware we selected in  
the previous step to the microcontroller on the Rosmaster expansion board. When the prompt  
appears on the right side [successfully run www.mcuisp.com to report to you, the command is  
executed, everything is normal], it means that the download is successful.  
Notice:  
①Before starting programming, please confirm that the serial port number of the Rosmaster is  
accessible, that is, there is no serial port assistant occupying it.  
②Rosmaster enters the burning mode operation, first press and hold the BOOT0 key on the  
expansion board, then press the RESET key, and finally release the BOOT0 key.  

---

## 1. Virtual machine installation and use.pdf

1. Virtual machine installation and use  
1. Virtual machine installation and use 
1.1. virtual machine installation 
1.2. the use of virtual machines 
1.3. set auto-start 
1.4. virtual machine settings 
Username: yahboom  
Password: yahboom  
Virtual machine: Ubuntu18.04  
1.1. virtual machine installation  
The virtual machine VMware Workstation is a paid software, and we do not provide VMware  
downloads.  
Double-click the downloaded exe installation file  
Click Next  
 
Select the Receive Agreement and click Next  
 
Select the path to install locally, here it is installed under D:\Program Files(x86), click Next  
 
Keep clicking next until  
 

Click Continue to start the installation, wait a few minutes  
 
The above completes the installation of VMWare, you can open VMWare on the desktop, as  
follows  
 
1.2. the use of virtual machines  
Find a local hard disk with more than 40G free space, and decompress the system compressed  
file. 
After installing the VMware Workstation virtual machine, double-click to open it to enter the  
interface as shown in the figure.  
Click [Open a Virtual Machine]  
Select the unzipped system file, as shown in the following figure.  
Click [Power on this virtual machine]  
If the prompt shown in the figure below appears, directly select [I Copied It].  
 
Click 【 yahboom 】 
Enter the password [yahboom] and press [enter] on the keyboard to confirm.  
1.3. set auto-start  
After entering the system, click the inverted triangle icon in the upper right corner, find the  
[yahboom] column, click the triangle icon on the right, and select [Account Settings].  
Click the [Unlock] button at the top to pop up a dialog box  

Enter the password [yahboom] and click Enter to confirm.  
Click the slider on the right side of [Automatic Login], as shown in the figure below. At this point,  
click the [x] in the upper right corner to close it.  
At this point, the self-startup setting is completed. When you start the virtual machine again, you  
do not need to enter the login password.  
1.4. virtual machine settings  
You can edit the virtual machine settings before starting, as shown in the figure below, click [Edit  
virtual machine settings] at this time  
The [Virtual Machine Settings] dialog box will pop up. Since the situation of each computer is  
different, you can set it according to the actual situation.  
The higher the virtual machine memory setting, the faster the virtual machine will run, but do not  
exceed the Maximum recommended memory.  
Processors: The higher the number of cores (c) per processor, the smoother the virtual machine  
runs, but not more than the sum.  
USB controller: USB compatibility is set to [USB 3.1], so that you can use USB 3.0 devices.  

---

## 1.About expansion boardV3.0.pdf

1. Introduction to expansion board  
1. Introduction to expansion board
1.1 、 Schematic diagram of compo nent distribution on the front of expansion board
1.2 、 Schematic diagram of compo nent distribution on the back of the expansion board
1.3 、 Analysis of Frequently Asked Questions about Expansion Boards
1.1 、 Schematic diagram of component distribution on the
front of expansion board 
①T-shaped DC 12V power input interface: used as the main power input of the expansion board,  
connected to the DC 12V power supply or 12V battery.
②⑨ DC 12V power output: Provides DC 12V power to the outside.
③Power indicator light: Indicates whether the power supply is normal.
④Micro USB data interface: connects to host communication and burning program.
⑤Type-C interface: Provides DC 5V to the outside, supports the Raspberry Pi 5 power supply  
protocol, and provides 5.1V/5A power supply to the Raspberry Pi 5. It only provides power but  
cannot communicate.
⑥DC 5V output interface: can power Jetson Nano.
⑦I2C interface: can connect external I2C devices, such as OLED screens.
⑧Indicator light: data indicator light and 6.8V voltage indicator light.
⑩DC 12V power switch: main power switch.
⑪Button: Button KEY1: User function button, which can realize customized functions through  
programming. Button RESET: Onboard microcontroller reset button. Button BOOT0: The BOOT0  
key of the onboard microcontroller is used to enter the programming mode of the  
microcontroller.
⑫Nine-axis attitude sensor: Provides the current attitude of the expansion board.
⑬CAN interface: Connect to CAN equipment.
⑭RGB colorful light strip interface: Connect to RGB colorful light strip.
⑮SBUS interface: Connect to the model aircraft remote control receiver.
⑯ PWM servo voltage switching: Change the position of the jumper cap to select 6.8V or 5V  
voltage to power the PWM servo.
⑰PWM servo interface: It can connect to 6.8V or 5V voltage PWM servo. You need to select the  
corresponding voltage in ⑯  according to the servo voltage.
⑱Serial servo interface: Connect to the serial servo mechanical arm.
⑲Buzzer: used to sound the alarm.
⑳Four-way motor connection port: Connect four motors. Please refer to the corresponding  
course documents according to the connection methods of different models.
㉑SWD debugging interface: Connect to the SW interface on ST-Link or J-Link, used to debug the  
microcontroller or download the microcontroller firmware.
 
1.2 、 Schematic diagram of component distribution on the
back of the expansion board 
①Onboard microcontroller: Mainly responsible for controlling peripherals on the expansion  
board, such as buzzers, motor drives, etc.
②Debug interface: Connect to the SW interface on ST-Link or J-Link, used to debug the  
microcontroller or download the microcontroller firmware. Note: There is no warranty after  
welding.
 
1.3 、 Analysis of Frequently Asked Questions about
Expansion Boards 
A: How to control the expansion board of Jetson Nano? How to communicate with expansion  
board?
Answer: Jetson Nano sends serial port data and transmits it to the expansion board through the  
USB port. The expansion board integrates a microcontroller to receive and parse the serial port  
data and then process the specific commands to be executed.
B: How to power the robot? Does Jetson Nano need another power supply?
Answer: The car is equipped with a battery pack at the factory. Insert the battery pack into the DC  
12V power T-port of the expansion board, turn on the main power switch, and the expansion  
board integrates a voltage conversion chip to provide DC 5V power, which is transmitted to the  
Jetson Nano through the DC 5V power line. powered by.
C: Which functions on the expansion board are managed by the microcontroller? ?
Answer: The parts managed by the microcontroller on the expansion board include: robotic arm,  
active buzzer, attitude sensor, PWM steering gear, motor, RGB colorful light bar, button KEY1,  
RESET button, SBUS interface, CAN interface, etc. .
D: How to update the microcontroller firmware of the expansion board? Why update  
microcontroller firmware?
Answer: The microcontroller integrated with the expansion board has already been programmed  
with firmware before leaving the factory. Please do not update the firmware unless necessary. If  
you need to update the firmware, please refer to the tutorial on updating firmware to update the  
firmware of the microcontroller.
E: What is the difference between expansion board hardware versions V1.0 and V3.0?
Answer: Hardware version V3.0 mainly upgrades the IMU chip to ICM20948, adds a SWD  
debugging interface and an external switch interface, supports the Raspberry Pi 5 power supply  
protocol, and can provide 5.1V/5A power supply for the Raspberry Pi 5. It has been modified and  
optimized. Part of the circuit, etc. Version V3.0 requires the use of Python library V3.3.X or above.

---

## 1.Lidar basics.pdf

1. Radar basics  
1. Radar basics
1.1. Overview
1.2. Principle of single-line lidar
1.2.1. Trigonometric ranging method
1. Direct type
2. Oblique shot type
1.2.2. TOF time-of-flight ranging method
1.3. Use radar
1.3.1, 4ROS radar:
1.3.2, X3/X3Pro radar:
1.4, launch analysis
For different models of radar
Before using lidar, you need to declare the [RPLIDAR_TYPE] variable in advance in the [.bashrc] file  
according to different radar models. Open the [.bashrc] file
If there is no sentence below in [.bashrc], you need to add it manually according to the purchased  
radar model. If there is this sentence, modify the radar model directly. For example: 4ROS lidar
Note: For rosmaster series cars equipped with X3 or X3Pro radar, the startup methods of  
the two radars are the same, just change them to X3. 
After modification, refresh the environment variables
1.1. Overview  
Single-line lidar refers to a radar whose line beam emitted by the laser source is a single line. It  
can be divided into triangular ranging and TOF lidar. It is mainly used in the field of robotics. It has  
fast scanning speed, strong resolution and high reliability. Compared with multi-line lidar, single-
line lidar responds faster in angular frequency and sensitivity, so it is more accurate in measuring  
distance and accuracy of obstacles.
 #Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
sudo vim ~/.bashrc
export RPLIDAR_TYPE=4ROS # a1, a2, a3, s1, s2 ， 4ROS ， X3
source ~/.bashrc
1.2. Principle of single-line lidar  
The working principle of the single-wire mechanical rotating radar is as shown in the figure below:
1.2.1. Trigonometric ranging method  
The laser triangulation ranging method mainly uses a beam of laser to illuminate the measured  
target at a certain incident angle. The laser is reflected and scattered on the target surface. At  
another angle, a lens is used to converge and image the reflected laser. The spot is imaged on the  
CCD (Charge-coupled Device, photosensitive coupling component) on the position sensor. When  
the measured object moves along the direction of the laser, the light spot on the position sensor  
will move, and its displacement corresponds to the movement distance of the measured object.  
Therefore, the distance between the measured object and the baseline can be calculated from the  
light spot displacement distance through algorithm design. value. Since the incident light and the  
reflected light form a triangle, the geometric triangle theorem is used to calculate the spot  
displacement, so this measurement method is called the laser triangulation ranging method.
According to the angular relationship between the incident beam and the normal line of the  
surface of the measured object, the laser triangulation ranging method can be divided into two  
types: oblique type and direct type.
1. Direct type  
As shown in Figure 1, when the laser beam is vertically incident on the surface of the object to be  
measured, that is, when the incident light is collinear with the normal line of the surface of the  
object to be measured, it is a direct laser triangulation method.
2. Oblique shot type  
When the angle between the incident laser beam and the normal line of the surface of the object  
being measured is less than 90° in the optical path system, the incident mode is oblique. The  
optical path diagram shown in Figure 2 is a laser triangulation oblique optical path diagram.

The laser emitted by the laser is incident on the surface of the object being measured at a certain  
angle with the normal line of the object surface. The reflected (scattered) light is concentrated  
through the lens at B and is finally collected by the photosensitive unit.
Whether it is the direct or oblique laser triangulation ranging method, it can achieve high-
precision, non-contact measurement of the measured object, but the resolution of the direct type  
is not as high as that of the oblique type.
Silan Technology's RPLIDAR series lidar also uses the oblique laser triangulation ranging method.  
During each ranging process, the RPLIDAR series lidar will emit a modulated infrared laser signal.  
The reflection generated by the laser signal after hitting the target object will be received by the  
RPLIDAR visual acquisition system, and then processed by the DSP embedded inside the RPLIDAR.  
The device solves the problem in real time, and the distance value between the illuminated target  
object and the RPLIDAR and the current angle information will be output from the communication  
interface. Driven by the motor mechanism, the ranging core of RPLIDAR will rotate clockwise,  
thereby achieving 360-degree all-round scanning and ranging detection of the surrounding  
environment.
1.2.2. TOF time-of-flight ranging method  
TOF lidar is based on measuring the flight time of light to obtain the distance of the target. Its  
working principle is mainly as follows: a modulated laser signal is emitted through a laser  
transmitter. The modulated light is received by the laser detector after being reflected by the  
object being measured. The distance to the target can be calculated by measuring the phase  
difference between the emitted laser and the received laser. .
Under the condition of distant objects, its measurement accuracy remains accurate and stable. At  
the same time, TOF radar is not inferior in its ability to resist light interference due to its ultra-
short light pulse characteristics. It can achieve stable ranging and high-precision mapping even  
under strong light of 60Klx outdoors.
Generally speaking, triangular ranging lidar and TOF lidar have their own difficulties in  
implementation. In principle, TOF radar has a longer ranging distance. In some occasions where  
distance is required, TOF radar is the most common, while The manufacturing cost of triangular  
ranging lidar is relatively low, and its accuracy can meet most industrial-grade civilian  
requirements, so it has also attracted much attention in the industry.
1.3. Use radar  
1.3.1, 4ROS radar:  
Terminal run
<PI5 needs to open another terminal to enter the same docker container
We can print the topic data through the terminal to check whether the radar starts normally, enter  
the terminal,
If you want to view the scan results in RVIZ, enter in the terminal,roslaunch ydlidar_ros_driver TG.launch
rostopic echo /scan
roslaunch ydlidar_ros_driver lidar_view.launch
1.3.2, X3/X3Pro radar:  
Terminal run
<PI5 needs to open another terminal to enter the same docker container
We can print the topic data through the terminal to check whether the radar starts normally, enter  
the terminal,
If you want to view the scan results in RVIZ, enter in the terminal,roslaunch ydlidar_ros_driver X2.launch
rostopic echo /scan
1.4, launch analysis  
Path: ~/software/library_ws/src/ydlidar_ros_driver-master/launch
TG.launch fileroslaunch ydlidar_ros_driver lidar_view.launch
<launch>
   <arg name="frame_id" default="laser"/>
   <node name="ydlidar_lidar_publisher" pkg="ydlidar_ros_driver" 
type="ydlidar_ros_driver_node" output="screen" respawn="false" >
     <!-- string property -->
     <param name="port" type="string" value="/dev/ydlidar"/>
     <param name="frame_id" type="string" value="$(arg frame_id)"/>
     <!--param name="ignore_array" type="string" value="-90,90"/-->
     <param name="ignore_array" type="string" value=""/>
     <!--remap from="scan" to="scan_raw"/-->
     <!-- int property -->
     <param name="baudrate" type="int" value="512000"/>
     <!-- 0:TYPE_TOF, 1:TYPE_TRIANGLE, 2:TYPE_TOF_NET -->
     <param name="lidar_type" type="int" value="0"/>
     <!-- 0:YDLIDAR_TYPE_SERIAL, 1:YDLIDAR_TYPE_TCP -->
     <param name="device_type" type="int" value="0"/>
     <param name="sample_rate" type="int" value="20"/>
     <param name="abnormal_check_count" type="int" value="4"/>
     <!-- bool property -->
     <param name="resolution_fixed" type="bool" value="true"/>
     <param name="auto_reconnect" type="bool" value="true"/>
     <param name="reversion" type="bool" value="true"/>
     <param name="inverted" type="bool" value="true"/>
     <param name="isSingleChannel" type="bool" value="false"/>
Main debugging parameters:
angle_min parameter: radar left angle
angle_max parameter: radar right angle
For other parameters, please refer to the official documentation.
ydlidar_ros_driver/README.md at master · YDLIDAR/ydlidar_ros_driver· GitHub     <param name="intensity" type="bool" value="false"/>
     <param name="support_motor_dtr" type="bool" value="false"/>
     <param name="invalid_range_is_inf" type="bool" value="true"/>
     <param name="point_cloud_preservative" type="bool" value="false"/>
     <!-- float property -->
     <param name="angle_min" type="double" value="-90" />
     <param name="angle_max" type="double" value="90" />
     <param name="range_min" type="double" value="0.01" />
     <param name="range_max" type="double" value="50.0" />
     <param name="frequency" type="double" value="10.0"/>
   </node>
   <!--node pkg="tf" type="static_transform_publisher" name="base_link_to_laser4"
     args="0.0 0.0 0.2 3.14 0.0 0.0 /base_footprint /laser 40" /-->
</launch>

---

## 1.Lidar basics_1.pdf

1. Radar basics  
1. Radar basics
1.1. Overview
1.2. Silan radar compo nents
1.2.1. Laser
1.2.2. Receiver
1.2.3. Signal processing unit
1.2.4. Rotating mechanism
1.3. Principle of single-line lidar
1.3.1. Trigonometric ranging method
1. Direct type
2. Oblique shot type
1.3.2. TOF time-of-flight ranging method
1.4. Lidar A1M8
1.5. Application scenarios
1.6. Function package rplidar ros
1.6.1. Remap USB serial port
1.6.2. Code testing
1.6.3. Map construction test
1.7. Source code analysis
Slam lidar tutorial materials: http://www.slamtec.com/cn/Support
Lidar technology email address: support@slamtec.com
Lidar wiki: http://wiki.ros.org/rplidar
LiDAR SDK: https://github.com/Slamtec/rplidar_sdk
LiDAR ROS: https://github.com/Slamtec/rplidar_ros
Lidar tutorial: https://github.com/robopeak/rplidar_ros/wiki
For different models of radar
Before using lidar, you need to declare the [RPLIDAR_TYPE] variable in advance in the [.bashrc] file  
according to different radar models. Open the [.bashrc] file
If there is no sentence below in [.bashrc], you need to add it manually according to the purchased  
radar model. If there is this sentence, modify the radar model directly. For example: Silan a1 lidar
Note: For rosmaster series cars equipped with s2l radar, the radar startup method is the  
same as s2. Just change it to s2 here. #Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
sudo vim ~/.bashrc
export RPLIDAR_TYPE=a1 # a1, a2, a3, s1, s2
After modification, refresh the environment variables
1.1. Overview  
Single-line lidar refers to a radar whose line beam emitted by the laser source is a single line. It  
can be divided into triangular ranging and TOF lidar. It is mainly used in the field of robotics. It has  
fast scanning speed, strong resolution and high reliability. Compared with multi-line lidar, single-
line lidar responds faster in angular frequency and sensitivity, so it is more accurate in measuring  
distance and accuracy of obstacles.
1.2. Silan radar components  
Taking Silan Technology's single-line lidar as an example, it is mainly composed of four core  
components: laser, receiver, signal processing unit and rotating mechanism.
RPLIDAR adopts a coordinate system that follows the left-hand rule. The front of the sensor is  
defined as the x-axis of the coordinate system. The origin of the coordinate system is the rotation  
center of the ranging core. The rotation angle increases with clockwise rotation. The specific  
coordinate system definition is shown in the figure below: (For details, please see the supporting  
manual)source ~/.bashrc
| A1 | A2, A3 |
| :------------------------------------------------ -: | :------------------------------------------------- ---: |
| 
  | 
 |
| S1 | S2 |
| 
  | 
 |
1.2.1. Laser  
The laser is the laser emitting mechanism in lidar. During operation, it lights up in a pulsed  
manner. Silan Technology's RPLIDAR A3 series radar lights up and goes out 16,000 times per  
second.
1.2.2. Receiver  
After the laser emitted by the laser hits an obstacle, the reflected light will be concentrated on the  
receiver through the lens group through reflection from the obstacle.
1.2.3. Signal processing unit  
The signal processing unit is responsible for controlling the emission of the laser and processing  
the signals received by the receiver. Based on this information, the distance information of the  
target object is calculated.
1.2.4. Rotating mechanism  
The above three components constitute the core components of measurement. The rotating  
mechanism is responsible for rotating the above-mentioned core components at a stable speed to  
scan the plane and generate real-time floor plan information.
1.3. Principle of single-line lidar  
The working principle of the single-wire mechanical rotating radar is as shown in the figure below:
1.3.1. Trigonometric ranging method  
The laser triangulation ranging method mainly uses a beam of laser to illuminate the measured  
target at a certain incident angle. The laser is reflected and scattered on the target surface. At  
another angle, a lens is used to converge and image the reflected laser. The spot is imaged on the  
CCD (Charge-coupled Device, photosensitive coupling component) on the position sensor. When  
the measured object moves along the direction of the laser, the light spot on the position sensor  
will move, and its displacement corresponds to the movement distance of the measured object.  
Therefore, the distance between the measured object and the baseline can be calculated from the  
light spot displacement distance through algorithm design. value. Since the incident light and the  
reflected light form a triangle, the geometric triangle theorem is used to calculate the spot  
displacement, so this measurement method is called the laser triangulation ranging method.
According to the angular relationship between the incident beam and the normal line of the  
surface of the measured object, the laser triangulation ranging method can be divided into two  
types: oblique type and direct type.
1. Direct type  
As shown in Figure 1, when the laser beam is vertically incident on the surface of the object to be  
measured, that is, when the incident light is collinear with the normal line of the surface of the  
object to be measured, it is a direct laser triangulation method.
2. Oblique shot type  
When the angle between the incident laser beam and the normal line of the surface of the object  
being measured is less than 90° in the optical path system, the incident mode is oblique. The  
optical path diagram shown in Figure 2 is a laser triangulation oblique optical path diagram.
The laser emitted by the laser is incident on the surface of the object being measured at a certain  
angle with the normal line of the object surface. The reflected (scattered) light is concentrated  
through the lens at B and is finally collected by the photosensitive unit.
Whether it is the direct or oblique laser triangulation ranging method, it can achieve high-
precision, non-contact measurement of the measured object, but the resolution of the direct type  
is not as high as that of the oblique type.
Silan Technology's RPLIDAR series lidar also uses the oblique laser triangulation ranging method.  
During each ranging process, the RPLIDAR series lidar will emit a modulated infrared laser signal.  
The reflection generated by the laser signal after hitting the target object will be received by the  
RPLIDAR visual acquisition system, and then processed by the DSP embedded inside the RPLIDAR.  
The device solves the problem in real time, and the distance value between the illuminated target  
object and the RPLIDAR and the current angle information will be output from the communication  
interface. Driven by the motor mechanism, the ranging core of RPLIDAR will rotate clockwise,  
thereby achieving 360-degree all-round scanning and ranging detection of the surrounding  
environment.
1.3.2. TOF time-of-flight ranging method  
TOF lidar is based on measuring the flight time of light to obtain the distance of the target. Its  
working principle is mainly as follows: a modulated laser signal is emitted through a laser  
transmitter. The modulated light is received by the laser detector after being reflected by the  
object being measured. The distance to the target can be calculated by measuring the phase  
difference between the emitted laser and the received laser. .
Under the condition of distant objects, its measurement accuracy remains accurate and stable. At  
the same time, TOF radar is not inferior in its ability to resist light interference due to its ultra-
short light pulse characteristics. It can achieve stable ranging and high-precision mapping even  
under strong light of 60Klx outdoors.
Generally speaking, triangular ranging lidar and TOF lidar have their own difficulties in  
implementation. In principle, TOF radar has a longer ranging distance. In some occasions where  
distance is required, TOF radar is the most common, while The manufacturing cost of triangular  
ranging lidar is relatively low, and its accuracy can meet most industrial-grade civilian  
requirements, so it has also attracted much attention in the industry.
Indicator Description
Ranging radius Radar measuring distance range
Ranging sampling rateHow many ranging outputs are performed in one
second
Scanning frequency How many times the radar scans in one second
Angular resolution Angle step between two adjacent ranging
Measurement
resolution/accuracyMinimum distance change can be perceived1.4. Lidar A1M8  
As can be seen from the figure above, parameters such as measurement radius, sampling speed,  
rotation speed, and angular resolution are important indicators of radar performance.
A higher scanning frequency  can ensure that the robot equipped with lidar can move at a faster  
speed and ensure the quality of map construction. However, increasing the scanning frequency is  
not as simple as accelerating the rotation of the lidar's internal scanning motor. Correspondingly,  
it is necessary to increase the ranging sampling rate. Otherwise, when the sampling frequency is  
fixed, faster scanning speed will only reduce the angular resolution. In addition to ranging distance  
and scanning frequency, parameters such as measurement resolution and mapping accuracy are  
equally important to lidar performance. These are important parameters to ensure that the robot  
can have stable performance.
1.5. Application scenarios  
Thanks to the advancement of lidar technology, the measurement radius, ranging frequency,  
distance resolution and angular resolution of lidar have been greatly improved, which can help  
various applications obtain larger scenes and richer contour information. It plays an indispensable  
and important role in many fields such as robot autonomous positioning and navigation, space  
environment surveying and mapping, and security and defense.
1.6. Function package rplidar ros  
Clone this project into the src folder of your workspace and run catkin_make to build rplidarNode  
and rplidarNodeClient.
1.6.1. Remap USB serial port  
When starting the radar function package, you may encounter that the serial port permissions are  
not executable. There are two solutions: 1) Add permissions directly; 2) Remap the USB serial port
1. Directly add permission method
This method only works this time.
Check the permissions of rplidar serial port:
Add write permission: (such as /dev/ttyUSB0)
2. Remapping USB serial port method
This method works long term.
In the rplidar_ros function package path, install USB port remapping
Re-plug the LiDAR USB interface and use the following command to modify the remapping:ls -l /dev |grep ttyUSB
sudo chmod 777 /dev/ttyUSB0
./scripts/create_udev_rules.sh
1.6.2. Code testing  
Run the rplidar node and view it in rviz
You should see rplidar's scan results in rviz.
Run the rplidar node and see with the test application
<PI5 needs to open another terminal to enter the same docker containerls -l /dev | grep ttyUSB
roslaunch rplidar_ros view_rplidar.launch
roslaunch rplidar_ros rplidar.launch # Launch radar
You should see rplidar's scan results in the console
1.6.3. Map construction test  rosrun rplidar_ros rplidarNodeClient # Get and print radar data
roslaunch rplidar_ros test_gmapping.launch
rosrun rqt_tf_tree rqt_tf_tree
1.7. Source code analysis  
From the test in section [1.6], it can be seen that the lidar data does not have 360° data. There is a  
small gap. That is because the data behind the lidar is blocked.
rplidar.launch filerosrun rqt_graph rqt_graph
<launch>
     <arg name="lidar_type" value="$(env RPLIDAR_TYPE)" doc="lidar_type type 
[a1,a2,a3,s1,s2]"/>
     <arg name="frame_id" default="laser"/>
     <arg name="shielding_angle" default="30"/>
shielding_angle parameter: the angle for shielding radar data, range [0, 360], which can be  
adjusted according to the actual situation.
gmapping is only applicable to points where the number of two-dimensional laser points in a  
single frame is less than 1440. If the number of laser points in a single frame is greater than  
1440, then problems such as [[mapping-4] process has died] will occur. Therefore, when  
using S2 lidar, the number of S2 points needs to be diluted.
If you do not need to filter radar data, comment or delete the following content in the  
[rplidar.launch] file
Then modify the [rplidarNode] node
Users can handle it according to the actual situation.     <!-- scan filtering node -->
     <node name="scan_filter" pkg="rplidar_ros" type="scan_filter.py" 
output="screen" respawn="true">
         <param name="shielding_angle" type="double" value="$(arg 
shielding_angle)"/>
     </node>
     <node name="rplidarNode" pkg="rplidar_ros" type="rplidarNode" 
output="screen" respawn="true">
         <param name="serial_port" type="string" value="/dev/rplidar"/>
         <param name="serial_baudrate" type="int" value="115200" if="$(eval 
arg('lidar_type') == 'a1')"/>
         <param name="serial_baudrate" type="int" value="115200" if="$(eval 
arg('lidar_type') == 'a2')"/>
         <param name="serial_baudrate" type="int" value="256000" if="$(eval 
arg('lidar_type') == 'a3')"/>
         <param name="serial_baudrate" type="int" value="256000" if="$(eval 
arg('lidar_type') == 's1')"/>
         <param name="serial_baudrate" type="int" value="1000000" if="$(eval 
arg('lidar_type') == 's2')"/>
         <param name="frame_id" type="string" value="$(arg frame_id)"/>
         <param name="inverted" type="bool" value="false"/>
         <param name="angle_compensate" type="bool" value="true"/>
         <param name="scan_mode" type="string" value="Sensitivity" if="$(eval 
arg('lidar_type') == 'a3')"/>
         <param name="scan_mode" type="string" value=" " unless="$(eval 
arg('lidar_type') == 'a3')"/>
         <remap from="scan" to="scan_raw"/>
     </node>
</launch>
     <!-- scan filtering node -->
     <node name="scan_filter" pkg="rplidar_ros" type="scan_filter.py" 
output="screen" respawn="true">
         <param name="shielding_angle" type="double" value="$(arg 
shielding_angle)"/>
     </node>
<remap from="scan" to="scan_raw"/> <!-- Delete -->
<remap from="scan" to="scan"/> <!-- Add -->

---

## 2、Common commands for docker image containers.pdf

2. Common commands for docker image  
containers 
 
2. Common commands for docker image containers
2.1. Do not use sudo comma nd
2.2. Help comma nd
2.3. Mirror comma nd
2.4. Container comma nd
2.5. Other commo nly used comma nds
2.6. Comma nd summa ry
The operating environment and software and hardware reference configuration are as follows:
Reference model: ROSMASTER X3
Robot hardware configuration: Arm series main control, Silan A1 lidar, AstraPro Plus depth  
camera
Robot system: Ubuntu (no version required) + docker (version 20.10.21 and above)
PC virtual machine: Ubuntu (18.04) + ROS (Melodic)
Usage scenario: Use on a relatively clean 2D plane
 
2.1. Do not use sudo command  
Normally, to operate docker commands, you need to prefix sudo, as follows:
But after adding the docker user group, you don't need to add the sudo prefix. How to add the  
docker user group (run the command in the host running docker):
After adding the above command, use the [docker images] command to test. If no error is  
reported, it means that the sudo command is no longer needed. If the following error is reported:
Then execute the following command on the host machine to solve the problem:sudo docker version
sudo groupadd docker # Add docker user group
sudo gpasswd -a $USER docker # Add the current user to the docker user group, 
where $USER can automatically resolve to the currently logged in user
newgrp docker # Update docker user group
pi@ubuntu:~$ docker images
WARNING: Error loading config file: /home/pi/.docker/config.json: open 
/home/pi/.docker/config.json: permission denied
 
2.2. Help command  
 
2.3. Mirror command  
1. docker pull download image
 
2. docker images list imagessudo chown "$USER":"$USER" /home/"$USER"/.docker -R
sudo chmod g+rwx "/home/$USER/.docker" -R
docker info # Display Docker system information, including the number of images 
and containers. .
docker --help # help
# Download image
jetson@ubuntu:~$ docker pull ubuntu
Using default tag: latest # Do not write tag, the default is latest
latest: Pulling from library/ubuntu
cd741b12a7ea: Pull complete # Layered download
Digest: sha256:67211c14fa74f070d27cc59d69a7fa9aeff8e28ea118ef3babc295a0428a6d21
Status: Downloaded newer image for ubuntu:latest
docker.io/library/ubuntu:latest #Real location
# List the images on the local host
jetson@ubuntu:~$ docker images
REPOSITORY TAG IMAGE ID CREATED SIZE
yahboomtechnology/ros-melodic 1.4.1 f8c914ba3cff 7 weeks ago 23.1GB
hello-world latest 46331d942d63 13 months ago 9.14kB
# explain
REPOSITORY mirrored warehouse source
TAG The tag of the image
IMAGE ID ID of the image
CREATED Image creation time
SIZE image size
# The same warehouse source can have multiple TAGs, representing different 
versions of the warehouse source. We use REPOSITORY: TAG to define different 
images. If you do not define the tag version of the image, docker will use the 
lastest image by default!
#optional
-a: List all local images
-q: only display image id
--digests: Display summary information of the image
 
3. docker search search image
 
4. docker rmi deletes the image
 
2.4. Container command  
A container can only be created if you have a mirror. We use the ubuntu mirror to test here.  
Download the mirror:
 
1. docker run runs the image to start the container# Search for images
jetson@ubuntu:~$ docker search ros2
NAME DESCRIPTION STARS OFFICIAL AUTOMATED
osrf/ros2 **Experimental** Docker Images for ROS2 deve… 60 [OK]
tiryoh/ros2-desktop-vnc A Docker image to provide HTML5 VNC interfac… 11
althack/ros2 An assortment of development containers for … 7
tiryoh/ros2 unofficial ROS2 image 6
athackst/ros2 [Deprecated-> use althack/ros2] 5
uobflightlabstarling/starling-mavros2 ROS2 version of MAVROS 2
theosakamg7/ros2_java_docker Image base 1 [OK]
# docker search The name of a certain image corresponds to the image in the 
DockerHub warehouse
#optional
--filter=stars=50: List the images whose collection number is not less than the 
specified value.
# Delete image
docker rmi -f image id # Delete a single
docker rmi -f Image name: tag Image name: tag # Delete multiple
docker rmi -f $(docker images -qa) # Delete all
docker pull ubuntu
# Order
docker run [OPTIONS] IMAGE [COMMAND][ARG...]
# Common parameter descriptions
--name="Name" # Specify a name for the container
-d # Run the container in background mode and return the container id!
-i # Run the container in interactive mode, used with -t
-t #Reassign a terminal to the container, usually used with -i
-P # Random port mapping (uppercase)
-p #Specify port mapping (summary), generally there are four ways to write it
ip:hostPort:containerPort
ip::containerPort
 
2. docker ps lists all running containers
 
3. Exit the container
 
4. Enter the running container from multiple terminalshostPort:containerPort (commonly used)
containerPort
#test
jetson@ubuntu:~$ docker images
REPOSITORY TAG IMAGE ID CREATED SIZE
yahboomtechnology/ros-melodic 1.4.1 f8c914ba3cff 7 weeks ago 23.1GB
ubuntu latest bab8ce5c00ca 6 weeks ago 69.2MB
hello-world latest 46331d942d63 13 months ago 9.14kB
#Use ubuntu to start the container in interactive mode and execute the /bin/bash 
command in the container!
jetson@ubuntu:~$ docker run -it ubuntu:latest /bin/bash
root@c54bf9efae47:/# ls
bin boot dev etc home lib media mnt opt proc root run sbin srv sys tmp usr var
root@c54bf9efae47:/# exit # Use exit to exit the container and return to the host
exit
jetson@ubuntu:~$
# Order
docker ps [OPTIONS]
# Common parameter descriptions
-a # List all currently running containers + historically run containers
-l # Display recently created containers
-n=? # Display the last n created containers
-q # Silent mode, only the container number is displayed.
#test
jetson@ubuntu:~$ docker ps -a
CONTAINER ID IMAGE COMMAND CREATED STATUS PORTS NAMES
c54bf9efae47 ubuntu:latest "/bin/bash" 2 hours ago Up 4 seconds funny_hugle
3b9c01839579 hello-world "/hello" 3 hours ago Exited (0) 3 hours ago jovial_brown
exit # Container stops exiting
ctrl+P+Q # Container exits without stopping
# Command 1
docker exec -it container id bashShell
# test
jetson@ubuntu:~$ docker ps -a
CONTAINER ID IMAGE COMMAND CREATED STATUS PORTS NAMES
c54bf9efae47 ubuntu:latest "/bin/bash" 2 hours ago Up 4 seconds funny_hugle
 
5. Start and stop the container
 
6. Delete the container
 
 
2.5. Other commonly used commands  
1. View the process information running in the container and support ps command parameters.3b9c01839579 hello-world "/hello" 3 hours ago Exited (0) 3 hours ago jovial_brown
jetson@ubuntu:~$ docker exec -it c5 /bin/bash # The container ID can be 
abbreviated, as long as it can uniquely identify the container.
root@c54bf9efae47:/#
# Command 2
docker attach container id
# test
jetson@ubuntu:~$ docker ps -a
CONTAINER ID IMAGE COMMAND CREATED STATUS PORTS NAMES
c54bf9efae47 ubuntu:latest "/bin/bash" 2 hours ago Up 35 seconds funny_hugle
3b9c01839579 hello-world "/hello" 3 hours ago Exited (0) 3 hours ago jovial_brown
jetson@ubuntu:~$ docker attach c5 # The ID of the container can be abbreviated, 
as long as it can uniquely identify the container.
root@c54bf9efae47:/#
# the difference
# exec opens a new terminal in the container and can start a new process
# attach directly enters the terminal of the container startup command and will 
not start a new process.
docker start (container id or container name) # Start the container
docker restart (container id or container name) # Restart the container
docker stop (container id or container name) # Stop the container
docker kill (container id or container name) #Force stop the container
docker rm container id # Delete the specified container
docker rm -f $(docker ps -a -q) # Delete all containers
docker ps -a -q|xargs docker rm # Delete all containers
 
2. View the metadata of the container/image# Order
docker top container id
# test
jetson@ubuntu:~$ docker ps -a
CONTAINER ID IMAGE COMMAND CREATED STATUS PORTS NAMES
c54bf9efae47 ubuntu:latest "/bin/bash" 2 hours ago Up 2 minutes funny_hugle
3b9c01839579 hello-world "/hello" 3 hours ago Exited (0) 3 hours ago jovial_brown
jetson@ubuntu:~$ docker top c5
UID PID PPID C STIME TTY TIME CMD
root 9667 9647 0 14:20 pts/0 00:00:00 /bin/bash
# Order
docker inspect container id
#Test to view container metadata
jetson@ubuntu:~$ docker ps -a
CONTAINER ID IMAGE COMMAND CREATED STATUS PORTS NAMES
c54bf9efae47 ubuntu:latest "/bin/bash" 2 hours ago Up 4 minutes funny_hugle
3b9c01839579 hello-world "/hello" 3 hours ago Exited (0) 3 hours ago jovial_brown
jetson@ubuntu:~$ docker inspect c54bf9efae47
[
     {
     # The complete id, the container id above is the first few digits of the 
intercepted id.
         "Id": 
"c54bf9efae471071391202a8718b346d9af76cb1ff17741e206280603d6f0056",
         "Created": "2023-04-24T04:19:46.232822024Z",
         "Path": "/bin/bash",
         "Args": [],
         "State": {
             "Status": "running",
             "Running": true,
             "Paused": false,
             "Restarting": false,
             "OOMKilled": false,
             "Dead": false,
             "Pid": 9667,
             "ExitCode": 0,
             "Error": "",
             "StartedAt": "2023-04-24T06:20:58.508213216Z",
             "FinishedAt": "2023-04-24T06:19:45.096483592Z"
         },
. . . .
#Test to view image metadata
jetson@ubuntu:~$ docker images
REPOSITORY TAG IMAGE ID CREATED SIZE
ubuntu latest bab8ce5c00ca 6 weeks ago 69.2MB
hello-world latest 46331d942d63 13 months ago 9.14kB
jetson@ubuntu:~$ docker inspect bab8ce5c00ca
[
     {
         "Id": 
"sha256:bab8ce5c00ca3ef91e0d3eb4c6e6d6ec7cffa9574c447fd8d54a8d96e7c1c80e",
         "RepoTags": [
             "ubuntu:latest"
         ],
         "RepoDigests": [
             
"ubuntu@sha256:67211c14fa74f070d27cc59d69a7fa9aeff8e28ea118ef3babc295a0428a6d21"
         ],
         "Parent": "",
         "Comment": "",
         "Created": "2023-03-08T04:32:41.063980445Z",
         "Container": 
"094fd0c521be8c84d81524e4a5e814e88a2839899c56f654484d32d171c7195b",
         "ContainerConfig": {
             "Hostname": "094fd0c521be",
             .............
             "Labels": {
                 "org.opencontainers.image.ref.name": "ubuntu",
                 "org.opencontainers.image.version": "22.04"
             }
         },
         "DockerVersion": "20.10.12",
         "Author": "",
         "Config": {
             "Hostname": "",
             .........
             "Labels": {
                 "org.opencontainers.image.ref.name": "ubuntu",
                 "org.opencontainers.image.version": "22.04"
             }
         },
         "Architecture": "arm64",
         "Variant": "v8",
         "Os": "linux",
         "Size": 69212233,
         "VirtualSize": 69212233,
         "GraphDriver": {
             "Data": {
                 "MergedDir": 
"/var/lib/docker/overlay2/8418b919a02d38a64ab86060969b37b435977e9bbdeb6b0840d4eb6
98280e796/merged",
                 "UpperDir": 
"/var/lib/docker/overlay2/8418b919a02d38a64ab86060969b37b435977e9bbdeb6b0840d4eb6
98280e796/diff",
                 "WorkDir": 
"/var/lib/docker/overlay2/8418b919a02d38a64ab86060969b37b435977e9bbdeb6b0840d4eb6
98280e796/work"
             },
             "Name": "overlay2"
         },
         "RootFS": {
             "Type": "layers",
 
 
2.6. Command summary  
             "Layers": [
                 
"sha256:874b048c963ab55b06939c39d59303fb975d323822a4ea48a02ac8dc635ea371"
             ]
         },
         "Metadata": {
             "LastTagTime": "0001-01-01T00:00:00Z"
         }
     }
]

---

## 2. Astra camera calibration.pdf

2. Astra camera calibration  
2. Astra camera calibration
2.1. Preparation before calibration
2.2. Astra calibration
2.2.1. Color icon definition
2.2.2, ir infrared calibration
2.3. Single target setting
Wiki: http://wiki.ros.org/camera_calibration
Official website link: https://orbbec3d.com/develop/
Astra camera: https://github.com/orbbec/ros_astra_camera
Developer community: https://developer.orbbec.com.cn/download.html?id=53
Due to some internal and external reasons of the camera, the image will be greatly distorted,  
mainly radial deformation and tangential deformation, causing the straight line to become curved.  
The farther the pixel is from the center of the image, the more serious the distortion will be. In  
order to avoid errors caused by data sources, the parameters of the camera need to be calibrated.  
Calibration essentially uses a known and determined spatial relationship (calibration plate) to  
reversely deduce the inherent and real parameters of the camera (internal parameters) by  
analyzing the pixels of the photographed pictures.
Disadvantages of infrared depth camera ranging:
(1) It is impossible to accurately measure the distance of black objects because black substances  
can absorb infrared rays and the infrared rays cannot return, so the distance cannot be measured.
(2) It is impossible to accurately measure the distance of specular objects, because only when the  
depth camera is on the center vertical line of the specular object, the receiver can receive the  
reflected infrared rays, which will lead to overexposure.
(3) It is impossible to accurately measure the distance of transparent objects because infrared rays  
can pass through transparent objects.
(4) Unable to accurately measure distances for objects that are too close. Principle briefly
2.1. Preparation before calibration  
A large [checkerboard] of known dimensions( http://wiki.ros.org/camera_calibration/Tutorials/
MonocularCalibration?action=AttachFile&do=view&target=check-108.pdf ). This tutorial uses a  
9x6 checkerboard and a 20mm square, which needs to be flattened during calibration.
The calibration uses the internal vertices of the checkerboard, so a "10x7"  
checkerboard uses the internal vertex parameters "9x6", as shown in the example  
below. 
Any calibration board can be used, as long as the parameters are changed.
An open area without obstacles and calibration board patterns
Monocular camera for publishing images via ROS
Checkerboard (calibration board)
Launch file Launch camera model
astra.launch Astra, Astra S, Astra mini, Astra mini S
astraproplus.launch Astra plus/Astraproplus
astrapro.launch Astra pro
embedded_s.launch Deeyea
dabai_u3.launch Dabai
gemini.launch Gemini
Obi mid-range camera model and corresponding launch file
Device view
Depth camera ID: [2bc5:0403]lsusb
Color camera ID: [2bc5:0501]
The appearance of these two IDs indicates that the device is connected.
2.2. Astra calibration  
Start the camera before calibration, and then turn off the camera until all calibrations are  
completed.
Launch Astra Camera
This startup command includes an IR image conversion node. The conversion is because the IR  
infrared camera views a 16-bit image during calibration, and the picture cannot be clearly seen.  
The 16-bit needs to be normalized into a value range of 0-255. 8-bit picture so that you can see it  
clearly.
View Image Topics
<PI5 needs to open another terminal and enter the same docker container#Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
roslaunch yahboomcar_visual astra_calibration.launch
rostopic list
2.2.1. Color icon definition  
Start calibration node
size: Calibrate the number of internal corner points of the checkerboard, for example, 9X6, with a  
total of six rows and nine columns of corner points.
square: The side length of the checkerboard, in meters.
image and camera: Set the image topic published by the camera.
rosrun camera_calibration cameracalibrator.py image:=/camera/rgb/image_raw 
camera:=/camera/rgb --size 9x6 --square 0.02
Calibration interface
X: The left and right movement of the checkerboard in the camera field of view
Y: The checkerboard moves up and down in the camera field of view
Size: the movement of the checkerboard back and forth in the camera field of view
Skew: The tilt and rotation of the checkerboard in the camera's field of view
After successful startup, place the checkerboard in the center of the screen and change to  
different positions. The system will identify it independently. The best situation is that the lines  
under [X], [Y], [Size], and [Skew] will first change from red to yellow and then to green as the data  
is collected, filling them as fully as possible.
Click [CALIBRATE] to calculate the internal parameters of the camera. The more pictures you  
have, the longer it will take. Just wait. (Sixty or seventy is enough, too many can easily get  
stuck).
Click [SAVE] to save the calibration results to [/tmp/calibrationdata.tar.gz].
Click [COMMIT] to write the calibration file into the [.ros/camera_info/rgb_camera.yaml] file.  
The next time you start the camera, the calibration results will be automatically read.
After the calibration is completed, you can move out the [/tmp/calibrationdata.tar.gz] file to see  
the content.
After decompression, there are the image just calibrated, an ost.txt file and an ost.yaml file.
2.2.2, ir infrared calibration  
After the data normalization problem is dealt with, another problem will arise. Because the RGBD  
camera, which uses structured light as the depth imaging principle, the infrared light projected by  
it is a special disordered spot, causing the infrared receiving device to be unable to receive clear  
and complete images. screen content.
At this time we can have several special processing methods:
Forcibly find various angles and let the camera find the corners as much as possible (poor  
accuracy)
Spread the infrared light spots evenly by pasting some frosted translucent paper in front of  
the red hair emitter (moderate accuracy, more convenient)
Block the infrared projection camera and use an external infrared camera to fill in the light  
(high accuracy, additional equipment is required)
Choose the processing method according to your needs.
Start calibration nodesudo mv /tmp/calibrationdata.tar.gz ~
rosrun camera_calibration cameracalibrator.py image:=/camera/ir/image_mono8 --
size 9x6 --square 0.02
size: Calibrate the number of internal corner points of the checkerboard, for example, 9X6, with a  
total of six rows and nine columns of corner points.
square: The side length of the checkerboard, in meters.
image and camera: Set the image topic published by the camera.
The following operations are similar to color camera calibration, changing different poses. The  
system will identify it independently. The best situation is that the lines under [X], [Y], [Size], and  
[Skew] will first change from red to yellow and then to green as the data is collected, filling them  
as fully as possible.
Click [CALIBRATE] to calculate the internal parameters of the camera. The more pictures you  
have, the longer it will take. Just wait. (Sixty or seventy is enough, too many can easily get  
stuck).
Click [SAVE] to save the calibration results to [/tmp/calibrationdata.tar.gz].
Click [COMMIT] to write the calibration file into the [.ros/camera_info/ir_camera.yaml] file.  
The next time you start the camera, the calibration results will be automatically read.
After the calibration is completed, you can move out the [/tmp/calibrationdata.tar.gz] file to see  
the content.
After decompression, there are the image just calibrated, an ost.txt file and an ost.yaml file.sudo mv /tmp/calibrationdata.tar.gz ~
2.3. Single target setting  
The principle of setting the color map in section [2.2.1] is the same, except that the startup  
command and topic name are different. This section is suitable for monocular color image  
calibration.
Start monocular camera
Start calibration node
The single-purpose calibration results are stored in the file [.ros/camera_info/head_camera.yaml].roslaunch usb_cam usb_cam-test.launch
rosrun camera_calibration cameracalibrator.py image:=/usb_cam/image_raw 
camera:=/usb_cam --size 9x6 --square 0.02

---

## 2. Multi-machine navigation.pdf

2. Multi-machine navigation  
2. Multi-machine navigation
2.1. Introduction
2.2. Use
1.2.1. Start the robot
1.2.2. Enable multi-machine navigation
1.2.3. Set commu nication topic
1.2.4. Set initial pose
2.3, launch file
2.4. Framework analysis
2.1. Introduction  
Regarding how to configure multi-machine communication and synchronization time, please refer  
to the lesson [Multi-machine joystick control] for details; if there is a network, the network system  
time can be synchronized directly without setting.
When using multi-machine handle control, you first need to ensure that the robots are under the  
same LAN and configured with the same [ROS_MASTER_URI]; multiple robots can only have one  
host to control their movements. In the case of this section, the virtual machine is set as the host  
machine, and other robots are slave machines. There are several slave machines. Of course, you  
can also set a certain robot as the master machine and the others as slave machines.
According to different models, you only need to set the purchased model in [.bashrc], X1 (normal  
four-wheel drive) X3 (Mailun) Take X3 as an example
Open the [.bashrc] file
Find the [ROBOT_TYPE] parameters and modify the corresponding car model
2.2. Use  
Take the virtual machine as the host and the three robots as slaves as an example; a map must be  
available before use.
1.2.1. Start the robot  
Virtual machine side#Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
sudo vim .bashrc
export ROBOT_TYPE=X3 # ROBOT_TYPE: X1 X3 X3plus R2 X7
roscore
Start the command (robot1 side). For the convenience of operation, this section takes [mono +  
laser + yahboomcar] as an example.
Start the command (robot2 side). For the convenience of operation, this section takes [mono +  
laser + yahboomcar] as an example.
More robots and so on.
1.2.2. Enable multi-machine navigation  
For the handle control opening process, please refer to the lesson [Multi-machine handle control].
Virtual machine side
[use_rviz] parameter: whether to open rviz.
[map] parameters: map name, map to be loaded.
After turning on the multi-machine navigation, there is a dense mess. The pose of each robot is  
incorrect. First, the initial pose must be set for each robot.roslaunch yahboomcar_multi laser_bringup_multi.launch ns:=robot1 # laser + 
yahboomcar
roslaunch yahboomcar_multi laser_usb_bringup_multi.launch ns:=robot1 # mono + 
laser + yahboomcar
roslaunch yahboomcar_multi laser_astrapro_bringup_multi.launch ns:=robot1 # Astra 
+ laser + yahboomcar
roslaunch yahboomcar_multi laser_bringup_multi.launch ns:=robot2 # laser + 
yahboomcar
roslaunch yahboomcar_multi laser_usb_bringup_multi.launch ns:=robot2 # mono + 
laser + yahboomcar
roslaunch yahboomcar_multi laser_astrapro_bringup_multi.launch ns:=robot2 # Astra 
+ laser + yahboomcar
roslaunch yahboomcar_multi yahboomcar_nav_multi.launch use_rviz:=true map:=my_map
Tip: Start if it prompts my_map error
The terminal content path of the .yaml file should be changed correctly
  
1.2.3. Set communication topic  
Right-click on the toolbar (green frame) and a dialog box (red frame) will pop up, as shown in the  
picture above. Right-click on [2D Pose Estimate], and then select [Tool Properties], as shown  
below, a dialog box will pop up.
From top to bottom [2D Pose Estimate] and [2D Nav Goal] correspond to the [rviz] toolbar from  
left to right. Set the topic name pointed by the red arrow. This method is quick and convenient.
What should I do if there are 4 or more robots and the initial pose and navigation icons are not  
enough?
At this time, you need to open the [navigation_multi.rviz] file (open whichever rviz you use, take  
[navigation_multi.rviz] as an example) and find the following content. These contents correspond  
to the icons in the rviz toolbar one-to-one. If you want to add an icon, copy the entire [Class]  
content of that icon. You must not miss anything, and the format must be correct. 
Tools:
     - Class: rviz/MoveCamera
     - Class: rviz/Interact
       Hide Inactive Objects: true
     - Class: rviz/Select
     - Class: rviz/SetInitialPose
       Theta std deviation: 0.2617993950843811
       Topic: robot1/initialpose
       X std deviation: 0.5
       Y std deviation: 0.5
     - Class: rviz/SetGoal
       Topic: robot1/move_base_simple/goal
     - Class: rviz/SetInitialPose
       Theta std deviation: 0.2617993950843811
       Topic: robot2/initialpose
       X std deviation: 0.5
       Y std deviation: 0.5
     - Class: rviz/SetGoal
       Topic: robot2/move_base_simple/goal
     - Class: rviz/SetInitialPose
       Theta std deviation: 0.2617993950843811
       Topic: robot3/initialpose
       X std deviation: 0.5
       Y std deviation: 0.5
     - Class: rviz/SetGoal
       Topic: robot3/move_base_simple/goal
1.2.4. Set initial pose  
There are too many robots and the information on the map is very complicated. In this case, you  
can uncheck the check mark behind the robot in the [Displays] display item list on the left and set  
the pose of each robot in turn. The effect after setting is as follows. Once set up, you can navigate.
Use the [2D Pose Estimate] of the [rviz] tool to set the initial pose until the position of the car  
in the simulation is consistent with the position of the actual car.
Click [2D Nav Goal] of the [rviz] tool, and then select a target point on the map where there  
are no obstacles. Release the mouse to start navigation. Only one target point can be  
selected, and it will stop when it is reached.
2.3, launch file       - Class: rviz/Measure
<launch>
     <arg name="first_robot1" default="robot1"/>
     <arg name="second_robot2" default="robot2"/>
     <arg name="third_robot3" default="robot3"/>
     <!-- Whether to open rviz || Whether to open rviz -->
     <arg name="use_rviz" default="true"/>
     <!-- Map name || Map name -->
     <arg name="map" default="my_map"/>
     <!-- Load map || Load map -->
     <node name="map_server" pkg="map_server" type="map_server" args="$(find 
yahboomcar_nav)/maps/$(arg map).yaml"/>
     <node pkg="rviz" type="rviz" name="rviz" required="true"
           args="-d $(find yahboomcar_multi)/rviz/navigation_multi.rviz" 
if="$(arg use_rviz)"/>
     <!-- Multi machine handle control || Multi machine handle control-->
     <include file="$(find yahboomcar_multi)/launch/joy_multi.launch"/>
     <!-- Mobile app node || Mobile app node-->
     <include file="$(find yahboomcar_nav)/launch/library/app.launch"/>
You can modify it according to your own needs. If there are 4 or more robots, just follow the case  
of the first 3 robots and add relevant content.
2.4. Framework analysis  
Node view     <!-- ############################ first_robot1 ################ 
############# -->
     <include file="$(find 
yahboomcar_multi)/launch/library/move_base_multi.launch">
         <arg name="ns" value="$(arg first_robot1)"/>
     </include>
     <!-- ############################ second_robot2 ################ 
############# -->
     <include file="$(find 
yahboomcar_multi)/launch/library/move_base_multi.launch">
         <arg name="ns" value="$(arg second_robot2)"/>
     </include>
     <!-- ############################ third_robot3 ################ 
############# -->
     <include file="$(find 
yahboomcar_multi)/launch/library/move_base_multi.launch">
         <arg name="ns" value="$(arg third_robot3)"/>
     </include>
</launch>
rqt_graph
There are a lot of node diagrams, which look complicated, but are actually very organized. The  
internal nodes for each robot are almost the same, and the navigation of a single robot is also the  
same. It should be noted here that the map only needs to be loaded once, and does not need to  
be loaded for every robot. At the same time, I also enabled the node controlled by the handle. One  
handle controls multiple robots at the same time, and only needs to start the [joy_node] node  
once.
View tf tree
There are also a lot of tf coordinate systems, so you need to zoom in to see them. There is only  
one [map] globally, and each [amcl] locates each robot separately. From then on, it is the same as  
single robot navigation.rosrun rqt_tf_tree rqt_tf_tree

---

## 2. Voice Control Module Port Binding.pdf

2. Voice Control Module Port Binding  
Note: The voice interaction module is pre-wired at the factory and can be used directly without  
repeating the steps in this tutorial. The following binding steps are for demonstration purposes  
only and are for reference only.
Enter the following command in the terminal to check if the device is connected.
lsusb
 Then, we modify the /etc/udev/rules.d/usb.rules file and enter it in the terminal.
 Add the following content:
Save and exit. Enter the following command in the terminal to reload the system devices.
You can then query the port number bound to each device.sudo gedit /etc/udev/rules.d/usb.rules
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7522", 
MODE:="0777", SYMLINK+="myspeech"
sudo udevadm trigger
sudo service udev reload
sudo service udev restart
2.3. Testing  
Finally, the physical connection diagram of the bound ports is shown below.
After successfully connecting to the voice module. "Speech Serial Opened!  
Baudrate=115200" will be displayed on the terminal.
After saying "yahboom,yahboom" to the module, the voice will respond "Yes, I am."
After unplugging the voice control module, the program will exit with an error.
If the actual running results are consistent with the above three points, the device binding is  
successful.python3 voice_ctrl_test.py

---

## 2. Voice control module port binding_1.pdf

2. Voice control module port binding  
Foreword: Because the ID device numbers of the HUB board and the voice control module are the  
same, the ID device numbers cannot be bound to them according to the method in the previous  
tutorial. Not binding ports may cause port conflicts or device identification errors, and the  
bound ports cannot be changed at will , otherwise the binding will be invalid. This section uses  
Raspberry Pi as an example to demonstrate.
2.1. Bind HUB board device number  
First, without connecting to the voice control board, the radar and PCB boards available are  
ttyUSB0 and ttyUSB1
Then, we first check the port information of the HUB board, mainly to check the device path  
information and enter the terminal
 Get the following information, the red box shows the path information of the device,
 
Then, we modify the /etc/udev/rules.d/usb.rules file, first bind the port number of the HUB  
board, and enter it in the terminal.ll /dev/ttyUSB*
ll /dev/myserial
ll /dev/rplidar
udevadm info --attribute-walk --name=/dev/ttyUSB1 |grep devpath
Find the myseial column in the file, as shown in the picture below, and add the content.
 
Exit after saving, enter the following three commands in the terminal to reload the device
2.2. Bind the speech recognition board port number  
Enter the following command in the terminal to view the device number,
 Here we find that the system recognizes the voice board as /dev/ttyUSB2, then we enter the  
following command to view the device path information
 Get the following picture,
 
Then, we modify the /etc/udev/rules.d/myspeech.rules file, bind the port number of the voice  
board, and enter it in the terminal,
Add content as shown below,sudo gedit /etc/udev/rules.d/usb.rules
ATTRS{devpath}=="1.4"
sudo udevadm trigger
sudo service udev reload
sudo service udev restart
ll /dev/ttyUSB*
udevadm info --attribute-walk --name=/dev/ttyUSB2 |grep devpath
sudo gedit /etc/udev/rules.d/myspeech.rules
Exit after saving, enter the following statement in the terminal to reload the system device
2.3. Testing  
Finally, the physical connection diagram of the port after binding is shown in the figure below,
Note where they are wired. This is fixed after binding. Do not change the position at will  
later, otherwise the system will not be able to recognize the device. 
After successfully connecting the voice module. "Speech Serial Opened! Baudrate=115200"  
will be displayed on the terminal.
After saying "Hello, Xiaoya" to the module, the voice reply is "Yes".
Unplugging the voice control module will cause the program to exit with an error.
If the actual running result is consistent with the above three points, it means the device is  
successfully bound.
Note: The bound HUB and voice board cannot be plugged into other ports, otherwise the  
device number will not be recognized. KERNEL=="ttyUSB*",ATTRS{devpath}=="1.3",ATTRS{idVendor}=="1a86",ATTRS{idProduct}=
="7523",MODE:="0777",SYMLINK+="myspeech"
sudo udevadm trigger
sudo service udev reload
sudo service udev restart
python3 voice_ctrl_test.py

---

## 3、Docker images deeply understand and publish images.pdf

3. In-depth understanding of docker  
images and publishing images 
 
3. In-depth understanding of docker images and publishing images
3.1. Understanding of mirroring
3.2, UnionFS (Union File System)
3.3. Mirror layering
3.3.1. Hierarchical understanding
3.3.2. The benefits of layering docker images
3.4. Making and publishing images
3.4.1. Make an image
3.4.2. Release image
The operating environment and software and hardware reference configuration are as follows:
Reference model: ROSMASTER X3
Robot hardware configuration: Arm series main control, Silan A1 lidar, AstraPro Plus depth  
camera
Robot system: Ubuntu (no version required) + docker (version 20.10.21 and above)
PC virtual machine: Ubuntu (18.04) + ROS (Melodic)
Usage scenario: Use on a relatively clean 2D plane
 
3.1. Understanding of mirroring  
1. An image is a lightweight, executable independent software package that contains everything  
needed to run a certain software. We package the application and configuration into a  
formed, deliverable, and deployable operating environment, including code, libraries  
required for runtime, environment variables, and configuration files, etc. This large packaged  
operating environment is an image image file.
2. Docker container instances can only be generated through image files.
 
3.2, UnionFS (Union File System)  
1. Union File System (UnionFS) is a layered, lightweight, high-performance file system. It is the  
basis of docker images and supports modifications to the file system to be superimposed  
layer by layer as a single submission. At the same time, different directories can be mounted  
under the same virtual file system.
2. Images can be inherited through layering. Based on the basic image, various specific  
application images can be produced.
Features of the Union file system: multiple file systems are loaded at the same time, but from the  
outside, only one file system can be seen; Union loading will superimpose the file systems of each  
layer, so that the final file system will contain all layers. Files and directories.
 
3.3. Mirror layering  
When downloading an image, pay attention to the download log output. You can see that it is  
downloading layer by layer:
# To view the image layering method, you can use the command: docker image 
inspect image name
jetson@ubuntu:~$ docker image inspect mysql:latest
[
     {
         "Id": 
"sha256:5371f8c3b63eec64a33b35530be5212d6148e0940111b57b689b5ba1ffe808c8",
         .........
         "RootFS": {
             "Type": "layers",
             "Layers": [
                 
"sha256:d6d4fc6aef875958d6186f85f03d88e6bb6484ab2dd56b30a79163baceff2f6d",
                 
"sha256:05c3b0b311a02bc56ca23105a76d16bc9b8c1d3e6eac808f4efb1a2e8350224b",
                 
"sha256:7b80f7f05642477ebc7d93de9539af27caab7c41a768db250fe3fe2b5506ca2c",
                 
"sha256:50e037faefab22cb1c75e60abb388b823e96a845650f3abd6d0a27e07a5a1d5e",
                 
"sha256:66040abb3f7201d2cc64531349a8225412db1029447a9431d59d999c941d56f6",
                 
"sha256:857162425652837a362aa5f1c3d4974cc83702728793de52ba48176d5367a89b",
                 
"sha256:7eebed3016f6b6ab68aa8e6be35f0689a3c18d331b7b542984a0050b859eaf26",
                 
"sha256:2fc4c142633d57d795edc0f3fd457f99a35fa611eab8b8c5d75c66e6eb729bc2",
                 
"sha256:7fde2d12d484f0c14dabd9ca845da0bcdaf60bd773a58ca2d73687473950e7fe",
                 
"sha256:9319848a00d38e15b754fa9dcd3b6e77ac8506850d32d8af493283131b9745a3",
                 
"sha256:5ff94d41f068ea5b52244393771471edb6a9a10f7a4ebafda9ef6629874a899b"
 
3.3.1. Hierarchical understanding  
All docker images start from a basic image layer. When modifications are made or new  
content is added, a new image layer will be created on top of the current image layer.
To give a simple example, if a new image is created based on Ubuntu 20.04, this is the first  
layer of the new image; if a python package is added to the image, a second image layer will  
be created on top of the basic image layer; If you continue to add a security patch, a third  
image layer will be created.
Docker images are read-only, when the container starts, a new writable layer is loaded on top  
of the image! This layer is what we usually call the container layer, and everything below the  
container is called the mirror layer!
 
3.3.2. The benefits of layering docker images  
Resource sharing, for example, if multiple images are built from the same Base image, then the  
host only needs to keep one base image on the disk, and only one base image needs to be loaded  
in the memory, so that all Containers serve, and each layer of the image can be shared.
 
 
3.4. Making and publishing images  
 
3.4.1. Make an image  
Method 1. Submit an image from the container:             ]
         },
         "Metadata": {
             "LastTagTime": "0001-01-01T00:00:00Z"
         }
     }
]
# Order
docker commit -m="Description information submitted" -a="Author" Container id 
Target image name to be created: [Tag name] [The -m -a parameter can also be 
omitted]
# test
jetson@ubuntu:~$ docker ps -a
CONTAINER ID IMAGE COMMAND CREATED STATUS PORTS NAMES
c54bf9efae47 ubuntu:latest "/bin/bash" 3 hours ago Up 24 minutes funny_hugle
3b9c01839579 hello-world "/hello" 3 hours ago Exited (0) 3 hours ago jovial_brown
jetson@ubuntu:~$ docker commit c54bf9efae47 ubuntu:1.0
sha256:78ca7be949b6412f74ba12e8d16bd548aaa7c3fa25134326db3a67784f848f8f
Method 2. Create image using dockerfile:
 
3.4.2. Release image  
The docker repository is a centralized place for storing image files. The largest public repository is  
docker hub ( https://hub.docker.com/ ), which stores a large number of images for users to  
download. Domestic public warehouses include Alibaba Cloud, NetEase Cloud, etc.
 
Steps to publish the image to docker hub:
1. Address: https://hub.docker.com/ , register an account first
2. Ensure that the account can be logged in normally
3. Use the tag command to modify the image name.
The specifications for publishing images to docker hub are:
For example, my registered user name here is: pengan88, then I need to change the image name  
first.jetson@ubuntu:~$ docker images # Generated ubuntu:1.0 image
REPOSITORY TAG IMAGE ID CREATED SIZE
ubuntu 1.0 78ca7be949b6 5 seconds ago 69.2MB
yahboomtechnology/ros-foxy 3.4.0 49581aa78b6b 5 hours ago 24.3GB
yahboomtechnology/ros-foxy 3.3.9 cefb5ac2ca02 4 days ago 20.5GB
yahboomtechnology/ros-foxy 3.3.8 49996806c64a 4 days ago 20.5GB
yahboomtechnology/ros-foxy 3.3.7 8989b8860d17 5 days ago 17.1GB
yahboomtechnology/ros-foxy 3.3.6 326531363d6e 5 days ago 16.1GB
ubuntu latest bab8ce5c00ca 6 weeks ago 69.2MB
hello-world latest 46331d942d63 13 months ago 9.14kB
# Order
docker build -f dockerfile file path -t new image name: TAG . # There is a . at 
the end of the docker build command indicating the current directory
# test
docker build -f dockerfile-ros2 -t yahboomtechnology/ros-foxy:1.2 .
For information on writing dockerfile, please refer to: 
https://docs.docker.com/develop/develop-images/dockerfile_best-practices/
docker push registered user name/image name
# Order:
docker tag image ID modified image name
# test
jetson@ubuntu:~$ docker images
4 、登录 docker hub 发布镜像：
5 、访问 docker hub 可查看到已经发布成功
REPOSITORY TAG IMAGE ID CREATED SIZE
ubuntu 1.0 78ca7be949b6 5 seconds ago 69.2MB
ubuntu latest bab8ce5c00ca 6 weeks ago 69.2MB
hello-world latest 46331d942d63 13 months ago 9.14kB
jetson@ubuntu:~$ docker tag 78ca7be949b6pengan88/ubuntu:1.0
jetson@ubuntu:~$ docker images
REPOSITORY                   TAG       IMAGE ID       CREATED          SIZE
pengan88/ubuntu              1.0       78ca7be949b6   23 minutes ago   69.2MB
ubuntu                       1.0       78ca7be949b6   23 minutes ago   69.2MB
ubuntu                       latest    bab8ce5c00ca   6 weeks ago      69.2MB
hello-world                  latest    46331d942d63   13 months ago    9.14kB
jetson@ubuntu:~$ docker login -u pengan88
Password:     # 这里输入 docker hub 注册的账号密码
WARNING! Your password will be stored unencrypted in 
/home/jetson/.docker/config.json.
Configure a credential helper to remove this warning. See
https://docs.docker.com/engine/reference/commandline/login/#credentials-store
Login Succeeded
jetson@ubuntu:~$ docker push pengan88/ubuntu:1.0
The push refers to repository [docker.io/pengan88/ubuntu]
ca774712d11b: Pushed
874b048c963a: Mounted from library/ubuntu
1.0: digest: 
sha256:6767d7949e1c2c2adffbc5d3c232499435b95080a25884657fae366ccb71394d size: 736

---

## 3. Astra Color Tracking.pdf

3. Astra color tracking  
3. Astra color tracking
3.1. Introduction
3.1.1, HSV introduction
3.1.2, HSV hexagonal pyramid
3.2. Ope ration steps
3.2.1. Start
3.2.2. Identification
3.2.3. Color calibration
3.3. Program analysis
3.1. Introduction  
The Astra color tracking of the Yahboom mobile robot can recognize multiple colors at any time,  
and independently stores the currently recognized color, controls the car to follow the detected  
and recognized color, and maintains a certain distance from the object.
The color tracking of the Yahboom mobile robot can also realize the function of real-time control  
of HSV. By adjusting the high and low thresholds of HSV, the interfering colors are filtered out, so  
that the squares can be ideally identified in complex environments. If the color selection effect is  
not good, Ideally, at this time, the car needs to be moved to different environments and calibrated  
so that it can recognize the colors we need in complex environments.
3.1.1, HSV introduction  
HSV (Hue, Saturation, Value) is a color space created by A. R. Smith in 1978 based on the intuitive  
characteristics of color, also known as the Hexcone Model.
The parameters of color in this model are: hue (H), saturation (S), and lightness (V).
H: 0 — 180
S: 0 — 255
V: 0 — 255
Here some reds are classified into the purple range:
3.1.2, HSV hexagonal pyramid  
Hue H
Represents color information, that is, the position of the spectral color. This parameter is  
represented by an angle, with a value ranging from 0° to 360°, starting from red and counting in  
counterclockwise direction. Red is 0°, green is 120°, and blue is 240°. Their complementary colors  
are: yellow is 60°, cyan is 180°, and purple is 300°.
Saturation S
Saturation S is expressed as the ratio between the purity of the selected color and the maximum  
purity of that color. When S=0, there is only grayscale. 120 degrees apart. Complimentary colors  
are 180 degrees apart. A color can be thought of as the result of mixing a certain spectral color  
with white. The greater the proportion of spectral colors, the closer the color is to spectral colors,  
and the higher the saturation of the color. The saturation is high and the color is deep and vivid.  
The white light component of the spectral color is 0, and the saturation reaches the highest level.  
Usually the value range is 0% ~ 100%. The larger the value, the more saturated the color.
LightnessV
Brightness represents the brightness of a color. For light source color, the brightness value is  
related to the brightness of the luminous body; for object color, this value is related to the  
transmittance or reflectance of the object. Usually the value range is 0% (black) to 100% (white).  
One thing to note: there is no direct relationship between it and light intensity.
The three-dimensional representation of the HSV model evolves from the RGB cube. If you  
imagine looking from the white vertices of the RGB along the diagonal of the cube to the black  
vertices, you can see the hexagonal shape of the cube. The hexagonal borders represent color, the  
horizontal axis represents purity, and lightness is measured along the vertical axis.
3.2. Operation steps  
Note: [R2] on the remote control handle has the [pause/start] function for all gameplays. 
3.2.1. Start  
Note: When the image is displayed, press the [q] key to exit. 
Two startup methods, choose one, the demonstration case is method 2
method one#Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
robot side
Method 2
Can be controlled remotely for easy operation.
robot side
virtual machine
VideoSwitch parameter: whether to use the camera function package to start.
Set parameters according to needs, or modify the launch file directly, so there is no need to attach  
parameters when starting.
3.2.2. Identification  
After startup, the system defaults to [Target Detection Mode], as shown below:
Keyboard key control:
[r]: Color selection mode, you can use the mouse to select the area of the color to be recognized  
(cannot exceed the area range).
【i 】 : Target detection mode. The color image on the left (Color) and the binary image on the right  
(Binary).
[q]: Exit the program.
[Spacebar]: After identifying that there is no problem, click [Spacebar] on the keyboard to execute  
the color following program.
In the color selection mode, use the mouse to select the location of the colored object, as shown  
in the figure below, and release it to start recognition.roslaunch yahboomcar_astra colorTracker.launch VideoSwitch:=true
roslaunch yahboomcar_astra colorTracker.launch VideoSwitch:=false
roslaunch yahboomcar_astra colorHSV.launch
3.2.3. Color calibration  
<PI5 needs to open another terminal to enter the same docker container
Dynamic parameter tuning
rosrun rqt_reconfigure rqt_reconfigure
Select the [color_HSV] node and [color_Tracker] node. Generally, you only need to adjust [Hmin],  
[Smin], [Vmin], and [Hmax]. These four parameters can be easily identified. The slide bar is always  
in a dragging state and data will not be transferred to the system until it is released; you can also  
select a row and then slide the mouse wheel.
Parameter analysis:
[linear_Kp], [linear_Ki], [linear_Kd]: PID control of linear speed during car following.
[angular_Kp], [angular_Ki], [angular_Kd]: PID control of angular velocity during car following.
[minDist]: Follow the distance and keep this distance.
[scale]: PID scaling.
Parameter modification
When the parameters are adjusted to the optimal state, the corresponding parameters are  
modified into the file, and no adjustment is required when using again.
According to the optimal parameters of the [rqt_reconfigure] debugging tool, enter the [scripts]  
folder of the [yahboomcar_astra] function package and modify the parameters corresponding to  
the [colorTracker.py] file, as shown below
class color_Tracker:
     def __init__(self):
         rospy.on_shutdown(self.cleanup)
         self.bridge = CvBridge()
         self.minDist = 1.0
         ... ...
         self.linear_PID = (3.0, 0.0, 1.0)
         self.angular_PID = (0.5, 0.0, 2.0)
         self.scale = 1000
         self.PID_init()
Parameters Analysis Corresponding parameters
name name of the parameter "linear_Kp"
type parameter data type double_t
level a bitmask passed to the callback 0
description A description parameter "Kp in PID"
default Initial value for node startup 3.0
min parameter minimum value 0
max parameter maximum value 10.0[rqt_reconfigure] Modification of the initial value of the debugging tool
Enter the [cfg] folder of the [yahboomcar_astra] function package and modify the initial values of  
the parameters corresponding to the [ColorTrackerPID.cfg] file.
Take the above article as an example to analyze
Note: After modification, you must recompile and update the environment to be effective. 
3.3. Program analysis  
colorTracker.launch filegen = ParameterGenerator()
gen.add("linear_Kp", double_t, 0, "Kp in PID", 3.0, 0, 10.0)
gen.add("linear_Ki", double_t, 0, "Ki in PID", 0.0, 0, 10.0)
gen.add("linear_Kd", double_t, 0, "Kd in PID", 1.0, 0, 10.0)
gen.add("angular_Kp", double_t, 0, "Kp in PID", 0.5, 0, 10.0)
gen.add("angular_Ki", double_t, 0, "Ki in PID", 0.0, 0, 10.0)
gen.add("angular_Kd", double_t, 0, "Kd in PID", 2.0, 0, 10.0)
gen.add("minDist", double_t, 0, "minDist", 1.0, 0, 10)
gen.add("scale", int_t, 0, "scale", 1000, 0, 10000)
exit(gen.generate(PACKAGE, "colorTracker", "ColorTrackerPID"))
gen.add("linear_Kp", double_t, 0, "Kp in PID", 3.0, 0, 10.0)
cd ~/yahboomcar_ws
catkin_make
source devel/setup.bash
<launch>
     <arg name="VideoSwitch" default="false"/>
     <!-- Handle control node Handle control node-->
     <include file="$(find yahboomcar_ctrl)/launch/yahboom_joy.launch"/>
     <!-- depth camera node astra depth node-->
     <include file="$(find astra_camera)/launch/astra.launch"/>
     <!-- robot drive node-->
     <include file="$(find yahboomcar_bringup)/launch/yahboomcar.launch"/>
launch file analysis
If the [VideoSwitch] parameter is true, the monocular color camera will not be started and the  
[colorHSV] node will be started. The method of obtaining color images is directly implemented by  
the [colorHSV] node using [video*].
If the [VideoSwitch] parameter is false, the monocular color camera is started and the [colorHSV]  
node is not started. Support web page monitoring. At this time, the [colorHSV] node needs to be  
started separately.
Node view
 
Node 【 color_HSV 】     <!-- usb_cam color node-->
     <include file="$(find usb_cam)/launch/usb_cam-test.launch" unless="$(arg 
VideoSwitch)"/>
     <node pkg="yahboomcar_astra" type="colorTracker.py" name="colorTracker" 
required="true" output="screen"/>
     <node pkg="yahboomcar_astra" type="colorHSV.py" name="colorHSV" 
required="true" output="screen" if="$(arg VideoSwitch)">
         <param name="VideoSwitch" type="bool" value="$(arg VideoSwitch)"/>
     </node>
</launch>
rqt_graph
Subscribe to color images
Publish the location of the recognized object in the image
Issue control instructions (only stop commands are issued when no color is recognized in the  
picture)
Node 【 color_Tracker 】
Subscribe to depth images
Subscribe to handle control information
Subscribe to the location information of the recognized object in the image
Issue car follow control instructions

---

## 3. Multi-machine formation.pdf

3 Multi-machine formation  
3 Multi-machine formation 
3.1 Introduction 
3.2 Use 
3.2.1 Start the robot 
3.2.2 Ope n multi-machine formation 
3.2.3 Formation control 
3.3 launch file 
3.4 frame analysis 
3.1 Introduction  
For the problem of how to configure multi-machine communication and synchronization time,  
please refer to the lesson [Multi-machine handle control] for details; if there is a network, the  
network system time can be directly synchronized without setting.  
When using multi-machine handle control, it is first necessary to ensure that the robot is under  
the same local area network and configured with the same [ROS_MASTER_URI]; for multiple  
robots to control motion, there can only be one host. The example in this section sets the virtual  
machine as the host, and other robots as the slaves. There are several slaves. Of course, you can  
also set one robot as the master and others as the slaves.  
According to different models, you only need to set the purchased model in [.bashrc], X1(ordinary  
four-wheel drive) X3(Mike wheel) X3plus(Mike wheel mechanical arm) R2(Ackerman differential)  
and so on. Section takes X3 as an example  
Open the [.bashrc] file  
Find the [ROBOT_TYPE] parameter and modify the corresponding model  
3.2 Use  
Take the virtual machine as the host and the three robots as slaves as an example; a map must be  
available before use.  The three slaves are [robot1], [robot2], and [robot3], and [robot1] is set as  
the leader, and [robot2] and [robot3] are set as the follower. Make sure the field is large enough  
to avoid collisions when playing this feature. And no obstacle avoidance function. This routine  
does not support the Raspberry PI 5 board driver#Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker 
tutorial
~/run_docker.sh
sudo vim .bashrc 
export  ROBOT_TYPE=X3    # ROBOT_TYPE: X1 X3 X3plus R2 X7 
3.2.1 Start the robot  
virtual machine side  
Start the command(robot1 side), for the convenience of operation, this section takes [mono +  
laser + yahboomcar] as an example.  
Start command(robot2 side), for the convenience of operation, this section takes [mono + laser +  
yahboomcar] as an example.  
More bots and so on.  
3.2.2 Open multi-machine formation  
For the process of opening the handle control, please refer to the lesson [Multi-machine handle  
control].  
virtual machine side  
[use_rviz] parameter: whether to open rviz.  
[map] Parameters: map name, the map to be loaded.  
After startup, you need to initialize the pose setting of the robot. For the specific setting method,  
please refer to [Multi-machine Navigation This Lesson]. After setting, the following figure is  
shown.  roscore 
roslaunch  yahboomcar_multi  laser_bringup_multi.launch  ns:=robot1             
# laser + yahboomcar 
roslaunch  yahboomcar_multi  laser_usb_bringup_multi.launch  ns:=robot1         
# mono + laser + yahboomcar 
roslaunch  yahboomcar_multi  laser_astrapro_bringup_multi.launch  ns:=robot1   
 # Astra + laser + yahboomcar 
roslaunch  yahboomcar_multi  laser_bringup_multi.launch  ns:=robot2             
# laser + yahboomcar 
roslaunch  yahboomcar_multi  laser_usb_bringup_multi.launch  ns:=robot2         
# mono + laser + yahboomcar 
roslaunch  yahboomcar_multi  laser_astrapro_bringup_multi.launch  ns:=robot2   
 # Astra + laser + yahboomcar 
roslaunch yahboomcar_multi tf_queuebroad.launch use_rviz:=true map:=my_map 
3.2.3 Formation control  
Open the dynamic parameter adjustment tool  
rosrun rqt_reconfigure rqt_reconfigure 
parameter scope Parse
[teams] Default[vertical] Formation: [convoy, vertical, horizontal]
[robot_model] Default [omni] Model: [omni, diff]
[navigate] [False,True] whether to run in navigation mode
[Switch] [False,True] Function switch [Start/Pause]
[dist] [0.5, 2.0] distance between queuesThe tool can be set individually for each robot.  
parameter parsing  
[lin_Kp], [lin_Ki], [lin_Kd]: PID debugging of trolley linear speed.  
[ang_Kp], [ang_Ki], [ang_Kd]: PID debugging of car angular velocity.  
After the [Switch] function switch is turned on, the robot automatically maintains the formation  
set by the [teams] parameter, and the [teams] parameter only recognizes the last set value. As  
shown below  
a column(vertical)  
Left and right guards(convoy)  
Horizontal team(horizontal)  
After the formation is set, when we control [robot1], other robots automatically maintain the  
formation without control.  
3.3 launch file  
tf_queuebroad.launch  
< launch > 
    < arg  name = "first_robot1"  default = "robot1" /> 
    < arg  name = "second_robot2"  default = "robot2" /> 
    < arg  name = "third_robot3"  default = "robot3" /> 
    <!--  rviz  ||  Whether  to  open  rviz  --> 
    < arg  name = "use_rviz"  default = "true" /> 
    <!--  Map name  ||  Map  name  --> 
    < arg  name = "map"  default = "my_map" /> 
    <!--  Load map  ||  Load  map  --> 
    < node  name = "map_server"  pkg = "map_server"  type = "map_server"  args = 
"$(find yahboomcar_nav)/maps/$(arg map).yaml" /> 
    < node  pkg = "rviz"  type = "rviz"  name = "rviz"  required = "true"  args 
= "-d $(find yahboomcar_multi)/rviz/tf_multi.rviz"  if = "$(arg use_rviz)" /> 
    <!--  ############################# first_robot1 
############################# --> 
    < node  pkg = "yahboomcar_multi"  type = "broad_queue.py"  name = 
"QueueBroad"  output = "screen"  args = "$(arg first_robot1)" /> 
    < include  file = "$(find 
yahboomcar_multi)/launch/library/move_base_multi.launch" > 
        < arg  name = "ns"  value = "$(arg first_robot1)" /> 
    </ include > 
    <!--  ############################# second_robot2 
############################# --> 
    < node  pkg = "yahboomcar_multi"  type = "listener.py"  name = 
"RobotListener"  output = "screen" 
          args = "$(arg second_robot2) point1"  ns = "$(arg second_robot2)/" > 
        < rosparam  param = "linPIDparam" > [ 1.0, 0, 1.0 ] </ rosparam > 
        < rosparam  param = "angPIDparam" > [ 0.8, 0, 1.0 ] </ rosparam > 
    </ node > 
    < include  file = "$(find 
yahboomcar_multi)/launch/library/move_base_multi.launch" > 
        < arg  name = "ns"  value = "$(arg second_robot2)" /> 
    </ include > 
    <!--  ############################# third_robot3 
############################# --> 
    < node  pkg = "yahboomcar_multi"  type = "listener.py"  name = 
"RobotListener"  output = "screen" 
          args = "$(arg third_robot3) point2"  ns = "$(arg third_robot3)/" > 
        < rosparam  param = "linPIDparam" > [ 1.0, 0, 1.0 ] </ rosparam > 
        < rosparam  param = "angPIDparam" > [ 0.8, 0, 1.0 ] </ rosparam > 
    </ node > 
    < include  file = "$(find 
yahboomcar_multi)/launch/library/move_base_multi.launch" > 
        < arg  name = "ns"  value = "$(arg third_robot3)" /> 
    </ include > 
</ launch > 
3.4 frame analysis  
Node view  
[QueueBroad] The node subscribes to the queue instructions from each robot, and the node will  
only recognize the latest formation setting.  
View tf tree  
It can be seen from the above figure that [robot1] will send out two coordinate systems [point1]  
and [point2], [robot2] and [robot3] monitor the relationship between itself and the coordinate  
system in real time, and make its own coordinate system coincide with the coordinate system. In  
the [tf_queuebroad.launch] file, we can see that [robot2] follows the [point1] coordinate system;  
[robot3] follows the [point2] coordinate system.  
 rqt_graph 
rosrun rqt_tf_tree rqt_tf_tree 

---

## 3. Voice control car movement.pdf

3. 3. Voice control car movement  
3. 3. Voice control car movement
3.1. Description 
3.2. Start function
3.2.1. function package path 
3.2.2. start 
3.3. Voice control car 
3.3.1. Movement state 
3.3.2. Light strip effect 
3.1. Description  
Realize voice control car forward, backward, parking, turn left, turn right and light strip effects.
3.2. Start function  
3.2.1. function package path  
3.2.2. start  
Core code analysis: 
1. import the library of speech recognition ~/yahboomcar/src/yahboomcar_voice_ctrl/ 
#Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
roslaunch yahboomcar_voice_ctrl voice_ctrl_yahboomcar.launch 
2. Create speech recognition objects and drive control objects 
3. read the content recognized by the voice 
4. send voice broadcast content 
Program flow chart: 
The specific code can refer to 
3.3. Voice control car  
Say "Hi Yahboom" to ROSMASTER.
Waiting until the voice module reply "Hi , I'm here.".
We can control the car according to the commands in the table below.from Speech_Lib import Speech 
from Rosmaster_Lib import Rosmaster 
spe  =  Speech() 
car  =  ROsmaster() 
speech_r  =  spe.speech_read()
spe.void_write(speech_r) 
~/yahboomcar/src/yahboomcar_voice_ctrl/scripts/voice_Ctrl_Mcnamu_driver.py 
function word Speech Module Recognition Results Voice broadcast content
Robot stop 2 OK , I'm stop.
Go ahead 4 OK , let's go.
Back 5 OK , I‘m back.
Turn left 6 OK , I‘m turning left.
Turn right 7 OK , I‘m turning right.
Enter A mode 8 OK, I‘m working on A mode.
Enter B mode 9 OK, I‘m working on B mode.
function wordSpeech Module Recognition
ResultsVoice broadcast content
Close light 10 OK, light is closed.
Red light up 11 OK, red light is on.
Green light up 12 OK, green light is on.
Blue light up 13 OK, blue light is on.
Yellow light up 14 OK, yellow light is on.
light A 15 OK, light A is on.
light B 16 OK, light B is on.
light C 17 OK, light C is on.
Display battery
value18OK, battery value has been
display.3.3.1. Movement state  
3.3.2. Light strip effect  
 

---

## 4、Docker hardware interaction and data processing.pdf

4. Docker hardware interaction and data  
processing 
 
4. Docker hardware interaction and data processing
4.1. Hardware mounting (port binding)
4.2. Display of GUI in docker
4.3. Transfer files between docker container and host machine
4.3.1. Use cp naming
4.3.1.1. Copy files from the container to the host
4.3.1.2. Copy files from the host to the container
The operating environment and software and hardware reference configuration are as follows:
Reference model: ROSMASTER X3
Robot hardware configuration: Arm series main control, Silan A1 lidar, AstraPro Plus depth  
camera
Robot system: Ubuntu (no version required) + docker (version 20.10.21 and above)
PC virtual machine: Ubuntu (18.04) + ROS (Melodic)
Usage scenario: Use on a relatively clean 2D plane
 
4.1. Hardware mounting (port binding)  
1. Establish udev rules (/etc/udev/rules.d/) in the host machine, see chapter [6. Linux operating  
system ---- 6. Bind device ID]
2. Then when opening the container, mount the devices with the rules set into the docker  
container through --device=/dev/myserial --device=/dev/rplidar and other parameters.
3. The device can be found in the docker containerdocker run -it --device=/dev/myserial --device=/dev/rplidar ubuntu:latest 
/bin/bash
jetson@ubuntu:~$ docker images
REPOSITORY TAG IMAGE ID CREATED SIZE
ubuntu 1.0 78ca7be949b6 About an hour ago 69.2MB
pengan88/ubuntu 1.0 78ca7be949b6 About an hour ago 69.2MB
yahboomtechnology/ros-foxy 3.4.0 49581aa78b6b 6 hours ago 24.3GB
yahboomtechnology/ros-foxy 3.3.9 cefb5ac2ca02 4 days ago 20.5GB
yahboomtechnology/ros-foxy 3.3.8 49996806c64a 4 days ago 20.5GB
yahboomtechnology/ros-foxy 3.3.7 8989b8860d17 5 days ago 17.1GB
yahboomtechnology/ros-foxy 3.3.6 326531363d6e 5 days ago 16.1GB
mysql latest 5371f8c3b63e 6 days ago 592MB
ubuntu latest bab8ce5c00ca 6 weeks ago 69.2MB
hello-world latest 46331d942d63 13 months ago 9.14kB
jetson@ubuntu:~$ ll /dev | grep ttyUSB*
 
4.2. Display of GUI in docker  
1. Install on the host machine:
2. Execute in the host machine: xhost +
After the following picture is displayed normally, perform 3 steps:
  
3. Execute the command on the host to enter the container:
4. Test
 
 lrwxrwxrwx 1 root root 7 Apr 23 18:07 myserial -> ttyUSB0
lrwxrwxrwx 1 root root 7 Apr 23 18:07 rplidar -> ttyUSB1
crwxrwxrwx 1 root dialout 188, 0 Apr 23 18:07 ttyUSB0
crwxrwxrwx 1 root dialout 188, 1 Apr 23 18:07 ttyUSB1
jetson@ubuntu:~$ docker run -it --device=/dev/myserial --device=/dev/rplidar 
ubuntu:latest /bin/bash
root@03522257ba30:/# ls /dev # There are already myserial and rplidar in docker
console fd full mqueue myserial null ptmx pts random rplidar shm stderr stdin 
stdout tty urandom zero
sudo apt-get install tigervnc-standalone-server tigervnc-viewer
sudo apt-get install x11-xserver-utils
docker run -it \ # Interactively run the docker image
--env="DISPLAY" \ # Turn on the display GUI interface
--env="QT_X11_NO_MITSHM=1" \ # Use X11 port 1 for display
-v /tmp/.X11-unix:/tmp/.X11-unix \ # Map display service node directory
yahboomtechnology/ros-foxy:3.3.9 # The name of the image to be started
/bin/bash # Execute the /bin/bash command within the container
Execute in container: rviz2
4.3. Transfer files between docker container and  
host machine 
4.3.1. Use cp naming  
4.3.1.1. Copy files from the container to the host  
 
4.3.1.2. Copy files from the host to the container  # Order
docker cp container id: path within the container destination host path
# test
# Execute within the container and create a file test
jetson@ubuntu:~$ docker ps -a
CONTAINER ID IMAGE COMMAND CREATED STATUS PORTS NAMES
c54bf9efae47 ubuntu:latest "/bin/bash" 2 hours ago Up 9 minutes funny_hugle
3b9c01839579 hello-world "/hello" 3 hours ago Exited (0) 3 hours ago jovial_brown
jetson@ubuntu:~$ docker attach c5
root@c54bf9efae47:/# ls
bin boot dev etc home lib media mnt opt proc root run sbin srv sys tmp usr var
root@c54bf9efae47:/# cd
root@c54bf9efae47:~# ls
root@c54bf9efae47:~# touch test.txt
root@c54bf9efae47:~# ls
test.txt
root@c54bf9efae47:~# pwd
/root
root@c54bf9efae47:/# read escape sequence #Press ctrl+P+Q to exit the container 
without stopping.
jetson@ubuntu:~$ docker cp c54bf9efae47:/root/test.txt ~/
jetson@ubuntu:~$ ls # The test.txt file has been copied in
Desktop Documents Downloads fishros Music openvino Pictures Public rootOnNVMe 
run_docker.sh sensors snap temp Templates test.txt Videos
# Order
docker cp host file path container id: path within the container
#test
jetson@ubuntu:~$ docker ps -a
CONTAINER ID IMAGE COMMAND CREATED STATUS PORTS NAMES
c54bf9efae47 ubuntu:latest "/bin/bash" 2 hours ago Up 5 minutes funny_hugle
3b9c01839579 hello-world "/hello" 3 hours ago Exited (0) 3 hours ago jovial_brown
jetson@ubuntu:~$ ls
Desktop Documents Downloads fishros Music openvino Pictures Public rootOnNVMe 
run_docker.sh sensors snap temp Templates test.txt Videos
jetson@ubuntu:~$ touch 11.txt
jetson@ubuntu:~$ ls
11.txt Desktop Documents Downloads fishros Music openvino Pictures Public 
rootOnNVMe run_docker.sh sensors snap temp Templates test.txt Videos
jetson@ubuntu:~$ docker cp 11.txt c54bf9efae47:/root/
jetson@ubuntu:~$ docker attach c5
root@c54bf9e

---

## 4. Astra Object Tracking.pdf

4. Astra object tracking  
4. Astra object tracking
4.1. Introduction
4.2, KCF object tracking
4.2.1. How to use
4.2.2. Keyboard control
4.2.3. Node analysis
4.1. Introduction  
Function package: ~/yahboomcar_ws/src/yahboomcar_astra
Official website: https://learnopencv.com/object-tracking-using-opencv-cpp-python/#opencv-tracki
ng-api
Object Tracking
Object tracking is to locate an object in consecutive video frames. This definition sounds  
straightforward, but in computer vision and machine learning, tracking is a very broad term that  
encompasses conceptually similar but technically different concepts. For example, the following  
are all different but related ideas commonly studied under object tracking:
(1) Dense Optical flow DOF: These algorithms help estimate the motion vector of each pixel in a  
video frame.
(2) Sparse optical flow: For example, the Kanade-Lucas-Tomashi (KLT) feature tracking algorithm  
tracks the positions of several feature points in the image.
(3) Kalman Filtering: A very popular signal processing algorithm based on prior motion  
information, used to predict the position of moving targets. One of the early applications of this  
algorithm was in missile guidance! The onboard computer that guided the Apollo 11 lunar module  
to the moon had a Kalman filter.
(4) Meanshift and Camshift: This is an algorithm for locating the maximum value of the density  
function. They are also used for tracking.
(5) Single object trackers: In this type of trackers, the first frame is marked with a rectangle to  
indicate the location of the object to be tracked. The object is then tracked in subsequent frames  
using a tracking algorithm. In most real-world applications, these trackers are used together with  
object detectors.
(6) Multiple object track finding algorithms: When we have a fast object detector, detect multiple  
objects in each frame, and then run a tracking finding algorithm to identify which object in a frame  
It makes sense for the rectangle to correspond to the rectangle in the next frame.
Comparison of various algorithms
Algorithm Speed Accuracy Description
BOOSTING Slow PoorThe same machine learning algorithm used
behind Haar casades (AdaBoost),
but it has been around for more than ten years
and is a veteran algorithm.
MIL Slow PoorMore accurate than BOOSTING, but has a
higher failure rate.
KCF Fast HighFaster than BOOSTING and MIL, but does not
perform well in occlusion situations.
TLD General GeneralThere are many false positives, and the
phenomenon of skewing is serious.
MEDIANFLOWGeneral
+GeneralThere are very few false positives. For fast
jumping or fast moving objects, the model will
fail.
GOTURN General GeneralDeep learning based object detector, which
requires additional models to run.
MOSSE Fastest High -The speed is really fast, but it is not as accurate
as CSRT and KCF. If you are pursuing speed, you
can choose it.
CSRT Fast - Highest Slightly more accurate than KCF, but not as fast.
4.2, KCF object tracking  
KCF stands for Kernel Correlation Filter Kernel Correlation Filtering Algorithm. It was proposed by  
Joao F. Henriques, Rui Caseiro, Pedro Martins, and Jorge Batista in 2014. After the algorithm came  
out, it was considered a sensation. This algorithm has very outstanding performance in both  
tracking effect and tracking speed. Therefore, a large number of scholars have been studying this  
algorithm, and the industry has also gradually applied this algorithm in actual scenarios. This  
[Algorithm Home Page] ( http://www.robots.ox.ac.uk/~joao/circulant/index.html ) contains papers  
and codes that can be downloaded here, as well as some introductions, etc. This article was  
published by the author on TPAMI in 2015, so you may see two versions, but there are no changes  
and both can be seen. Paper download address  The related filtering algorithm is considered  
discriminative tracking. It mainly uses the given samples to train a discriminative classifier to  
determine whether the target or the surrounding background is tracked. information. The rotation  
matrix is mainly used to collect samples, and the fast Fourier transform is used to accelerate the  
calculation of the algorithm.
4.2.1. How to use  
Note: [R2] on the remote control handle has the [pause/start] function for this gameplay. 
#Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
One-click startup (robot side)
After starting, enter the selection mode, use the mouse to select the location of the object, as  
shown in the figure below, release it to start recognition.
4.2.2. Keyboard control  
<PI5 needs to open another terminal to enter the same docker container
【r 】 : Reset mode, you can use the mouse to select the area to identify the target.
[q]: Exit the program.roslaunch yahboomcar_astra KCFTracker.launch
[Spacebar]: Target tracking; just move the target slowly while following. If you move too fast, you  
will lose the target.
Parameter analysis:
[linear_Kp], [linear_Ki], [linear_Kd]: PID control of linear speed during car following.
[angular_Kp], [angular_Ki], [angular_Kd]: PID control of angular velocity during car following.
[minDist]: Follow the distance and keep this distance.
Parameter modification
When the parameters are adjusted to the optimal state, the corresponding parameters are  
modified into the file, and no adjustment is required when using again.
According to the optimal parameters of the [rqt_reconfigure] debugging tool, enter the [src] folder  
of the [yahboomcar_astra] function package and modify the parameters corresponding to the  
[KCF_Tracker.cpp] file, as shown below
The minDist parameter is modified in the [KCF_Tracker.h] filerosrun rqt_reconfigure rqt_reconfigure
ImageConverter::ImageConverter(ros::NodeHandle &n) {
     KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
     float linear_KP=0.9;
     float linear_KI=0.0;
     float linear_KD=0.1;
     float angular_KP=0.5;
     float angular_KI=0.0;
     float angular_KD=0.2;
float minDist = 1.0;
Parameters Analysis Corresponding parameters
name name of the parameter "linear_Kp"
type parameter data type double_t
level a bitmask passed to the callback 0
description A description parameter "Kp in PID"
default Initial value for node startup 0.9
min parameter minimum value 0
max parameter maximum value 10.0[rqt_reconfigure] Modification of the initial value of the debugging tool
Enter the [cfg] folder of the [yahboomcar_astra] function package and modify the initial values of  
the parameters corresponding to the [KCFTracker.cfg] file.
Take the above article as an example to analyze
Note: After modification, you must recompile and update the environment to be effective. 
4.2.3. Node analysis  gen.add("linear_Kp", double_t, 0, "Kp in PID", 0.9, 0, 10.0)
gen.add("linear_Ki", double_t, 0, "Ki in PID", 0.0, 0, 10.0)
gen.add("linear_Kd", double_t, 0, "Kd in PID", 0.1, 0, 10.0)
gen.add("angular_Kp", double_t, 0, "Kp in PID", 0.5, 0, 10.0)
gen.add("angular_Ki", double_t, 0, "Ki in PID", 0.0, 0, 10.0)
gen.add("angular_Kd", double_t, 0, "Kd in PID", 0.2, 0, 10.0)
gen.add("minDist", double_t, 0, "minDist", 1.0, 0, 10.0)
exit(gen.generate(PACKAGE, "KCFTracker", "KCFTrackerPID"))
gen.add("linear_Kp", double_t, 0, "Kp in PID", 0.9, 0, 10.0)
cd ~/yahboomcar_ws
catkin_make
source devel/setup.bash
rqt_graph
【KCF_Tracker 】 Node analysis
Subscribe to RGB color images
Subscribe to depth images
Subscribe to handle control information
Issue car speed control instructions

---

## 4. Multi-machine communication configuration.pdf

4. Multi-machine communication configuration  
4. Multi-machine commu nication configuration 
4.1. Configuration 
4.2. effect demonstration 
Note: Before logging in remotely, you must know the IP of the robot, which can be  
displayed on an external monitor or OLED.  
For example, the following figure: Username [jetson], hostname [yahboom]. 
 
4.1. Configuration  
Require: 
All masters are under the same network 
Choose one as the master, the others are all slaves 
Install ssh and chrony packages on each device for synchronization 
For example: jetson nano is the host machine, the virtual machine is the slave machine, and the IP  
of jetson nano is known. 
Next, just modify the .bashrc file of the slave machine (virtual machine). 
Add at the bottom, where [IP] refers to the IP of [jetson nano]. 
After setting the IP, it is best to refresh the environment variables. #Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to the ROS/07.Docker 
tutorial
~/run_docker.sh
sudo apt-get install chrony openssh-server 
sudo vim ~/.bashrc 
export ROS_MASTER_URI=http://IP:11311 
 source ~/.bashrc 
4.2. effect demonstration  
Note: ROS Master must be started on the host. 
Use ssh to remotely log in to jetson nano and start rosmaster 
ubuntu (virtual machine) 
 roscore 
rosrun turtlesim turtlesim_node  
rosrun turtlesim turtle_teleop_key  

---

## 4. Multi-machine surround.pdf

4. Multi-machine surround  
4. Multi-machine surround 
4.1 Introduction 
4.2 Use 
4.2.1 Start the robot 
4.2.2 Turn on multi-machine surround 
4.2.3 Formation control 
4.3 launch file 
4.4 frame analysis 
4.1 Introduction  
For the problem of how to configure multi-machine communication and synchronization time,  
please refer to the lesson [Multi-machine handle control] for details; if there is a network, the  
network system time can be directly synchronized without setting.  
When using multi-machine handle control, it is first necessary to ensure that the robot is under  
the same local area network and configured with the same [ROS_MASTER_URI]; for multiple  
robots to control motion, there can only be one host. The example in this section sets the virtual  
machine as the host, and other robots as the slaves. There are several slaves. Of course, you can  
also set one robot as the master and others as the slaves.  
According to different models, you only need to set the purchased model in [.bashrc], X1(ordinary  
four-wheel drive) X3(Mike wheel) X3plus(Mike wheel mechanical arm) R2(Ackerman differential)  
and so on. Section takes X3 as an example  
Open the [.bashrc] file  
Find the [ROBOT_TYPE] parameter and modify the corresponding model  
4.2 Use  
Take the virtual machine as the host and the three robots as slaves as an example; a map must be  
available before use.  The two slaves are [robot1] and [robot2] respectively, and [robot1] is set as  
the leader, and [robot2] is set as the follower. Make sure the field is large enough to avoid  
collisions when playing this feature. And no obstacle avoidance function. This routine does not  
support the Raspberry PI 5 board driver#Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker 
tutorial
~/run_docker.sh
sudo vim .bashrc 
export  ROBOT_TYPE=X3    # ROBOT_TYPE: X1 X3 X3plus R2 X7 
4.2.1 Start the robot  
virtual machine side  
Start the command(robot1 side), for the convenience of operation, this section takes [mono +  
laser + yahboomcar] as an example.  
Start command(robot2 side), for the convenience of operation, this section takes [mono + laser +  
yahboomcar] as an example.  
More bots and so on.  
4.2.2 Turn on multi-machine surround  
For the process of opening the handle control, please refer to the lesson [Multi-machine handle  
control].  
virtual machine side  
[use_rviz] parameter: whether to open rviz.  
[map] Parameters: map name, the map to be loaded.  
After startup, you need to initialize the pose setting of the robot. For the specific setting method,  
please refer to [Multi-machine Navigation This Lesson]. After setting, the following figure is  
shown.  roscore 
roslaunch  yahboomcar_multi  laser_bringup_multi.launch  ns := robot1           
  # laser + yahboomcar 
roslaunch  yahboomcar_multi  laser_usb_bringup_multi.launch  ns := robot1       
  # mono + laser + yahboomcar 
roslaunch  yahboomcar_multi  laser_astrapro_bringup_multi.launch  ns := robot1   
 # Astra + laser + yahboomcar 
roslaunch  yahboomcar_multi  laser_bringup_multi.launch  ns := robot2           
  # laser + yahboomcar 
roslaunch  yahboomcar_multi  laser_usb_bringup_multi.launch  ns := robot2       
  # mono + laser + yahboomcar 
roslaunch  yahboomcar_multi  laser_astrapro_bringup_multi.launch  ns := robot2   
 # Astra + laser + yahboomcar 
roslaunch yahboomcar_multi tf_roundbroad.launch use_rviz:=true map:=my_map 
4.2.3 Formation control  
Open the dynamic parameter adjustment tool  
The tool can be set individually for each robot.  
parameter parsing  
[lin_Kp], [lin_Ki], [lin_Kd]: PID debugging of trolley linear speed.  
[ang_Kp], [ang_Ki], [ang_Kd]: PID debugging of car angular velocity.  rosrun rqt_reconfigure rqt_reconfigure 
parameter scope Parse parameter scope Parse
[teams] Default[vertical] This parameter is invalid under this function
[robot_model] Default [omni] Model: [omni, diff]
[navigate] [False,True] whether to run in navigation mode
[Switch] [False,True] Function switch [Start/Pause]
[dist] [0.5, 2.0] wrap radius
After the [Switch] function switch is turned on, the robot automatically obtains its own position  
and the position launched by the pilot robot. When it reaches the closest point for the second  
time, it will follow the robot to automatically circle. As shown below  
4.3 launch file  
tf_roundbroad.launch  
< launch > 
    < arg  name = "first_robot1"  default = "robot1" /> 
    < arg  name = "second_robot2"  default = "robot2" /> 
    <!--  rviz  ||  Whether  to  open  rviz  --> 
    < arg  name = "use_rviz"  default = "true" /> 
    <!--  Map name  ||  Map  name  --> 
    < arg  name = "map"  default = "my_apm" /> 
    <!--  Load map  ||  Load  map  --> 
    < node  name = "map_server"  pkg = "map_server"  type = "map_server"  args = 
"$(find yahboomcar_nav)/maps/$(arg map).yaml" /> 
4.4 frame analysis  
Node view  
View tf tree  
     < node  pkg = "rviz"  type = "rviz"  name = "rviz"  required = "true"  args 
= "-d $(find yahboomcar_multi)/rviz/tf_multi.rviz"  if = "$(arg use_rviz)" /> 
    <!--  ############################# first_robot1 
############################# --> 
    < node  pkg = "yahboomcar_multi"  type = "broad_round.py"  name = 
"RoundBroad"  output = "screen"  args = "$(arg first_robot1)" /> 
    < include  file = "$(find 
yahboomcar_multi)/launch/library/move_base_multi.launch" > 
        < arg  name = "ns"  value = "$(arg first_robot1)" /> 
    </ include > 
    <!--  ############################# second_robot2 
############################# --> 
    < node  pkg = "yahboomcar_multi"  type = "listener.py"  name = 
"RobotListener"  output = "screen" 
          args = "$(arg second_robot2) point1"  ns = "$(arg second_robot2)/" > 
        < rosparam  param = "linPIDparam" > [ 1.0, 0, 1.0 ] </ rosparam > 
        < rosparam  param = "angPIDparam" > [ 0.8, 0, 1.0 ] </ rosparam > 
    </ node > 
    < include  file = "$(find 
yahboomcar_multi)/launch/library/move_base_multi.launch" > 
        < arg  name = "ns"  value = "$(arg second_robot2)" /> 
    </ include > 
</ launch > 
rqt_graph 
rosrun rqt_tf_tree rqt_tf_tree 
It can be seen from the above figure that [robot1] will send out the [point1] coordinate system,  
and [robot2] monitors the relationship between itself and the coordinate system in real time, and  
makes its own coordinate system coincide with the coordinate system. In the  
[tf_roundbroad.launch] file, we can see that [robot2] follows the [point1] coordinate system.  
 
 

---

## 4. Voice control Autopilot.pdf

4. Voice controlled autonomous driving  
4.1. Function description  
By interacting with the voice recognition module on ROSMASTER, you can turn on or off  
ROSMASTER's red/blue/green/yellow line patrol function by voice. The R2 key on the handle can  
cancel/turn on this function at any time.
4.2. Start  
4.2.1. Function package path  
4.2.2. Start  
<PI5 needs to open another terminal to enter the same docker container
#Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
~/yahboomcar_ws/src/yahboomcar_voice_ctrl/
roslaunch yahboomcar_voice_ctrl voice_ctrl_followline.launch #Launch chassis 
control + radar + wireless handle node
python3 
~/yahboomcar_ws/src/yahboomcar_voice_ctrl/scripts/voice_Ctrl_follow_line.py 
#Enable line follow function
Taking the yellow line patrol as an example, place the ROSMASTER on the yellow line, adjust the  
camera position, and move the camera downwards. After the program is started, call ROSMASTER  
"Hello, Xiaoya" to wake up the module. When it broadcasts "in", it means waking up the module.  
Next, you can say "Patrol Yellow Line" to it, and ROSMASTER will broadcast "Okay, it's on." Yellow  
line patrol function".
Then, we release the handle's control of ROSMASTER by pressing the R2 key of the handle, and  
ROSMASTER begins to patrol the yellow line. If there is no remote control, you can also enter the  
following command through the terminal,
If you want to cancel the line patrol function, say "Turn off line patrol" to ROSMASTER. ROSMASTER  
will stop and the voice will broadcast "OK, line patrol function has been turned off".
4.2.3. Dynamic parameter adjustment  
Open the dynamic adjustment parameters, select the LineDetect column, and then adjust the  
parameters inside. Manually modify the adjusted parameters to voice_Ctrl_follow_line.py, and  
restart the program to use the adjusted parameters.rostopic pub /JoyState std_msgs/Bool False
rosrun rqt_reconfigure rqt_reconfigure
4.3. Core code analysis:  
4.3.1. Import the speech recognition library and create speech  
recognition objects 
4.3.2. Modify the value of hsv_range (process function) based on the  
content read through speech recognition, and then obtain the value of  
circle from Speech_Lib import Speech
self.spe = Speech()
self.command_result = self.spe.speech_read()
self.spe.void_write(self.command_result)
if self.command_result == 23 :
self.model = "color_follow_line"
     print("red follow line")
     self.hsv_range = [(0, 106, 175), (180, 255, 255)]
            
elif self.command_result == 24 :
     self.model = "color_follow_line"
     print("green follow line")
     self.hsv_range = [(55, 105, 136), (95, 255, 255)]
    
elif self.command_result == 25 :
     self.model = "color_follow_line"
     print("bule follow line")
     self.hsv_range = [(55, 134, 218), (125, 253, 255)]
            
elif self.command_result == 26 :
     self.model = "color_follow_line"
     print("yellow follow line")
     self.hsv_range = [(17, 55, 187), (81, 255, 255)]
rgb_img, binary, self.circle = self.color.line_follow(rgb_img, self.hsv_range)
Note: The value of hsv here can be modified according to the actual situation. Since the  
camera is sensitive to light, the hsv value here may be different, and the line following  
effect is not very good. Users can adjust the maximum and minimum values of HSV with  
dynamic parameters, modify the max and min values of the calibrated color HSV into the  
above code, and use the calibrated values after restarting the program. 
4.3.3. Release the speed to the chassis (execute function)  
According to the value of self.circle obtained, it is passed into execute as an actual parameter, data  
is processed, whether to avoid obstacles is judged, and finally the speed topic data is released.if color_radius == 0: self.ros_ctrl.pub_cmdVel.publish(Twist())
     else:
     twist = Twist()
         b = Bool()
         [z_Pid, _] = self.PID_controller.update([(point_x - 320)/16, 0])
         if self.img_flip == True: twist.angular.z = +z_Pid
         else: twist.angular.z = -z_Pid
         twist.linear.x = self.linear
         if self.warning > 10:
             rospy.loginfo("Obstacles ahead!!!")
             self.ros_ctrl.pub_cmdVel.publish(Twist())
             self.Buzzer_state = True
             b.data = True
             self.pub_Buzzer.publish(b)
         else:
             if self.Buzzer_state == True:
                 b.data = False
                 for i in range(3): self.pub_Buzzer.publish(b)
                 self.Buzzer_state = False
             self.ros_ctrl.pub_cmdVel.publish(twist)
Function wordsSpeech recognition
module resultsVoice broadcast content
Turn off the line
following function22OK, the line following function has
been turned off
Turn on the red line
patrol function23OK, the red line patrol function
has been turned on
Turn on the green line
patrol function24OK, the green line patrol function
has been turned on
Turn on the blue line
patrol function25OK, the blue line patrol function
has been turned on
Turn on the yellow line
patrol function26OK, the yellow line patrol function
has been turned on4.3.4, flow chart  
For complete code, please refer to:
4.3.5. Function module communication table  ~/yahboomcar_ws/src/yahboomcar_voice_ctrl/scripts/voice_Ctrl_follow_line.py

---

## 5、Enter the bot's docker container.pdf

5. Enter the robot’s docker container  
*Raspberry Pi PI5 Master Control ’s ROS1 courses are all in docker containers. *
5. Enter the robot’s docker container
5.1, related concepts
5.2. How to query the docker image version used by the robot
5.3. Binding peripherals
5.4. Check the peripheral connection status
5.5. Edit script
5.6. Execute script
5.7. Switch models, radars and cameras
5.8. Multiple terminals enter the same docker container
5.9. How to open a container that is already in the [Exited] state
5.9.1. Need to use camera
5.9.2. No need to use camera
5.9.3. Containers that enter the [Exited] closed state again
The operating environment and software and hardware reference configuration are as follows:
Reference model: ROSMASTER X3
Robot hardware configuration: Arm series main control, Silan A1 lidar, AstraPro Plus depth  
camera
Robot system: Ubuntu (no version required) + docker (version 20.10.21 and above)
PC virtual machine: Ubuntu (18.04) + ROS (Melodic)
Usage scenario: Use on a relatively clean 2D plane
 
5.1, related concepts  
1. What is a docker host?
2. What is GUI?
3. What is the robot’s docker container?The host is the server where we call the command to create the container using 
the image. This refers to the main control on our car (jetson or Raspberry Pi, 
etc.). The hosts mentioned below all refer to this.
GUI is the graphical user interface, which mainly refers to: the image window 
displayed by opencv, rviz interface, rqt interface, etc.
The robot here is the Rosmaster car, which is the Rosmaster car container that 
has been configured with various development dependency environments.
4. Before operating the tutorial in this chapter, please make sure that you have mastered the  
knowledge of the following chapters, otherwise you may find it difficult to learn. If this  
happens, please review the following pre-knowledge content repeatedly. Once you master it,  
you will feel very relaxed. Come on, you are the best!
 
5.2. How to query the docker image version used by  
the robot 
1. The docker image version used by the robot is also the image version used on the car. After  
the user burns the system image of the car and starts it, execute:
You will see multiple docker image versions. Please select the name [yahboomtechnology/ros-
melodic]. The version with the highest tag is the latest image version of the robot. As queried here,  
use the [yahboomtechnology/ros-melodic:1.4.1] version,.
2. Why can’t we just put one docker image in the car system?
If you have read the tutorial in this chapter [07. Docker ------- 3. In-depth understanding of docker  
images and publishing images], you should know that docker images are a layered mechanism,  
that is, the image of a subsequent tag depends on the image of the previous tag. Mirror.  
Therefore, there may be multiple versions of docker images in the host machine, and the tags of  
these images will be updated incrementally.
In the future, we will update new courses and update functions by releasing new docker images.
 
5.3. Binding peripherals  
 
First make sure that the car has connected various peripherals and has done port binding on  
the peripherals. The port binding is processed on the docker host (car)
Common peripherals include: serial port equipment, laser radar, RGBD camera, voice control  
module, joystick remote control, etc.
By default, the car has been bound to Astra camera, lidar and serial device . If you need  
to bind other devices, please refer to the port binding tutorial.
For the steps of port binding, please refer to the tutorial chapter [6. Linux operating system --
---- 06. Binding device ID]
 #PI5 ROS1
pi@yahboom:~ $ docker images
REPOSITORY TAG IMAGE ID CREATED SIZE
yahboomtechnology/ros-melodic 1.4.1 f8c914ba3cff 12 days ago 23.1GB
Port binding has been configured in the host. If you need to modify it, you can check the  
content and modify it:
 
5.4. Check the peripheral connection status  
 
This step is performed on the host machine:
1. This is to view the peripherals other than the camera. There is no voice control module  
connected here. If it is connected, the [myspeech] device will be displayed.
 
2. Check the ports of the AstraPro Plus camera as follows:
 
5.5. Edit script  
Since the port number will often change after the AstraPro Plus camera is plugged in and  
unplugged, you need to re-edit the script to configure the port of the AstraPro Plus camera.
 
Edit the script to run docker. This step is performed on the host machine:
1. The script to run docker [run_docker.sh] is generally placed in the root directory of the car's  
owner directory. Here I am in the path below. If not, you can create the file yourself, and  
remember to give the script executable permissions after creation.ll /dev | grep ttyUSB*
jetson@ubuntu:~$ ll /dev/astra*
lrwxrwxrwx 1 root root 15 May 5 17:42 /dev/astradepth -> bus/usb/001/007
lrwxrwxrwx 1 root root 15 May 5 17:42 /dev/astrauvc -> bus/usb/001/009
The content of the [run_docker.sh] script is as follows:
Those without comments can be copied directly and modified as needed.
Note: When adding a host device to the container below, if the host is not connected to the device,  
you need to remove the corresponding addition operation before the container can be opened.
Raspberry Pi PI5  ROS1
Annotated script description:
Note: When adding a host device to the container below, if the host is not connected to the device,  
you need to remove the corresponding addition operation before the container can be opened.chmod +x run_docker.sh #Give the script executable permissions
#!/bin/bash
xhost+
docker run -it \
--net=host \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v /home/pi/temp:/root/temp \
-v /dev/bus/usb/003/006:/dev/bus/usb/003/006 \
-v /dev/bus/usb/003/008:/dev/bus/usb/003/008 \
--device=/dev/myserial \
--device=/dev/rplidar \
--device=/dev/astradepth \
--device=/dev/astrauvc \
--device=/dev/input \
--device=/dev/video0 \
--device=/dev/video1 \
-p 9090:9090 \
-p 8888:8888 \
yahboomtechnology/ros-melodic:1.4.1 /bin/bash
#!/bin/bash
xhost + # xhost is used to support GUI display in docker
docker run -it \ # Interactively run the docker image
--net=host \ # Container network is set to host mode
--env="DISPLAY" \ # Turn on the display GUI interface
--env="QT_X11_NO_MITSHM=1" \ # Use X11 port 1 for display
-v /tmp/.X11-unix:/tmp/.X11-unix \ # Map display service node directory
-v /home/pi/temp:/root/temp \ # As a directory for the host and container to 
temporarily transfer files, you can use this directory if you need to transfer 
files.
-v /dev/bus/usb/001/010:/dev/bus/usb/001/010 \ # Add host device to the 
container. This is the astrpro plus device port. If the car is not connected to 
the camera, please remove this line.
 
2. Modify the above script. These two lines are the port numbers of the AstraPro Plus camera.  
Since the port number will change after the camera is plugged in and out, you need to  
reconfigure the camera port.
It is the camera port queried in step 5.4 2. This port may change after the camera is plugged in  
and out, so everyone's port is different and needs to be configured by yourself.-v /dev/bus/usb/001/011:/dev/bus/usb/001/011 \ # Add host device to the 
container. This is the astrpro plus device port. If the car is not connected to 
the camera, please remove this line.
--device=/dev/astradepth \ # Add host device to the container. Here is the 
astrpro plus device port. If the car is not connected to the camera, please 
remove this line.
--device=/dev/astrauvc \ # Add host device to the container. Here is the astrpro 
plus device port. If the car is not connected to the camera, please remove this 
line.
--device=/dev/video0 \ # Add host device to the container. Here is the astrpro 
plus device port. If the car is not connected to the camera, please remove this 
line.
--device=/dev/video1 \ # Add host device to the container. Here is the astrpro 
plus device port. If the car is not connected to the camera, please remove this 
line.
--device=/dev/myserial \ # Add host device to the container. Here is the serial 
device port. If the car is not connected to the serial port, please remove this 
line.
--device=/dev/rplidar \ # Add host device to the container. Here is the radar 
device port. If the car is not connected to the radar, please remove this line.
--device=/dev/myspeech \ # Add host device to the container. Here is the voice 
control device port. If the car is not connected to the voice control device, 
please remove this line.
--device=/dev/input \ # Add host device to the container. Here is the handle 
device port. If the car is not connected to the handle, please remove this line.
-p 9090:9090 \ # Open port
-p 8888:8888 \
yahboomtechnology/ros-melodic:1.4.1 /bin/bash # The name of the image to be 
started, based on the modification queried in step 5.2; execute the /bin/bash 
command in the container
#Note: When adding the host device to the container above, if the host is not 
connected to the device, you need to remove the corresponding addition operation 
before the container can be opened.
-v /dev/bus/usb/001/010:/dev/bus/usb/001/010 \ # Mount the storage volume to the 
container and mount it to a directory in the container. What is mounted here is 
the RGB and RGB of the camera. depth port
-v /dev/bus/usb/001/011:/dev/bus/usb/001/011 \
-v /dev/bus/usb/001/007:/dev/bus/usb/001/007 \ # Mount the storage volume to the 
container and mount it to a directory in the container. What is mounted here is 
the RGB and RGB of the camera. depth port
-v /dev/bus/usb/001/009:/dev/bus/usb/001/009 \
 
 
5.6. Execute script  
 
After step 5.5 is completed, open the terminal on the docker host machine [i.e. the car, which can  
be on VNC or on the car screen]
Note: This must be executed on the VNC of the car or on the screen of the car. It cannot be  
executed on the car terminal remotely entered through ssh.Execute in the terminal (such as the  
car terminal entered through MobaXterm), otherwise the GUI image may not be displayed in the  
container. As follows, after entering the car terminal in MobaXterm and executing run_docker.sh  
to enter the container, rviz cannot be displayed.
 
Execute in the VNC interface of the car or on the car screen:
You can correctly enter the container and display the GUI screen. You can execute the rviz  
command test again.
 
If the GUI cannot be displayed after executing the rviz command, the following error is displayed:  
(generally possible in the Raspberry Pi master)./run_docker.sh
You need to add another parameter to the startup script:
Raspberry Pi PI5  ROS1
Then run the script again to enter the container and display the GUI screen.
 
 
5.7. Switch models, radars and cameras  
Note: Since the ROSMASTER series robots are divided into multiple types of robots and multiple  
types of equipment, the factory system has been configured with routines for multiple types of  
equipment. However, since the product cannot be automatically identified, the machine type and  
radar model need to be manually set.
After entering the container: Make the following modifications according to the car model, radar  
type and camera type:--security-opt apparmor:unconfined
#!/bin/bash
xhost+
docker run -it \
--net=host \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
-v /tmp/.X11-unix:/tmp/.X11-unix \
--security-opt apparmor:unconfined \ # Added this parameter
-v /home/pi/temp:/root/temp \
-v /dev/bus/usb/003/006:/dev/bus/usb/003/006 \
-v /dev/bus/usb/003/008:/dev/bus/usb/003/008 \
--device=/dev/myserial \
--device=/dev/rplidar \
--device=/dev/astradepth \
--device=/dev/astrauvc \
--device=/dev/input \
--device=/dev/video0 \
--device=/dev/video1 \
-p 9090:9090 \
-p 8888:8888 \
yahboomtechnology/ros-melodic:1.4.1 /bin/bash
root@ubuntu:/# cd
root@ubuntu:~# vim .bashrc
After the modification is completed, save and exit vim, and then execute:
You can see the current modified car model, radar type and camera type
 
Robot project files are stored in the following directory:
 
5.8. Multiple terminals enter the same docker  
container 
 
1. In the above steps, a docker container has been opened. You can open another terminal on  
the host (car) to view:
2. Now enter the docker container in the newly opened terminal:
After successfully entering the container, you can open countless terminals to enter the container.root@ubuntu:~# source .bashrc
-------------------------------------------------- ------
ROS_DOMAIN_ID: 12
my_robot_type: x3 | my_lidar: a1 | my_camera: astraplus
-------------------------------------------------- ------
root@ubuntu:~#
/root/yahboomcar_ws
docker ps -a
docker exec -it 5b698ea10535 /bin/bash
Robot project files are stored in the following directory:
 
3. Note:
(1) When executing the command in step 2, make sure the container is in the [UP] state
(2) If the container is in the [Exited] closed state, please refer to step 1.6 below.
 
 
5.9. How to open a container that is already in the  
[Exited] state 
 
There are two situations: still need to use the camera and no longer need to use the camera
 
5.9.1. Need to use camera  
First, you need to check whether the port of the AstraPro Plus camera has changed according to  
the guidance in the above step [5.3. Check the peripheral connection status].
1. If the port of the Astra Pro camera is changed, it will not be possible to enter the container  
again.
(1) If there are some modifications in the container that need to be retained, you can refer to the  
following command to generate a new image,
(2) If there are no modifications that need to be retained, directly refer to the steps [5.2 to 5.5] in  
this chapter to enter the container.
 
2. If the port of the AstraPro Plus camera has not changed, then directly refer to the steps of  
[5.7.3, Entering the [Exited] Closed State Container Again]./root/yahboomcar_ws
Submit an image from the container:
docker commit container id Target image name to be created: [label name]
For example: docker commit 66c40ede8c68 yahboomtechnology/ros-melodic:1.1 #The 
label name is incremented according to your own situation
Then run this new image into the container: refer to the steps [5.2 to 5.5] in 
this chapter to perform
 
5.9.2. No need to use camera  
Directly refer to the steps of [5.7.3, Entering the [Exited] Closed State Container Again] to perform.
 
 
5.9.3. Containers that enter the [Exited] closed state again  
Open the terminal on the docker host machine [that is, the car, which can be on VNC or on the car  
screen]
Note: This must be executed on the VNC of the car or on the car screen. It cannot be executed in  
the car terminal remotely entered through ssh (such as the car terminal entered through  
MobaXterm). Otherwise, the GUI image may not be displayed in the container. Of course, how can  
you There is no need to display the GUI image, that's fine.
1. First check the status of the container
2. Enable GUI access
3. Open the container [The ID of the container here can be abbreviated, as long as it can  
uniquely identify the currently existing container]
4. Enter the container again
5. Open rviz to see if the GUI screen can be opened.
6. The specific implementation is as follows:docker ps -a
xhost+
docker start 5b
docker exec -it 5b /bin/bash
rviz
jetson@ubuntu:~$ docker ps -a
CONTAINER ID IMAGE COMMAND CREATED STATUS PORTS NAMES
5b698ea10535 yahboomtechnology/ros-melodic:1.4.1 "/bin/bash" 3 days ago Exited 
(0) 8 seconds ago ecstatic_lewin
jetson@ubuntu:~$ xhost +
access control disabled, clients can connect from any host
jetson@ubuntu:~$ docker start 5b
5b
jetson@ubuntu:~$ docker exec -it 5b /bin/bash
-------------------------------------------------- ------
my_robot_type: x3 | my_lidar: a1 | my_camera: astrapro
-------------------------------------------------- ------
root@ubuntu:/# rviz
QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-root'
[INFO] [1682298616.634096279] [rviz]: Stereo is NOT SUPPORTED
[INFO] [1682298616.634576375] [rviz]: OpenGl version: 3.1 (GLSL 1.4)
[INFO] [1682298617.959654036] [rviz]: Stereo is NOT SUPPORTED

---

## 5. Voice control color recognition.pdf

5. Voice control color recognition  
5. Voice control color recognition 
5.1. Description 
5.2. Steps
5.2.1. Function package path 
5.2.2. Start 
5.3. Code analysis 
5.3.1. Impo rt the speech recognition library and create speech recognition objects 
5.3.2. Get mouse events and specify the area selected by the mouse 
5.3.3. Get the HSV value of the selected area 
5.3.4. Determine the area where the HSV value is located, and broadcast the identification result 
according to the interval 
5.3.5. Program flow chart 
5.3.6. Voice module commu nication protocol
5.1. Description  
Start or stop the color recognition function of ROSMASTER by voice module.
5.2. Steps  
5.2.1. Function package path  
5.2.2. Start  
1. After the program is run, we put the object to be identified in front of the camera, select the  
color area of the object with the mouse, keep the mouse and do not release it.
2. Then, say "Hi Yahboom" to the voice module, wait until the voice module replies saying "Hi,  
i‘m here".
3. We can say "What color is this?" and the  voice module will announce the color of the area  
selected by the mouse.
Note: Since the camera is more sensitive to light, the recognition results of the same color  
will be different in environments with different intensities of light.~/yahboomcar_ws/src/yahboomcar_voice_ctrl/ 
#Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
cd ~/yahboomcar_ws/src/yahboomcar_voice_ctrl/scripts #switch directory 
python3 voice_Ctrl_color_identify.py #Run program 
5.3. Code analysis  
5.3.1. Import the speech recognition library and create speech  
recognition objects  
5.3.2. Get mouse events and specify the area selected by the mouse  
This step is mainly to get the value of self.Roi_init, which is used to obtain the HSV value of the  
area 
5.3.3. Get the HSV value of the selected area  
5.3.4. Determine the area where the HSV value is located, and broadcast  
the identification result according to the interval  from Speech_Lib import Speech 
self.spe = Speech() 
def onMouse(self, event, x, y, flags, param):
    if event == 1:
        self.select_flags = True
        self.Mouse_XY = (x,y)
    if event == 4:
        self.select_flags = False
    if self.select_flags == True:
        self.cols = min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y)
        self.rows = max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y)
        self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])
if self.Roi_init[0]!=self.Roi_init[2] and self.Roi_init[1]!=self.Roi_init[3]:
    HSV = cv.cvtColor(rgb_img,cv.COLOR_BGR2HSV)
    for i in range(self.Roi_init[0], self.Roi_init[2]):
        for j in range(self.Roi_init[1], self.Roi_init[3]):
            H.append(HSV[j, i][0])
            S.append(HSV[j, i][1])
            V.append(HSV[j, i][2])
    H_min = min(H); H_max = max(H)
    S_min = min(S); S_max = 253
    V_min = min(V); V_max = 255
command_result = self.spe.speech_read()
    if command_result !=999:
        print(command_result)                
    if command_result == 60:
        if H_min == 0 and H_max == 179 : 
            self.spe.void_write(61)
            print("red")
        elif H_min >= 23 and H_min <= 56:
            print("yellow")
            self.spe.void_write(64)
        elif H_min >= 56 and S_min < 200:
            print("green")
function
wordSpeech Recognition
Module ResultsVoice broadcast content
What color is
this?60Reply according to the color identified,
as following table
color The host sends the result of the recognition Voice broadcast content
red 61 This is red
blue 62 This is blue
green 63 This is green
yellow 64 This is yellow5.3.5. Program flow chart  
Code path:
5.3.6. Voice module communication protocol              self.spe.void_write(63)
        elif H_min >= 60 and S_min >200: 
            print("blue")
            self.spe.void_write(62)
~/yahboomcar_ws/src/yahboomcar_voice_ctrl/scripts/voice_Ctrl_color_identify.py 
 

---

## 6、Robot development environment construction in Docker.pdf

6. Building a robot development  
environment in docker 
 
6. Building a robot develo pment environment in docker
6.1. Use jupyter lab to access docker
6.2. Use vscode to access docker
6.2.1. Remote configuration
6.2.2, vscode configuration
6.2.2.1. Download and install vscode
6.2.2.2, vscode configures ssh to remotely log in to the host machine
6.2.2.3. Enter the robot container
6.2.2.4, vscode remote host configuration docker environment
6.2.2.5. Configure password-free login
The operating environment and software and hardware reference configuration are as follows:
Reference model: ROSMASTER X3
Robot hardware configuration: Arm series main control, Silan A1 lidar, AstraPro Plus depth  
camera
Robot system: Ubuntu (no version required) + docker (version 20.10.21 and above)
PC virtual machine: Ubuntu (18.04) + ROS (Melodic)
Usage scenario: Use on a relatively clean 2D plane
 
6.1. Use jupyter lab to access docker  
1. Enter the container, see [5. Enter the robot’s docker container], and execute the following  
command:
Note: When using jupyter lab in a docker container, you must run the docker container in host  
networking mode: add the "--net=host" parameter when running the container.
2. To view on other devices, open it in a windows or ubuntu browser (must be on the same LAN,  
192.168.2.102 is the IP address in the docker container)root@ubuntu:/# jupyter lab --allow-root
[I 2023-04-24 09:27:45.265 ServerApp] Package jupyterlab took 0.0001s to import
[I 2023-04-24 09:27:45.277 ServerApp] Package jupyter_server_fileid took 0.0096s 
to import
[I 2023-04-24 09:27:45.297 ServerApp] Package jupyter_server_terminals took 
0.0190s to import
[I 2023-04-24 09:27:45.429 ServerApp] Package jupyter_server_ydoc took 0.1301s to 
import
[I 2023-04-24 09:27:45.431 ServerApp] Package nbclassic took 0.0001s to import
.....................
 
The following directory is the project path of the robot:
/root/yahboomcar_ws
 
 http://192.168.2.102:8889/lab
Enter the password: yahboom to enter jupyter lab
6.2. Use vscode to access docker  
Here we take configuring vscode to access the docker container in windows as an example to  
explain. The steps to access docker in ubuntu are basically the same.
 
6.2.1. Remote configuration  
See Chapter [6. Linux Operating System ---- 3. Remote Control]
Make sure that windows can remotely log in to the docker host [car]:
 
6.2.2, vscode configuration  
 
6.2.2.1. Download and install vscode  
vscode official website: https://code.visualstudio.com/ , just download and install the windows  
version
 
6.2.2.2, vscode configures ssh to remotely log in to the host machine  
1. Open vscode, click the icon with the arrow below on the left, then enter remote in the search  
box, select the Remote Development plug-in, and click Install to install the plug-in.
After vscode is installed by default, it is the English version. You can install the Chinese plug-in to  
localize it:Open cmd in windows and enter the ssh command to test: ssh jetson@192.168.2.102 
(change the username and ip to your own)
Or use remote tools: putty, xshell, securecrt, winscp, mobaxterm, finalshell, 
etc.
 
2. Press the shortcut key [ctrl + shift + p] in vscode to open the command input window, enter:  
remote, and then follow the instructions in the figure below to log in to the remote host [car].

If you see the screen above, it means you have successfully logged in to the host computer  
remotely.
 
6.2.2.3. Enter the robot container  
See the tutorial in the chapter [5. Entering the robot’s docker container] to enter.
 
6.2.2.4, vscode remote host configuration docker environment  
1. Install the docker plug-in on the remote host [car]
 
2. After the installation is complete, a docker icon will appear in the left navigation bar.
 
3. Click the docker icon
 
4. Right-click the running container and operate as shown below:
 
5. A new window will open. When you see the following, you have entered the container.
 
6. Open the folder
/root/yahboomcar_ws # This is the project path of the robot
 
7. Similarly, the plug-ins we need can also be installed in the container to facilitate our  
development.
In addition to ros, the recommended plug-ins to install here are:
After completing the above steps, you can operate the code files in the container to develop and  
learn.
 
6.2.2.5. Configure password-free login  
Some of the above steps may require you to enter the host's password. Let's optimize it again and  
configure password-free login.
1. First test using ssh to log in to the host [car] in Windows. The instructions are as follows:
At this time you will find that you need to enter the host password
 ssh jetson@192.168.2.102 (change the username and IP to your own)
2. Next configure password-free login
(1) Add environment variables
Open the environment variable properties page and click New in the user variable section. The  
variable is HOME and the value is C:\Users\name, where name is the user name. You can check  
the user name of your computer by yourself. The key pair generated will be saved by default in  
under this directory.
 
(2) Generate key pair
Open the cmd command line and run it in the directory where the ssh program is located, or after  
adding the system environment, run [ssh-keygen -t rsa] anywhere. This command is used to  
generate the key, and then press Enter all the way. When you see a rectangular diagram  
generated, Then the key generation is successful. At this time, there will be two more files in the  
.ssh folder in the user directory, namely id_rsa (private key) and id_rsa.pub (public key)
 
(3) Add the public key to the host machine
Open the cmd command line similarly and enter
This command first logs in to the host machine, and then adds the public key of the local machine,  
that is, win, to the personal directory of the host account, thereby achieving password-free login.  
Note that this step requires entering the password for the host account.
 
(4) Verification
Test again and use SSH to log in to the host [car] in Windows. The instructions are as follows:
At this time, you will find that you no longer need to enter a password.
Restart vscode, and you no longer need to enter a password where you need to enter a password.ssh username@host "cat >> ~/.ssh/authorized_keys" < C:\Users\name\.ssh\id_rsa.pub
#For example: modify according to your own situation
ssh jetson@192.168.2.102 "cat >> ~/.ssh/authorized_keys" < 
C:\Users\Admin\.ssh\id_rsa.pub
ssh jetson@192.168.2.102 (change the username and IP to your own)

---

## 6. Voice control color tracking.pdf

6. Voice control color tracking  
6.1. Function description  
By interacting with the voice recognition module on ROSMASTER, you can turn on or off  
ROSMASTER's red/blue/green/yellow color tracking function by voice. The R2 key on the handle  
can cancel/enable this function at any time.
6.2. Start  
6.2.1. Function package path  
6.2.2. Start  
<PI5 needs to open another terminal to enter the same docker container
After the program is started, call ROSMASTER "Hello, Xiaoya" to wake up the module. When it  
broadcasts "on", it means waking up the module. Taking tracking red as an example, you can then  
say "Start tracking red" to it, and ROSMASTER will broadcast "Okay, start tracking red". Then, we  
release the controller's control of ROSMASTER by pressing the R2 key of the controller, and  #Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
~/yahboomcar_ws/src/yahboomcar_voice_ctrl/
roslaunch yahboomcar_voice_ctrl voice_ctrl_colorTracker.launch
python 
~/yahboomcar_ws/src/yahboomcar_voice_ctrl/scripts/voice_Ctrl_color_tracker.py
ROSMASTER begins to track red. If there is no remote control, you can also enter the following  
command through the terminal,
If you want to cancel the color tracking function, say "cancel tracking" to ROSMASTER, ROSMASTER  
stops, and the voice will broadcast "OK, cancel tracking".
6.2.3. Color calibration  
The camera is very sensitive to light, so sometimes the color recognition will be inaccurate. In this  
case, the red, green, yellow, and blue colors need to be recalibrated. Terminal input,
Find the colorHSV column and drag the slider to modify the HSV value.
Open the voice_Ctrl_color_tracker.py program and find the following sections,rostopic pub /JoyState std_msgs/Bool False
rosrun rqt_reconfigure rqt_reconfigure
if command_result == 73 :
     self.model = "color_follow_line"
     print("tracker red")
     self.hsv_range = [(0, 185, 175), (180, 253, 255)]
elif command_result == 74 :
     self.model = "color_follow_line"
     print("tracker green")
     self.hsv_range = [(54, 92, 75), (125, 255, 255)]
elif command_result == 75 :
     self.model = "color_follow_line"
     print("tracker bule")
     self.hsv_range = [(55, 204, 177), (125, 253, 255)]
elif command_result == 72 :
     self.model = "color_follow_line"
Modify the calibrated HSV value just recorded to the position of the corresponding color, save it,  
and use the calibrated value the next time you start it.
6.3. Core code analysis voice_Ctrl_color_tracker.py       print("tracker yellow")
     self.hsv_range = [(18, 128, 168), (125, 253, 255)]
command_result = self.spe.speech_read()
         self.spe.void_write(command_result)
         if command_result == 73 :
             self.model = "color_follow_line"
             print("tracker red")
             self.hsv_range = [(20, 215, 111), (180, 253, 255)]
             self.dyn_update = True
         elif command_result == 74 :
             self.model = "color_follow_line"
             print("tracker green")
             self.hsv_range = [(44, 138, 91), (84, 255, 255)]
             self.dyn_update = True
         elif command_result == 75 :
             self.model = "color_follow_line"
             print("tracker bule")
             self.hsv_range = [(83, 217, 196), (141, 253, 255)]
             self.dyn_update = True
         elif command_result == 72 :
             self.model = "color_follow_line"
             print("tracker yellow")
             self.hsv_range = [(18, 55, 187), (81, 253, 255)]
             self.dyn_update = True
         elif command_result == 76 :
             self.model = "Stop"
             #self.ros_ctrl.Joy_active == False
             #self.ros_ctrl.pub_cmdVel.publish(Twist())
         self.command_result = 999
         if self.dyn_update == True :
             params = {'Hmin': self.hsv_range[0][0], 'Hmax': self.hsv_range[1]
[0],
                           'Smin': self.hsv_range[0][1], 'Smax': 
self.hsv_range[1][1],
                           'Vmin': self.hsv_range[0][2], 'Vmax': 
self.hsv_range[1][2]}
             self.dyn_client.update_configuration(params)
             self.dyn_update = False
         if self.model == "color_follow_line":
             self.ros_ctrl.Joy_active == False
             #self.model == "General"
             rgb_img, binary, self.circle = self.color.object_follow(rgb_img, 
self.hsv_range)
             if self.ros_ctrl.Joy_active == False :
                 if self.circle[2] != 0: threading.Thread(
                 target=self.execute, args=(self.circle[0], self.circle[1], 
self.circle[2])).start()
                 if self.point_pose[0] != 0 and self.point_pose[1] != 0: 
threading.Thread(
Function words Speech recognition module results Voice broadcast content
Start tracking yellow 72 OK, start tracking yellow
Start tracking red 73 OK, start tracking red
Start tracking green 74 OK, start tracking green6.3.1. Program flow chart  
For complete code, please refer to:
6.4. Function module communication table                   target=self.execute, args=(self.point_pose[0], 
self.point_pose[1], self.point_pose[2])).start()
             #threading.Thread(target=self.execute, args=(self.circle[0], 
self.circle[2])).start()
         return rgb_img, binary
def execute(self, x, y, z):
position = Position()
position.angleX = x
position.angleY = y
position.distance = z
self.pub_position.publish(position)
~/yahboomcar_ws/src/yahboomcar_voice_ctrl/scripts/voice_Ctrl_color_tracker.py
Function words Speech recognition module results Voice broadcast content
Start tracking blue 75 OK, start tracking blue
Cancel Tracking 76 OK, Cancel Tracking

---

## 7、How customers can update docker images in the future.pdf

7 、How customers can update docker  
images in the future 
7 、How customers can update docker images in the future
7.1 、 Method 1
7.2 、 Method 2
7.3 、 Method 3
Currently, ROS2's courses are all placed in docker containers , and customers can experience  
learning to use containerized development methods.
In the future, new functional modules will continue to be added to docker, and these new  
functional modules will be put into the new docker image, and users need to experience these  
new functions, and there are three ways to update the docker image:
7.1 、 Method 1  
When a new docker image is updated, an img image with a host will be released, and the new  
docker image has been downloaded from the host, and customers can directly use this img to  
experience it.
7.2 、 Method 2  
When updating a new docker image, you can manually update the image without flashing the  
image:
Use commands on the host:
This new image version number Please check the directory of this section: [Latest docker image  
version number and tar fileLatest docker image version number .txt], open [Latest docker image  
version number .txt], how to see the version number is higher than the current host, indicating  
that there is an update, you can update the image.
 docker pulls the latest image version number
For example:
docker pull yahboomtechnology/ros-melodic:1.4.1 # The latest image version number 
here is modified according to the actual view
The [latest docker image version number and tar file] here is updated in real time, and it may be  
greater than version 3.5.3 when you see it, according to what you actually see. This method  
requires downloading the docker image from the Internet, which takes a long time and may time  
out and the download will not come down, if this happens, use the other two methods.
After the pull execution is complete, execute:
You can view the downloaded images to experience the new features
7.3 、 Method 3  
When updating a new docker image, you can manually update the image without flashing the  
image:
The new docker image will provide a [xxx.tar] file, which stores the new docker image, which is  
placed in this section directory [latest docker image version number and tar file], if the version  
number of the file is higher than the current host, it means that there is an update, and the image  
can be updated. Download the file to the host.
 
The tar file here is updated in real time, and it may have been greater than version 3.5.3 when you  
see it, according to what you actually see. Use the command in the directory where the [xxx.tar]  
file of the host is located:
The operation takes some time, but it rarely fails.
After the docker load execution is complete, execute:
You can view the updated image and experience the new features.docker images
docker load -i xxx.tar
docker images

---

## 7. Voice control multi-point navigation.pdf

7. Voice control multi-point navigation  
7.1. Function description  
By interacting with the voice recognition module on ROSMASTER, you can use your voice to let  
ROSMASTER navigate to point 1/2/3 on an established map. The R2 button on the handle can  
cancel/enable this function at any time. .
7.2. Start  
7.2.1. Function package path  
7.2.2. Calibrate the target point for voice navigation  
robot side
<PI5 needs to open another terminal to enter the same docker container
[use_rviz] Parameter: whether to open rviz.
[map] Parameters: map name, map to be loaded.#Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
~/yahboomcar_ws/src/yahboomcar_voice_ctrl/
roslaunch yahboomcar_nav laser_bringup.launch #laser + yahboomcar
roslaunch yahboomcar_nav yahboomcar_navigation.launch use_rviz:=false map:=house 
#Open navigation, change house to the map name for mapping
Turn on the visual interface (virtual machine side)
1. In the map rviz, adjust the initial posture of ROSMASTER;
  2), terminal input
3. Use the 2D Nav Goal tool in rviz to give the car a target point in rviz, recorded as position 1. At  
this time, the coordinates of the target point will be printed out in the terminal window where  
you just viewed the data of /move_base_simple/goal. ,As shown below,
Just record the pose part of the data. Later, you need to send this part manually through the  
program.
4. Open ~/yahboomcar_ws/src/yahboomcar_voice_ctrl/scripts/voice_Ctrl_send_mark.py, and  
modify the pose data just recorded to the corresponding location.
In addition to the two BC points, the position is also calibrated first, and then the calibrated value  
is modified into the function as an actual parameter.
7.2.2. Voice navigation  roslaunch yahboomcar_nav view_navigate.launch
rostopic echo /move_base_simple/goal
  pose.pose.position.x = 2.15381097794
  pose.pose.position.y = -5.02386903763
  pose.pose.orientation.z = 0.726492681307
  pose.pose.orientation.w = 0.687174202082
roslaunch yahboomcar_nav laser_bringup.launch #laser + yahboomcar
roslaunch yahboomcar_nav yahboomcar_navigation.launch use_rviz:=false map:=house 
#Open navigation, change house to the map name for mapping
python ~/yahboomcar_ws/src/yahboomcar_voice_ctrl/scripts/voice_Ctrl_send_mark.py
After calibrating the initial pose in rviz, say "Hello, Xiaoya" to ROSMASTER to wake up the voice  
module. After hearing the voice module's feedback broadcast of "is", say to ROSMASTER "Navigate  
to position 1"; the voice module There will be a feedback message saying "Okay, heading to  
location 1". The same goes for other locations, as long as the coordinate values of the destinations  
of other points are written in the program.
7.3. Core code analysis  
7.3.1. For the principle of navigation, please refer to the tutorial " 11.  
Lidar \ 12. Navigation and Obstacle Avoidance "  The content of this  
section is mainly about judging the speech recognition results and  
packaging and sending 
target point data,
#Pack target point data
         speech_r = spe.speech_read()
         if speech_r == 19 :
             print("goal to one")
             spe.void_write(speech_r)
             pose.pose.position.x = 2.15381097794
             pose.pose.position.y = -5.02386903763
             pose.pose.orientation.z = 0.726492681307
             pose.pose.orientation.w = 0.687174202082
             pub_goal.publish(pose)
         elif speech_r == 20 :
             print("goal to show")
             spe.void_write(speech_r)
             pose.pose.position.x = 1.57744419575
             pose.pose.position.y = 4.8174996376
             pose.pose.orientation.z = -0.683335654604
             pose.pose.orientation.w = 0.730104364558
             pub_goal.publish(pose)
            
         elif speech_r == 21 :
             print("goal to three")
             spe.void_write(speech_r)
             pose.pose.position.x = -1.08106160164
             pose.pose.position.y = 1.30198049545
             pose.pose.orientation.z = -0.0132771070267
             pose.pose.orientation.w = 0.99991185533
             pub_goal.publish(pose)
            
         elif speech_r == 32 :
             print("goal to four")
             spe.void_write(speech_r)
             pose.pose.position.x = -1.08106160164
             pose.pose.position.y = 1.30198049545
             pose.pose.orientation.z = -0.0132771070267
             pose.pose.orientation.w = 0.99991185533
             pub_goal.publish(pose)
         elif speech_r == 33 :
             print("goal to Origin")
Function words Speech recognition module results Voice broadcast content
Navigate to location 1 19 OK, heading to location 1
Navigate to location 2 20 OK, heading to location 2
Navigate to No. 3 21 OK, heading to No. 3
Navigate to No. 4 32 OK, heading to No. 4
Return to origin 33 OK, returning to origin7.3.2. Program flow chart  
For complete code, please refer to:
7.3.3. Function module communication table               spe.void_write(speech_r)
             pose.pose.position.x = -1.08106160164
             pose.pose.position.y = 1.30198049545
             pose.pose.orientation.z = -0.0132771070267
             pose.pose.orientation.w = 0.99991185533
             pub_goal.publish(pose)
         elif speech_r == 0 :
             pub_cmdVel.publish(Twist())S
~/yahboomcar_ws/src/yahboomcar_voice_ctrl/scripts/voice_Ctrl_send_mark.py

---

## 0.How to use old voice module in new image.pdf

How to use old voice module in new image  
1. Tutorial Notes  
As of September 2025, all our rosmaster car images have been updated with the latest voice  
interaction module. If you would like to use the old secondary microphone interaction module  
with the new image, please refer to this tutorial.
2. New/Old Voice Interaction Module  
The image below shows the new voice interaction module. Be careful not to confuse them.
The picture below is the old two-microphone interaction module.
3. Steps to Convert Old Second Microphones to Compatible
with the New Image 
Note: The driver library environment for the new voice interaction module is already  
installed and set up at the factory. If you wish to revert to the old voice interaction module,  
please refer to the following tutorial
1. You can find the corresponding Python driver library for the voice module in the source code  
summary and reinstall it.
The driver library for the old voice interaction module is py_install_V0.0.1
The driver library corresponding to the new voice interaction module is py_install_V0.0.2
2. Python Driver Library Installation Steps
You can find the compressed package py_install_V0.0.1.rar in the source code section of the  
course materials. Place this package in the user directory of the corresponding board. Open a  
terminal and unzip it. (If the ROS environment is installed and running in a Docker container, you  
will need to place this package in the Docker container for decompression and installation.)
The image above shows a successful decompression.
Then install it on your system using the following command:
Enter the user password and press Enter to confirm. If you see the installation prompt  
Speech_Lib=x.x.x version number, it means the installation is successful.sudo apt install unrar
unrar x py_install_V0.0.1.rar
cd py_install_V0.0.1/
sudo python3 setup.py install
 
 
 
 

---



## 3. ROS Control
# Ros Control Manual

## 1. PID algorithm theory.pdf

1 PID algorithm theory  
1 PID algorithm theory 
1.1 algorithm introduction 
1.1.1 Scale part 
1.1.2 Integral part 
1.1.3 Differential part 
1.2 Selection of PID algorithm 
1.2.1 Position PID algorithm 
1.2.2 Incremental PID algorithm 
1.2.3 the difference between incremental and positional 
1.3 Debugging of PID parameters 
1.3.1 Determine the proportional coefficient Kp 
1.3.2 Determine the integral time constant Ti 
1.3.3 Determine the differential time constant Td 
1.3.4 System no-load and on-load joint debugging 
1.1 algorithm introduction  
PID is to perform proportional, integral and derivative operations on the input deviation, and the  
superposition results of the operations are used to control the actuator. The formula is as follows:  
$u(t)=K_p[e(t)+\frac{1}{T_i}\int_{0}^{t}e(t)dt+Td\frac{de(t)}{dt}]$  
It consists of three parts:  
P is the ratio, which is the input deviation multiplied by a coefficient;  
I is the integral, which is the integral operation of the input deviation;  
D is the derivative, and the input deviation is differentiated.  
The following figure shows a basic PID controller:  
1.1.1 Scale part  
The mathematical expression for the proportional part is: $K_p*e(t)$  
In an analog PID controller, the role of the proportional link is to react instantaneously to  
deviations. Once the deviation occurs, the controller will immediately take control, so that the  
control amount changes in the direction of reducing the deviation. The strength of the control  
effect depends on the proportionality coefficient. The larger the proportionality coefficient, the  
stronger the control effect, the faster the transition process and the smaller the static deviation of  
the control process; however, the larger the proportionality factor, the easier it is to generate  
oscillations and destroy the stability of the system. sex. Therefore, the selection of case  
coefficients must be appropriate, so that the transition time is small, the static difference is small  
and stable.  
Advantages: Adjust the open-loop proportional coefficient of the system, improve the steady-state  
accuracy of the system, reduce the inertia of the system, and speed up the response speed.  
Disadvantage: Only use P controller, too large open-loop proportional coefficient will not only  
increase the overshoot of the system, but also make the stability margin of the system smaller or  
even unstable.  
1.1.2 Integral part  
The mathematical expression of the integral part is: $\frac{K_p}{T_i}\int_{0}^{t}e(t)dt$  The larger  
the integral constant, the weaker the accumulation effect of the integral. At this time, the system  
is in transition. No oscillation will occur; however, increasing the integral constant will slow down  
the process of eliminating static errors, and it will take longer to eliminate deviations, but it can  
reduce overshoot and improve system stability. When Ti is small, the integral effect is stronger,  
and oscillation may occur in the transition time of the system, but the time required to eliminate  
the deviation is short. Therefore, Ti must be determined according to the specific requirements of  
actual control.  
Advantage: Eliminates steady-state errors.  
Disadvantage: The addition of the integral controller will affect the stability of the system and  
reduce the stability margin of the system.  
1.1.3 Differential part  
The mathematical expression of the differential part is: $K_p*Td\frac{de(t)}{dt}$  
The role of the differential link prevents the variation of the deviation. It is controlled according to  
the change trend(change speed) of the deviation. The faster the deviation changes, the greater  
the output of the differential controller, and can be corrected before the deviation becomes  
larger. The introduction of differential action will help to reduce overshoot, overcome oscillation  
and stabilize the system, especially for high-order systems, which speed up the tracking speed of  
the system. However, the function of the differential is very sensitive to the noise of the input  
signal. Generally, the differential is not used for those systems with large noise, or the input signal  
is filtered before the differential works. The action of the differential part is determined by the  
differential time constant Td.  The larger the Td, the stronger the effect of suppressing the  
deviation change; the smaller the Td, the weaker the effect of resisting the deviation change. The  
differential part obviously has a great effect on system stability. Appropriate selection of the  
differential constant Td can optimize the differential action.  
Advantages: Make the response speed of the system faster, reduce the overshoot, reduce the  
oscillation, and have a "prediction" effect on the dynamic process.  
1.2 Selection of PID algorithm  
Digital PID control algorithms can be divided into position PID and incremental PID control  
algorithms. So before we decide to choose which PID algorithm to use, we should first understand  
its principle:  
1.2.1 Position PID algorithm  
$u(k)=K_pe(k)+K_I\sum_{i=o}e(i)+K_D[e(k)-e(k-1)]$
e(k): the value set by the user(target value) - the current state value of the control object  
Proportion P : e(k)  
Integral I : Accumulation of ∑ e(i) errors  
Differential D : e(k) - e(k-1) this time error - last time error  
That is, the position PID is the actual position of the current system, the deviation from the  
expected position you want to achieve, and PID control is performed.  
Because there is an error integral ∑ e(i), which is always accumulated, that is, the current output  
u(k) is related to all the past states, and the accumulated value of the error is used;(error e will  
have error accumulation), the output u(k) corresponds to the actual position of the actuator. Once  
the control output is wrong(the current state value of the control object has a problem), a large  
change in u(k) will cause a large change in the system and the positional PID will reach the integral  
term. When saturated, the error will still continue to accumulate under the integral action. Once  
the error starts to change in the reverse direction, the system needs a certain time to exit from  
the saturation region. Therefore, when u(k) reaches the maximum and minimum, the integral  
action should be stopped and the integral should be required. Limiting and output limiting So  
when using position PID, we generally use PD control directly  
The positional PID is suitable for objects with no integral components in the actuator, such as the  
control of the upright and temperature control systems of the steering gear and the balance  
trolley  
Advantages: Position PID is a non-recursive algorithm, which can directly control the  
actuator(such as the balance car), the value of u(k) and the actual position of the actuator(such as  
the current angle of the car) are in one-to-one correspondence, so Works well in objects with  
actuators without integral components  
Disadvantages: Each output is related to the past state, e(k) must be accumulated during  
calculation, and the computational workload is large.  
1.2.2 Incremental PID algorithm  
$\Delta u(k)=u(k)-u(k-1)=K_P[e(k)-e(k-1)]+K_ie(k)+K_D[e(k)-2e(k-1)+e(k-2)]$  
Proportion P : $e(k)-e(k-1)$ this time error - last time error  
Integral I : $e(k)$ bias
Differential D : $e(k)-2e(k-1)+e(k-2)$ This time error-2*last error + last time error  
Incremental PID can be well seen according to the formula, once KP, TI, TD are determined, as  
long as the deviation of the three measured values before and after is used, the control amount  
$\Delta u(k)$ corresponds to the increment of the position error in recent times, not the deviation  
from the actual position. There is no error accumulation. That is to say, accumulation is not  
required in incremental PID. The determination of the control increment $\Delta u(k)$ is only  
related to the last three sampling values, and it is easy to obtain a better control effect through  
weighted processing, and when a problem occurs in the system, the incremental method will not  
seriously affect the system. Work  
Summary: Incremental PID is to increment the positional PID. At this time, the controller outputs  
the difference between the position values calculated at two adjacent sampling times, and the  
result obtained is an increment, that is, in the last control On the basis of the amount, you need to  
increase(negative value means decrease) the control amount.  
 advantage:  
①The influence of malfunction is small, and the error data can be removed by the method of  
logical judgment if necessary.  
②The impact of manual/automatic switching is small, which is convenient to realize disturbance-
free switching. When the computer fails, the original value can still be maintained.  
③ There is no need to accumulate in the formula. The determination of the control increment  
Δu(k) is only related to the last three sampling values.  
shortcoming:  
① The integral truncation effect is large, and there is a steady-state error;  
②The influence of overflow is great. For some controlled objects, it is not very good to use the  
incremental method;  
1.2.3 the difference between incremental and positional  
(1) The incremental algorithm does not need to do accumulation, the determination of the control  
amount increment is only related to the recent deviation sampling values, and the calculation  
error has little influence on the control amount calculation. The positional algorithm uses the  
accumulated value of past deviations, which is prone to large accumulated errors.  
(2) The incremental algorithm obtains the increment of the control quantity. For example, in the  
valve control, only the change part of the valve opening is output, and the influence of  
misoperation is small. If necessary, the output can be limited or prohibited by logical judgment.,  
will not seriously affect the work of the system. The positional output directly corresponds to the  
output of the object, so it has a greater impact on the system.  
(3) The output of incremental PID control is the increment of the control amount, and there is no  
integral action, so this method is suitable for objects with integral components of the actuator,  
such as stepper motors, etc., while the positional PID is suitable for the actuator without Objects  
that integrate components, such as electrohydraulic servo valves.  
(4) When performing PID control, the positional PID needs to have integral limiter and output  
limiter, while the incremental PID only needs to output the limiter  
Positional PID and incremental PID are just two implementations of digital PID control algorithms,  
and they are essentially the same. The main difference is that the integral items are stored in  
different ways. The positional PID integral items are stored separately, and the incremental PID  
integral items are stored as part of the output. Various players on the Internet also have their own  
unique opinions on the use of positional and incremental. Opinion, it depends on which algorithm  
is suitable for our specific application scenario.  
1.3 Debugging of PID parameters  
There are many methods for PID controller parameter selection, such as trial and error method,  
critical proportionality method, and extended critical proportionality method. However, for PID  
control, the selection of parameters is always a very complicated task, which requires continuous  
adjustment to obtain a more satisfactory control effect. Based on experience, the general steps  
for PID parameter determination are as follows:  
1.3.1 Determine the proportional coefficient Kp  
When determining the proportional coefficient Kp, first remove the integral term and differential  
term of the PID, so that Ti=0 and Td=0, making it a pure proportional adjustment. The input is set  
to 60% to 70% of the maximum allowable output of the system, and the proportional coefficient  
Kp gradually increases from 0 until the system oscillates; and vice versa, the proportional  
coefficient Kp decreases gradually from this time until the system oscillation disappears. Record  
the proportional coefficient Kp at this time, and set the proportional coefficient Kp of the PID to  
be 60% to 70% of the current value.  
1.3.2 Determine the integral time constant Ti  
After the proportional coefficient Kp is determined, set a larger integral time constant Ti, and then  
gradually decrease Ti until the system oscillates, and then reversely increase Ti until the system  
oscillation disappears. Record the Ti at this time, and set the integral time constant Ti of the PID to  
be 150% to 180% of the current value.  
1.3.3 Determine the differential time constant Td  
Differential time constant Td generally does not need to be set, it can be set to 0. At this time, PID  
adjustment is converted into PI adjustment. If it needs to be set, it is the same as the method for  
determining Kp, taking 30% of its value when it is not oscillating.  
1.3.4 System no-load and on-load joint debugging  
Fine-tune the PID parameters until the performance requirements are met.  
Of course, this is just my personal debugging method, which is not necessarily suitable for  
everyone and every environment. It is only provided for your reference; however, there are also  
classic PID debugging formulas circulating on the Internet, and I also post them for your  
reference:  
Parameter tuning to find the best, in order from small to large.  
First proportional, then integral, and finally add the differential.  
The curve oscillates very frequently, and the proportional dial should be enlarged.  
The curve floats around the big bend, and the proportional dial turns to the small turn.  
When the curve deviates, the recovery is slow, and the integration time decreases.  
The curve fluctuation period is longer, and the integration time is longer.  
The oscillation frequency of the curve is fast, so reduce the differential first.  
If the momentary difference is large, the fluctuation will be slow, and the differentiation time  
should be lengthened.  
The ideal curve has two waves, with the front high and the back low four to one.  
One look at the second adjustment and more analysis, the adjustment quality will not be low.  
PID is proportional(P), integral(I), and differential(D) control algorithms. It is not necessary to have  
these three algorithms at the same time. It can also be PD, PI, or even only P algorithm control.  
One of the simplest ideas for closed-loop control I used to have was only P control. The current  
result is fed back and subtracted from the target. If it is positive, it will decelerate, and if it is  
negative, it will accelerate. Of course, this is just the simplest closed-loop control algorithm, that  
is, go back to the positional and incremental summary in the previous section. For details, please  
refer to our current control environment. Because of the differences of each control system, the  
parameters that can make our system achieve the most stable effect are of course OK.  

---

## 1.ROS introduction.pdf

1.ROS introduction  
1.ROS introduction
1.1 Main features of ROS
1.2 Overall architecture of ROS
1.2.1 Calculation graph level
1.2.2 File system level
1.2.3  Ope n source commu nity level
1.3 Commu nication mechanism
1.3.1 、 Topic
1.3.2 、 Service
1.3.3 、 Action
1.4 Commo n compo nents
1.5 Release version
ROS wiki : http://wiki.ros.org/
ROS (Robot Operating System, referred to as "ROS") is an open source operating system suitable  
for robots. It provides the services that an operating system should have, including hardware  
abstraction, low-level device control, implementation of common functions, inter-process  
message passing, and package management. It also provides the tools and library functions  
needed to obtain, compile, write, and run code across computers.
The main goal of ROS is to provide support for code reuse for robotics research and  
development. ROS is a framework of distributed processes (also known as "nodes") that are  
encapsulated in packages and function packages that are easy to share and publish. ROS also  
supports a federated system similar to a code repository, which can also enable project  
collaboration and release. This design allows the development and implementation of a project to  
be completely independent from the file system to the user interface (not restricted by ROS). At  
the same time, all projects can be integrated by ROS basic tools.
1.1 Main features of ROS  
(1) Distributed architecture (each working process is regarded as a node and is managed  
uniformly using the node manager),
(2) Multi-language support (such as C++, Python, etc.),
(3) Good scalability (you can write one node, or organize many nodes into a larger project through  
roslaunch),
(4) Open source code (ROS follows the BSD protocol and is completely free for individual and  
commercial applications and modifications).
1.2 Overall architecture of ROS  
Open source community level: mainly includes developer knowledge, code, and algorithm  
sharing.
File system level: used to describe the code and executable programs that can be found on the  
hard disk.
Computational graph level: reflects the communication between processes and processes and  
systems.
1.2.1 Calculation graph level  
Node
Nodes are the main computing execution processes. ROS is composed of many nodes. After  
multiple nodes are started, you can use the following command to view the topic  
communication between each node.
Information
Nodes realize logical connections and data exchange with each other through messages.
Topic (theme)
Topics are a way of delivering messages (publish/subscribe). Each message must be  
published to the corresponding topic, and each topic is strongly typed. ROS topic messages  
can be transmitted using TCP/IP or UDP. The default transmission method used by ROS is  
TCP/IP. Transmission based on TCP is called TCPROS, which is a long connection method;  
transmission based on UDP is called UDPROS, which is a low-latency, high-efficiency  
transmission method, but it is easy to lose data and is suitable for remote operations.
Services
Services are used in the request-reply model and must also have a unique name. When a  
node provides a service, all nodes can communicate with it through code written using ROS  
clients.
Message record packagerqt_graph
File system level
Comprehensive functional package
Functional package
Function package list Message Service Code OthersMessage recording package is a file format used to save and playback ROS message data,  
which is saved in .bag file. It is an important mechanism for storing data.
Parameter server
The parameter server is a shared multi-variable dictionary accessible over the network,  
stored by key on the node manager.
Node Manager (Master)
The node manager is used for registration and search of topics and service names, etc. If  
there is no node manager in the entire ROS system, there will be no communication between  
nodes.
1.2.2 File system level  
Dependencies can be configured between function packages. If function package A depends on  
function package B, then when building the system in ROS, B must be built earlier than A, and A  
can use the header files and library files in B.
The file system level concepts are as follows:
Function pack list:
This list indicates the dependencies of the function package, source file compilation flag  
information, etc. The package.xml file in the function package is a function package list.
Function pack:
Function packages are the basic form of software organization in the ROS system, including  
running nodes and configuration files.
Comprehensive feature package
Several functional packages are organized together to form a comprehensive functional  
package.
Message type
When sending messages between nodes in ROS, a message description is required in  
advance. ROS provides standard types of messages, which can also be defined by yourself.  
The description of the message type is stored in the msg file under the function package.
Service type
Defines the data structure provided by each process in the ROS system regarding service  
requests and responses.
1.2.3 Open source community level  
Distribution: ROS distribution is a series of comprehensive function packages that can be  
installed independently and have version numbers. ROS distributions serve a similar role as  
Linux distributions. This makes it easier to install ROS software and maintain consistent  
versions through a collection of software.
Software repository (Repository): ROS relies on websites or hosting services that share open  
source code and software libraries, where different organizations can publish and share  
their own robot software and programs.
ROS Wiki: The ROS Wiki is the primary forum for recording information about the ROS  
system. Anyone can register an account, contribute their own files, provide corrections or  
updates, write tutorials, and other actions.
Bug Ticket System: If you find a problem or want to propose a new feature, ROS provides this  
resource to do this.
Mailing list: The ROS user mailing list is the main communication channel about ROS. It can  
exchange various questions or information from ROS software updates to ROS software use  
like a forum.
ROS Answer: Users can use this resource to ask questions
1.3 Communication mechanism  
1.3.1 、 Topic  
The asynchronous publish-subscribe communication mode is widely used in ros. Topic is  
generally used for one-way, message flow communication. Topics generally have strong type  
definitions: a topic of one type can only accept/send messages of a specific data type (message  
type). Publisher is not required to have type consistency, but when accepting, subscriber will  
check the md5 of the type and report an error.
1.3.2 、 Service  
Service is used to handle synchronous communication in ros communication, using server/client  
semantics. Each service type has two parts: request and response. For the server in the service,  
ros will not check the name conflict. Only the last registered server will take effect and establish a  
connection with the client.
Features Topic Service Action
Response
mechanismNone Result responseProgress response,
result response
Synchronicity Asynchronous Synchronous Asynchronous
Communication
modelPublisher ，SubscriberClient ，ServerClient ，Server
Node
correspondenceMany to manyMany （ Client ） to
one （ Server ）Many （ Client ） to
one （ Server ）
1.3.3 、 Action  
Actions are composed of multiple topics and are used to define tasks. Task definitions include  
goals (Goal), task execution process status feedback (Feedback), and results (Result). Compiling  
action will automatically generate 7 structures: Action, ActionGoal, ActionFeedback, ActionResult,  
Goal, Feedback, and Result structures.
Features of Action:
A question and answer communication mechanism
With continuous feedback
Can be terminated during a mission
Implementation of message mechanism based on ROS
Action interface:
goal: publish task goal
cancel: Request to cancel the task
status: Notify the client of the current status
feedback: Monitoring data for periodic feedback task running
result: Send the execution result of the task to the client and only publish it once.
Comparison of communication mode features
1.4 Common components  
launch startup file; TF coordinate transformation; Rviz; Gazebo; QT toolbox; Navigation; MoveIt!
launch ： Launch File is a way to start multiple nodes at the same time in ROS.It can also  
automatically start the ROS Master node manager, and can implement various configurations of  
each node, providing great convenience for the operation of multiple nodes.
TF coordinate transformation: There are often a large number of component elements in the  
robot body and the robot's working environment. The positions and postures of different  
components are involved in robot design and robot applications. TF is a function package that  
allows users to track multiple coordinate systems over time , which uses a tree data structure to  
buffer and maintain coordinate transformation relationships between multiple coordinate  
systems based on time, which can help developers complete coordinate transformations such as  
points and vectors between coordinate systems at any time.
QT toolbox: In order to facilitate visual debugging and display, ROS provides a Qt-based  
background graphics tool suite - rqt_common_plugins,It contains many practical tools: log output  
tool (rqt_console), calculation graph visualization tool (rqt_graph), data drawing tool (rqt_plot),  
parameter dynamic configuration tool (rqt_reconfigure)
Rviz: rviz is a 3D visualization tool that is well compatible with various robot platforms based on  
the ROS software framework. In rviz, you can use XML to describe the size, quality, position,  
material, joints and other attributes of any physical objects such as robots and surrounding  
objects, and present them in the interface. At the same time, rviz can also display robot sensor  
information, robot motion status, changes in the surrounding environment, etc. in a graphical  
manner in real time.
Gazebo: Gazebo is a powerful three-dimensional physics simulation platform with a powerful  
physics engine, high-quality graphics rendering, convenient programming and graphics interfaces,  
and most importantly, it is open source and free. Although the robot model in Gazebo is the same  
as the model used by rviz, the physical properties of the robot and the surrounding environment,  
such as mass, friction coefficient, elastic coefficient, etc., need to be added to the model. The  
robot's sensor information can also be added to the simulation environment in the form of plug-
ins and displayed in a visual manner.
Navigation: Navigation is a two-dimensional navigation function package of ROS. Simply put, it  
calculates safe and reliable robot speed control instructions through the navigation algorithm  
based on the information flow of the input odometer and other sensors and the global position of  
the robot.
Moveit ： Moveit ！ Function package is the most commonly used tool package, mainly used for  
trajectory planning. Move it! The configuration assistant is used to configure some files that need  
to be used in planning, which is very important.
1.5 Release version  
Reference link ：http://wiki.ros.org/Distributions
ROS distribution refers to the version of the ROS software package.The concept of it and Linux  
distribution version (such as Ubuntu ) similar.The purpose of a ROS distribution is to allow  
developers to work with a relatively stable codebase until they are ready to upgrade  
everything.Therefore, after each release version is launched, ROS developers usually only fix the  
bugs in this version and provide a small number of improvements to the core software  
Version nameRelease
dateVersion
lifecycleOperating system platform
ROS Noetic
NinjemysMay 2020 May 2023 Ubuntu 20.04
ROS Melodic
MoreniaMay 23,
2018May 2023Ubuntu 17.10, Ubuntu 18.04, Debian 9,
Windows 10
ROS Lunar
LoggerheadMay 23,
2017May 2019Ubuntu 16.04, Ubuntu 16.10, Ubuntu
17.04,Debian 9
ROS Kinetic
KameMay 23,
2016April 2021 Ubuntu 15.10, Ubuntu 16.04, Debian 8
ROS Jade
TurtleMay 23,
2015May 2017Ubuntu 14.04, Ubuntu 14.10, Ubuntu
15.04
ROS Indigo
IglooJuly 22, 2014 April 2019 Ubuntu 13.04, Ubuntu 14.04
ROS Hydro
MedusaSeptember
4, 2013May 2015Ubuntu 12.04, Ubuntu 12.10, Ubuntu
13.04
ROS Groovy
GalapagosDecember
31, 2012July 2014Ubuntu 11.10, Ubuntu 12.04, Ubuntu
12.10
ROS Fuerte
TurtleApril 23,
2012--Ubuntu 10.04, Ubuntu 11.10, Ubuntu
12.04
ROS Electric
EmysAugust 30,
2011--Ubuntu 10.04, Ubuntu 10.10, Ubuntu
11.04, Ubuntu 11.10
ROS
DiamondbackMarch 2,
2011--Ubuntu 10.04, Ubuntu 10.10, Ubuntu
11.04
ROS C TurtleAugust 2,
2010--Ubuntu 9.04, Ubuntu 9.10, Ubuntu
10.04, Ubuntu 10.10
ROS Box TurtleMarch 2,
2010--Ubuntu 8.04, Ubuntu 9.04, Ubuntu 9.10,
Ubuntu 10.04package.As of October 2019, the version names, release times and version life cycles of major  
ROS releases are as shown in the following table:
 

---

## 2. Robot PID debugging.pdf

2. Robot PID debugging  
Note: The PID has been debugged before the product leaves the factory. It is not  
recommended to adjust it yourself, as it may cause problems with subsequent functions.  
But you can learn
Learn debugging methods and operating procedures. 
According to different models, just set the purchased model in [.bashrc], X1 (normal four-
wheel drive) X3 (Mailun) X3plus (Mailun robotic arm) R2 (Ackerman differential) etc. , this  
section takes X3 as an example: 
Find the [ROBOT_TYPE] parameter and modify the corresponding model
2.1. Start the chassis  
2.1.1. Function package path  
2.1.2. Start  
1), first check the source code Mcnamu_driver.py
Find the line self.car.set_pid_param and remove the comment # in front of it to debug the robot's  
PID.
2. First select the debugging parameters. After entering the name of each parameter as shown  
in the figure below, click [+] to add, [-] to delete.sudo vim .bashrc
export ROBOT_TYPE=X3 # ROBOT_TYPE: X1 X3 X3plus R2 X7
#Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
~/yahboom_ws/src/yahboom_bringup/
roslaunch yahboomcar_bringup bringup.launch #robotchassisstart
rosrun rqt_reconfigure rqt_reconfigure # Parameter adjuster
rqt_plot # rqt visualization tool
rosrun yahboomcar_ctrl yahboom_keyboard.py #Keyboard control node
def dynamic_reconfigure_callback(self, config, level):
         # self.car.set_pid_param(config['Kp'], config['Ki'], config['Kd'])
         # print("PID: ", config['Kp'], config['Ki'], config['Kd'])
         self.linear_max = config['linear_max']
         self.linear_min = config['linear_min']
         self.angular_max = config['angular_max']
         self.angular_min = config['angular_min']
         return config
/cmd_vel/angular/z: target angular velocity
/cmd_vel/linear/x: Linear velocity in the target x direction
/cmd_vel/linear/y: Linear velocity in the target y direction
/vel_raw/angular/z: Current angular velocity
-/vel_raw/linear/x: current linear velocity in the x direction
-/vel_raw/linear/y: current linear velocity in the y direction
Note: The speed in the y direction is only available for the Mecanum wheel car X1
3. In the rqt_reconfigure window, as shown in the figure, select the [driver_node] node. We only  
need to adjust the three parameters [Kp], [Ki], and [Kd], and do not adjust the others.
Debugging steps:
Use the keyboard keys on the keyboard control terminal to drive the car to move [forward],  
[backward], [turn left], and [turn right]
Observe the changes in the [rqt_plot] window image so that the current angular velocity is  
close to the target angular velocity, and the current linear velocity is close to the target linear  
velocity. It is impossible to overlap. The target speed is in an ideal state without any  
interference, and the speed can be increased instantly.
Use the keyboard [q], [z], [w], [x], [e], [c] to increase or decrease the speed and test the status  
of different speeds. [t] switches the car linear speed X/Y axis direction, [s] stops keyboard  
control. Note that the linear velocity range of the car's XY axis is v_x=[-1.0, 1.0], v_y=
[-1.0, 1.0], and the angular velocity range is v_z=[-5.0, 5.0]. 
Observe the changes of [rqt_plot], adjust [Kp], [Ki], and [Kd] data through [rqt_reconfigure],  
test multiple times, and select the optimal data.
Keyboard control instructions ( Before keyboard control, you need to click with the mouse to  
open the keyboard control terminal before you can control it )
Directional control
[i] or [I] [linear, 0] [u] or [U] [linear, angular]
[,] [-linear, 0] [o] or [O] [linear, - angular]
[j] or [J] [0, angular] [m] or [M] [- linear, - angular]
[l] or [L] [0, - angular] [.] [- linear, angular]
Button Speed change Button Speed change
[q]Linear speed and angular  
speed are both increased by  
10%[z]Linear speed and angular  
speed are both reduced by  
10%
[w]Only the linear speed  
increases by 10%[x]Only the linear speed  
decreases by 10%
[e]Only the angular velocity  
increases by 10%[c]Only the angular velocity  
decreases by 10%
[t]Line speed X-axis/Y-axis  
direction switching[s] Stop keyboard controlspeed control
After debugging, the PID is automatically stored in the PCB. Just comment out the PID setting part  
according to the way you originally viewed the source code.
Note: Other gameplay requires PID debugging, the theory is the same, theoretical  
reference. 

---

## 2.ROS installation.pdf

2.ROS installation  
Select the corresponding ros version according to the Ubuntu system. For the corresponding  
relationship, please refer to [1. ROS Introduction]-[1.5. Release Version] to select.This course takes  
the installation of ROS-Noetic  on Ubuntu20.04  as an example to explain how to install ros.
2.1 Set ros source  
Terminal input,
2.2  Set the key  
Terminal input,
2.3 Update source  
Terminal input,
2.4  Install ROS-Noetic  
What is installed here is the basic desktop version, terminal input,
2.5 Set up ROS environment  
Add the path of ROS to the environment variable so that when you open the terminal in the  
future, you can find the running environment of ROS and enter it in the terminal.
Then reopen the terminal or source to refresh the environment variables, and enter in the  
terminal,#Raspberry Pi 5 master control, ROS tutorials are all used in the DOCKER 
container, just follow the ROS/07 and docker usage tutorials.
#ubuntu23.10 has not updated ROS yet. The host machine cannot install the ROS 
environment for the time being. Waiting for subsequent updates...
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" 
> /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key 
C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop -y
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
2.6 Verification  
Terminal input,
If the following screen appears, the installation can be successful.
roscore

---

## 3. Install the Rosmaster driver library.pdf

3. Install the Rosmaster driver library  
3. Install the Rosmaster driver library 
3.1. Declaration before installing the driver library 
3.2. Download the Python driver library file 
3.3. Transfer files to Jetson Nano 
3.4. start the installation 
3.5. impo rt library file 
3.6. Basic usage of driver library 
3.7. Introduction to API 
3.1. Declaration before installing the driver library  
The latest driver library has been installed in the factory mirror system of the car, so there is no  
need to install it repeatedly. If you are not using the factory image, or if the driver library has  
updated content, you only need to install the driver library.  
The driver library storage path that comes with the factory system: ~/Software/py_install  
For the method of installing the driver library, please refer to the following steps. Here, the  
installation of version V1.5.8 is used as an example.  
3.2. Download the Python driver library file  
The latest version of the Rosmaster Python driver library is available in this course material,  
named py_install.zip.  
The compressed package contains the following files:  
3.3. Transfer files to Jetson Nano  
If you use the browser that comes with Jetson Nano to download the file, please download the file  
to a user-operable path, such as the desktop.  
If you use the driver library zip file in the data, or download the driver library file with a computer  
browser, you can drag the driver library zip file into the Jetson Nano desktop through WinSCP  
software.  
After the installation is successful, the driver library file can be deleted.  
 
3.4. start the installation  
Open the terminal of Jetson Nano and enter the following command to unzip it.  
Go to the desktop and check if the file exists, the red box is the target file  
unzip files  
Note: The entire documentation routine takes the py_install.zip compressed package placed on  
the desktop of the Jetson Nano system as an example. If you store the compressed package in a  
different path, please enter the corresponding directory according to the actual path to operate.  
Go to the driver library folder  
Run the installation command and see the installation version number indicated at the end,  
indicating that the installation is successful. This command will overwrite the Rosmaster_Lib  
driver library that has been installed before.  cd  ~/Desktop &&  ls 
unzip py_install.zip 
cd  py_install 
sudo  python3 setup.py install 
 
3.5. import library file  
The name of the Rosmaster driver library is Rosmaster_Lib, and Rosmaster_Lib is used in the  
program to import the library.  
 
3.6. Basic usage of driver library  
Check out the video tutorial for this lesson for details.  
Source code path: Rosmaster/Samples/3.test_rosmaster.ipynb  
 
3.7. Introduction to API  
The following is an introduction to the API of the driver library, and the function usage and  
parameter content will be introduced in the control course later.  from  Rosmaster_Lib  import  Rosmaster 
|  __del __ (self) 
| 
 |  __init__(self, car_type=1, com='/dev/myserial', delay=0.002, debug=False) 
 |      Initialize self.  See help(type(self)) for accurate signature. 
| 
 |  clear_auto_report_data(self) 
| # Clear the cached data automatically sent by the microcontroller 
 |      # Clear the cache data automatically sent by the MCU 
| 
 |  create_receive_threading(self) 
| # Start a thread for receiving and processing data 
 |      # Start the thread that receives and processes data 
| 
 |  get_accelerometer_data(self) 
| # Get the three-axis data of the accelerometer, return a_x, a_y, a_z 
 |      # Get accelerometer triaxial data, return a_x, a_y, a_z 
| 
 |  get_battery_voltage(self) 
| # Get the battery voltage value 
 |      # Get the battery voltage 
| 
 |  get_gyroscope_data(self) 
| # Get the three-axis data of the gyroscope, return g_x, g_y, g_z 
 |      # Get the gyro triaxial data, return g_x, g_y, g_z 
| 
 |  get_magnetometer_data(self) 
| # Get the magnetometer three-axis data, return m_x, m_y, m_z 
| 
 |  get_motion_data(self) 
| # Get the speed of the car, val_vx, val_vy, val_vz 
 |      # Get the car speed, val_vx, val_vy, val_vz 
| 
 |  get_motion_pid(self) 
| # Get the motion PID parameters of the car, return [kp, ki, kd] 
 |      # Get the motion PID parameters of the dolly and return [kp, ki, kd] 
| 
 |  get_motor_encoder(self) 
| # Get four-way motor encoder data 
 |      # Obtain data of four-channel motor encoder 
| 
 |  get_uart_servo_angle(self, s_id) 
| # Read the angle of the bus servo, s_id indicates the ID number of the servo to 
be read, s_id=[1-6] 
 |      # Read the Angle of the bus steering gear, s_id indicates the ID number 
of the steering gear to be  
read, s_id=[1-6] 
| 
 |  get_uart_servo_angle_array(self) 
| # Read the angles of six servos at one time [xx, xx, xx, xx, xx, xx], if a 
servo is wrong, the bit is -1 
 |      # Read the angles of three steering gear [xx, xx, xx, xx, xx, xx] at one 
time. If one steering gear 
 is wrong, that one is -1 
| 
 |  get_uart_servo_value(self, servo_id) 
| # Read the bus servo position parameter, servo_id=[1-250], return: read ID, 
current position parameter 
 |      # Read bus servo position parameters, servo_id=[1-250], return: read ID, 
current position parameter 
s 
| 
 |  get_version(self) 
| # Get the version number of the underlying MCU, such as 1.1 
 |      # Get the underlying microcontroller version number, such as 1.1 
| 
 |  reset_flash_value(self) 
| # Reset the data saved in the car's flash and restore the factory default 
values. 
 |      # Reset the car flash saved data, restore the factory default value 
| 
 |  set_auto_report_state(self, enable, forever=False) 
| # The MCU automatically returns the data status bit, the default is ON, if it 
is set to OFF, it will affect part of the data read function. 
| # enable=True, the underlying expansion board will send a packet of data every 
10 milliseconds, a total of four packets of different data, so each packet of 
data is refreshed every 40 milliseconds.  e 
nable=False, do not send. 
| # forever=True for permanent storage, =False for temporary use. 
 |      # The MCU automatically returns the data status bit, which is enabled by 
default. If the switch is  
closed, the data reading function will be affected.   
 |      # enable=True, The underlying expansion board sends four different 
packets of data every 10 millise 
conds, so each packet is refreshed every 40 milliseconds.  
 |      # If enable=False, the report is not sent.   
 |      # forever=True for permanent, =False for temporary 
| 
 |  set_beep(self, on_time) 
| # Buzzer switch, on_time=0: off, on_time=1: keep ringing, 
| # on_time>=10: Automatically turn off after xx milliseconds (on_time is a 
multiple of 10). 
 |      # Buzzer switch. On_time =0: the buzzer is off. On_time =1: the buzzer 
keeps ringing 
 |      # On_time >=10: automatically closes after xx milliseconds (on_time is a 
multiple of 10) 
| 
 |  set_car_motion(self, v_x, v_y, v_z) 
| # Car motion control, v_x=[-1.0, 1.0], v_y=[-1.0, 1.0], v_z=[-5.0, 5.0] 
 |      # Car movement control, v_x = [-1.0, 1.0], v_y = [-1.0, 1.0], v_z = 
[-5.0, 5.0] 
| 
 |  set_car_run(self, state, speed, adjust=False) 
| # Control the car to move forward, backward, left, right, etc. 
| # state=[0~6],=0 stop,=1 forward,=2 backward,=3 left,=4 right,=5 left,=6 right
 
| # speed=[-100, 100],=0 to stop. 
| # adjust=True to enable the gyroscope to assist the movement direction.  =False 
to disable. 
 |      # Control the car forward, backward, left, right and other movements. 
 |      # State =[0~6],=0 stop,=1 forward,=2 backward,=3 left,=4 right,=5 spin 
left,=6 spin right 
 |      # Speed =[-100, 100], =0 Stop. 
 |      # Adjust =True Activate the gyroscope auxiliary motion direction.  If 
=False, the function is disab 
led. 
| 
 |  set_car_type(self, car_type) 
| # Set the car type 
 |      # Set car Type 
| 
 |  set_colorful_effect(self, effect, speed=255, parm=255) 
| # RGB programmable light strip special effects display. 
| # effect=[0, 6], 0: stop light effect, 1: running water light, 2: marquee 
light, 3: breathing light, 4: gradient light, 5: starlight, 6: battery display 
| # speed=[1, 10], the smaller the value, the faster the speed changes. 
| # parm, optional, as an additional parameter. Usage 1: Enter [0, 6] for the 
breathing light effect to modify the color of the breathing light. 
 |      # RGB programmable light band special effects display. 
 |      # Effect =[0, 6], 0: stop light effect, 1: running light, 2: running 
horse light, 3: breathing ligh 
t, 4: gradient light, 5: starlight, 6: power display  
 |      # Speed =[1, 10], the smaller the value, the faster the speed changes 
 |      # Parm, left blank, as an additional argument.  Usage 1: The color of 
breathing lamp can be modifie 
d by the effect of breathing lamp [0, 6] 
| 
 |  set_colorful_lamps(self, led_id, red, green, blue) 
| # RGB programmable light strip control, which can be controlled individually or 
as a whole. Before the control, you need to stop the RGB light effects. 
| # led_id=[0, 13], control the corresponding number of RGB lights; led_id=0xFF, 
control all lights. 
| # red,green,blue=[0, 255], representing the color RGB value. 
 |      # RGB programmable light belt control, can be controlled individually or 
collectively, before contr 
ol need to stop THE RGB light effect. 
 |      # Led_id =[0, 13], control the CORRESPONDING numbered RGB lights;  Led_id 
=0xFF, controls all light 
s. 
 |      # Red,green,blue=[0, 255], indicating the RGB value of the color. 
| 
 |  set_motor(self, speed_1, speed_2, speed_3, speed_4) 
| # Controls the motor PWM pulses, thus controlling the speed (not using the 
encoder to measure the speed).  speed_X=[-100, 100] 
 |      # Control PWM pulse of motor to control speed (speed measurement without 
encoder). speed_X=[-100, 1 
00] 
| 
 |  set_pid_param(self, kp, ki, kd, forever=False) 
| # PID parameter control will affect the change of the motion speed of the car 
controlled by the set_car_motion function. By default it can be left unadjusted.
 
| # kp ki kd = [0, 10.00], you can enter decimals. 
| # forever=True for permanent storage, =False for temporary use. 
| # Since the permanent storage needs to be written into the chip flash, the 
operation time is long, so add the delay time to avoid the problem of packet loss 
caused by the microcontroller. 
| # Temporary action has a quick response and is valid for one time, and the data 
is no longer maintained after restarting the single chip. 
 |      # PID parameter control will affect the set_CAR_motion function to 
control the speed change of the  
car.  This parameter is optional by default.   
,  # KP ki kd = [0, 10.00] 
 |      # forever=True for permanent, =False for temporary.   
 |      # Since permanent storage needs to be written into the chip flash, which 
takes a long time to opera 
te, delay is added to avoid packet loss caused by MCU.   
 |      # Temporary effect fast response, single effective, data will not be 
maintained after restarting th 
e single chip 
| 
 |  set_pwm_servo(self, servo_id, angle) 
| # Servo control, servo_id: corresponding to the ID number, angle: corresponding 
to the angle value of the servo 
 |      # servo_id=[1, 4], angle=[0, 180] 
 |      # Servo control, servo_id: corresponding, Angle: corresponding servo 
Angle value 
| 
 |  set_pwm_servo_all(self, angle_s1, angle_s2, angle_s3, angle_s4) 
| # Control the angle of four PWM channels at the same time, angle_sX=[0, 180] 
 |      # At the same time control four PWM Angle, angle_sX=[0, 180] 
| 
 |  set_uart_servo(self, servo_id, pulse_value, run_time=500) 
| # Control the bus servo.  servo_id:[1-255], indicates the ID number of the 
servo to be controlled, when id=254, it controls all connected servos. 
| # pulse_value=[96,4000] indicates the position to which the servo should run. 
| # run_time indicates the running time (ms), the shorter the time, the faster 
the servo rotates. Minimum is 0, maximum is 2000 
 |      # Control bus steering gear.  Servo_id :[1-255], indicating the ID of the 
steering gear to be contr 
olled. If ID =254, control all connected steering gear.   
 |      # pulse_value=[96,4000] indicates the position to which the steering gear 
will run.   
 |      # run_time indicates the running time (ms). The shorter the time, the 
faster the steering gear rota 
tes.  The minimum value is 0 and the maximum value is 2000 
| 
 |  set_uart_servo_angle(self, s_id, s_angle, run_time=500) 
| # Set the bus servo angle interface: id:7-9, angle: 7:[0, 225], 8:[30, 270], 9:
[30, 180], set the angle to which the servo should move 
. 
| # Set the vertical and upward clamping state, the three servos are all 180 
degrees, the 7/8th turn clockwise (down) to decrease, counterclockwise (up) to 
increase, and the clamp is released 
To reduce, clamp to increase. 
| # run_time indicates the running time (ms), the shorter the time, the faster 
the servo rotates. Minimum is 0, maximum is 2000 
 |      # Set bus steering gear Angle interface: ID :7-9, Angle :7 :[0, 225], 8:
[30, 270], 9:[30, 180], set 
 steering gear to move to the Angle.   
 |      # Set up the vertical clamping state, the three steering gear are 180 
degrees, 7/8 clockwise (down) 
 to decrease, counterclockwise (up) to increase, clip release to decrease, 
clamping to increase.   
 |      # run_time indicates the running time (ms). The shorter the time, the 
faster the steering gear rota 
tes.  The minimum value is 0 and the maximum value is 2000 
| 
 |  set_uart_servo_angle_array(self, angle_s=[90, 90, 90, 90, 90, 180], 
run_time=500) 
| # Control the angles of all servos of the robotic arm at the same time. 
 |      # Meanwhile, the Angle of all steering gear of the manipulator is 
controlled 
| 
 |  set_uart_servo_id(self, servo_id) 
| # Set the ID number of the bus servo, servo_id=[1-250]. 
| # Please make sure to connect only one bus servo before running this function, 
otherwise all connected bus servos will be set to the same ID, resulting in 
control confusion 
. 
 |      # Set the bus servo ID, servo_id=[1-250].   
 |      # Before running this function, please confirm that only one bus actuator 
is connected. Otherwise,  
all connected bus actuators will be set to the same ID, resulting in confusion of 
control 
| 
 |  set_uart_servo_offset(self, servo_id) 
| # Set the median deviation of the robotic arm, servo_id=0~6, =0 all restore the 
factory default value 
 |      # Run the following command to set the mid-bit deviation of the 
manipulator: servo_id=0 to 6, =0 Re 
store the factory default values 
| 
 |  set_uart_servo_torque(self, enable) 
| # Turn off/on bus servo torque, enable=[0, 1]. 
| # enable=0: Turn off the steering gear torque, you can turn the steering gear 
by hand, but the command cannot control the rotation; 
| # enable=1: Turn on the torque force, the command can control the rotation, and 
the servo cannot be turned by hand. 
 |      # Turn off/on the bus steering gear torque force, enable=[0, 1].   
 |      # enable=0: Turn off the torque force of the steering gear, the steering 
gear can be turned by hand 
, but the command cannot control the rotation;   
 |      # enable=1: Turn on torque force, command can control rotation, can not 
turn steering gear by hand 

---

## 3.ROS common command tools.pdf

3.ROS common command tools  
We introduced the four important concepts of nodes, messages, topics, and services in [1.  
Introduction to ROS] - [1.2.1. Computational graph level]. During the running process of ROS, it is  
often necessary to use some commands to debug the program. This section introduces the  
common debugging commands of ros.
3.1 Node rosnode  
When developing and debugging, you often need to view the current node and node information,  
so please remember these common commands. If you can't remember it, you can also check the  
rosnode command usage through rosnode help.
Terminal input,
3.2 Message rosmsg  
msg is equivalent to a data type. You can check which msg is available in the current ros  
environment. Some common usages are as follows:rosnode
Terminal input,
3.3 Topic rostopic  
Topics are one of the commonly used communication methods between nodes in ros. Rostopic  
provides tools for printing topic messages (equivalent to subscribers), tools for publishing  
messages (equivalent to publishers), and other tools.It is convenient for us to debug and check  
whether the subscriber or publisher of the node subscribes and publishes the message normally.  
The common usage is as follows:
Terminal input,rosmsg
rostopic
/topic represents the name of the topic. For example, if you need to output the /hello message of  
the topic name, enter it in the terminal.
3.4 Service rosservice  
Service is also one of the commonly used communication methods between nodes in ros. It is  
different from the topic in that it has a response value, that is, node A requests node B to provide  
services. After B provides services, There needs to be a response value (response), which can be  
understood as a return value. Commonly used usages are as follows:
Terminal input,
rostopic echo /hello
rosservice
Similarly, /service here is also replaced based on the actual service name. For example, when  
displaying the service information of /add_two_ints of the service name, enter it in the terminal,
 rosservice info /add_two_ints

---

## 4.ROS workspace.pdf

4.ROS workspace  
The ros workspace is a folder used to store ros function packages. It usually ends with ws. Let's  
create a space named ros_ws as an example to explain how to create a ros workspace.
4.1 Create workspace folder  
Take creating the ros_ws space in the ~ directory as an example, and enter in the terminal,
4.2 Create src to store function packages  
We create a src folder in the src directory of the workspace to store the function package created  
later, and enter it in the terminal.
4.3 Initialize workspace  
Terminal input,
4.4 Compile workspace  
Use the catkin_make command to compile the functions in the entire workspace. You need to  
compile in the workspace directory ，cd
mkdir ros_ws
cd ~/ros_ws
mkdir src
cd ~/ros_ws/src
catkin_init_workspace
cd ~/ros_ws
catkin_make
4.5 Add workspace to environment variables  
Add the workspace to the environment variable so that you can find the function package when  
you open the terminal. Otherwise, you need to source the workspace every time to find the  
function package and related programs. Enter in the terminal.
Enter the following command to refresh the environment variables or reopen the terminal to take  
effect. Enter in the terminal,
 echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

---

## 5. Control PWM servo.pdf

5. Control PWM servo  
5. Control PWM servo 
5.1. Experimental objectives 
5.2. Experiment preparation 
5.3. Experimental effect 
5.4. Program source code 
5.1. Experimental objectives  
Control the rotation of the PWM servo on the robot.  
 
5.2. Experiment preparation  
The position in the red box in the picture below is the interface of the PWM servo, including one  
servo voltage switching interface and four servo interfaces. The jumper cap can be inserted into  
the servo voltage switching interface to select 5V or 6.8V voltage. If the jumper cap is not inserted,  
the PWM servo cannot be controlled. The black interface of the servo interface is GND, the red  
interface is the positive pole of the 5V power supply, and the yellow interface is the signal.  
The servo interface must be inserted according to the color, and cannot be inserted backwards.  
Rosmaster_Lib library functions that PWM servo gimbal needs to use:  
Parameter explanation: servo control, servo_id: corresponding ID number: S1 = 1, S2 = 2, S3 = 3,  
S4 = 4, angle: corresponding to the angle value of the servo  
servo_id=[1, 4] ， angle=[0, 180]  
Return value: None.  
 
Parameter explanation: control the angle of four PWM channels at the same time, angle_sX=[0,  
180]  
Return value: None.  
 
5.3. Experimental effect  
Check out the course accompanying video.  
 
5.4. Program source code  
Power on the Rosmaster robot, and open the browser of the Jetson Nano or remote computer to  
enter the Jupyter lab editor.  
Reference code path: Rosmaster/Samples/5.pwm_servo.ipynb  set_pwm_servo ( servo_id ,  angle ) 
set_pwm_servo_all ( angle_s1 ,  angle_s2 ,  angle_s3 ,  angle_s4 ) 

---

## 5. Opencv application.pdf

5. OpenCV application  
5. Ope nCV application 
5.1. Overview 
5.2. QR code 
5.2.1 Introduction of QR code 
5.2.2 The structure of QR code 
5.2.3. Features of QR code 
5.2.4. QR code creation and recognition 
5.3. Human Pose Estimation 
5.3.1. Overview 
5.3.2. Principle 
5.3.3. Start 
5.4, target detection 
5.4.1. Model structure 
5.4.2. code analysis 
5.4.3. Start 
5.1. Overview  
OpenCV is a cross-platform computer vision and machine learning software library released  
under the BSD license (open source) and can run on Linux, Windows, Android and MacOS  
operating systems.  [1] It is lightweight and efficient - it consists of a series of C functions and a  
small number of C++ classes, and provides interfaces in languages such as Python, Ruby, and  
MATLAB, and implements many general algorithms in image processing and computer vision.  
5.2. QR code  
5.2.1 Introduction of QR code  
QR code is a kind of two-dimensional barcode. QR comes from the abbreviation of "Quick  
Response" in English, which means quick response. It comes from the inventor's hope that QR  
code can make its content quickly decoded.  QR code not only has large information capacity, high  
reliability and low cost, but also can represent various text information such as Chinese  
characters and images. It has strong confidentiality and anti-counterfeiting and is very convenient  
to use. What's more, the QR code technology is open source.  
5.2.2 The structure of QR code  
picture Parse
Positioning  markings indicate the direction of the QR code.
Alignment  markings If the QR code is large, these additional elements help with
positioning.
pattern  With these lines, the scanner can identify how big the matrix is.
Version information  (Version information) here specifies the version number of
the QR code in use. There are currently 40 different version numbers of the QR
code. Version numbers for the sales industry are usually 1-7.
Format  information Format patterns contain information about fault tolerance
and data mask patterns and make scanning codes easier.
Data  and error correction keys These modes hold the actual data.
Quiet  zone This zone is very important for the scanner, its role is to separate
itself from the surrounding.
5.2.3. Features of QR code  
The data values in the QR code contain duplicate information (redundant values). Therefore, even  
up to 30% of the QR code structure is destroyed without affecting the readability of the QR code.   
The storage space of the QR code is up to 7089 bits or 4296 characters, including punctuation  
marks and special characters, can be written into the QR code. In addition to numbers and  
characters, words and phrases (such as URLs) can also be encoded. As more data is added to the  
QR code, the code size increases and the code structure becomes more complex.  
5.2.4. QR code creation and recognition  
Source path: ~/yahboomcar_ws/src/yahboomcar_visual/simple_qrcode  
Install  
create  
Create qrcode object  python3 -m pip install qrcode pyzbar 
sudo apt-get install libzbar-dev 
qrcode QR code to add logo  
Note: When using Chinese, you need to add Chinese characters  
Identify      '''
    参 数 含 义 ：
    version ： 值 为 1~40 的 整 数 ， 控 制 二 维 码 的 大 小 （ 最 小 值 是 1 ， 是 个 12×12 的 矩 阵 ） 。
             如 果 想 让 程 序 自 动 确 定 ， 将 值 设 置 为  None 并 使 用  fit 参 数 即 可 。
    error_correction ： 控 制 二 维 码 的 错 误 纠 正 功 能 。 可 取 值 下 列 4 个 常 量 。
    ERROR_CORRECT_L ： 大 约 7% 或 更 少 的 错 误 能 被 纠 正 。
    ERROR_CORRECT_M （ 默 认 ） ： 大 约 15% 或 更 少 的 错 误 能 被 纠 正 。
    ROR_CORRECT_H ： 大 约 30% 或 更 少 的 错 误 能 被 纠 正 。
    box_size ： 控 制 二 维 码 中 每 个 小 格 子 包 含 的 像 素 数 。
    border ： 控 制 边 框 （ 二 维 码 与 图 片 边 界 的 距 离 ） 包 含 的 格 子 数 （ 默 认 为 4 ， 是 相 关 标 准 规 定 的 最 小 值 ）
    '''
    qr = qrcode.QRCode( version=1, 
error_correction=qrcode.constants.ERROR_CORRECT_H, box_size=5, border=4,)
    # If the logo address exists, add the logo image 
    my_file = Path(logo_path)
    if my_file.is_file(): img = add_logo(img, logo_path)
Just use the python3 + py file to execute, then enter the content to be 
generated, and press Enter to confirm. 
def  decodeDisplay ( image ,  font_path ): 
    gray  =  cv . cvtColor ( image ,  cv . COLOR_BGR2GRAY ) 
Effect demonstration  
    # You need to convert the output Chinese characters into Unicode encoding 
first 
    barcodes  =  pyzbar . decode ( gray ) 
    for  barcode  in  barcodes : 
        # Extract the position of the bounding box of the QR code 
        ( x ,  y ,  w ,  h ) =  barcode . rect 
        # draw the bounding box of the barcode in the image 
        cv . rectangle ( image , ( x ,  y ), ( x  +  w ,  y  +  h ), ( 225 ,  0 
,  0 ),  5 ) 
        encoding  =  'UTF-8' 
        # To draw it, you need to convert it to a string first 
        barcodeData  =  barcode . data . decode ( encoding ) 
        barcodeType  =  barcode . type 
        # Plot the data and type on the image 
        pilimg  =  Image . fromarray ( image ) 
        # create brush 
        draw  =  ImageDraw . Draw ( pilimg )  
        # Parameter 1: font file path, parameter 2: font size 
        fontStyle  =  ImageFont . truetype ( font_path ,  size = 12 ,  encoding 
= encoding ) 
        # Parameter 1: print coordinates, parameter 2: text, parameter 3: font 
color, parameter 4: font 
        draw . text (( x ,  y  -  25 ),  str ( barcode . data ,  encoding ), 
 fill =( 255 ,  0 ,  0 ),  font = fontStyle ) 
        # PIL image to cv2 image 
        image  =  cv . cvtColor ( np . array ( pilimg ),  cv . COLOR_RGB2BGR ) 
        # Print barcode data and barcode type to terminal 
        print ( "[INFO] Found {} barcode: {}" . format ( barcodeType , 
 barcodeData )) 
    return  image 
Just use python3 + py file to execute it 
5.3. Human Pose Estimation  
Source path: ~/yahboomcar_ws/src/yahboomcar_visual/detection  
5.3.1. Overview  
Human Posture Estimation (Human Posture Estimation) is to estimate the human posture by  
correctly linking the detected human key points in the picture. The key points of the human body  
usually correspond to joints with a certain degree of freedom on the human body, such as neck,  
shoulder, elbow, wrist, waist, knee, ankle, etc., as shown in the figure below.  
5.3.2. Principle  
Input an image, extract features through a convolutional network, and obtain a set of feature  
maps, which are then divided into two forks, and use the CNN network to extract Part Confidence  
Maps and Part Affinity Fields respectively;  after obtaining these two information, we use the  
graph theory in Bipartite Matching (even matching) Find the Part Association, connect the joint  
points of the same person, due to the vector nature of the PAF itself, the resulting bipartite  
matching is very correct, and finally merged into the overall skeleton of a person;  Finally, based  
on PAFs, Multi- Person Parsing—>Convert the Multi-person parsing problem into a graphs  
problem—>Hungarian Algorithm (Hungarian Algorithm)  (The Hungarian algorithm is the most  
common algorithm for partial graph matching. The core of the algorithm is to find an  
augmentation path. An algorithm for finding the maximum matching of a bipartite graph with a  
wide path.)  
5.3.3. Start  
After clicking on the image box, use the keyboard [f] key to toggle target detection.  
input image  cd ~/yahboomcar_ws/src/yahboomcar_visual/detection 
python target_detection.py 
if  action == ord ( 'f' ) or  action == ord ( 'F' ): state = not  state   # 
function switch 
output image  
5.4, target detection  
The main problem in this section is how to use the dnn module in OpenCV to import a trained  
target detection network. But there are requirements for the version of opencv.  
At present, there are three main methods for target detection using deep learning:  
Faster R-CNNs  
You Only Look Once(YOLO)  
Single Shot Detectors(SSDs)  
Faster R-CNNs are the most commonly heard of deep learning based neural networks. However,  
this approach is technically difficult to understand (especially for deep learning newbies), difficult  
to implement, and difficult to train.  
In addition, even using the "Faster" method to implement R-CNNs (where R stands for Region  
Proposal), the algorithm is still relatively slow, about 7FPS.  
If we are after speed, we can turn to YOLO, because it is very fast, can reach 40-90 FPS on  
TianXGPU, and the fastest version may reach 155 FPS. But the problem with YOLO is that its  
accuracy has yet to be improved.  
SSDs were originally developed by Google and can be said to be a balance between the above  
two. Compared to Faster R-CNNs, its algorithm is more straightforward. Compared with YOLO, it  
is more accurate.  
5.4.1. Model structure  
The main work of MobileNet is to replace the past standard convolutions (standard convolutions)  
with depthwise sparable convolutions (depth-level separable convolutions) to solve the problems  
of computational efficiency and parameter size of convolutional networks. The MobileNets model  
is based on depthwise sparable convolutions (depth-level separable convolutions), which can  
decompose standard convolutions into a depthwise convolution and a point convolution (1 × 1  
convolution kernel). Depthwise convolution applies each kernel to each channel, while 1 × 1  
convolution is used to combine the outputs of channel convolutions.  
Batch Normalization (BN) is added to the basic components of MobileNet, that is, at each SGD  
(stochastic gradient descent), the standardization process is performed so that the mean of the  
result (each dimension of the output signal) is 0 and the variance is 1. Generally, you can try BN to  
solve the problem that the convergence speed is very slow or the gradient explosion cannot be  
trained during the training of the neural network. In addition, in general use cases, BN can also be  
added to speed up the training speed and improve the model accuracy.  
In addition, the model also uses the ReLU activation function, so the basic structure of the  
depthwise separable convolution is shown in the following figure:  
The MobileNets network is composed of many depthwise separable convolutions shown above.  
Its specific network structure is shown in the following figure:  
5.4.2. code analysis  
List of recognized objects  
Load the category [object_detection_coco.txt], import the model [frozen_inference_graph.pb], and  
specify the deep learning framework [TensorFlow]  [person, bicycle, car, motorcycle, airplane, bus, train,
 truck, boat, traffic light, fire hydrant, street sign,
 stop sign, parking meter, bench, bird, cat, dog, horse,
 sheep, cow, elephant, bear, zebra, giraffe, hat, backpack,
 umbrella, shoe, eye glasses, handbag, tie, suitcase,
 frisbee, skis, snowboard, sports ball, kite, baseball bat,
 baseball glove, skateboard, surfboard, tennis racket,
 bottle, plate, wine glass, cup, fork, knife, spoon, bowl,
 banana, apple, sandwich, orange, broccoli, carrot, hot dog,
 pizza, donut, cake, chair, couch, potted plant, bed, mirror,
 dining table, window, desk, toilet, door, tv, laptop, mouse,
 remote, keyboard, cell phone, microwave, oven, toaster,
 sink, refrigerator, blender, book, clock, vase, scissors,
 teddy bear, hair drier, toothbrush]
Import the image, extract the height and width, calculate the 300x300 pixel blob, and pass this  
blob into the neural network  
5.4.3. Start  
After clicking the image frame, use the keyboard [f] key to switch the human pose  
estimation.  with open('object_detection_coco.txt', 'r') as f: class_names = 
f.read().split('\n')
COLORS = np.random.uniform(0, 255, size=(len(class_names), 3))
model = cv.dnn.readNet(model='frozen_inference_graph.pb', 
config='ssd_mobilenet_v2_coco.txt', framework='TensorFlow')
def  Target_Detection ( image ): 
    image_height ,  image_width ,  _  =  image . shape 
    # create blob from image 
    blob  =  cv . dnn . blobFromImage ( image = image ,  size =( 300 ,  300 ), 
 mean =( 104 ,  117 ,  123 ),  swapRB = True ) 
    model . setInput ( blob ) 
    output  =  model . forward () 
    # iterate over each detection 
    for  detection  in  output [ 0 ,  0 , :, :]: 
        # Extract the confidence of the detection 
        confidence  =  detection [ 2 ] 
        # Draw bounding box only if detection confidence is above a certain 
threshold, skip otherwise 
        if  confidence  >  .4 : 
            # Get the class ID 
            class_id  =  detection [ 1 ]  
            # map class id to class 
            class_name  =  class_names [ int ( class_id )  -  1 ] 
            color  =  COLORS [ int ( class_id )] 
            # Get bounding box coordinates 
            box_x  =  detection [ 3 ]  *  image_width 
            box_y  =  detection [ 4 ]  *  image_height 
            # Get the width and height of the bounding box 
            box_width  =  detection [ 5 ]  *  image_width 
            box_height  =  detection [ 6 ]  *  image_height 
            # draw a rectangle around each detected object 
            cv . rectangle ( image , ( int ( box_x ),  int ( box_y )), ( int ( 
box_width ),  int ( box_height )),  color ,  thickness = 2 ) 
            # Write the class name text on the detected object 
            cv . putText ( image ,  class_name , ( int ( box_x ),  int ( box_y 
 -  5 )),  cv . FONT_HERSHEY_SIMPLEX ,  1 ,  color ,  2 ) 
    return  image 
cd ~/yahboomcar_ws/src/yahboomcar_visual/detection 
python target_detection.py 
if action == ord('f') or action == ord('F'):state = not state   # function switch 

---

## 5.ROS function package.pdf

5.ROS function package  
The function package is a folder used to store each node program. This section explains how to  
create a function package in the src directory of the workspace ros_ws created in the previous  
section.
5.1 Create function package  
The function package is stored in the src folder under the workspace folder. Enter the following  
command to create it. Take creating the learn_topic function package as an example. Terminal  
input,
Explain what the command line consists of:
catkin_create_pkg ： create command
learn_topic ： the name of the function package
roscpp 、 rospy 、 geometry_msgs turtlesim ： This is a dependent library of the function  
package. It can also be added to the CMakeLists.txt file in the function package directory  
when needed later.
5.2 Function package folder description  
There are the following files under the created function package:cd ~/ros_ws/src
catkin_create_pkg learn_topic std_msgs rospy roscpp geometry_msgs turtlesim
include: stores header files that need to be compiled
src: stores the source code files for compiling node programs
CMakeLists.txt: files required for compilation, including declaring which libraries need to be  
connected, which dependencies are required, which programs are generated, etc.
package.xml: declares some dependencies required for compilation and some information  
about the function package, including the function package version, the author of the  
function package creation, etc.
Later, we will learn how to write a node program. There will be a C++ version and a Python  
version. Generally, the C++ version of the program source code is stored in src.The source code of  
the Python version is stored in the scripts folder. This folder is created by yourself under the  
function package path. You can enter the following command to create it,
cd ~/ros_ws/src/learn_topic
mkdir scripts
Including the launch files and customized message files that will be mentioned later, the  
corresponding launch files and msg folders need to be created in the directory of the function  
package for storage.

---

## 6.ROS node.pdf

6.ROS node  
In [1. Introduction to ROS]-[1.2.1. Computational graph level], we introduced the concept of  
nodes. In the previous section, the function package we created is the folder that stores the node  
program. You can use Python or C++ to write the node program, then compile it into an  
executable file, and finally run it as what we call the node program.
6.1 roscore  
Before running all ros programs, you need to start roscore (this is not required when running the  
launch file later, roscore will be started when the launch file is started), enter in the terminal,
Only one roscore can be run. If roscore is started in multiple terminals, it will prompt that roscore  
has been started, as shown in the figure below.
roscore
6.2 rosnode  
After starting roscore, a node program is started. We have introduced several common tools of  
rosnode in the previous [3. ROS common command tools]-[3.1. Node rosnode].We can use  
rosnode list  to view and query all currently running nodes, and enter in the terminal,
Only one node is started here, which is /rosout. This is the node that we run after starting the  
roscore program. You can use rosnode info node_name to view the information of the node.
(node_name represents the node name, modify it according to the actual node name that needs  
to be queried), enter the terminal,
As shown in the figure above, some relevant information about the node will be printed and  
listed, such as:
Posted topics and related data types
Publications:  
/rosout_agg [rosgraph_msgs/Log]
Subscribed topics and related data types
Subscriptions:  
/rosout [unknown type]
Services provided and related data types
Services:  
/rosout/get_loggers
/rosout/set_logger_level
6.3 rosrun  rosnode list
rosnode info /rosout
Rosrun is the command to start the ros node program. The previous roscore is special. You can  
start it by inputting roscore in the terminal. However, most other ros node programs are started  
by rosrun. The command format is as follows:
pkg_name ： function package name, the name of the function package folder of the subsequent  
executable program
executable_program ： The name of the executable program, which can be a file generated by C++  
compilation, or a file written in Python with a .py executable file at the end.
For example, taking the classic little turtle as an example, after starting roscore, we enter in  
another terminal,
After successful startup, a little turtle will appear. We can check the current nodes through  
rosnode list and enter in the terminal.
Compared with the previous /rosout, there is an additional /turtlesim. You can also use the  
rosnode info tool to view node information and enter it in the terminal.rosrun pkg_name executable_program
rosrun turtlesim turtlesim_node 
rosnode list
There will be more content here, but basically the content is similar. The [Publications] section  
explains which topics the node has published and the corresponding topic data types;  
[Subscriptions] section explains which topics the node subscribes to and the corresponding topic  
data types; the [Services] section explains what services the node provides.
Making more use of the rosnode tool to query running nodes and related information is a very  
important point in the process of debugging ros. For example, we wrote a program and ran it  
without problems and reported errors, but the topic communication between nodes did not run  
as we expected. At this time, you can use rosnode info to check whether it is caused by  
inconsistent topic names or other content. Only by running these tools flexibly can the problem  
be solved faster.rosnode info /turtlesim

---

## 7.ROS topic publisher.pdf

7.ROS topic publisher  
As mentioned in the previous section, a node program has both publishing and subscribing  
messages. In this section, we will explain how to declare a publisher in the node program and  
publish topic messages. We proceed based on the previously established workspace ros_ws and  
learn_topic function packages.
7.1 Create a publisher  
General creation steps are as follows:
Initialize ROS nodes
Create handle
Register node information with ROS Master, including the published topic name, message  
type in the topic, and queue length
Create and initialize message data
Send messages cyclically with a certain frequency
7.2 C++ version  
7.2.1 Writing source code  
In the src folder of the function package learn_topic, create a C++ file (the file suffix is .cpp), name  
it turtle_velocity_publisher.cpp, and paste the following content into turtle_velocity_publisher.cpp.
/*Create a small turtle speed publisher*/
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_velocity_publisher");//ROS node initialization
    ros::NodeHandle n;//Here is create handle
    //Create a Publisher, publish a topic named /turtle1/cmd_vel, the message 
type is geometry_msgs::Twist, and the queue length is 10
    ros::Publisher turtle_vel_pub = n.advertise<geometry_msgs::Twist>
    ("/turtle1/cmd_vel", 10);
    ros::Rate loop_rate(10);//Set the frequency of the loop
    while (ros::ok())
    {
        //Initialize the message to be published, the type must be consistent 
with Publisher
        geometry_msgs::Twist turtle_vel_msg;
        turtle_vel_msg.linear.x = 0.8;
        turtle_vel_msg.angular.z = 0.6;
        turtle_vel_pub.publish(turtle_vel_msg);// Publish speed news
        //Print published speed content
        ROS_INFO("Publsh turtle velocity command[%0.2f m/s, %0.2f rad/s]",
        turtle_vel_msg.linear.x, turtle_vel_msg.angular.z);
        loop_rate.sleep();//Delay according to cycle frequency
    }
    return 0;
}
7.2.2 Modify CMakelist.txt file  
Configure in CMakelist.txt, under the build area, add the following content,
add_executable shows that the generated executable program file is turtle_velocity_publisher,  
and the compiled source code is turtle_velocity_publisher.cpp in the src directory.
target_link_libraries specifies the libraries that need to be linked when compiling and generating  
an executable file.
7.2.3 Compile  
Terminal input,
After the compilation is passed, you need to re-source the current environment variables to find  
or update the program. Enter in the terminal.
7.2.4 Running the program  
Open roscore ，
Run the little turtle node program,
Run the publisher node program and continue to send speed to the little turtle.add_executable(turtle_velocity_publisher src/turtle_velocity_publisher.cpp)
target_link_libraries(turtle_velocity_publisher ${catkin_LIBRARIES})
cd ~/ros_ws
catkin_make
cd ~/ros_ws
source devel/setup.bash
roscore
rosrun turtlesim turtlesim_node
rosrun learn_topic turtle_velocity_publisher
As shown in the figure above, after receiving the published message, the little turtle will move at  
the specified speed. We can check which nodes are running through rosnode list and enter in the  
terminal,
/turtle_velocity_publisher is the program we write, compile, and run. The node name here is  
consistent with ros::init(argc, argv, "turtle_velocity_publisher") in the code.
7.2.5 Program flow chart  rosnode list
7.3 Python version  
7.3.1 Writing source code  
Create a new python file (file suffix .py) in the scripts folder under the function package  
learn_topic, name it turtle_velocity_publisher.py, copy and paste the following program code into  
the turtle_velocity_publisher.py file,
#!/usr/bin/env python
# -*- coding: utf-8 -*-
# This routine will publish to the turtle1/cmd_vel topic, message type 
geometry_msgs::Twist
import rospy
from geometry_msgs.msg import Twist
def turtle_velocity_publisher():
    rospy.init_node('turtle_velocity_publisher', anonymous=True) # ROS node 
initialization
The python program does not need to be compiled, but it needs to add executable permissions  
and enter it in the terminal.
7.3.2 Run  
Open roscore,
Run the little turtle node,
Run the publisher node program and continue to send speed to the little turtle.    # Create a small turtle speed publisher and publish a topic named 
/turtle1/cmd_vel. The message type is geometry_msgs::Twist, and 8 represents the 
message queue length.
    turtle_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=8)
    rate = rospy.Rate(10) #Set the frequency of the loop
    while not rospy.is_shutdown():
        # Initialize geometry_msgs::Twist type message
        turtle_vel_msg = Twist()
        turtle_vel_msg.linear.x = 0.8
        turtle_vel_msg.angular.z = 0.6
        # release the news
        turtle_vel_pub.publish(turtle_vel_msg)
        rospy.loginfo("linear is :%0.2f m/s, angular is :%0.2f rad/s",
        turtle_vel_msg.linear.x, turtle_vel_msg.angular.z)
        rate.sleep()# Delay according to cycle frequency
if __name__ == '__main__':
    try:
        turtle_velocity_publisher()
    except rospy.ROSInterruptException:
        pass
cd ~/ros_ws/src/learn_topic/scripts
sudo chmod a+x turtle_velocity_publisher.py
roscore
rosrun turtlesim turtlesim_node
rosrun learn_topic turtle_velocity_publisher.py
7.3.3 Program flow chart  

---

## 8. Control robot movement.pdf

8. Control robot movement  
8. Control robot movement
8.1. Experimental objectives 
8.2. Experiment preparation 
8.3. Experimental effect 
8.4. Program source code 
8.1. Experimental objectives  
Control the forward and backward on the Rosmaster, left and right, and set the relevant  
parameters of the car.  
 
8.2. Experiment preparation  
The motor interface has an anti-reverse connection function, which can be connected to the  
motor using Rosmaster's motor cable.  
The interface line sequence corresponding to the motor is shown in the following figure:  
Rosmaster_Lib library functions required to control the Rosmaster motor:  
Parameter explanation: trolley motion control, this function will read the pulse data of the  
encoder to calculate the speed of the trolley motion. There may be differences according to  
different trolleys. Here is an example of a Mecanum wheel trolley: v_x represents the longitudinal  
speed of the trolley, in m/s, positive forward, negative backward; v_y control represents the lateral  
speed of the trolley, in m/s , positive numbers are left, negative numbers are right; v_z represents  
the rotation speed of the car, the unit is rad/s, positive numbers are left rotation, negative  
numbers are right rotation.  
v_x=[-1.0, 1.0], v_y=[-1.0, 1.0], v_z=[-5.0, 5.0]  
Return value: None.  
 set_car_motion ( v_x ,  v_y ,  v_z ) 
Parameter explanation: PID parameter control will affect the change of the movement speed of  
the car controlled by the set_car_motion function. By default it can be left unadjusted.  
kp ki kd = [0, 10.00], decimals can be entered.  
forever=True save forever, =False temporary effect.  
Permanent storage is to write the data into the Flash of the MCU chip, the data will not be lost  
after restarting, and the writing time is long, so the delay time is added to avoid the problem of  
packet loss caused by the MCU. Temporary action has fast response and is effective once, and the  
data is no longer maintained after restarting the single chip.  
Return value: None.  
 
Parameter explanation: The MCU automatically returns the data status bit, which is enabled by  
default. If it is set to be disabled, it will affect the function of reading data.  
enable=True, the underlying expansion board will send data every 40 milliseconds.  enable=False,  
do not send.  
forever=True save forever, =False temporary effect.  
Return value: None.  
 
Parameter explanation: Clear the cached data automatically sent by the microcontroller.  
Return value: None.  
 
Parameter explanation: reset the data saved in the car's flash and restore the factory default  
value. This function can also be achieved by long pressing the K2 key on the expansion board for  
about 10 seconds.  
Return value: None.  
 
The following functions all return data, which can only be read when create_receive_threading() is  
started normally:  
Parameter Explanation: Get the three-axis data of the accelerometer  set_pid_param ( kp ,  ki ,  kd ,  forever = False ) 
set_auto_report_state ( enable ,  forever = False ) 
clear_auto_report_data () 
reset_flash_value () 
get_accelerometer_data () 
Return statement: a_x, a_y, a_z  
 
Parameter explanation: Get the three-axis data of the gyroscope  
Return value: g_x, g_y, g_z  
 
Parameter explanation: Get the speed of the car, return val_vx, val_vy, val_vz  
Return value: val_vx, val_vy, val_vz  
 
Parameter explanation: Get the motion PID parameters of the car, and return [-1, -1, -1] if the  
error is read.  
Return value: kp, ki, kd  
 
8.3. Experimental effect  
Check out the course accompanying video.  
Note: If you modify a part of the configuration in this course and set it to be permanently saved, it  
will cause an abnormal situation later (for example, set the automatic return data status  
enable=False, and permanently save it as True, so that the data cannot be read normally). Please  
call the reset_flash_value() function, or press and hold the KEY1 key on the expansion board for  
about 10 seconds to restore the factory configuration.  
 
8.4. Program source code  
Power on the Rosmaster robot, and open the browser of the Jetson Nano or remote computer to  
enter the Jupyter lab editor.  
Reference code path: Rosmaster/Samples/8.car_motion.ipynb  get_gyroscope_data () 
get_motion_data () 
get_motion_pid () 

---

## 8. Robot URDF model.pdf

8 Robot URDF model  
8 Robot URDF model 
8.1 URDF overview 
8.1.1 Introduction 
8.1.2 Compo nents 
8.1.3 links 
8.1.4 joints 
8.2 URDF visualization 
According to different models, you only need to set the purchased model in [.bashrc],  
X1(ordinary four-wheel drive) X3(Mike wheel) X3plus(Mike wheel mechanical arm)  
R2(Ackerman differential) and so on. Section takes X3 as an example:  
Find the [ROBOT_TYPE] parameter and modify the corresponding model  
8.1 URDF overview  
Function package reference path: ~/yahboomcar_ws/src/yahboomcar_description  
8.1.1 Introduction  
URDF, the full name of Unified Robot Description Format, translated into Chinese as Unified  
Robot Description Format, is a robot model file described in xml format, similar to DH  
parameters.  
The first line is required for xml, which describes the version information of xml.  
The second line describes the current robot name; all information about the current robot is  
contained in the [robot] tag.  #Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker 
tutorial
~/run_docker.sh
sudo vim .bashrc 
export ROBOT_TYPE=X3   # ROBOT_TYPE: X1 X3 X3plus R2 X7
<?xml  version="1.0" encoding="utf-8"?> 
< robot  name = "yahboomcar" > 
</ robot > 
8.1.2 Components  
1) link, connecting rod, can be imagined as a human arm.  
2) joint, joint, can be imagined as a human elbow.  
The relationship between link and joint: two links are connected by joints.  
8.1.3 links  
1 Introduction  
In the URDF descriptive language, links are used to describe physical properties.  
describe the visual display, <visual>Label.  
describe collision properties, <collision>Label.  
describe physical inertia, <inertial>Labels are not commonly used.  
Links can also describe the link size(size)\color(color)\shape(shape)\inertial matrix(inertial  
matrix)\collision properties(collision properties) etc. Each Link will become a coordinate system.  
2) sample code: ~/yahboomcar_ws/src/yahboomcar_description/urdf/yahboomcar_X3.urdf  
3) label introduction  
origin  
Describes the pose information; xyzThe attribute describes the coordinate position in the  
environment, rpyAttributes describe their own posture.  
mess      < link  name = "front_left_wheel" > 
        < inertial > 
            < origin  xyz = "2.3728E-06 -9.4228E-07 0.00064068"  rpy = "0 0 0" 
/> 
            < mass  value = "0.051543" /> 
            < inertia  1.4597E = -05"  ixy = "-4.7945E-10"  ixz = "-2.4786E-10" 
                     iyy = "1.4598E-05"  iyz = "1.7972E-09"  izz = "2.4267E-05" 
/> 
        </ inertial > 
        < visual > 
            < origin  xyz = "0 0 0"  rpy = "0 0 0" /> 
            < geometry > 
                < mesh  filename = 
"package://yahboomcar_description/meshes/mecanum/front_left_wheel.STL" /> 
            </ geometry > 
            < material  name = "" > 
                < color  rgba = "0.7 0.7 0.7 1" /> 
            </ material > 
        </ visual > 
        < collision > 
            < origin  xyz = "0 0 0"  rpy = "0 0 0" /> 
            < geometry > 
                < mesh  filename = 
"package://yahboomcar_description/meshes/mecanum/front_left_wheel.STL" /> 
            </ geometry > 
        </ collision > 
    </ link > 
Describes the quality of the link.  
inertia  
The inertial reference frame, due to the symmetry of the rotational inertia matrix, only needs  
6 upper triangular elements ixx, ixy, ixz, iyy, iyz, izz as attributes.  
geometry  
The label describes the shape; meshThe main function of the attribute is to load the texture  
file, filenameThe file address of the attribute texture path. The label also includes other  
label descriptions:  
material  
The label describes the material; nameAttributes are required , can be empty, and can be  
repeated. Through the [color] tag in rgbaAttributes to describe red, green, blue, and  
transparency, separated by spaces. The range of colors is [0-1].  
8.1.4 joints  
1 Introduction  
Describe the relationship between two joints, motion position and velocity limits, kinematic and  
dynamic properties.  
Joint Type:  
fixed: fixed joints. Movement is not allowed and acts as a connection.  
continuous: Rotate the joint. It can be rotated continuously, and there is no limit to the  
rotation angle.  
revolute: Rotate the joint. Similar to continuous, there is a limit to the rotation angle.  
prismatic: sliding joints. Move along a certain axis, there is a position limit.  
floating: floating joints. With six degrees of freedom, 3T3R.  
planar: Planar joints. Allows translation or rotation above the plane orthogonal.  
2) sample code  
In the [joint] tag, the name attribute is required , which describes the name of the joint and is  
unique.
In the type attribute of the [joint] tag, fill in the corresponding six joint types.<box size="1 2 3"/>     #box box, describe the length, width and height of 
the box through the size attribute.       
<cylinder length="1.6" radius="0.5"/>    #cylinder is cylindrical, the 
height of the cylinder is described by the `length` property, and the radius 
of the cylinder is described by the `radius` property. 
<sphere radius="1"/>     #sphere is spherical, and the radius of the sphere 
is described by the `radius` attribute. 
< joint  name = "front_right_joint"  type = "continuous" > 
        < origin  xyz = "0.08 -0.0845 -0.0389"  rpy = "-1.5703 0 3.14159" /> 
        < parent  link = "base_link" /> 
        < child  link = "front_right_wheel" /> 
        < axis  xyz = "0 0 1"  rpy = "0 0 0" /> 
        < limit  effort = "100"  velocity = "1" /> 
    </ joint > 
3) label introduction  
origin  
subtab, referring to the rotation joint in parentThe relative position of the coordinate  
system.  
parent,child  
The parent and child sub-labels represent two links to be connected; parent is the reference,  
and child rotates around the praent.  
axis 
The child label indicates which axis(xyz) the corresponding link of the child rotates around  
and the amount of rotation around the fixed axis.  
limit  
The child tag is mainly to limit the child. lowerproperties and upperThe property limits the  
radian range of rotation, effortThe property limits the force range during rotation.  
(positive and negative value, the unit is cattle or N) velocityThe property limits the speed  
at which it turns, in meters/second or m/s.  
mimic  
Describes the relationship of this joint to existing joints.  
safety_controller  
Describe the safety controller parameters. Protect the movement of the robot joints.  
8.2 URDF visualization  
8.2.1. Start  
8.2.2. Sample pictures  
The red axis is the X axis ; the green axis is the Y axis ; the blue axis is the Z axis ; the coordinate  
system formed by the three axes is called the base coordinate system .Adjusting the  
[joint_state_publisher_gui] component can control the rotation of the wheel.  
roslaunch yahboomcar_description display.launch 

---

## 8. ROS+Opencv foundation.pdf

8. ROS+Opencv foundation  
8. ROS+Ope ncv foundation 
8.1. Overview 
8.2. Astra 
8.2.1. Start Astra Camera 
8.2.2. Start the color map subscription node 
8.2.3. Start the depth graph subscription node 
8.2.4. Start color image inversion 
This lesson takes the Astra camera as an example, which is similar to ordinary cameras.  
8.1. Overview  
Wiki ：  http://wiki.ros.org/cv_bridge/ 
Teaching: http://wiki.ros.org/cv_bridge/Tutorials 
Source code: https://github.com/ros-perception/vision_opencv.git 
Feature pack location: ~/yahboomcar_ws/src/yahboomcar_visual 
ROS has already integrated versions above Opencv3.0 during the installation process, so the  
installation configuration hardly needs to be considered too much. ROS transmits images in its  
own sensor_msgs/Image message format and cannot directly process images, but the provided  
[CvBridge] ] Can perfectly convert and be converted image data formats. [CvBridge] is a ROS  
library, equivalent to a bridge between ROS and Opencv. 
Opencv and ROS image data conversion is shown in the following figure: 

topic name type of data
/camera/depth/image_raw sensor_msgs/Image
/camera/depth/image sensor_msgs/Image
/camera/rgb/image_raw sensor_msgs/Image
/camera/depth/image_raw/compressedDepth sensor_msgs/CompressedImageAlthough the installation configuration does not need to be considered too much, the use  
environment still needs to be configured, mainly the two files [package.xml] and [CMakeLists.txt].  
This function package not only uses [CvBridge], but also needs [Opencv] and [PCL], so it is  
configured together. 
package.xml 
Add the following 
[cv_bridge]: Image conversion dependency package. 
CMakeLists.txt 
There are many configuration contents in this file. For details, please refer to the source file. 
8.2. Astra  
8.2.1. Start Astra Camera  
View threads 
You can see a lot of topics, just a few commonly used in this section   < build_depend > sensor_msgs </ build_depend > 
  < build_export_depend > sensor_msgs </ build_export_depend > 
  < exec_depend > sensor_msgs </ exec_depend > 
  < build_depend > std_msgs </ build_depend > 
  < build_export_depend > std_msgs </ build_export_depend > 
  < exec_depend > std_msgs </ exec_depend > 
  < build_depend > cv_bridge </ build_depend > 
  < build_export_depend > cv_bridge </ build_export_depend > 
  < exec_depend > cv_bridge </ exec_depend > 
  < exec_depend > image_transport </ exec_depend > 
#Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
#Multiple ros commands require multiple terminals to be executed in the same 
docker container. Please refer to the tutorials in Sections 07/5 and 5.8.
roslaunch astra_camera astrapro.launch 
rostopic list 
topic name type of data
/camera/rgb/image_raw/compressed sensor_msgs/CompressedImage
Check the encoding format of the topic: rostopic echo +[topic]+encoding, for example 
The topic with [compressed] or [compressedDepth] after the topic is a compressed topic. When  
ROS transmits images, data packets may be lost due to factors such as the network, the running  
speed of the host, the running memory of the host, and the huge amount of video stream data.  
off topic. So there is no way, I can only subscribe to the compressed topic. Open two images at the  
same time to subscribe to different topics for testing. If the device performance is good and the  
network is also good, there will be no change. Otherwise, you will find that the topics after image  
compression will be much smoother. 
rostopic echo /camera/rgb/image_raw/encoding 
rostopic echo /camera/depth/image_raw/encoding 
8.2.2. Start the color map subscription node  
version parameter: optional [py, cpp] different codes have the same effect. 
View Node Graph 
When opening the node graph, there will be dense nodes and relationships between nodes. At  
this time, we use the part linked to the topic [/camera/rgb/image_raw], and  
[/astra_rgb_Image_cpp] is the node we wrote. 
py code analysis roslaunch yahboomcar_visual astra_get_rgb.launch version:=cpp 
rqt_graph 
Create a subscriber: The topic of subscription is ["/camera/rgb/image_raw"], the data type is  
[Image], and the callback function [topic()] 
Use [CvBridge] for data conversion. What should be paid attention to here is the encoding format.  
If the encoding format is incorrect, the converted image will have problems. 
c++ code analysis 
similar to py code 
8.2.3. Start the depth graph subscription node  
View Node Graph 
Opening the node graph will show dense nodes and relationships between nodes. At this time, we  
use the part linked to the [/camera/depth/image_raw] topic.  , [/astra_depth_Image_cpp] is the  
node we wrote. sub = rospy.Subscriber("/camera/rgb/image_raw", Image, topic) 
frame = bridge.imgmsg_to_cv2(msg, "bgr8") 
//Create a receiver.
ros::Subscriber subscriber = n.subscribe<sensor_msgs::Image>
("/camera/rgb/image_raw", 10, RGB_Callback);
// create cv_bridge example 
cv_bridge::CvImagePtr cv_ptr;
// data conversion 
cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
roslaunch yahboomcar_visual astra_get_depth.launch version:=cpp 
rqt_graph 
py code analysis 
Create a subscriber: The topic of subscription is ["/camera/depth/image_raw"], the data type is  
[Image], and the callback function [topic()] 
Use [CvBridge] for data conversion. What should be paid attention to here is the encoding format.  
If the encoding format is incorrect, the converted image will have problems. 
c++ code analysis 
similar to py code sub = rospy.Subscriber("/camera/depth/image_raw", Image, topic)
# Encoding format 
encoding = ['16UC1', '32FC1']
# You can switch between different encoding formats to test the effect 
frame = bridge.imgmsg_to_cv2(msg, encoding[1])
8.2.4. Start color image inversion  
image view 
py code analysis 
Two subscribers and two publishers are created here, one for general image data and one for  
compressed image data. 
1. Create subscribers 
The subscribed topic is ["/camera/rgb/image_raw"], the data type is [Image], and the callback  
function [topic()]. 
The topic of subscription is ["/camera/rgb/image_raw/compressed"], data type  
[CompressedImage], and callback function [compressed_topic()]. 
2. Create a publisher 
The published topic is ["/camera/rgb/image_flip"], data type [Image], queue size [10]. 
The posted topic is ["/camera/rgb/image_flip/compressed"], data type [CompressedImage], queue  
size [10]. //Create a receiver. 
ros::Subscriber subscriber = n.subscribe<sensor_msgs::Image>
("/camera/depth/image_raw", 10, depth_Callback); 
// create cv_bridge example 
cv_bridge::CvImagePtr cv_ptr;
// data conversion 
cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
roslaunch yahboomcar_visual astra_image_flip.launch    
rqt_image_view 
sub_img = rospy.Subscriber("/camera/rgb/image_raw", Image, topic)
pub_img = rospy.Publisher("/camera/rgb/image_flip", Image, queue_size=10)
sub_comimg = rospy.Subscriber("/camera/rgb/image_raw/compressed", 
CompressedImage, compressed_topic)
pub_comimg = rospy.Publisher("/camera/rgb/image_flip/compressed", 
CompressedImage, queue_size=10)
3. Callback function 
 # Normal image transfer processing 
def topic(msg):
    if not isinstance(msg, Image):
        return
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    frame = cv.resize(frame, (640, 480))
    frame = cv.flip(frame, 1)
    # opencv mat ->  ros msg
    msg = bridge.cv2_to_imgmsg(frame, "bgr8")
    pub_img.publish(msg)
    
# Compressed image transmission processing 
def compressed_topic(msg):
    if not isinstance(msg, CompressedImage): return
    bridge = CvBridge()
    frame = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
    frame = cv.resize(frame, (640, 480))
    frame = cv.flip(frame, 1)
    # Create CompressedIamge 
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.data = np.array(cv.imencode('.jpg', frame)[1]).tostring()
    pub_comimg.publish(msg)

---

## 8.ROS topic subscribers.pdf

8.ROS topic subscribers  
The previous section introduced how to write a publisher node program in ros. This section  
introduces how to write a subscriber program.
8.1 Create a subscriber  
The general creation steps are as follows:
Initialize ROS nodes
Create handle
Subscribe to the topics you need
Wait in a loop for topic messages, and enter the callback function after receiving the  
message
Complete message processing in the callback function
The callback function is an important part of the subscriber. It processes the subscribed message,  
then parses the message data, and determines how to proceed with the subsequent program  
based on the parsed message data.
8.2 C++ version  
8.2.1 Writing source code  
In the src folder of the function package learn_topic, create a C++ file (the file suffix is .cpp), name  
it turtle_pose_subscriber.cpp, and paste the following content into turtle_pose_subscriber.cpp,
/*Create a small turtle to receive the current pose information*/
#include <ros/ros.h>
#include "turtlesim/Pose.h"
// After receiving the message, you will enter the message callback function, 
which will process the received data.
void turtle_poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    // Print received messages
    ROS_INFO("Turtle pose: x:%0.3f, y:%0.3f", msg->x, msg->y);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_pose_subscriber");// Initialize ROS node
    ros::NodeHandle n;//Here is create handle
    // Create a subscriber. The topic of subscription is the topic of 
/turtle1/pose. poseCallback is the callback function.
    ros::Subscriber pose_sub = n.subscribe("/turtle1/pose", 
10,turtle_poseCallback);
    ros::spin(); // Loop waiting for callback function
    return 0;
}
8.2.2 Modify CMakelist.txt file  
Configure in CMakelist.txt, under the build area, add the following content,
add_executable shows that the generated executable program file is turtle_pose_subscriber, and  
the compiled source code is turtle_pose_subscriber.cpp in the src directory.
target_link_libraries specifies the libraries that need to be linked when compiling and generating  
an executable file.
8.2.3 Compile  
Terminal input,
After the compilation is passed, you need to re-source the current environment variables to find  
or update the program. Enter in the terminal.
8.2.4 Running the program  
Run roscore
Run the little turtle nodeadd_executable(turtle_pose_subscriber src/turtle_pose_subscriber.cpp)
target_link_libraries(turtle_pose_subscriber ${catkin_LIBRARIES})
cd ~/ros_ws
catkin_make
cd ~/ros_ws
source devel/setup.bash
roscore
rosrun turtlesim turtlesim_node
Run the turtle speed control node
Run the subscriber node to continuously receive the pose data sent by the little turtle.
After all programs are running, click on the terminal running the turtle speed control node and  
press [up, down, left, and right] on the keyboard to control the movement of the turtle. The  
terminal running the subscriber will print the xy coordinate value of the turtle to the terminal in  
real time.As shown below,
We can use the rqt_graph tool to view the communication between nodes and terminal input.
rosrun turtlesim turtle_teleop_key
rosrun learn_topic turtle_pose_subscriber
rqt_graph
The inside of the ellipse represents the node name, the inside of the rectangle represents the  
topic name, and the arrow represents the direction of topic message transmission. The subscriber  
node program turtle_pose_subscriber we wrote is subscribed to the topic message of  
/turtle1/pose.This is consistent with the code ros::Subscriber pose_sub = 
n.subscribe("/turtle1/pose", 10, turtle_poseCallback);, and then in the callback function,  
print the data, the code is as follows,
8.2.5 Program flow chart  
 
8.3 Python version  
8.3.1 Writing source code  
Create a new python file (file suffix .py) in the scripts folder under the function package  
learn_topic, name it turtle_pose_subscriber.py, copy and paste the following program code into  
the turtle_pose_subscriber.py file,
The python program does not need to be compiled, but it needs to add executable permissions  
and enter it in the terminal.void turtle_poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    // Print received messages
    ROS_INFO("Turtle pose: x:%0.3f, y:%0.3f", msg->x, msg->y);
}
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from turtlesim.msg import Pose
def poseCallback(msg):
    rospy.loginfo("Turtle pose: x:%0.3f, y:%0.3f", msg.x, msg.y)
def turtle_pose_subscriber():
    rospy.init_node('turtle_pose_subscriber', anonymous=True)# ROS node 
initialization
    # Create a Subscriber, subscribe to the topic named /turtle1/pose, and 
register the callback function poseCallback
    rospy.Subscriber("/turtle1/pose", Pose, poseCallback)
    rospy.spin()# Loop waiting for callback function
if __name__ == '__main__':
    turtle_pose_subscriber()
cd ~/ros_ws/src/learn_topic/scripts
sudo chmod a+x turtle_pose_subscriber.py
8.3.2 Run  
Open roscore ，
Run the little turtle node,
Run the turtle speed control node,
Run the subscriber node and continue to receive the pose data sent by the little turtle.
After all programs are running, click on the terminal running the turtle speed control node and  
press [up, down, left, and right] on the keyboard to control the movement of the turtle. The  
terminal running the subscriber will print the xy coordinate value of the turtle to the terminal in  
real time.As shown below,
8.3.3 Program flow chart  roscore
rosrun turtlesim turtlesim_node
rosrun turtlesim turtle_teleop_key
rosrun learn_topic turtle_pose_subscriber.py

---

## 9. Control Serial Servo.pdf

Control Serial Servo  
Control Serial Servo 
1. Experimental goal 
2. Experiment preparation 
3. Experimental effect 
4. Program source code 
1. Experimental goal  
Control the movement of the serial port servo on the Rosmaster, control the serial port servo  
through the slider, and read the current angle value of the serial port servo.  
 
2. Experiment preparation  
The position of the red box in the picture below is the interface of the serial servo. This interface  
has the function of anti-reverse connection. There is no need to worry about the problem of  
reverse insertion when using Rosmaster's servo cable.  
For more information about serial servos, please check the following webpages:  
15kg Serial Bus Smart Servo  
The Rosmaster_Lib library functions needed to control the serial port servos have the following  
contents:  
set_uart_servo_angle ( s_id ,  s_angle ,  run_time = 500 ) 
Parameter explanation: Control a serial servo, s_id: corresponding ID number: 1~6, run_time  
controls the running time of the servo, within the valid range, the smaller the time, the faster the  
servo rotates, the unit is milliseconds, and the minimum value is 0.  
s_angle: corresponds to the angle value of the steering gear. The range is different according to  
the different s_id, 1~4:[0, 180], 5:[0, 270], 6:[30, 180]  
Return value: None.  
Parameter explanation: control six serial servos, angle_s controls the angle value of six servos, the  
control range is the same as the above range, run_time controls the running time of the servos,  
within the effective range, the smaller the time, the faster the servos rotate. The unit is  
milliseconds, and the minimum value is 0.  
Return value: None.  
Parameter explanation: close/open the torque force of the serial port servo, enable=[0, 1].  
enable=0: Turn off the torque force of the steering gear, you can turn the steering gear by hand,  
but the command cannot control the rotation; enable=1: Turn on the torque force, the command  
can control the rotation, but you cannot turn the steering gear by hand.  
Return value: none  
Parameter explanation: read the angle of the serial servo, s_id indicates the ID number of the  
servo to be read, s_id=[1-6]  
Return value: Returns the current angle of the input ID, and returns -1 for read errors.  
Parameter explanation: read the angles of six servos at one time [xx, xx, xx, xx, xx, xx], if a servo is  
wrong, the digit is -1  
Return value: [angle_1, angle_2, angle_3, angle_4, angle_5, angle_6].  
 
3. Experimental effect  
Check out the course accompanying video.  
 set_uart_servo_angle_array ( angle_s =[ 90 ,  90 ,  90 ,  90 ,  90 ,  180 ],  
run_time = 500 ) 
set_uart_servo_torque(enable) 
get_uart_servo_angle(s_id) 
get_uart_servo_angle_array() 
4. Program source code  
Power on the Rosmaster robot, and open the browser of the Jetson Nano or remote computer to  
enter the Jupyter lab editor.  
Reference code path: Rosmaster/Samples/9.arm_servo.ipynb  

---

## 9. ROS+Opencv application.pdf

9. ROS+Opencv application  
9. ROS+Ope ncv application 
9.1. Overview 
9.2. Use 
9.2.1. Start 
9.2.2. Display method 
9.2.3. Effect display 
9.3. Node 
9.3.1. edge detection algorithm 
9.3.2. Contour moment 
9.3.3. Face Recognition 
9.1. Overview  
wiki: http://wiki.ros.org/opencv_apps 
Source code: https://github.com/ros-perception/opencv_apps.git 
Most of the code was originally taken from 
https://github.com/Itseez/opencv/tree/master/samples/cpp 
Feature pack: ~/software/library_ws/src/opencv_apps 
The topic subscribed by this function package is [/image]. What we need to do is to open the  
camera node, write a node that converts the camera topic into a [/image] node, and publish the  
[/image] topic. 
The path to the node that enables the camera and publishes the [/image] topic: 
The opencv_apps program provides various nodes that run opencv's functions internally and  
publish the results to a ROS topic. When using the opencv_apps program, according to your own  
business needs, you only need to run a launch file, so you don't have to write program codes for  
these functions. 
ROS Wiki has related node analysis, topic subscription and topic publishing of corresponding  
nodes, introduction of related parameters, etc. See the ROS WiKi for details. ~/yahboomcar_ws/src/yahboomcar_visual/scripts/pub_image.py 
9.2. Use  
9.2.1. Start  
Step 1: Start camera
img_flip parameter: Whether the image needs to be flipped horizontally, the default is false. 
The [usb_cam-test.launch] file opens the [web_video_server] node by default, and you can directly  
use the [IP:8080] webpage to view the image in real time. 
Step 2: Start the function of Opencv_apps 
<PI5 needs to open another terminal and enter the same docker container#Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
roslaunch yahboomcar_visual opencv_apps.launch img_flip:=false 
roslaunch  opencv_apps  face_recognition.launch              # face recognition 
roslaunch  opencv_apps  corner_harris.launch                 # harris corner 
detection 
roslaunch  opencv_apps  camshift.launch                      # target tracking 
algorithm 
roslaunch  opencv_apps  contour_moments.launch               # contour moments 
roslaunch  opencv_apps  convex_hull.launch                   # polygon outline 
roslaunch  opencv_apps  discrete_fourier_transform.launch    # discrete Fourier 
transform algorithm 
roslaunch  opencv_apps  edge_detection.launch                # edge detection 
algorithm 
roslaunch  opencv_apps  face_detection.launch                # face detection 
algorithm 
roslaunch  opencv_apps  fback_flow.launch                    # Optical flow 
detection algorithm 
roslaunch  opencv_apps  find_contours.launch                 # contour detection 
roslaunch  opencv_apps  general_contours.launch              # general contour 
detection 
roslaunch  opencv_apps  goodfeature_track.launch             # feature point 
tracking 
roslaunch  opencv_apps  hls_color_filter.launch              # HLS color filter 
roslaunch opencv_apps  hough_circles .launch #                 Hough circle 
detection 
roslaunch  opencv_apps  hough_lines.launch                   # Hough line 
detection 
roslaunch  opencv_apps  hsv_color_filter.launch              # HSV color filter 
roslaunch  opencv_apps  lk_flow.launch                       # LK optical flow 
algorithm 
roslaunch  opencv_apps  people_detect.launch                 # human detection 
algorithm 
roslaunch  opencv_apps  phase_corr.launch                    # Phase correlation 
displacement detection 
roslaunch  opencv_apps  pyramids.launch                      # Image pyramid 
sampling algorithm 
roslaunch  opencv_apps  rgb_color_filter.launch              # RGB color filter 
Almost every functional case will have a parameter [debug_view], Boolean type, whether to use  
Opencv to display the picture, which is displayed by default. 
Set to [False] if no display is required, for example 
However, after this is started, some cases cannot be displayed in other ways, because in the  
source code, some [debug_view] is set to [False], which will turn off image processing. 
9.2.2. Display method  
rqt_image_view 
Enter the following command to select the corresponding topic 
opencv 
The system defaults to display, no need to do anything. 
web viewing 
(same as the local area network) Enter IP+port in the browser, for example: 
9.2.3. Effect display  
Optical flow detection algorithm 
Move the screen and observe the phenomenon. roslaunch  opencv_apps  segment_objects.launch               # clear background 
detection algorithm 
roslaunch  opencv_apps  simple_flow.launch                   # Simplified optical 
flow algorithm 
roslaunch  opencv_apps  smoothing.launch                     # simple filter 
roslaunch  opencv_apps  threshold.launch                     # threshold image 
processing 
roslaunch  opencv_apps  watershed_segmentation.launch        # watershed 
segmentation algorithm 
roslaunch opencv_apps contour_moments.launch  debug_view:=False 
rqt_image_view 
192.168.2.79:8080 
Feature point tracking 
Hough circle detection 
Hough Line Detection 
The lower the threshold, the more lines, and the easier the picture gets stuck. 
Phase-dependent displacement detection 
The faster the camera moves, the larger the radius of the circle. 
watershed segmentation algorithm 
Use the mouse to select different objects, and the system will automatically distinguish them. 
parameter type default Parse
~use_camera_info bool trueSubscribe to the topic [camera_info] to get the
default coordinate system ID, otherwise use
the image information directly.
~debug_view bool falsewhether to create a window to display the
node image
~edge_type int 0Specify the edge detection method: 0: Sobel
operator, 1: Laplacian operator, 2: Canny edge
detection
~canny_threshold1 int 100 Specify the second canny threshold
~canny_threshold2 int 200 Specify the first canny threshold
~apertureSize int 3 The aperture size of the Sobel operator.
~apply_blur_pre bool True whether to apply blur() to the input image
~ postBlurSize double 3.2 Enter the image aperture size
~apply_blur_post bool Falsewhether to apply GaussianBlur() to the input
image
~L2gradient bool False Parameters of canny
~queue_size int 3 queue size
9.3. Node  
Each case in this section will have a topic of subscribing to images and publishing images. 
9.3.1. edge detection algorithm  
parameter type default Parse
~use_camera_info bool trueSubscribe to the topic [camera_info] to get
the default coordinate system ID, otherwise
use the image information directly.
~debug_view bool falsewhether to create a window to display the
node image
~canny_low_threshold int 0 Canny edge detection low threshold
~queue_size int 3 queue size
9.3.2. Contour moment  
parameter type default Parse
~approximate_sync bool falseSubscribe to the
topic [camera_info]
to get the default
coordinate system
ID, otherwise use
the image
information
directly.
~queue_size int 100Queue size for
subscribing to
topics
~model_method string "eigen"Methods of face
recognition:
"eigen", "fisher" or
"LBPH"
~use_saved_data bool trueLoad training data
from ~data_dir
path
~save_train_data bool trueSave the training
data to the
~data_dir path for
retraining
~data_dir string "~/opencv_apps/face_data"Save training data
path
~face_model_width int 190width of training
face images
9.3.3. Face Recognition  
This case is self-training and real-time recognition through real-time collection of human images,  
and the steps are slightly complicated. 
parameter type default Parse
~face_model_height int 90height of training
face images
~face_padding double 0.1Fill ratio for each
face
~model_num_components int 0The number of
components of the
face recognizer
model (0 is
considered
unlimited)
~model_threshold double 8000.0face recognition
model threshold
~lbph_radius int 1Radius parameter
(only for LBPH
method)
~lbph_neighbors int 8Neighborhood
parameter (only for
LBPH method)
~lbph_grid_x int 8grid x parameter
(only for LBPH
method)
~lbph_grid_y int 8grid y parameter
(only for LBPH
method)
~queue_size int 100Image subscriber
queue size
Steps: 
1. First, enter the character name after the colon in the following figure: Yahboom 
2. Confirm the name: y 
3. Then place the face in the center of the image and click OK. 
4. Add a photo cyclically: y, click OK. 
5. To end the image collection, enter: n, and click OK. 
6. Close the launch file and restart. 
If you need to enter the recognized recognition, cycle 1 to 5 in turn, until all recognition personnel  
are completed, and then perform the sixth step. 
Step 3: To ensure that the face can be recognized 
The final recognition effect 
 

---

## 9. Timer interrupt to control PWM servo.pdf

9. Timer interrupt to control PWM servo  
9. Timer interrupt to control PWM servo 
9.1. Purpose of the experiment 
9.2, configuration pin information 
9.3. Analysis of the experimental flow chart 
9.4. core code explanation 
9.5. Hardware connection 
9.6. Experimental effect 
9.1. Purpose of the experiment  
Use the basic timer interrupt function of STM32 to simulate the output PWM signal and control  
the PWM servo.  
9.2, configuration pin information  
1. Import the ioc file from the Beep project and name it PwmServo.  
According to the schematic diagram, the servos S1 S2 S3 S4 are connected to the PC3 PC2 PC1  
PC0 pins of the STM32 respectively.  
Set the PC0 PC1 PC2 PC3 pins as output mode, the specific parameters are shown in the following  
figure:  
2. Next, we need to configure timer 7. The specific configuration parameters are shown in the  
figure below.  
Turn on the timer global interrupt setting.  
9.3. Analysis of the experimental flow chart  
9.4. core code explanation  
1. Create new bsp_pwmServo.h and bsp_pwmServo.c, and add the following content in  
pwmServo.h:  
Among them, SERVO_X_HIGH() means output high level, SERVO_X_LOW() means output low level.  
2. Create the following content in the bsp_pwmServo.c file:  
The PwmServo_Init() function initializes the PWM position to 90 degrees.  
The PwmServo_Angle_To_Pulse() function converts the angle into a PWM duty cycle value.  
3. The PwmServo_Set_Angle() function sets the pwm servo angle, index=0~3, and angle is 0-180.  
 
4. The PwmServo_Set_Angle_All() function sets the angle of all pwm servos, angle_s1  
corresponds to the angle of S1, the range is 1-180, and the other three parameters  
correspond to the angle values of S2, S3 and S4 respectively.  
5. The PwmServo_Handle() function needs to be called in the interrupt of the timer to simulate  
the output PWM signal and control the servo according to the angle value of the servo set  
above.  
6. Create a new HAL_TIM_PeriodElapsedCallback() function. The name of this function cannot  
be changed, otherwise the function will be found. The PwmServo_Handle() function is called  
by the timer 7 interrupt to generate the PWM signal.  
7. Add the following content to the Bsp_Loop() function and press the button to control the  
PWM servo.  
9.5. Hardware connection  
Since the PWM servos have different voltage drive values, the expansion board has added a  
voltage switching function. According to the jumper cap on the expansion board, the PWM output  
voltage can be modified to 5V or 6.8V. To use the PWM servo, the corresponding voltage must be  
selected with a jumper cap to avoid burning the servo. The PWM servos cannot be controlled  
without the jumper caps inserted.  The pins of the PWM servo are: yellow->signal, red->power  
positive, black->power negative.  
Since the power of the PWM servo is relatively large, the expansion board should not be powered  
by USB 5V directly, but must be powered by DC 12V.  
 
9.6. Experimental effect  
After the program is programmed, the LED light flashes every 200 milliseconds. Press the button  
multiple times, the PWM servo will go back and forth between 50 degrees and 150 degrees.  

---



## 4. Mapping Systems
# Mapping Manual

## 8. ORB_SLAM2 basics.pdf

8. ORB_SLAM2 basics  
8. ORB_SLAM2 basics
8.1. Introduction
8.2. Official case
8.2.1. Monocular test
8.2.2. Binocular test
8.2.3, RGBD test
8.3, ORB_SLAM2_ROS camera test
8.3.1. Internal parameter modification
8.3.2. Monocular
8.3.3. Monocular AR
8.3.4, RGBD
Official website: http://webdiis.unizar.es/~raulmur/orbslam/
ASL Dataset: https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
mono Dataset: https://vision.in.tum.de/data/datasets/rgbd-dataset/download
stereo Dataset: http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01
_easy/
orb_slam2_ros: http://wiki.ros.org/orb_slam2_ros
ORB-SLAM: https://github.com/raulmur/ORB_SLAM
ORB-SLAM2: https://github.com/raulmur/ORB_SLAM2
ORB-SLAM3: https://github.com/UZ-SLAMLab/ORB_SLAM3
8.1. Introduction  
ORB-SLAM is mainly used for monocular SLAM;
The ORB-SLAM2 version supports three interfaces: monocular, binocular and RGBD;
The ORB-SLAM3 version adds IMU coupling and supports fisheye cameras.
All steps of ORB-SLAM use the ORB features of the image uniformly. The ORB feature is a very fast  
feature extraction method that is rotation invariant and can use pyramids to build scale  
invariance. Using unified ORB features helps the SLAM algorithm to be consistent in steps such as  
feature extraction and tracking, key frame selection, three-dimensional reconstruction, and  
closed-loop detection. The system is also robust to severe motion and supports wide-baseline  
closed-loop detection and relocalization, including fully automatic initialization. Since the ORB-
SLAM system is a SLAM system based on feature points, it can calculate the camera's trajectory in  
real time and generate sparse three-dimensional reconstruction results of the scene.
On the basis of ORB-SLAM, ORB-SLAM2 contributes:
1. The first open source SLAM system for monocular, binocular and RGBD cameras, including  
loopback, relocation and map reuse.
2. The results of RGBD show that more accuracy can be obtained by using BA than ICP or  
minimization based on photometric and depth errors.
3. By using the far point and near point in binoculars and monocular observation, the binocular  
results are more accurate than the direct binocular SLAM algorithm.
4. Light positioning mode can effectively reuse maps.
ORB-SLAM2 includes modules common to all SLAM systems: tracking, mapping, relocalization, and  
loop closing. . The figure below is the process of ORB-SLAM2.
8.2. Official case  
Open the terminal and enter ORB_SLAM2
8.2.1. Monocular test  #Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
cd ~/software/ORB_SLAM2
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUM3.yaml 
~/software/ORB_SLAM_data/rgbd_dataset_freiburg3_long_office_household
The blue frame is the key frame, the green frame is the camera orientation, the black point is the  
saved point, and the red point is the point currently seen by the camera.
After the test is completed, the keyframes are saved to the KeyFrameTrajectory.txt file in the  
current directory.
8.2.2. Binocular test  
The blue frame is the key frame, the green frame is the camera orientation, the black point is the  
saved point, and the red point is the point currently seen by the camera.
After the test is completed, the keyframes are saved to the CameraTrajectory.txt file in the current  
directory.# Timestamp position (x y z) + attitude (x y z w)
1341847980.722988 -0.0000464 0.0001060 0.0000110 -0.0000183 0.0001468 -0.0000286 
1.0000000
./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml 
~/software/ORB_SLAM_data/MH_01_easy/mav0/cam0/data 
~/software/ORB_SLAM_data/MH_01_easy/mav0/cam1/data 
Examples/Stereo/EuRoC_TimeStamps /MH01.txt
8.2.3, RGBD test  
Merge the depth data and color image data into rgbd data and save it to the associations.txt file
Back to ORB_SLAM2
test command
8.3, ORB_SLAM2_ROS camera test  
The internal parameters of the camera have been modified before the product leaves the factory.  
If you want to learn how to do this, please refer to the section [8.3.1, Modification of Internal  
Parameters]. It can be handheld or robot used as a mobile carrier for mobile testing.
If it is held, there is no need to execute the next command, otherwise, it will be executed. (Robot  
side)
<PI5 needs to open another terminal to enter the same docker container# Timestamp position (x y z) + attitude (x y z w)
1403636597.963556 -0.020445164 0.127641633 0.107868195 -0.136788622 -0.074876986 
-0.044620439 0.986757994
cd ~/software/ORB_SLAM_data/rgbd_dataset_freiburg3_long_office_household
python2 associate.py
cd ~/software/ORB_SLAM2
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM3.yaml 
~/software/ORB_SLAM_data/rgbd_dataset_freiburg3_long_office_household 
~/software/ORB_SLAM_data/rgbd_dataset_freiburg3_long_office_household/association
s.txt
roslaunch yahboomcar_slam camera_driver.launch
Start the camera ORB_SLAM2 test (Robot side or virtual machine)
[orb_slam_type] parameters: [mono, monoAR, rgbd], there are three types available,  
monocular, monocular AR, rgbd.
8.3.1. Internal parameter modification  
The camera requires the internal parameters of the camera before running ORBSLAM, so the  
camera must be calibrated first. The specific method can be found in the lesson [02, Astra Camera  
Calibration].
Start monocular camera
Start calibration node
After calibration, move the [calibrationdata.tar.gz] file to the [home] directory.
After unzipping, open [ost.yaml] in the folder and find the camera internal parameter matrix, for  
example: the following content.roslaunch yahboomcar_slam camera_orb_slam.launch orb_slam_type:=mono
roslaunch usb_cam usb_cam-test.launch
rosrun camera_calibration cameracalibrator.py image:=/usb_cam/image_raw 
camera:=/usb_cam --size 9x6 --square 0.02
sudo mv /tmp/calibrationdata.tar.gz ~
Camera internal parameter matrix
Modify the data in data to the values corresponding to [astra.yaml] and [astra1.0.yaml] in the  
[param] folder under the [yahboomcar_slam] function package.
8.3.2. Monocular  
When the command is executed, there is only a green box in the [ORB_SLAM2:Map Viewer]  
interface, and the [ORB_SLAM2:Current Frame] interface is trying to initialize. At this time, slowly  
move the camera up, down, left, and right to find feature points in the screen and initialize slam. .
camera_matrix:
   rows: 3
   cols: 3
   data: [ 683.90304, 0. , 294.56102,
             0. , 679.88513, 228.05956,
             0. , 0. , 1. ]
# fx 0 cx
# 0 fy cy
# 0 0 1
Camera.fx: 683.90304
Camera.fy: 679.88513
Camera.cx: 294.56102
Camera.cy: 228.05956
As shown in the picture above, when you enter the [SLAM MODE] mode, you must continuously  
acquire each frame of image to position the camera when running the monocular. If you select the  
pure positioning mode of [Localization Mode] in the upper left picture, the camera will not be able  
to find its own position. You have to start over to get the keyframes.
8.3.3. Monocular AR  
When the command is executed, there is only one interface and [slam not initialized] is displayed.  
slam is not initialized. Click the box to the left of [Draw Points] in the left column to display feature  
points. At this time, slowly move the camera up, down, left, and right to find feature points in the  
picture and initialize slam.
As shown in the picture above, enter [SLAM ON] mode at this time. Click [Insert Cube] on the  
screen to insert an AR cube where it is considered to be a plane. And the AR block will always be in  
a fixed position in the scene.
, not a fixed position on the camera screen. Click [Clear All] to clear.
8.3.4, RGBD  
RGBD does not have to continuously acquire each frame of image like running a monocular. If you  
select the pure positioning mode [Localization Mode] in the upper left picture, you can position  
the key frame just acquired.

---

## 8.karto mapping algorithm.pdf

8. karto mapping algorithm  
8. karto mapping algorithm
8.1. Introduction
8.2. Use
8.2.1. Start
8.2.1. Controlling the robot
8.2.1. Map saving
8.3. Topics and services
8.4. Configuration parameters
8.5, TF transformation
karto: http://wiki.ros.org/slam_karto
map_server: https://wiki.ros.org/map_server
8.1. Introduction  
Karto is a 2D laser SLAM solution based on a sparse graph optimization method with loop closure  
detection. The graph optimization method uses the mean value of the graph to represent the  
map. Each node represents a position point of the robot trajectory and a sensor measurement  
data set. Each new node is added and the calculation is updated. Karto uses spa (karto_slam) or  
g2o (nav2d) optimization library, and the front-end and back-end use a single thread.
The ROS version of Karto_SLAM, in which the Spare Pose Adjustment (SPA) used is related to scan  
matching and loop closure detection. The more landmarks there are, the greater the memory  
requirements. However, compared with other methods, the graph optimization method has  
greater advantages in mapping in large environments because it only contains point graphs (robot  
poses), and the map is obtained after the pose is obtained.
overall program framework
8.2. Use  
Note: When building a map, the slower the speed, the better the effect (note that the  
rotation speed should be slower). If the speed is too fast, the effect will be poor. 
According to different models, you only need to set the purchased model in [.bashrc], X1 (normal  
four-wheel drive) X3 (Mailun) Take X3 as an example
Open the [.bashrc] file#Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
sudo vim .bashrc
Find the [ROBOT_TYPE] parameters and modify the corresponding car model
Note: Due to the difference in radar laser data between 4ROS radar and A1 radar, you need  
to modify the karto source code part before it can run normally and modify the file.
Locate line 4165, the source code is,
Need to be changed to,
After saving, switch to the ~/software/library_ws directory and use catkin_make to compile.
8.2.1. Start  
Start the command (robot side). For the convenience of operation, this section takes [mono + laser  
+ yahboomcar] as an example.
Mapping command (robot side)
<PI5 needs to open another terminal to enter the same docker containerexport ROBOT_TYPE=X3 # ROBOT_TYPE: X1 X3 X3plus R2 X7
sudo gedit ~/software/library_ws/src/open_karto-
melodic/include/open_karto/Karto.h
void Update()
     {
       m_NumberOfRangeReadings = static_cast<kt_int32u>
(math::Round((GetMaximumAngle() -
                                                                     
GetMinimumAngle())
                                                                     / 
GetAngularResolution())+1);
     }
  void Update()
     {
       m_NumberOfRangeReadings = static_cast<kt_int32u>
(math::Round((GetMaximumAngle() -
                                                                     
GetMinimumAngle())
                                                                     
/GetAngularResolution()));
     }
cd ~/software/library_ws
catkin_make
roslaunch yahboomcar_nav laser_bringup.launch # laser + yahboomcar
roslaunch yahboomcar_nav laser_usb_bringup.launch # mono + laser + yahboomcar
roslaunch yahboomcar_nav laser_astrapro_bringup.launch # Astra + laser + 
yahboomcar
[use_rviz] parameter: whether to enable rviz visualization.
[map_type] parameter: Set the mapping algorithm [karto].
Turn on the visual interface (virtual machine side)
roslaunch yahboomcar_nav yahboomcar_map.launch use_rviz:=false map_type:=karto
roslaunch yahboomcar_nav view_map.launch
The gap at the back of the robot is due to the obstruction caused by the installation position of the  
display screen, so a certain range of radar data is blocked. The shielding range can be adjusted, or  
it can not be blocked according to the actual situation. For specific operations, see [01. Radar Basic  
Course].
8.2.1. Controlling the robot  
Keyboard controls robot movement
Control the robot movement with the handle
Make the robot cover the area to be mapped and the map should be as closed as possible.
There may be some scattered points during the mapping process. If the mapping environment is  
well closed, relatively regular, and the movement is slow, the scattering phenomenon will be much  
smaller.
8.2.1. Map saving  
The map will be saved to the ~/yahboomcar_ws/src/yahboomcar_nav/maps/ folder, a pgm image  
and a yaml file.
map.yaml
Parameter analysis:
image: The path of the map file, which can be an absolute path or a relative path.
resolution: resolution of the map, meters/pixel
Origin: 2D pose (x, y, yaw) in the lower left corner of the map. The yaw here is rotated  
counterclockwise (yaw=0 means no rotation). Many parts of the current system ignore the  
yaw value.
negate: whether to reverse the meaning of white/black and free/occupied (the interpretation  
of the threshold is not affected)
occupied_thresh: Pixels with an occupation probability greater than this threshold will be  
considered fully occupied.
free_thresh: Pixels with occupancy probability less than this threshold will be considered  
completely free.rosrun teleop_twist_keyboard teleop_twist_keyboard.py # System integration
roslaunch yahboomcar_ctrl yahboom_keyboard.launch # Custom
rosrun map_server map_saver -f ~/yahboomcar_ws/src/yahboomcar_nav/maps/my_map # 
The first way
bash ~/yahboomcar_ws/src/yahboomcar_nav/maps/map.sh # The second way
image: map.pgm
resolution: 0.05
origin: [-15.4,-12.2,0.0]
Negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
Topic Subscription Type Description
scan sensor_msgs/LaserScan Depth data of lidar scan
tf tf/tfMessageUsed for conversion
between lidar coordinate
system, base coordinate
system, and odometer
coordinate system
Topic Post Type Description
map_metadata nav_msgs/MapMetaData Publish map Meta data
map nav_msgs/OccupancyGrid Publish map raster data
visualization_marker_array visualization_msgs/MarkerArray Publish pose diagram
Service Type Description
dynamic_map nav_msgs/GetMap Get map data
Parameters Type Default value Description
~base_frame string "base_link" Robot base coordinate system
~map_frame string "map" Map coordinate system
~odom_frame string "odom" Odometer coordinate system
~throttle_scans int 1Process 1 per this many scans (set this to a higher number to skip more
scans)8.3. Topics and services  
Node view
8.4. Configuration parameters  
Common parametersrqt_graph
Parameters Type Default value Description
~map_update_interval float 5.0The time in seconds between map updates. Lowering this number will
update the occupancy grid more frequently, but will increase the
computational load.
~resolution float 0.05 Map resolution (meters per occupied grid block)
~delta float 0.05Map resolution (meters per occupied grid block). Same as resolution.
Defined for compatibility with gmapping parameter names.
~transform_publish_period float 0.05 The time in seconds between transform publications.
use_scan_matching bool trueWhether to use the scan matching algorithm, generally set to true, the
mapper algorithm can correct the noise and errors of the odometer and
laser. In some simulation environments with accurate sensor data, the
scan matching algorithm will achieve worse results (because the use of
Gaussian blur reduces the observation confidence of high-precision
sensors), and it is recommended to turn it off (just add noise to the
simulation environment).
use_scan_barycenter bool trueDefines the distance between scans using the centroid of the scan
endpoints.
minimum_travel_distance double 0.2 Set the minimum travel between scans.
minimum_travel_heading double deg2rad(10)=0.087266461 Set the minimum angle between scans.
scan_buffer_size int 70Set the length of ScanBuffer, approximately equal to
scan_buffer_maximum_scan_distance/minimum_travel_distance
scan_buffer_maximum_scan_distance double 20.0 Setting the maximum length of ScanBuffer is similar to Size
link_match_minimum_response_fine double 0.8 Set the minimum response threshold for the minimum scans connection
link_scan_minimum_distance double 10.0Set the maximum distance between scans of two connections. If it is
greater than this value, the response threshold of the two will not be
considered
loop_search_maximum_distance double 4.0Maximum distance for loop detection. Scans less than this distance from
the current position will be considered matching loop closures.
do_loop_closing bool true Whether to enable loop closing detection
loop_match_minimum_chain_size int 10 The lowest chain size for loop detection
loop_match_maximum_variance_coarse double math::Square(0.4)=0.16The maximum covariance value of coarse matching during loop matching.
If it is less than this value, it is considered a feasible solution
loop_match_minimum_response_coarse double 0.8The minimum response for coarse matching during loop matching. A
response value greater than this value will start coarse precision loop
optimization
loop_match_minimum_response_fine double 0.8The minimum response threshold for loop matching. High accuracy will
only start when it is greater than this value
Parameters TypeDefault
valueDescription
correlation_search_space_dimension double 0.3Set the search range
size of Correlation
Grid
correlation_search_space_resolution double 0.01Set the resolution of
the Correlation Grid
correlation_search_space_smear_deviation double 0.03Set the Correlation
Grid blur levelCorrection parameters
Loopback parameters
Parameters TypeDefault
valueDescription
loop_search_space_dimension double 8.0Loop detection space
range size
loop_search_space_resolution double 0.05Loop detection spatial
resolution
loop_search_space_smear_deviation double 0.03 Loop detection blur level
Parameters Type Default value Description
distance_variance_penalty double sqrt(0.3)=0.09 (less than 1.0)Compensation
coefficient for
odometer during
scan-matching
angle_variance_penalty double sqrt(deg2rad(20))=0.17453292Compensation
coefficient for angle
during scan-
matching
fine_search_angle_offset double deg2rad(0.2)=0.0017453292Fine search angle
range
coarse_search_angle_offset double deg2rad(20)=0.17453292Coarse search angle
range
coarse_angle_resolution double deg2rad(2)=0.017453292Coarse search angle
resolution
minimum_angle_penalty double 0.9minimum angle
penalty
minimum_distance_penalty double 0.5Minimum distance
penalty
use_response_expansion bool falseWhether to increase
the search scope if
no good matches
are found
Required TF
transformationDescription
laser-->base_linkUsually a fixed value, the transformation between the lidar
coordinate system and the base coordinate system, generally
published by robot_state_publisher or static_transform_publisherScan Matcher parameters
8.5, TF transformation  
Required TF
transformationDescription
base_link-->odomTransformation between the map coordinate system and the robot
odometer coordinate system, estimating the robot's pose in the map
Published by TF
TransformDescription
map-->odomThe current estimate of the robot pose within the map frame (only
provided if parameter "pub_map_odom_transform" is true).
View tf tree
rosrun rqt_tf_tree rqt_tf_tree

---

## 9、ORB_SLAM2_Octomap_en.pdf

9. ORB_SLAM2_Octomap  
9. ORB_SLAM2_Octomap
9.1. Introduction
9.2. Use
9.3, octomap_server
9.3.1, Topics and Services
9.3.2. Configuration parameters
9.3.3, TF transformation
9.4. Expansion testing
octomap official website: http://octomap.github.io/
octomap source code: https://github.com/OctoMap/octomap
octomap wiki: http://wiki.ros.org/octomap
octomap_server: http://wiki.ros.org/octomap_server
9.1. Introduction  
Octomap uses the octree data structure to store the probabilistic occupancy map of the three-
dimensional environment. OctoMap library  implements a 3D occupancy grid mapping method,  
providing data structures and mapping algorithms in C++, which is particularly suitable for robots.  
The map implementation is based on octree.
It elegantly compresses and updates maps with adjustable resolution! It stores maps in the form  
of octotree (will be discussed later), which can save a lot of space compared to point clouds. The  
map created by octomap probably looks like this: (different resolutions from left to right)
Precautions
Note: When building a map, moving the robot slowly and losing key frames may cause the  
map building to fail. 
According to different models, you only need to set the purchased model in [.bashrc], X1 (normal  
four-wheel drive) X3 (Mailun) Take X3 as an example
Open the [.bashrc] file
Find the [ROBOT_TYPE] parameters and modify the corresponding car model
9.2. Use  
Start orb_slam and the underlying driver (Robot side)
[bUseViewer] parameter: whether to open the visualization window of orbslam. If true, you  
can clearly view the key points. If the positioning is unsuccessful, you can reset the key points.  
Click [Reset] on the left side of the picture below.
<PI5 needs to open another terminal to enter the same docker container#Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
sudo vim .bashrc
export ROBOT_TYPE=X3 # ROBOT_TYPE: X1 X3 X3plus R2 X7
roslaunch yahboomcar_slam robot_orb_slam.launch bUseViewer:=true
Start octree mapping (Robot side)
[frame_id] parameter: coordinate system name, available by default and no need to set.
[use_rviz] parameter: whether to enable rviz.
Turn on the visual interface (virtual machine side)
roslaunch yahboomcar_slam robot_orb_octomap.launch frame_id:=odom use_rviz:=false
roslaunch yahboomcar_slam view_orb_octomap.launch
Subscription topic Type Description
cloud_in sensor_msgs/PointCloud2Incoming 3D point
cloud for scan
integration.
Post Topic Type Description
octomap_binary octomap_msgs/OctomapComplete
maximum
likelihood
occupancy map as
a compact octal
map binary
stream, encoding
free space and
occupied space.
Binary messages
only differentiate
between free
space and
occupied space,
but smaller.
octomap_full octomap_msgs/OctomapFull maximum
likelihood
occupancy map as
a compact octal
map binary
stream, encoding
free space and
occupied space.
The complete
message contains
the complete
probabilities and
all additional data
stored in the tree.
occupied_cells_vis_array visualization_msgs/MarkerArrayIn RViz, all
occupied voxels
are marked as
visualization
"boxes"Due to the octree, its map looks like it is composed of many small squares (much like Minecraft).  
When the resolution is high, the squares are small; when the resolution is low, the squares are  
large. Each square represents the probability of that square being occupied.
9.3, octomap_server  
9.3.1, Topics and Services  
Subscription topic Type Description
octomap_point_cloud_centers sensor_msgs/PointCloud2The centers of all
occupied voxels
serve as point
clouds, useful for
visualization. Note
that there will be
gaps as points
have no volume
size and octagonal
voxels can have
different
resolutions!
map nav_msgs/OccupancyGridProject a 2D
occupancy map
downward from a
3D map.
Service Type Description
octomap_binary octomap_msgs/GetOctomapThe complete
maximum
likelihood
occupancy map as
a compact binary
stream of octal
maps, encoding
free space and
occupied space.
clear_bbx octomap_msgs/BoundingBoxQueryClears an area in
the 3D occupancy
map, setting all
voxels in the area
to "free"
reset std_srvs/EmptyReset the entire
map
Node view
rqt_graph
Parameters Type Default value Description
frame_id string /mapThe static global frame in which the
map will be published. When
building a map dynamically, sensor
data needs to be converted to this
frame.
resolution float 0.05The resolution of the map in meters
when starting from an empty map.
Otherwise the resolution of the
loaded file will be used.
base_frame_id string base_footprintThe robot base that performs
ground plane detection (if enabled)
height_map bool trueWhether the visualization should
encode the height with different
colors
color/[r/g/b/a] float  When ~heigh_map=False, display
the color of occupied cells in the
[0:1] range
sensor_model/max_range float -1 (unlimited)The maximum range (in meters) for
inserting point cloud data when
building a map dynamically.
Limiting the range to a useful range
(e.g. 5 meters) prevents false error
points away from the robot.
sensor_model/[hit|miss] float 0.7 / 0.4Hit and miss probabilities in the
sensor model when building the
map dynamically
9.3.2. Configuration parameters  
Parameters Type Default value Description
sensor_model/[min|max] float 0.12 / 0.97Minimum and maximum
probability of clamping when
building a map dynamically
latch boolTrue for static
mapping, false
if no initial
mapping is
givenWhether the topic is locked for
publishing or only published once
per change. For best performance
when building maps (which update
frequently), set this to false. When
set to true, all topics and
visualizations will be created on all
changes on each map.
filter_ground bool falseWhether the ground plane should
be detected and ignored from scan
data when dynamically building a
map using
pcl::SACMODEL_vertical_plane. This
clears everything on the ground,
but does not insert the ground into
the map as an obstacle. If this
feature is enabled, the
~ground_filter/... parameters can be
further configured.
ground_filter/distance float 0.04The distance threshold of the point
(z direction) to be segmented to the
ground plane
ground_filter/angle float 0.15The angle threshold between the
detected plane and the horizontal
plane detected as the ground
ground_filter/plane_distance float 0.07Distance threshold at z=0 for a
plane to be detected as ground
(fourth coefficient of the PCL plane
equation)
pointcloud [ m i n | m a x ]z float -/+ infinityInsert the minimum and maximum
height to be considered in the
callback.
occupancy [ m i n | m a x ]z float -/+ infinityThe minimum and maximum height
to be considered in the final map.
Required TF
transformationDescription
data sensor frame --
> /mapIf scanning integration is done, the sensor data needs to be
converted into a global map frame. This information needs to be
obtained from external SLAM or localization nodes.9.3.3, TF transformation  
View tf tree
9.4. Expansion testing  
Note: This case is a test version and is related to device performance. If the performance is  
too low, it may not start properly. 
Start the underlying driver + orb_slam (virtual machine side)
[bUseViewer] parameter: whether to open the visualization window of orbslam.
Start octree mapping (virtual machine side)
[use_viewer] parameter: whether to open the visualization window of pcl_viewer.
[local_frame_id] parameter: local map coordinate system.
[global_frame_id] parameter: global map coordinate system.
After opening, the [viewer] window will pop up, as shown belowrosrun rqt_tf_tree rqt_tf_tree
roslaunch yahboomcar_slam test_orb_slam.launch
roslaunch yahboomcar_slam test_pcl_mapping.launch
 
Move the camera slowly, when a picture appears, as shown below
The coordinate system needs to be scaled and rotated as shown in the figure below
Sliding wheel: zoom in or out
Press and hold scroll wheel: pan
Left mouse button: rotate
Right mouse button: zoom in or out
Slowly move the camera to build the map as shown below. After the build is completed, [Ctrl+c]  
closes and the pcd point cloud file is automatically saved. The path file name under this function  
package is [resultPointCloudFile.pcd]
pcl_viewer installation command
View and enter the directory where the pcd file is located
sudo apt-get install pcl-tools
pcl_viewer resultPointCloudFile.pcd

---

## 9.cartographer mapping algorithm.pdf

9. Cartographer mapping algorithm  
9. Cartographer mapping algorithm
9.1. Introduction
9.2. Use
9.2.1. Start
9.2.2. Controlling the robot
9.2.3. Map saving
9.2.4. Node View
9.2.5. View tf tree
9.3. Configuration parameters
Cartographer: https://google-cartographer.readthedocs.io/en/latest/
Cartographer ROS: https://google-cartographer-ros.readthedocs.io/en/latest/
map_server: https://wiki.ros.org/map_server
9.1. Introduction  
Cartographer is a 2D and 3D SLAM (simultaneous localization and mapping) library supported by  
Google's open source ROS system. A graph-building algorithm based on graph optimization (multi-
threaded backend optimization, problem optimization of ceiling construction). Data from multiple  
sensors (such as LIDAR, IMU, and cameras) can be combined to simultaneously calculate the  
sensor's position and map the environment around the sensor.
The source code of cartographer mainly includes three parts: cartographer, cartographer_ros and  
ceres-solver (back-end optimization).
cartographer
Cartographer uses the mainstream SLAM framework, which is a three-stage structure of feature  
extraction, closed-loop detection, and back-end optimization. A certain number of LaserScans  
form a submap, and a series of submaps constitute the global map. The cumulative error in the  
short-term process of using LaserScan to construct a submap is not large, but the long-term  
process of using a submap to construct a global map will have a large cumulative error. Therefore,  
closed-loop detection is needed to correct the positions of these submaps. The basic unit of  
closed-loop detection is submap, closed-loop detection uses the scan_match strategy. The focus of  
cartographer is the creation of submap subgraphs that integrate multi-sensor data (odometry,  
IMU, LaserScan, etc.) and the implementation of the scan_match strategy for closed-loop  
detection.
cartographer_ros
The cartographer_ros package runs under ROS and can accept various sensor data in the form of  
ROS messages. After processing, it can be published in the form of messages to facilitate  
debugging and visualization.
9.2. Use  
Note: When building a map, the slower the speed, the better the effect (note that the  
rotation speed should be slower). If the speed is too fast, the effect will be poor. 
According to different models, you only need to set the purchased model in [.bashrc], X1 (normal  
four-wheel drive) X3 (Mailun) Take X3 as an example
Open the [.bashrc] file#Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
Find the [ROBOT_TYPE] parameters and modify the corresponding car model
To replace the configuration file, copy the launch file and lua file required by cartographer to the  
specified directory. It is configured before leaving the factory.
9.2.1. Start  
Start the command (robot side). For the convenience of operation, this section takes [mono + laser  
+ yahboomcar] as an example.
Mapping command (robot side)
<PI5 needs to open another terminal to enter the same docker container
[use_rviz] parameter: whether to enable rviz visualization.
[map_type] parameter: Set the mapping algorithm [cartographer].
Turn on the visual interface (virtual machine side)sudo vim .bashrc
export ROBOT_TYPE=X3 # ROBOT_TYPE: X1 X3 X3plus R2 X7
sudo bash ~/yahboomcar_ws/src/yahboomcar_nav/scripts/copy_carto.sh
roslaunch yahboomcar_nav laser_bringup.launch # laser + yahboomcar
roslaunch yahboomcar_nav laser_usb_bringup.launch # mono + laser + yahboomcar
roslaunch yahboomcar_nav laser_astrapro_bringup.launch # Astra + laser + 
yahboomcar
roslaunch yahboomcar_nav yahboomcar_map.launch use_rviz:=false 
map_type:=cartographer
The gap at the back of the robot is due to the obstruction caused by the installation position of the  
display screen, so a certain range of radar data is blocked. The shielding range can be adjusted, or  
it can not be blocked according to the actual situation. For specific operations, see [01. Radar Basic  
Course].
9.2.2. Controlling the robot  
Keyboard controls robot movement
Control the robot movement with the handle
Make the robot cover the area to be mapped and the map should be as closed as possible.
There may be some scattered points during the mapping process. If the mapping environment is  
well closed, relatively regular, and the movement is slow, the scattering phenomenon will be much  
smaller.
9.2.3. Map saving  
The map will be saved to the ~/yahboomcar_ws/src/yahboomcar_nav/maps/ folder, a pgm image  
and a yaml file.
map.yamlroslaunch yahboomcar_nav view_cartographer.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py # System integration
roslaunch yahboomcar_ctrl yahboom_keyboard.launch # Custom
bash ~/yahboomcar_ws/src/yahboomcar_nav/maps/carto_map.sh
Parameter analysis:
image: The path of the map file, which can be an absolute path or a relative path.
resolution: resolution of the map, meters/pixel
Origin: 2D pose (x, y, yaw) in the lower left corner of the map. The yaw here is rotated  
counterclockwise (yaw=0 means no rotation). Many parts of the current system ignore the  
yaw value.
negate: whether to reverse the meaning of white/black and free/occupied (the interpretation  
of the threshold is not affected)
occupied_thresh: Pixels with an occupation probability greater than this threshold will be  
considered fully occupied.
free_thresh: Pixels with occupancy probability less than this threshold will be considered  
completely free.
9.2.4. Node View  
9.2.5. View tf tree  
image: map.pgm
resolution: 0.05
origin: [-15.4,-12.2,0.0]
Negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
rqt_graph
rosrun rqt_tf_tree rqt_tf_tree
Parameters Description
map_frame map coordinate system
tracking_frame Convert all sensor data to this coordinate system
published_frame The coordinate system pointed to by map
odom_frameDo you provide odom's tf? If it is true, the tf tree is map-
>odom->footprint; if it is false, the tf tree is map-
>footprint
provide_odom_frameIf true, the local, non-loop-closed, continuous pose will
be published as odom_frame in map_frame?
publish_frame_projected_to_2dIf enabled, published poses will be restricted to pure 2D
poses (no roll, pitch or z-offset)
use_odometryWhether to use odometer, if required, you must have
odom tf
use_nav_sat Whether to use gps
use_landmarks Whether to use landmarks
num_laser_scans Whether to use single-line laser data
num_multi_echo_laser_scans Whether to use multi_echo_laser_scans data
num_subdivisions_per_laser_scan1 frame of data is divided into several times for
processing, usually 1
num_point_clouds Whether to use point cloud data
lookup_transform_timeout_sec Timeout when looking up tf
submap_publish_period_sec Time interval for publishing submap (seconds)
pose_publish_period_secThe time interval for publishing pose, when the value is
5e-3, it is 200HZ
trajectory_publish_period_secThe time interval for publishing trajectory markers
(trajectory nodes), the value is 30e-3 is 30ms
rangefinder_sampling_ratio Fixed sampling frequency for lidar messages
odometry_sampling_ratio Fixed sampling frequency of odometry messages
fixed_frame_pose_sampling_ratioFixed sampling frequency of fixed coordinate system
messages
imu_sampling_ratio Fixed sampling frequency of IMU messages
landmarks_sampling_ratio Fixed sampling frequency of landmark messages9.3. Configuration parameters  
lua file

---



## 5. Race Loop & TEB Navigation
# Race Loop Manual

## 0.ReadMe.pdf

Notice:
1.ThemodeltrainingautopilotcourseisonlysuitableforJetsonnano,JetsonXavierNX,andJetsonTX2NX
motherboards,andisnotcompatiblewithRaspberryPi.
2.ThemodeltrainingautopilotcourseisonlyapplicabletotheR2LcarandYahboom'sautopilotmap.
Ifyouuseothermodelsorothermaps,youneedtodevelopanddebugthecodeyourself,andthecode
providedinthischaptercannotbeuseddirectly.
ThethreefoldersintheAutopilot_Annexfilearetheyolov5s.engineandlibmyplugins.sofilesgeneratedby
Yahboom,correspondingtothethreemotherboards.
best_traffic&crossline.ptisatrainedmodelthatincludestrafficsignsandsidewalks.Themodelinthe
autopilotsourcecodedoesnotdetectsidewalks.
Ifyouneedtodetectsidewalksduringautopilot,youneedtogenerateyolov5s.engineandlibmyplugins.so
filesandmodifytheyamlfileaccordingtothetutorial.
Thepicturesintraffic_picarepicturesofeachtrafficsignandsidewalk.Ifyouneedtoretrainthemodel,
youcantrainbasedonthepictureshere.

---

## 1. Getting started with Open Source CV.pdf

1. Getting started with Open Source CV  
1. Getting started with Ope n Source CV 
1.1. Introduction to Ope nCV 
1.2. Ope nCV image reading and display 
1.2.1. Image reading: 
1.2.2. Image display 
1.2.3. Code and actual effect display 
1.3. Ope nCV image writing 
1.3.1. Function method: cv2.imwrite('new_img_name', img) 
1.3.2. Code and actual effect display 
1.4. Ope nCV camera to read and display video 
1.4.1. Camera reading 
1.4.2. Display camera video 
1.4.3. Code and actual effect display 
1.5. Ope nCV pixel operation 
1.5.1. Pixel operation, we can change any position to a new pixel color. 
1.5.2. Code and actual effect display 
1.1. Introduction to OpenCV  
What is OpenCV? Its full name is Open source Computer Vision Library, an open source computer  
vision library. As shown in the figure above, what we see is the logo of OpenCV, and we can see  
three small circles with three distinct primary colors of R, G, and B. Composition, that is, it is a set  
of open source API function library about computer vision. This also means,  
(1) Whether it is scientific research or commercial application, it can be used for development;  
(2) The source code of all API functions is public, and you can see the program steps of its internal  
implementation;  
(3) You can modify the source code of OpenCV to compile and generate the specific API functions  
you need.  
The image processing on ROSMASTER uses some functions of the OpenCV function library, or it  
can be said that its existence is inseparable in most image processing design fields. As early as  
many years ago, it has been used in intrusion detection and specific target tracking. , target  
detection, face detection, face recognition, face tracking and other fields, OpenCV can be said to  
show its talents, and these are only the tip of the iceberg of its application. Now that we realize  
that OpenCV is so general, in this chapter, we will introduce you to some very basic image  
processing functions that we use in our courses, and also some general functions. Here we first  
have a general understanding of these knowledge. After a while, there are two practical projects  
in the back: color recognition and tracking, face recognition and tracking to teach you how to get  
started, but the powerful application functions provided by OpenCV are far more than that. If you  
are interested in the development of Opencv computer vision library, you want to go deeper If  
you understand, here are a few websites for your reference and study:  
OpenCV official homepage: https://www.opencv.org  
OpenCV Chinese Forum: http://www.opencv.org.cn  
OpenCV CSDN Forum: https://bbs.csdn.net/forums/OpenCV  
1.2. OpenCV image reading and display  
1.2.1. Image reading:  
img = cv2.imread('yahboom.jpg', 0) The first parameter is the path of the image, and the second  
parameter is how to read the image.  
cv2.IMREAD_UNCHANGED: keep the original format unchanged, -1;  
cv2.IMREAD_GRAYSCALE: read the image in grayscale mode, which can be represented by 0;  
cv2.IMREAD_COLOR: read a color image, which can be represented by 1; the default value  
cv2.IMREAD_UNCHANGED: Read in an image, including its alpha channel, which can be  
represented by 2.  
1.2.2. Image display  
cv.imshow('frame', frame): Open a window named frame and display frame frame data  
(image/video data)  
Parameter meaning:  
The first parameter is the name of the window to be created  
The second parameter indicates the image to be displayed  
1.2.3. Code and actual effect display  
run the program  
python ~/yahboomcar_ws/src/yahboomcar_astra/scripts/opencv/1_1.py
import cv2 as cv
if __name__ == '__main__':
    img = cv.imread('yahboom.jpg')
    while True :
        cv.imshow("frame",img)
        action = cv.waitKey(10) & 0xFF
        if action == ord('q') or action == 113:
            break
    img.release()
    cv.destroyAllWindows()
1.3. OpenCV image writing  
1.3.1. Function method: cv2.imwrite('new_img_name', img)  
Parameter meaning:  
The first parameter is the name of the file to save  
The second parameter is the saved image  
1.3.2. Code and actual effect display  
run the program  
python ~/yahboomcar_ws/src/yahboomcar_astra/scripts/opencv/1_2.py
import cv2 as cv
if __name__ == '__main__':
    img = cv.imread('yahboom.jpg')
    cv.imwrite("yahboom_new.jpg",img)   
    new_img = cv.imread('yahboom_new.jpg')
    while True :
        cv.imshow("frame",img)
        cv.imshow("new_frame",new_img)
        action = cv.waitKey(10) & 0xFF
        if action == ord('q') or action == 113:
            break
    img.release()
    cv.destroyAllWindows()
1.4. OpenCV camera to read and display video  
1.4.1. Camera reading  
capture=cv.VideoCapture(0)  
Parameter meaning:  
The parameter in VideoCapture() is 0, which means to open the built-in camera of the notebook,  
and the parameter is the video file path to open the video, such as cap =  
cv2.VideoCapture(“../test.avi”)  
1.4.2. Display camera video  
ret,img = frame.read()  
Return value meaning:  
ret: ret is a bool value to determine whether to read back the correct frame  
img: image data for each frame  
1.4.3. Code and actual effect display  
run the program  
python ~/yahboomcar_ws/src/yahboomcar_astra/scripts/opencv/1_3.py
import cv2 as cv
if __name__ == '__main__':
    frame = cv.VideoCapture(0)
    while frame.isOpened():
        ret,img = frame.read()
        cv.imshow('frame',img)
        action = cv.waitKey(10) & 0xFF
        if action == ord('q') or action == 113:
            break
    frame.release()
    cv.destroyAllWindows()
1.5. OpenCV pixel operation  
1.5.1. Pixel operation, we can change any position to a new pixel color.  
First, we need to read the image, then modify the value of bgr and assign an area to be black.  
1.5.2. Code and actual effect display  
run the program  
python ~/yahboomcar_ws/src/yahboomcar_astra/scripts/opencv/1_4.py
import cv2
if __name__ == '__main__':
    img = cv2.imread('yahboom.jpg')
    (b,g,r) = img[100,100]
    print(b,g,r)
    i=j=0
    for j in range(1,255):
        img[i,j] = (0,0,0)
        for j in range(1,255):
            img[i,j] = (0,0,0)
    while True :
        cv2.imshow("frame",img)
        action = cv2.waitKey(10) & 0xFF
        if action == ord('q') or action == 113:
            break
    img.release()
    cv2.destroyAllWindows()
The red box part is the modified color value.  

---

## 1. KNN recognizes handwritten digits.pdf

1 KNN recognizes handwritten digits  
1 KNN recognizes handwritten digits 
1.1 KNN(K-nearest neighbor algorithm) to recognize handwritten digits 
1.1.1 Introduction to KNN 
1.1.2 Taking the recognition of handwritten digits as an example to introduce the 
implementation of the KNN algorithm 
1.1 KNN(K-nearest neighbor algorithm) to recognize
handwritten digits 
1.1.1 Introduction to KNN  
1. KNN(K-Nearest Neighbor) is a supervised learning method. Its working mechanism is very  
simple, and it does not need to train a training set. It is one of the simpler classical machine  
learning algorithms. Can handle regression and classification problems.  
2. method ideas  
In the feature space, if most of the k nearest(that is, the nearest neighbors in the feature space)  
samples near a sample belong to a certain category, the sample also belongs to this category.  
In official words, the so-called K-nearest neighbor algorithm is to give a training data set, for a  
new input instance, find the K instances closest to the instance in the training data set(that is, the  
K neighbors mentioned above).), the majority of these K instances belong to a certain class, and  
the input instance is classified into this class.  
3. working principle  
There is a sample data set, also known as a training sample set, and each data in the sample set  
has a label, that is, we know the relationship between each data in the sample set and the  
category to which it belongs. After inputting data without labels, compare each feature in the new  
data with the features corresponding to the data in the sample set, and extract the classification  
labels of the most similar data(nearest neighbors) in the sample set. Generally speaking, we only  
select the top K most similar data in the sample data set, which is the origin of K in the K nearest  
neighbor algorithm, usually K is an integer not greater than 20. Finally, the classification with the  
most occurrences in the K most similar data is selected as the classification of the new data.  
4. KNN advantages and disadvantages  
advantage  
Flexible usage, convenient for small sample prediction, high precision, insensitive to outliers,  
no data input assumptions  
shortcoming  
Lack of training phase, unable to cope with multiple samples, high computational complexity  
and high space complexity  
5. KNN implementation steps:  
Calculate distance  
Euclidean distance, that is  
Sort by increasing distance  
Select the K points with the smallest distance(generally no more than 20)  
Determine the frequency of occurrence of the category in which the first K points belong,  
frequency = a category/k  
Returns the category with the highest frequency in the top K points as the predicted  
classification of the test data  
1.1.2 Taking the recognition of handwritten digits as an example to
introduce the implementation of the KNN algorithm 
1. data set  
Training set  
The training set is already classified data, you can refer to the directory ~/KNN/knn-
digits/trainingDigits  
Note: The training set has been trained. If the user wants to train by himself, he needs to  
back up the original training set first. The number of training sets is large.  
test set  
 The test set is used to test the algorithm, you can refer to the directory ~/KNN/knn-
digits/testDigits  
2. use the drawing function under windows to make handwritten digital pictures  
Open the drawing software, adjust the resolution to 32*32, or other resolutions, but it is best  
to draw to fill the entire interval, otherwise the error rate is very high.  
Hold down ctrl and slide the mouse wheel up to enlarge the picture to the maximum. Pick  
the Paint Bucket Tool and choose a black color to fill the entire screen.  

Pick the Pencil Tool, choose White for the Color, and the Maximum Width for the Line  
Thickness. As shown below:  
After drawing, save it as a png image(this example takes 8.png as an example), and copy it to the  
project directory through the WinSCP tool  
3. Convert the image(.img) to text(.txt)  
code  
Code location: ~/KNN/img2file.py  
img_path: image file name  
txt_name: After conversion, save it in the ~/KNN/knn-digits/testDigits directory, the name is  
8_1.txt  
run the program  
After the program runs, a file named 8_1.txt will be generated in the KNN directory. Double-click  
to open it and you can see that this is the case.  from  PIL  import  Image 
import  numpy  as  np 
 def img2txt(img_path, txt_name): 
    im  =  Image.open(img_path). convert('1'). resize((32, 32))   # 
type:Image.Image 
    data  =  np.asarray(im) 
    np.savetxt(txt_name, data, fmt = '%d', delimiter = '') 
img2txt("8.png", "./knn-digits/testDigits/8_1.txt") 
cd KNN 
python img2file.py 
It can be seen that the part with the number 8 is roughly surrounded by a number 8.  
4. run the KNN recognition algorithm program  
code  
Code location: ~/KNN/knn.py  In the ~/KNN directory, open a terminal and run 
python knn.py 
#!/usr/bin/env python3 
# -*- coding: utf-8 -*- 
""" 
Created on Sun Apr 16 14:53:46 2017 
@author: jiangkang 
""" 
import  numpy 
import  operator 
import  os 
import  matplotlib.pyplot  as  plt 
 def img2vector(filename): 
    returnVect  =  numpy.zeros((1, 1024)) 
    file  =  open(filename) 
    for  i  in  range(32): 
        lineStr  =  file.readline() 
        for  j  in  range(32): 
            returnVect [ 0, 32  *  i  +  j ] =  int(lineStr [ j ]) 
    return  returnVect 
 def classifier(inX, dataSet, labels, k): 
    dataSetSize  =  dataSet.shape [ 0 ] 
    diffMat = numpy.tile(inX,(dataSetSize, 1)) -  dataSet 
    sqDiffMat = diffMat  **  2 
    sqDistances  =  sqDiffMat.sum(axis = 1) 
    distances  =  sqDistances  **  0.5 
    sortedDistIndicies  =  distances.argsort() 
    classCount  = {} 
    for  i  in  range(k): 
        voteIlabel  =  labels [ sortedDistIndicies [ i ]] 
        classCount [ voteIlabel ] =  classCount.get(voteIlabel, 0)  +  1 
    sortedClassCount  =  sorted(classCount.items(), key = 
operator.itemgetter(1), reverse = True) 
    return  sortedClassCount [ 0 ][ 0 ] 
#Train first, then test recognition, k represents the k value in the algorithm - 
select the top K most similar data in the sample data set, the k value is 
generally an integer not greater than 20 
 def handWritingClassTest(k): 
    hwLabels  = [] 
    trainingFileList = os.listdir('knn-digits/trainingDigits') training set data 
    m  =  len(trainingFileList) 
    trainingMat  =  numpy.zeros((m, 1024)) 
    for  i  in  range(m): 
        fileNameStr  =  trainingFileList [ i ] 
        fileStr  =  fileNameStr.split('.')[ 0 ] 
        classNumStr  =  int(fileStr.split('_')[ 0 ]) 
        hwLabels.append(classNumStr) 
        trainingMat [ i, :] =  img2vector("knn-digits/trainingDigits/%s"  % 
 fileNameStr) 
    testFileList = os.listdir('knn-digits/testDigits') test set data 
    errorCount  =  0.0 
    mTest  =  len(testFileList) 
    for  i  in  range(mTest): 
        fileNameStr  =  testFileList [ i ] 
        fileStr  =  fileNameStr.split('.')[ 0 ] 
        classNumStr  =  int(fileStr.split('_')[ 0 ]) 
        vectorTest  =  img2vector("knn-digits/testDigits/%s"  %  fileNameStr) 
        result  =  classifier(vectorTest, trainingMat, hwLabels, k) 
        print("The classification result is: %d, the real result is: %d"  %
(result, classNumStr)) 
        if  result  !=  classNumStr : 
            errorCount  +=  1.0 
    '''fileStr = "2.txt" 
run screenshot  
As shown in the figure, the identification is 8, and the identification is correct. If the recognition  
result is different from the real result, please copy the converted txt file to knn-
digits/trainingDigits/ and name it as follows: 8_801.txt, and then retrain it to recognize it normally.  
Program Description  
The name of the text file converted from the picture, taking 8_1 as an example, knn.py will  
parse the file name in the program, the underscore is the limit, the front 8 represents the  
real number, and the latter part can be customized, see the code,  
  Train first, then identify.      classNumStr = int(fileStr.split('.')[0]) 
    vectorTest = img2vector("./2.txt") 
    result = classifier(vectorTest, trainingMat, hwLabels, 3)''' 
    print("Total number of errors: %d"  %  errorCount) 
    print("Error rate: %f"  %(errorCount / mTest)) 
    return  errorCount 
handWritingClassTest(1) 
 classNumStr = int(fileStr.split('_')[0]) 

---

## 2. Linux basics.pdf

2. Linux basics  
2. Linux basics 
2.1. Introduction to Linux system 
2.2. Ubuntu overview 
2.3. Ubuntu file system 
2.4. Commo n comma nds 
2.4.1. increase 
2.4.2. delete 
2.4.3. change 
2.4.4. Check 
2.4.5. Others 
2.5. Editor 
2.5.1.  vim 
2.5.2. nano 
2.5.3. gedit 
2.6. Ubuntu software operation comma nd 
2.1. Introduction to Linux system  
Linux is an open source operating system whose kernel was first released on October 5, 1991 by  
Linus Benadict Torvalds. It inherits the network-centric design idea of Unix and is a performance  
Stable multi-user network operating system.  
In March 1994, Linux 1.0 was released with a code volume of 170,000 lines. At that time, it was  
released under a completely free and free agreement, and then the GPL agreement was officially  
adopted.  
In January 1995, Bob Young founded RedHat (Little Red Hat), with GNU and Linux as the core,  
integrating more than 400 open source program modules, and came up with a brand of Linux,  
namely RedHat Linux, called RedHat Linux. Linux distribution, sold in the market.  
In June 1996, the Linux 2.0 kernel was released, which can support multiple processors.  
Main features of Linux  
Free and open source; fully compatible with POSIX 1.0 standard; multi-user, multi-task; has a good  
interface; supports multiple platforms.  
Linux major version  
There are currently about 300 Linux distributions, almost all of which can be run as server  
systems.  Linux distributions rarely copy each other, and the popular Linux server distributions  
are mainly the following:  
Red Hat Enterprise Linux: This is the first Linux distribution for the commercial market. It is  
available in server versions and supports numerous processor architectures.  
Debian: Debian is extremely stable, which makes it ideal for servers.  
CentOS: CentOS is an enterprise Linux distribution rebuilt from free source code from Red Hat  
Enterprise Linux. This refactored version completely removes the registered trademark and a very  
subtle change in the Binary package.  
Ubuntu: Ubuntu is a derivative of Debian that focuses on its use in this market, common on  
servers, cloud computing, and even some mobile devices running Ubuntu Linux.  
2.2. Ubuntu overview  
Ubuntu is a Linux operating system based on desktop applications.  Ubuntu is based on the  
Debian distribution and the Gnome desktop environment, and since version 11.04, the Ubuntu  
distribution has abandoned the Gnome desktop environment in favor of Unity. Since Ubuntu  
18.04 LTS, Ubuntu distributions are back to using the GNOME3 desktop environment. Debian-
based Ubuntu has a place on almost every Linux-related list.  Canonical's Ubuntu trumps all other  
Linux server distributions - From simple installation, excellent hardware discovery, to world-class  
commercial support, Ubuntu sets standards that are hard to match  
2.3. Ubuntu file system  
Ubuntu is different from Windows, there is no concept of drive letter, there is only one root  
directory [/], all files are under it  
├──  bin           # bin is the abbreviation of Binary. Stores the most commonly 
used executable files (binaries) in the system. 
├──  boot          # The Linux kernel and system boot files are stored here, 
including Grub and lilo launcher programs. 
├──  dev           # dev is the abbreviation of Device. This directory stores 
the external devices of Linux, such as hard disk, partition, keyboard, mouse, 
usb, etc. 
├──  etc           # This directory is used to store all configuration files and 
subdirectories required for system management, such as passwd, hostname, etc. 
├──  home          # The home directory of the user. In Linux, each user has a 
directory of his own. Generally, the directory name is named after the user's 
account. 
│    └──  yahboom         # user 
│        ├──  Desktop     # Desktop 
│        ├──  Documents   # Documents 
│        ├──  Downloads   # Download 
│        ├──  Music       # Music 
│        ├──  Pictures    # Pictures 
│        ├──  Public      # Share 
│        ├──  Templates   # Templates 
│        ├──  Videos      # Video 
│ ... 
│    ...  
├──  lost + found    # This directory is usually empty. When the system is shut 
down illegally, some scattered files are stored here. 
├──  lib           # Stores shared library files, including many library files 
used by programs in /bin and /sbin. 
├──  media         # The CD-ROM and USB devices automatically mounted by the 
ubuntu system, which store the temporarily read files. 
├──  mnt           # As the mount point of the mounted file system. 
├──  opt           # As a storage directory for optional files and programs, it 
is mainly used by third-party developers to easily install and uninstall their 
software. 
├──  proc          # This directory is a virtual directory, which is the mapping 
of the system memory, where all processes marked as files are stored, and cpuinfo 
stores the data of the current working state of the cpu. 
├──  root          # This directory is the home directory of the system 
administrator, also known as the super-authorized user. 
-i to execute interactively
-f Force delete, ignore non-existing files without prompting
-r Recursively delete the contents of a directory2.4. Common commands  
2.4.1. increase  
create a new file  
new folder  
copy  
2.4.2. delete  ├──  sbin          # s means Super User, which stores system management programs 
used by system administrators, such as system management, directory query and 
other key command files. 
├──  srv           # Store the service data provided by the system. 
├──  sys           # System device and file hierarchy, and provide detailed 
kernel data information to user programs. 
├──  usr           # Store files and directories related to system users. 
│    ├──  bin       # Standard commands for users and administrators 
│    ├──  games     # Stores the small games that come with XteamLinux 
│    ├──  include   # Used to store the header files needed to develop and 
compile applications under Linux, c or c++ 
│    ├──  lib       # Link library for applications and packages 
│    ├──  local     # System administrator installed application directory 
│    ├──  sbin      # Stores the management program used by the root superuser 
│    └──  src       # Linux open source code 
│    └──  share     # Store shared data 
│    ... 
├──  var           # Variable-length files, especially log data such as log 
files and printer files. 
│    ├──  backups 
│    ├──  cache     # Application cache directory 
│    ├──  crash     # System error message 
│    ├──  log       # log file 
│    ├──  mail      # email 
│    └──  tmp       # Temporary file directory 
│    ... 
├──  tmp           # This directory is used to store some temporary files, and 
all users have read and write permissions to this directory. 
... 
touch test.txt 
mkdir test          # create a file 
mkdir -p test/src   # create the test folder and create the src folder in the 
test folder 
sudo cp test.txt test_copy.txt # copy a file 
symbol meaning
+ Increase permissions
- Revoke permission
= set permissions
Letter
permissionsmeaning
rread means read permission. For a directory, if there is no r permission, it
means that the contents of this directory cannot be viewed through ls.
inwrite means writable permission. For a directory, if there is no w
permission, it means that new files cannot be created in the directory.
xexecute means executable permission. For a directory, if there is no x
permission, it means that the directory cannot be entered through cd.2.4.3. change  
mv move, rename  
chmod changes file permissions  
Permission setting  
rwx 
Shortcut to add all permissions  
change Password  
set root password  
Set username and password  sudo rm test.txt      # delete file
sudo rm -r test      # delete the folder and its contents 
sudo mv test test_new           # Change the test folder to test_new 
sudo mv test.txt test_new.txt   # Modify the test.txt file to test_new.txt 
sudo chmod +rwx test.txt   
sudo chmod 777 test.txt   
sudo passwd root 
sudo passwd username 
2.4.4. Check  
View system version  
View hardware information  
View file information  
tree installation command  
find files  
2.4.5. Others  
tar command  
tar uses the format: tar [parameter] package file name file  lsb_release -a      # release version number 
uname -a            # Kernel version and system bitness 
cat /proc/version   # kernel version and gcc version 
curl  cip.cc or ifconfig     # View IP address 
cat /proc/cpuinfo or lscpu   # cpu info 
sudo dmidecode -t memory     # memory information 
df -h                        # View the space of all mounted file systems 
which python3                # View command location 
v4l2-ctl --list-formats-ext  # View camera device parameters 
nproc                        # View the number of cores 
la              # Display all subdirectories and files in the specified 
directory, including hidden files 
ll              # display the details of the file in a list 
ls -h           # cooperate to display the file size in a user-friendly way 
cat test.txt    # View file content 
tree            # View the file directory (requires tree installation) 
sudo apt install tree 
find ./ -name  test.sh    # Find all files or directories named test.sh in the 
current directory 
find ./ -name  '*.sh'     # Find all files or directories with the suffix .sh in 
the current directory 
find ./ -name  "[A-Z]*"   # Find all files or directories starting with an 
uppercase letter in the current directory 
-c # Generate archive file, create package file 
-v # List the detailed process of archive unarchiving, showing progress 
-f # Specify the name of the archive file, f must be followed by a .tar file, so 
you must put the option at the end 
-t # list the files contained in the 
-x # unpack   the archive 
Pack  
unpack  
zip, unzip commands  
Compressed file: zip [-r] destination file (without extension) source file  
Unzip the file: unzip -d unzip the directory file compressed file  
ln command  
Soft link: Soft link does not occupy disk space, and the soft link is invalid if the source file is  
deleted. Commonly used, you can create files or folders  
Hard links: Hard links can only link ordinary files, not directories. Linked file exists even though  
source file is deleted  
scp remote copy  
wget file download  
Just search for an image address on Baidu as an example.  tar -cvf xxx.tar *                 # all files in the current directory 
tar -cvf xxx.tar *.txt             # files ending in .txt 
tar -cvf xxx.tar my-file my-dir    # Package the specified directory or file 
tar -xvf xxx.tar              # unpack to current directory 
tar -xvf xxx.tar -C my-dir    # Unpack to the specified directory (need to 
create the my-dir directory first) 
zip bak *        # All files in the current directory, you can also specify a 
file 
zip -r bak *     # all files & directories in current directory recursively 
unzip -d ./target_dir bak.zip  # Unzip to the   specified directory 
unzip bak.zip                  # unzip to current directory 
ln  -s source-file  link-file 
ln source-file link-file 
scp jetson@192.168.16.66:/home/jetson/xxx/yahboom/xxx.tar.gz  /home/yahboom/ # 
Copy  from remote to local 
scp /home/yahboom/xxx.png jetson@192.168.16.66:/home/jetson/ # local to  remote 
scp -r jetson@192.168.16.66:/home/jetson/test /home/yahboom/    # from remote to 
local -r 
scp -r /home/yahboom/test jetson@192.168.16.66:/home/jetson/    # from local to 
remote -r 
other  
2.5. Editor  
2.5.1. vim  
vim is an upgraded version of vi. The most common difference is that it can display some special  
information of system files in multiple colors.  
install command  
Three main modes  
Command mode (edit mode): default mode, move the cursor, cut/paste text (interface  
performance: the file name is displayed in the lower left corner or empty)  Insert mode (input  
mode): modify text (interface performance: the lower left corner displays —INSERT– ) In the insert  
mode, press the ESC key to return to the  last line mode of the command mode (extended mode):  
save, exit, etc. (interface performance: the lower left corner displays -VISUAL-) In the last line  
mode, press the ESC key twice continuously to return to the last line mode  
Mode switch  
Switch from command mode to edit mode  
Switch from command mode to last line mode  
Switch from last line mode to command mode: press 【 esc 】 
Switch from editing mode to command mode: press [esc]  wget 
"https://img0.baidu.com/it/u=3911542037,2006161295&fm=224&fmt=auto&gp=0.jpg"     
             # Download ordinary files (Baidu links should be double-quoted) 
wget -O yahboom.jpg 
"https://img0.baidu.com/it/u=3911542037,2006161295&fm=224&fmt=auto&gp=0.jpg"   # 
Save the file with the specified file name 
nautilus . # open the current file 
cd ~                 # Switch to the current user's home directory (/home/user 
directory) 
cd .                 # Change to current directory 
cd -                 # can enter the last directory 
cd /                 # Change to the system root directory/ 
pwd                  # show the current path 
echo "HelloWorld"    # Output HelloWorld information to the console 
which                # View command location 
sudo apt install vim 
i     # Insert mode to enter edit mode 
a     # Append mode to enter edit mode 
o     # Start editing at the beginning of the next line of the current line 
O     # Start editing at the beginning of the previous line of the current line 
: # Enter a colon [:] 
Esc built: exit to the current mode  
Esc build Esc build: always return to command mode  
last line mode  
command mode  w            # save 
q            # exit 
q!           # force quit 
x            # save and exit 
set nu       # display line number 
set          # nonu hide line numbers 
0,$d         # vim delete the entire contents of the file: 
/string      # Start looking for the string string backwards from the cursor; 
press n to locate the next one, and shfit+n to locate the previous one. 
g/string     # Retrieve string. Stops the cursor at the beginning of the first 
retrieved string of strings. 
yy         # copy 
p          # paste 
3yy        # Copy 3 lines 
2p         # paste 2 times 
dd         # cut 
3dd        # cut 3 lines 
u          # undo 
Ctrl + r   # Undo 
dd         # delete the current line 
dG         # delete the current line to the end of the file 
dH         # delete the current line to the beginning of the file 
gg         # jump to the first line of the current document 
G          # jump to the end line of the current document 
^          # Jump to the beginning of the current line 
$          # jump to the end of the current line 
h          # move left one character 
j          # move down one line 
k          # move up one line 
l          # move one character to the right 
PageDown(or Ctrl + F) # scroll down a screen 
PageUp(or Ctrl + B)   # page up one screen 
2.5.2. nano  
nano is a text editor for Unix and Unix-like systems, a clone of Pico.  
Install  
new/open file  
control commands  
2.5.3. gedit  
gedit is no different from using Notepad under Windows.  
In the editor, we can click the "Open" button to browse the list of recently opened files and open  
the file; click the "Save" button to save the file currently being edited; click the menu bar on the  
right to perform more operations and so on.  The shortcut keys are the same as under Windows:  
The gedit editor must be started without a remote interface such as ssh, jupyter, and putty when  
the interface can be displayed.  
2.6. Ubuntu software operation command  sudo apt install nano 
nano path + file name 
For example: nano test_nano.txt 
Ctrl + v      # Next page 
Ctrl + y      # Previous page 
Ctrl + w      # Search for a word or phrase 
Ctrl + x      # close the current text, exit nano, return to shell 
Ctrl + \      # Search and replace 
Ctrl + k      # cut line of text 
Ctrl + u      # Paste line of text 
Ctrl + c      # Display cursor position in text 
Ctrl + s             save the file 
Ctrl + Shift + s     save as 
Ctrl + f             to search text content 
sudo  apt-get update                             # update source 
sudo  apt-get install package                    # install package 
sudo  apt-get remove package                     # remove the package 
sudo  apt-cache search package                   # search for packages 
sudo  apt-cache show package                     # Get information about the 
package, such as description, size, version, etc. 
sudo  apt-get install package --reinstall        # reinstall the package 
sudo  apt-get -f install                         # repair installation 
sudo  apt-get remove package --purge             # Remove packages, including 
configuration files, etc. 
sudo  apt-get build-dep package                  # Install the relevant 
compilation environment 
sudo  apt-get upgrade                            # update installed packages 
sudo  apt-get dist-upgrade                       # upgrade the system 
 
 sudo  apt-cache depends package                  # Know which packages to use 
this package depend on 
sudo  apt-cache rdepends package                 # See which packages the 
package depends on 
sudo  apt-get source package                     # Download the source code of 
the package 
sudo  apt-get clean && sudo apt-get autoclean    # clean useless packages 
sudo  apt-get check                              # Check for broken dependencies 

---

## 2. Open Source CV geometric transformation.pdf

2. Open Source CV geometric transformation  
2. Ope n Source CV geometric transformation 
2.1. Ope nCV image scaling 
2.1.1. In Ope nCV, the function to achieve image scaling is:  cv2.resize(InputArray src, 
OutputArray dst, Size, fx, fy, interpolation) 
2.1.2. Code and actual effect display 
2.2. Ope nCV image cropping 
2.2.1. picture cutting 
2.2.2. Code and actual effect display 
2.3. Ope nCV image translation 
2.3.1. In Ope nCV, image translation is achieved through affine transformation. The method 
used is cv2.warpAffine(src, M, dsize[,dst[, flags[,  borderMode[, borderValue]]]]) 
2.3.2. How to get the transformation matrix M? An example is given below, 
2.3.3. Code and actual effect display 
2.4. Ope nCV image mirroring 
2.4.1 The principle of image mirroring 
2.4.2. Taking vertical transformation as an example, let's see how Python is written 
2.1. OpenCV image scaling  
2.1.1. In OpenCV, the function to achieve image scaling is:
cv2.resize(InputArray src, OutputArray dst, Size, fx, fy, interpolation) 
Parameter meaning:  
InputArray src: input image  
OutputArray ds: output image  
Size: output image size  
fx,fy: scaling factors along the x-axis, y-axis  
interpolation: interpolation method, you can choose INTER_NEAREST (nearest neighbor  
interpolation), INTER_LINEAR (bilinear interpolation (default setting)), INTER_AREA (resampling  
using pixel area relationship), INTER_CUBIC (bicubic interpolation of 4x4 pixel neighborhood),  
INTER_LANCZOS4 (Lanczos interpolation of 8x8 pixel neighborhood)  
requires attention:  
1. The output size format is (width, height)  
2. The default interpolation method is: bilinear interpolation  
2.1.2. Code and actual effect display  
run the program  
python ~/yahboomcar_ws/src/yahboomcar_astra/scripts/opencv/2_1.py
import cv2
if __name__ == '__main__':
    img = cv2.imread('yahboom.jpg')
2.2. OpenCV image cropping  
2.2.1. picture cutting  
First read the image, and then get the pixel area from the array. In the following code, select the  
shape area X: 300-500 Y: 500-700, note that the image size is 800*800, so the selected area should  
not exceed this resolution.  
2.2.2. Code and actual effect display  
run the program      print(img.shape)
    x, y = img.shape[0:2]
    img_test1 = cv2.resize(img, (int(y / 2),   int(x / 2)))
    while True :
        cv2.imshow("frame",img)
        cv2.imshow('resize0', img_test1)
        action = cv2.waitKey(10) & 0xFF
        if action == ord('q') or action == 113:
            break
    img.release()
    cv2.destroyAllWindows()
python ~/yahboomcar_ws/src/yahboomcar_astra/scripts/opencv/2_2.py
import cv2
if __name__ == '__main__':
    img = cv2.imread('yahboom.jpg')
    dst = img[0:100,100:200]
    while True :
        cv2.imshow("frame",img)
        cv2.imshow('dst', dst)
        action = cv2.waitKey(10) & 0xFF
        if action == ord('q') or action == 113:
            break
    img.release()
    cv2.destroyAllWindows()
2.3. OpenCV image translation  
2.3.1. In OpenCV, image translation is achieved through affine
transformation. The method used is cv2.warpAffine(src, M, dsize[,dst[,
flags[, borderMode[, borderValue]]]]) 
Parameter meaning:  
src - the input image.  M - Transformation matrix.  dsize - the size of the output image.  flags -  
combination of interpolation methods (int type!)  borderMode - border pixel mode (int type!)   
borderValue - (emphasis!) border padding value; by default it is 0.  
Among the above parameters: M is used as an affine transformation matrix, which generally  
reflects the relationship of translation or rotation, and is a 2×3 transformation matrix of  
InputArray type. In daily affine transformation, only the first three parameters are set, such as  
cv2.warpAffine(img,M,(rows,cols)), the basic affine transformation effect can be achieved.  
2.3.2. How to get the transformation matrix M? An example is given
below, 
Convert the original image src to the target image dst through the transformation matrix M:  
  dst(x, y) = src(M11x + M12y+M13, M21x+M22y+M23)  
Move the original image src to the right by 200 pixels and down by 100 pixels, then the  
corresponding relationship is:  
  dst(x, y) = src(x+200, y+100)  
Complete the above expression, namely:  
  dst(x, y) = src(1·x + 0·y + 200, 0·x + 1·y + 100)  
According to the above expression, the value of each element in the corresponding  
transformation matrix M can be determined as:  
M11 = 1  
M12=0  
M13=200  
M21=0  
M22=1  
M23=100  
Substituting the above values into the transformation matrix M, we get:  
                                 M = [    ]  
2.3.3. Code and actual effect display  
run the program  
2.4. OpenCV image mirroring  
2.4.1 The principle of image mirroring  
There are two types of image mirroring transformations: horizontal mirroring and vertical  
mirroring. Horizontal mirroring takes the vertical center line of the image as the axis, and swaps  
the pixels of the image, that is, swaps the left and right halves of the image. Vertical mirroring  
takes the horizontal midline of the image as the axis and reverses the upper and lower parts of  
the image.  
Transformation principle:  
Let the width of the image be width and the length be height.  (x, y) are the transformed  
coordinates, (x0, y0) are the coordinates of the original image  
Horizontal mirror transformation  
Forward mapping: x=width-x0-1, y=y0  python ~/yahboomcar_ws/src/yahboomcar_astra/scripts/opencv/2_3.py
import cv2
import numpy as np
if __name__ == '__main__':
    img = cv2.imread('yahboom.jpg')
    imgInfo = img.shape
    height = imgInfo[0]
    width = imgInfo[1]
    matShift = np.float32([[1,0,10],[0,1,10]])# 2*3
    dst = cv2.warpAffine(img, matShift, (width,height))
    while True :
        cv2.imshow("frame",img)
        cv2.imshow('dst', dst)
        action = cv2.waitKey(10) & 0xFF
        if action == ord('q') or action == 113:
            break
    img.release()
    cv2.destroyAllWindows()
Backward mapping: x0=width-x-1,y0=y  
Vertical Mirror Transformation  
Mapping up: x=x0,y=height-y0-1  
Mapping down: x0=x, y0=height-y-1  
Summarize:  
In the horizontal mirror transformation, the entire image is traversed, and then each pixel is  
processed according to the mapping relationship. In fact, the horizontal mirror transformation is  
to change the column of image coordinates to the right, and the column on the right to the left,  
which can be transformed in units of columns. The same is true for vertical mirror  
transformations, which can be transformed in units of rows.  
2.4.2. Taking vertical transformation as an example, let's see how Python
is written 
run the program  
python ~/yahboomcar_ws/src/yahboomcar_astra/scripts/opencv/2_4.py
import cv2
import numpy as np
if __name__ == '__main__':
    img = cv2.imread('yahboom.jpg')
    imgInfo = img.shape
    height = imgInfo[0]
    width = imgInfo[1]
    deep = imgInfo[2]
    newImgInfo = (height*2,width,deep)
    dst = np.zeros(newImgInfo,np.uint8)#uint8
    for i in range(0,height):
        for j in range(0,width):
            dst[i,j] = img[i,j]
            dst[height*2-i-1,j]   = img[i,j]
    while True :
        cv2.imshow("frame",img)
        cv2.imshow('dst', dst)
        action = cv2.waitKey(10) & 0xFF
        if action == ord('q') or action == 113:
            break
    img.release()
    cv2.destroyAllWindows()
 

---

## 2.Basic use of TensorFlow.pdf

2. TensorFlow  
2.1. What is TensorFlow?  
2.1.1. Definition  
TensorFlow™ is a symbolic mathematics system based on data flow programming  (dataflow  
programming) and is widely used in various types of machine learning  Programming  
implementation of (machine learning) algorithm, its predecessor is the neural network of Google  
Network algorithm library DistBelief.
Tensorflow has a multi-level structure and can be deployed on various servers , PC terminals and 
web pages  and supports GPU  and TPU high performance Numerical calculation , is widely used in  
Google’s internal product development and scientific research in various fields
2.1.2. Core components  
The core components of distributed TensorFlow (core runtime) include: distribution center  
(distributed master), executor (dataflow executor/worker service), kernel application (kernel  
implementation) and the bottom [device layer] ( https://baike . baidu.com/item/Device  
Layer/8983070) (device layer)/ Network Layer  (networking layer).
1), distribution center (distributed master)
The distribution center clips subgraphs from the input data flow graph, divides them into  
operational fragments and starts executors. When the distribution center processes the data flow  
graph, it will perform preset operation optimizations, including common subexpression  
elimination (common subexpression elimination), constant folding (constant folding), etc.
2. The executor is responsible for running graph operations in processes and devices, and  
sending and receiving results from other executors. Distributed TensorFlow has a parameter  
server to aggregate and update model parameters returned by other executors. The  
executor will choose to perform parallel computing  and GPU acceleration when scheduling  
local devices.
Tensor type Description
tf.float32 32-bit floating point number
tf.float64 64-bit floating point number
tf.int64 64-bit signed integer type
tf.int32 32-bit signed integer type
tf.int16 16-bit signed integer type
tf.int8 8-bit signed integer type
tf.uint8 8-bit unsigned integer type
tf.string Variable length byte array3. The kernel application is responsible for a single graph operation, including mathematical  
calculations, array manipulation, control flow and state management operations. The kernel  
application uses Eigen  to perform parallel calculations of tensors, cuDNN libraries, etc. to  
perform GPU acceleration, and gemmlowp to perform low numerical precision calculations.  
In addition, users can perform low numerical precision calculations in the kernel application.  
Register additional kernels (fused kernels) to improve the efficiency of basic operations such  
as activation functions and their gradient calculations.
2.2, TensorFlow 2  
2.2.1. Introduction  
TensorFlow is a deep learning open source tool released by Google in November 2015. We can  
use it to quickly build  deep neural networks and train deep learning models . The main purpose  
of using TensorFlow and other open source frameworks is to provide us with a module toolbox  
that is more conducive to building deep learning networks, so that the code can be simplified  
during development, and the final model will be more concise and easy to understand.
2.2.2. Upgrade direction  
1. Use Keras and Eager Execution to easily build models.
2. Achieve robust production environment model deployment on any platform.
3). Provide powerful experimental tools for research.
4. Simplify the API by cleaning up obsolete APIs and reducing duplication.
2.3. TensorFlow basic concept syntax  
2.3.1. Tensor  
1. Tensor is the core data unit of TensorFlow, which is essentially an array of arbitrary  
dimensions. We call a 1-dimensional array a vector, a 2-dimensional array a matrix, and a  
tensor can be regarded as an N-dimensional array.
2. In TensorFlow, each Tensor has two basic attributes: data type (default: float32) and shape.  
The data types are roughly as shown in the following table,
Tensor type Description
tf.bool Boolean type
tf.complex64 Real and imaginary numbers
3). According to different uses, there are two main tensor types in TensorFlow, namely
tf.Variable: variable Tensor, which needs to specify an initial value and is often used to define  
variable parameters, such as the weights of neural networks.
tf.constant: constant Tensor, the initial value needs to be specified to define a tensor that  
does not change
4), define a variable Tensor
Create a new python file, name it Tensor_Variable, and then give it execution permissions.
Paste the following code inside,
run the test,
Note: ROSMASTER must use python3 to properly use tensorflow2.0 or above
output,
5), define a constant Tensor
Create a new python file, name it Tensor_constant, and then give it execution permissions.
Paste the following code inside,
run the test,sudo chmod a+x Tensor_Variable.py
import tensorflow astf
v = tf.Variable([[1, 2], [3, 4]]) # Two-dimensional variable with shape (2, 2)
print(v)
python3Tensor_Variable.py
<tf.Variable 'Variable:0' shape=(2, 2) dtype=int32, numpy=
array([[1, 2],
        [3, 4]], dtype=int32)>
sudo chmod a+x Tensor_constant.py
import tensorflow astf
v = tf.constant([[1, 2], [3, 4]]) # Two-dimensional variable with shape (2, 2)
print(v)
python3 Tensor_constant.py
output,
If you look closely, you will find that the output tensor has three attributes: shape, data type dtype,  
and NumPy array.
6. Commonly used methods to create new special constant tensors:
tf.zeros: Create a new constant Tensor with the specified shape and all 0s
Example: c = tf.zeros([2, 2]) # 2x2 constant Tensor with all 0s
output,
tf.ones_like: Refer to a certain shape and create a new constant Tensor with all 1’s
 Example: v = tf.ones_like(c) # A constant Tensor that is consistent with the shape (shape) of tensor  
c and is all 1. Note that the shape here refers to the attribute shape of the tensor
Output,
tf.fill: Create a new constant Tensor with a specified shape and all scalar values.
 Example: a = tf.fill([2, 3], 6) # 2x3 is a constant Tensor of all 6
Output,
tf.linspace: Create an equally spaced sequence
Example: c = tf.linspace(1.0, 10.0, 5, name="linspace")
Output,
tf.range: Create a sequence of numbers
Example: c = tf.range(start=2, limit=8, delta=2)
output,<tf.Tensor: id=9, shape=(2, 2), dtype=int32, numpy=
array([[1, 2],
        [3, 4]], dtype=int32)>
<tf.Tensor: id=12, shape=(2, 2), dtype=float32, numpy=
array([[0., 0.],
        [0., 0.]], dtype=float32)
<tf.Tensor: id=15, shape=(3, 3), dtype=float32, numpy=
array([[1., 1.],
        [1., 1.]],dtype=float32)
<tf.Tensor: id=18, shape=(2, 3), dtype=int32, numpy=
array([[6, 6, 6],
        [6, 6, 6]], dtype=int32)>
<tf.Tensor: id=22, shape=(5,), dtype=float32, numpy=array([ 1. , 3.25, 5.5 , 
7.75, 10. ], dtype=float32)>
For this part of the code, please refer to:
2.3.2. Use Eager Execution (dynamic graph) mechanism to perform  
operations on tensors 
1. Changes in the tensor operation mechanism of TensorFlow2 and TensorFlow1.x
The dynamic graph mechanism is the biggest difference between TensorFlow2.x and  
TensorFlow1.x. It is similar to PyTorch and simplifies the code and execution process.
2. Take tensor addition as an example to illustrate,
Create a new python file, name it tensor_plus, and then give it execution permissions.
Paste the following code inside,
run the test,
It can be found that the execution process of this python is the same, and if it is in Tersorflow1.x, a  
session needs to be established, and the addition operation in the session is performed.
3. Commonly used APIs of TensorFlow:
tf.math: Mathematical calculation module, which provides a large number of mathematical  
calculation functions.
tf.linalg: Linear algebra module, which provides a large number of linear algebra calculation  
methods and classes.
tf.image: Image processing module, which provides classes such as image cropping,  
transformation, encoding, and decoding.<tf.Tensor: id=26, shape=(5,), dtype=int32, numpy=array([2, 4, 6, 8], 
dtype=int32)>
~/TensorFlow_demo/tensor_init.py
sudo chmod a+x tensor_plus.py
a = tf.fill([2, 3], 6)
b = tf.fill([2, 3], 2)
c = a + b
print(a)
print(b)
print(c)
python3 tensor_plus.py
tf.train: Provides components for training, such as optimizers, learning rate decay strategies,  
etc.
tf.nn: Provides underlying functions for building neural networks to help implement various  
functional layers of deep neural networks.
tf.keras: the high-level API of the original Keras framework. Contains high-order neural  
network layers in the original tf.layers.
tf.data: Input data processing module, which provides classes such as tf.data.Dataset for  
encapsulating input data, specifying batch size, etc.
For usage of these commonly used APIs, please refer to the official documentation:
Module: tf | TensorFlow Core v2.8.0 (google.cn)
2.3.3. Neural network  
1. Neural network  is a mathematical model that exists in the nervous system of a computer. It  
is connected by a large number of neurons and performs calculations. It changes the internal  
structure based on external information and is often used to process input. Model complex  
relationships between outputs. The structure of a basic neural network has an input layer, a  
hidden layer, and an output layer. The picture below is a neural network diagram,
2. Input layer: receives sensory information;
3. Hidden layer: processing of input information;
4). Output layer: outputs the computer’s understanding of the input information.
2.3.4. Build and train neural network ( kear )
1), import data
For more information about data, please refer to: tf.data.Dataset | TensorFlow Core v2.8.0  
(google.cn)
2. Define a structural network that describes the neural networkx_train = datasets.load_iris().data
y_train = datasets.load_iris().target
Example:
The input parameters represent the network structure from the input layer to the output layer,  
generally including the following three:
Straighten layers: tf.keras.layers.Flatten()
Please refer to the official documentation: tf.keras.layers.Flatten | TensorFlow Core v2.8.0  
(google.cn)
Fully connected layer: tf.keras.layers.Dense()
Please refer to the official documentation: tf.keras.layers.Dense | TensorFlow Core v2.8.0  
(google.cn)
Convolution layer: tf.keras.layers.Conv2D()
Please refer to the official documentation: tf.keras.layers.Conv2D | TensorFlow Core v2.8.0  
(google.cn)
3. Configure the training method for training the neural network
Example:
The input parameters are composed of the following three parts:
optimizer: optimizer
Mainly set the learning rate lr, learning decay rate decay and momentum parameters.
loss: loss function
metrics: accuracy
Multiple accuracy rates can be specified.
For the specific values of the three parameters, please refer to: tf.keras.Model | TensorFlow Core  
v2.8.0 (google.cn)
4. Execute the training processmodel = tf.keras.models.Sequential()
model = tf.keras.Sequential([
     tf.keras.layers.Dense(3, activation='softmax', 
kernel_regularizer=tf.keras.regularizers.l2())
])
model.compile( optimizer = optimizer, loss = loss function, metrics = 
["accuracy"])
model.compile(optimizer=tf.keras.optimizers.SGD(lr=0.1),
               
loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=False),
               metrics=[tf.keras.metrics.sparse_categorical_accuracy])
Example:
For specific parameter settings, please refer to: tf.keras.Model | TensorFlow Core v2.8.0  
(google.cn)
5), print network structure and parameter statistics
For specific parameter settings, please refer to: tf.keras.Model | TensorFlow Core v2.8.0  
(google.cn)
2.3.4. Training neural network example - classic example of training cat  
and dog images 
1), code path reference
2), run the program
3. Screenshots of program running
When pictures of kittens and puppies appear, in the picture display window, press the q key  
to continue executing the program. model.fit (x=input features of the training set, y=label of the training set, 
batch_size=specifies the number of samples included in each batch when performing 
gradient descent,
epochs = value at the end of training, validation_data = (input features of the 
test set, label of the test set),
validation_split = What proportion is divided from the training set to the test 
set,
validation_freq = number of epoch intervals for testing)
model.fit(x_train, y_train, batch_size=32, epochs=5, validation_data=(x_test, 
y_test), validation_freq=1)
model.summary()
~/TensorFlow_demo/cats_dogs_demo.py
cd TensorFlow_demo/
python3 cats_dogs_demo.py
The number of training epochs for this model is 10, and the number of sample batches is 10. The  
coordinate curve on the left shows that as the number of training times increases, the accuracy  
acc and error loss will increase and decrease.

---

## 2.Lidar avoiding.pdf

2. Radar obstacle avoidance  
2. Radar obstacle avoidance
2.1. How to use
2.2. Source code analysis
Introduction to radar obstacle avoidance gameplay:
Set lidar detection angle and response distance
After turning on the car, the car will drive straight when there are no obstacles.
Determine the direction in which the obstacle appears on the car (front left, front right, right  
in front)
Respond according to the position of the car where obstacles appear (turn left, turn right,  
turn left in a large circle, turn in a large right circle)
2.1. How to use  
Note: The [R2] of the remote control handle has the [pause/start] function of this gameplay.  
Due to the problem of movement method, the case in this section does not support the  
Ackerman model. Different models will have different parameter ranges, but the principle  
is the same; take the X3 McLunner as an example. 
Start with one click (robot side). After executing the command, the car starts to move.
<PI5 needs to open another terminal to enter the same docker container
#Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
roslaunch yahboomcar_laser laser_Avoidance.launch
Parameters Range Parse
[linear] [0.0, 1.0] Car linear speed
[angular] [0.0, 5.0] Car angular speed
[LaserAngle] [10, 90] Lidar detection angle (left and right angles)
[ResponseDist] [0.0, 8.0] Car response distance
[switch] [False, True] Car movement [Start/Pause]Dynamic debugging parameters
Parameter analysis:
In the box in front of [switch], click the value of [switch] to True, and the car will stop. [switch]  
Default is False, the car moves.
Parameter modification
When the parameters are adjusted to the optimal state, the corresponding parameters are  
modified into the file, and no adjustment is required when using again.
According to the optimal parameters of the [rqt_reconfigure] debugging tool, enter the [scripts]  
folder of the [yahboomcar_laser] function package and modify the parameters corresponding to  
the [laser_Avoidance.py] file, as shown belowrosrun rqt_reconfigure rqt_reconfigure
class laserAvoid:
     def __init__(self):
         rospy.on_shutdown(self.cancel)
         ... ...
         self.linear = 0.5
         self.angular = 1.0
         self.LaserAngle = 40
         self.ResponseDist = 0.55
Parameters Analysis Corresponding parameters
name The name of the parameter "linear"
type parameter data type double_t
level a bitmask passed to the callback 0
description A description parameter "linear in robot"
default Initial value for node startup 0.5
min parameter minimum value 0
max parameter maximum value 1.0[rqt_reconfigure] Modification of the initial value of the debugging tool
Enter the [cfg] folder of the [yahboomcar_laser] function package and modify the initial values of  
the parameters corresponding to the [laserAvoidancePID.cfg] file.
Take the above article as an example to analyze
Note: After modification, you must recompile and update the environment to be effective. 
Node view
【laser_Aviodance 】 node analysis
Subscribe to lidar datagen.add("linear", double_t, 0, "linear in robot", 0.5, 0, 1.0)
gen.add("angular", double_t, 0, "angular in robot", 1.0, 0, 5.0)
gen.add("LaserAngle", int_t, 0, "LaserAngle", 40, 10, 90)
gen.add("ResponseDist", double_t, 0, "ResponseDist", 0.55, 0, 8)
gen.add("switch", bool_t, 0, "switch in rosbot", False)
gen.add("linear", double_t, 0, "linear in robot", 0.5, 0, 1.0)
cd ~/yahboomcar_ws
catkin_make
source devel/setup.bash
rqt_graph
ParametersDefault
valueJudgment
self.front_warning Default is 0When the value is greater than 10, it means there is an
obstacle ahead.
self.Left_warning Default is 0When the value is greater than 10, it means there is an
obstacle in front of the left.
self.Right_warning Default is 0When the value is greater than 10, it means there is an
obstacle on the right front.Subscribe to handle information
Publish car speed
2.2. Source code analysis  
launch file
-base.launch
laser_Avoidance.launch
laser_Avoidance.py source code parameter analysis:
Programming flow chart:<launch>
     <!-- Start lidar node -->
     <!-- Activate the lidar node -->
     <include file="$(find rplidar_ros)/launch/rplidar.launch"/>
     <!-- Start the car chassis drive node-->
     <!-- Start the car chassis drive node -->
     <include file="$(find yahboomcar_bringup)/launch/yahboomcar.launch"/>
     <!-- Handle control node -->
     <!-- Handle control node -->
     <include file="$(find yahboomcar_ctrl)/launch/yahboom_joy.launch"/>
</launch>
<launch>
     <!-- Launch base.launch file -->
     <!-- Launch the base.launch file -->
     <include file="$(find yahboomcar_laser)/launch/base.launch"/>
     <!-- Start lidar obstacle avoidance node -->
     <!-- Activate lidar obstacle avoidance node -->
     <node name='laser_Avoidance' pkg="yahboomcar_laser" 
type="laser_Avoidance.py" required="true" output="screen"/>
</launch>
The car moves forward autonomously and avoids surrounding obstacles. For specific program  
design, please see the source code of [laser_Avoidance.py].

---

## 2.STM32 Development environment.pdf

2. STM32 development environment  
2. STM32 developme nt environment 
2.1. Introduction to STM32CubeIDE 
2.2. download the installation package 
2.3. start the installation 
2.4. New construction 
2.5. pin configuration 
2.6. write code 
2.7. Compiler 
2.8. Burn program 
2.9. Program phenomenon 
2.1. Introduction to STM32CubeIDE  
STM32CubeIDE is an all-in-one multi-OS development tool, an advanced C/C++ development  
platform with peripheral configuration, code generation, code compilation and debugging  
functions for STM32 microcontrollers and microprocessors. It is based on the Eclipse®/CDT™  
framework and GCC toolchain for development, and GDB for debugging, selects an STM32  
microprocessor and then creates a project and generates initialization code, supports graphical  
configuration of STM32 clocks and pins, etc. Content, support Windows, Linux, Mac mainstream  
platforms, very powerful and practical.  
2.2. download the installation package  
Open the following link with a computer browser:  
https://www.st.com/en/development-tools/stm32cubeide.html  
According to the download of the computer system, here we take the installation of version 1.9.0  
in the Win10 system as an example, select the latest version to download.  
If you have an account on my.st.com, you can log in directly and download the software. If you  
don't want to log in now, simply provide your name and email address in the form below to  
download the software.  
2.3. start the installation  
Double-click to open the installation package. Then just follow the tutorial.  
The installation path can be modified according to the actual situation, be careful not to have  
Chinese.  
Check the driver, and then click Install to install.  
Then wait for the installation to complete.  
 
2.4. New construction  
1. Double-click the shortcut on the desktop to open STM32CubeIDE. You need to select a  
workspace. The save path can be selected according to the actual path (do not bring  
Chinese).  
2. Click File->New->STM32 Project.  
3. Search and select the chip STM32F103RCT6, then click Next in the lower right corner to enter  
the next step.  
4. Enter the project name, take LED as an example here, other parameters can be defaulted.  
5. Click Yes, and the graphical content will be loaded.  
After completion, it will look like the following picture:  
 
2.5. pin configuration  
1. First need Debug information, click SYS->Debug under Pinout&Configuration and select  
Serial Wire.  
2. Modify the system clock of STM32. According to the schematic diagram, the external crystal  
oscillator is 8M frequency.  
Select RCC->HSE in Pinout&Configuration and select Crystal/Ceramic Resonator. The HSE is the  
external clock, and the LSE is the internal clock. Using the external clock can be more stable and  
efficient than the internal clock.  
Switch to Clock Configuration to modify the frequency of HCLK to 72, and press Enter to confirm.  
3. Increase the LED pin configuration. It can be seen from the schematic diagram that the LED  
is connected to the PC13 pin.  
Set the PC13 pin to GPIO_Output, and modify the Label to LED for convenience.  
Then press Ctrl+S to save, tick Remember my decision, and click Yes. This will automatically  
generate the code every time you save.  
2.6. write code  
1. Since the initialization code of the system has been generated by the graphical configuration  
in the previous step, we only need to add the functions to be implemented.  
Find the main function in the main.c file, and add the content to control the LED light under  
while(1). The function is that the LED light flashes every 200 milliseconds. Press Ctrl+S to save the  
code.  
Note: The code content needs to be added between USER CODE BEGIN and USER CODE END.  
Otherwise, the content of the code will be overwritten after the code is generated by the graphical  
tool next time, and the content added between USER CODE BEGIN and USER CODE END will not  
be overwritten. Don't write Chinese comments in it, there may be garbled characters.  
2.7. Compiler  
1. Add the function of generating HEX files.  
Click Project->Properties->C/C++ Build->Settings->MCU Post build outputs in turn, and then check  
the box before Convert to Intel Hex file(-O ihex), as shown in the following figure.  

2. Click the hammer in the toolbar to start compiling the project.  
STM32CubeIDE will pop up the Console console and see 0 errors in compilation, and 0 warnings  
means the compilation is successful. As shown in the figure below, the file name generated by the  
project is LED.hex, and this file is saved in the Debug folder of the project directory.  

2.8. Burn program  
1. Install the CH340 driver  
Since the USB communication of the Rosmaster expansion board uses the CH340 chip, the driver  
of the CH340 chip needs to be installed. If the computer has already installed the CH340 driver,  
there is no need to install it again.  
Unzip the [Uart drive (CH340).zip] in the course materials, double-click to open the CH341SER.EXE  
program  
 
Click Install. After the installation is complete, you will be prompted that the installation was  
successful.  
2. Download the burning software  
This time burning Rosmaster expansion board microcontroller firmware requires mcuisp (or  
flymcu) burning software, please go to http://www.mcuisp.com website to download mcuisp (or  
flymcu) burning software; you can also use the data provided directly mcuisp software.  
The mcuisp software is a green version software, which does not need to be installed. Double-
click to open it to use.  
3. Connect the device  
Before connecting the Rosmaster to the computer, please unplug the Micro USB data cable and  
power cable connecting the expansion board to the Jetson Nano.  
Insert one end of the USB data cable into the USB port of the computer, and the other end into  
the Micro USB port of the Rosmaster expansion board.  
4. Configure the burning software  
When searching for a serial port, if there are multiple serial port numbers, it is not confirmed  
which one is the Rosmaster. Solution 1: Unplug other USB ports and search again; Solution 2: First  
unplug the Rosmaster USB data cable, click Search Serial Port, write down the searched serial port  
number, insert the Rosmaster USB data cable, search the serial port again, and compare before  
and after Twice, the newly added serial port number is the serial port number of the Rosmaster.  
When selecting firmware, you need to select the LED.hex file in the Debug folder of the project  
directory.  
The last is the configuration selection at the bottom. Be sure to select the option of [DTR low-level  
reset, RTS high-level into BootLoader] option, otherwise the download may fail.  
5. Burn program  
Please put the microcontroller on the expansion board into the programming mode first:  
First press and hold the BOOT0 key on the expansion board, then press the RESET key, and finally  
release the BOOT0 key.  
Click [Start Programming], and the mcuisp burning software will burn the firmware we selected in  
the previous step to the microcontroller on the Rosmaster expansion board. When the prompt  
appears on the right side [ www.mcuisp.com:Mission  Complete,Anything Ok!!!], it means that the  
download is successful.  
Notice:  
①Before starting programming, please confirm that the serial port number of the Rosmaster is  
accessible, that is, there is no serial port assistant occupying it.  
②Rosmaster enters the burning mode operation, first press and hold the BOOT0 key on the  
expansion board, then press the RESET key, and finally release the BOOT0 key.  
2.9. Program phenomenon  
The LEDs on the expansion board flash every 200ms.  

---

## 2.Turn off the self-starting process at startup.pdf

2. Close the large self-starting program at boot  
2. Close the large self-starting program at boot
2.1. What is a large program that starts automatically at boot?
2.2. Tempo rarily close large programs
2.3. Permanently close large programs
2.4. Set up the startup program
2.5. Tempo rarily start a large program
2.1. What is a large program that starts automatically at  
boot? 
In order to facilitate the experience of the APP function of the car, a program has been added to  
the system. This program integrates the control functions and gameplay of the APP, so it is called  
the "big program". Moreover, when the motherboard system is turned on, this program will  
automatically start, so it is called Do "self-starting large programs at boot".
The large program that automatically starts at startup is just for the convenience of experiencing  
the functions of the mobile APP. In actual development, the large program needs to be closed,  
otherwise it will occupy the device and cause unpredictable errors. Therefore, before developing a  
program, please manually close the large auto-start program at startup.
There are two ways to close the large self-starting program at startup. One is to close it only once  
and it will start automatically the next time you turn on the computer. This is called temporary  
shutdown. The other way is to close it and it will not start automatically again the next time you  
turn on the computer unless you manually restart it. Turning on is called permanent turning off.
The default username and password of the factory system are as follows:
jetson series motherboard
Username: jetson Password: yahboom
Raspberry Pi motherboard
Username: pi Password: yahboom
 
2.2. Temporarily close large programs  
If you have a 7-inch touch screen or display and mouse and keyboard, log in to the desktop after  
connecting.
If there is no display screen, mouse and keyboard, please use a computer in the same LAN to  
open the VNC Viewer software and log in to the desktop remotely to enter the desktop.
For detailed operations on the VNC remote login method, please refer to the following web page.  
The configuration content has been configured. Jump directly to step 6 to connect.
VNC Remote Desktop Configuration (yahboom.com)
Username: jetson
Password:yahboom
After entering the desktop, you will see a terminal. Just click the X symbol in the upper left corner  
to close the terminal to close the large program. Sometimes you may be prompted that closing  
the terminal will close the running program. Just select Confirm to close.
 
2.3. Permanently close large programs  
First, close the large running program by temporarily closing it.
After opening the Ubuntu system application, search for Startup Applications, and uncheck the  
check mark in front of start_rosmaster_app, as shown in the figure below, to permanently close  
the large program.
Raspberry Pi 5
 
2.4. Set up the startup program  
Open the Ubuntu system application, search for Startup Applications, and check the box in front  
of start_rosmaster_app, as shown in the figure below, then the large program will automatically  
start next time the system is turned on.sudo rm -rf /home/pi/.config/autostart/rosmaster.desktop
Raspberry Pi 5
 
2.5. Temporarily start a large program  
If you need to manually run a large program, please open the Ubuntu terminal first, and then  
enter the following command:
Raspberry Pi 5sudo cp -r /home/pi/Rosmaster/rosmaster/rosmaster.desktop 
/home/pi/.config/autostart/
sudo chown -R pi:pi /home/pi/.config/autostart/rosmaster.desktop
python3 ~/Rosmaster-App/rosmaster/rosmaster_main.py
python3 /home/pi/Rosmaster/rosmaster/rosmaster_main.py

---

## 3. Basic use of PyTorch.pdf

3 Basic use of PyTorch  
3 Basic use of PyTorch 
3.1 About PyTorch 
3.1.1 Introduction 
3.1.2 Features 
3.2 Tensors in PyTorch 
3.2.1 Tensor 
3.2.2 create a tensor 
3.3 torchvision package introduction 
3.3.1 torchvision is a library dedicated to processing images in Pytorch, including four 
categories: 
3.4 Convolutional Neural Networks 
3.4.1 Neural network 
3.4.2 Convolutional Neural Networks 
3.5 Build the LetNet neural network and train the data set 
3.5.1 Preparation before implementation 
3.5.2 Implementation process 
3.5.3 Running the program 
The Raspberry Pi motherboard series does not support the PyTorch function yet.  
3.1 About PyTorch  
3.1.1 Introduction  
PyTorch is an source open Python machine learning library, based on Torch, for applications such  
as natural language processing.  
3.1.2 Features  
1. powerful GPU-accelerated tensor computing  
2. deep neural network of automatic derivation system  
3. dynamic graph mechanism  
3.2 Tensors in PyTorch  
3.2.1 Tensor  
The English of tensor is Tensor, which is the basic operation unit in PyTorch. Like Numpy's  
ndarray, it represents a multi-dimensional matrix. The biggest difference from ndarray is that  
PyTorch's Tensor can run on GPU, while numpy's ndarray can only run on CPU, and running on  
GPU greatly speeds up the operation.  
3.2.2 create a tensor  
1. There are many ways to create tensors. Calling APIs of different interfaces can create  
different types of tensors.  
a = torch.empty(2,2): create an uninitialized 2*2 tensor  
b = torch.rand(5, 6) : creates a uniformly distributed initialization tensor with each element from  
0-1 
c = torch.zeros(5, 5, dtype=torch.long): Create an initialized all-zero tensor and specify the type of  
each element as long  
d = c.new_ones(5, 3, dtype=torch.double) : create a new tensor d based on a known tensor c  
d.size(): Get the shape of the tensor d  
2. Operations between tensors  
The operation between tensors is actually the operation between matrices. Due to the dynamic  
graph mechanism, mathematical calculations can be performed directly on the tensors. For  
example,  
Add two tensors:  
Multiply two tensors  
This part of the code can be referred to: ~/Pytorch_demo/torch_tensor.py  
run the code,  c = torch.zeros(5,3,dtype=torch.long) 
d = torch.ones(5,3,dtype=torch.long) 
e = c + d 
print(e) 
c = torch.zeros(5,3,dtype=torch.long) 
d = torch.ones(5,3,dtype=torch.long) 
e = c * d 
print(e) 
python3.6 torch_tensor.py
3.3 torchvision package introduction  
3.3.1 torchvision is a library dedicated to processing images in Pytorch,
including four categories: 
1. torchvision.datasets: load datasets, Pytorch has many datasets such as CIFAR, MNIST, etc.,  
you can use this class to load datasets, the usage is as follows:  
2. torchvision.models: Load the trained model, the model includes the following VCG, ResNet,  
etc. The usage is as follows:  
3. torchvision.transforms: the class of image transformation operation, the usage is as follows:  
4. torchvision.untils: Arrange the pictures into a grid shape, the usage is as follows:  
For more information on the use of the torchvision package, you can refer to the official website  
documentation: https://pytorch.org/vision/0.8/datasets.html  
3.4 Convolutional Neural Networks  cifar_train_data  =  torchvision.datasets.CIFAR10(root = './data', train = True, 
                                          download = False, transform = 
transform) 
import  torchvision.models  as  models 
resnet18  =  models.resnet18() 
transform  =  transforms.Compose(
    [ transforms.ToTensor(), 
     transforms.Normalize((0.5, 0.5, 0.5),(0.5, 0.5, 0.5))]) 
torchvision.utils.make_grid(tensor, nrow = 8, padding = 2, normalize = False, 
range = None, scale_each = False, pad_value = 0) 
3.4.1 Neural network  
1. The difference between neural network and machine learning  
Both neural networks and machine learning are used for classification tasks. The difference is that  
neural networks are more efficient than machine learning, the data is simpler, and fewer  
parameters are required to perform tasks. The following points are explained:  
Efficiency: The efficiency of neural networks is reflected in the extraction of features. It is  
different from the features of machine learning. It can be trained and "corrected" by itself.  
We only need to input data, and it will continuously update the features by itself.  
Data simplicity: In the process of machine learning, we need to perform some processing on  
the data before inputting the data, such as normalization, format conversion, etc., but in the  
neural network, it does not require too much processing.  
Fewer parameters to perform tasks: In machine learning, we need to adjust penalty factors,  
slack variables, etc. to tune to the most suitable effect, but for neural networks, only a weight  
w and bias term b need to be given, During the training process, these two values will be  
continuously revised and adjusted to the optimum, so that the error of the model is  
minimized.  
3.4.2 Convolutional Neural Networks  
1. convolution kernel  
Convolution kernels can be understood as feature extractors, filters(digital signal processing), and  
so on. The neural network has three layers(input layer, hidden layer, output layer), and the  
neurons in each layer can share the convolution kernel, so it is very convenient to process high-
level data. We only need to design the size, number and sliding step size of the convolution kernel  
to make it train by itself.  
2. the three basic layers of the convolutional neural network:  
convolutional layer  
Perform convolution operation, inner product operation of two convolution kernel-sized  
matrices, multiply the numbers in the same position and then add and sum. A small number  
of convolution kernels are set in the convolutional layer close to the input layer, and the  
further the convolutional layer is, the more convolutional kernels are set in the convolutional  
layer.  
pooling layer  
By downsampling, the image and parameters are compressed, but the quality of the image is  
not destroyed. There are two pooling methods, MaxPooling(that is, taking the largest value in  
the sliding window) and AveragePooling(taking the average of all the values in the sliding  
window).  
Flatten layer & Fully Connected layer  
This layer is mainly a stack of layers. After the pooling layer, the image is compressed, and  
then goes to the Flatten layer; the output of the Flatten layer is put into the Fully Connected  
layer, and softmax is used to classify it.  
Filename file usage
batches.meta.betThe file stores the English name of each category. It can be opened and
viewed with Notepad or other text file readers
data batch 1.binThese 5 files are training data in the CIFAR-10 dataset. Each file stores
10,000 32 x 32 color images and their corresponding class labels in
binary format.
data batch 2.bin .
data batch 3.bin .
data batch 4.bin .
data batch 5.bin A total of 50,000 training images
test batch binThis file stores the test images and the labels of the test images. A total
of 10,000 sheets
readme. html Dataset introduction file3.5 Build the LetNet neural network and train the data set  
3.5.1 Preparation before implementation  
1. the environment  
The ROSMASTER-jetson development board series are all installed with the project development  
environment, including:  
python 3.6+  
torch 1.8.0  
torchvision 0.9.0  
2. data set  
CIFAR-10, 50,000 training images of 32*32 size, and 10,000 test images  
Note: The dataset is saved in the ~/Pytorch_demo/data/cifar-10-batches-py directory,  
Tensor type describe
tf.float32 32-bit floating point number
tf.float64 64-bit floating point
tf.int64 64-bit signed integer
tf.int32 32-bit signed integer
tf.int16 16-bit signed integer
tf.int8 8-bit signed integer
tf.uint8 8-bit unsigned integer
tf.string variable length byte array
tf.bool boolean
tf.complex64 real and imaginary numbers
3.5.2 Implementation process  
1. import related modules  
2. load the dataset  
3. package data set  
4. build a convolutional neural network  import  torch 
import  torch.nn  as  nn 
import  torch.nn.functional  as  F 
import  torchvision 
import  torchvision.transforms  as  transforms 
import  torch.optim  as  optim 
cifar_train_data  =  torchvision.datasets.CIFAR10(root = './data', train = True, 
                                          download = False, transform = 
transform) 
cifar_test_data  =  torchvision.datasets.CIFAR10(root = './data', train = False, 
                                          transform = transform) 
train_data_loader  =  torch.utils.data.DataLoader(cifar_train_data, batch_size = 
32, shuffle = True) 
test_data_loader  =  torch.utils.data.DataLoader(cifar_test_data, batch_size = 
32, shuffle = True 
class  LeNet(nn.Module): 
    #Define the operation operators required by the network, such as convolution,
fully connected operators, etc. 
     def __init__(self): 
        super(LeNet, self). __init__() 
5. configure the loss function and optimizer for training  
6. start training and testing  
3.5.3 Running the program  
1. reference code path  
2. run the program  
        #Conv2d parameter meaning: number of input channels, number of output 
channels, kernel size 
        self.conv1  =  nn.Conv2d(3, 6, 5) 
        self.conv2  =  nn.Conv2d(6, 16, 5) 
        self.fc1  =  nn.Linear(16 * 5 * 5, 120) 
        self.fc2  =  nn.Linear(120, 84) 
        self.fc3  =  nn.Linear(84, 10) 
        self.pool  =  nn.MaxPool2d(2, 2) 
     def forward(self, x): 
        x  =  F.relu(self.conv1(x)) 
        x  =  self.pool(x) 
        x  =  F.relu(self.conv2(x)) 
        x  =  self.pool(x) 
        x  =  x.view(- 1, 16 * 5 * 5) 
        x  =  F.relu(self.fc1(x)) 
        x  =  F.relu(self.fc2(x)) 
        x  =  self.fc3(x) 
        return  x 
criterion  =  nn.CrossEntropyLoss() 
optimizer  =  optim.SGD(net.parameters(), lr = 0.005, momentum = 0.9) 
~/Pytorch_demo/pytorch_demo.py 
cd ~/Pytorch_demo 
python3.6 pytorch_demo.py 
We have only trained 2 times here. You can modify the eTOCh value to modify the training times.  
The more training times, the higher the accuracy.  
 

---

## 3. Lidar guard.pdf

3. Radar guard  
3. Radar guard
3.1. How to use
3.2. Source code analysis
Radar Guard gameplay introduction:
Set lidar detection angle and response distance.
After turning on the car, the car faces the target closest to the car.
When the distance between the target and the car is less than the response distance, the  
buzzer will sound until there is no target within the response distance.
The angular speed PID of the car can be adjusted to achieve the best rotation effect of the  
car.
3.1. How to use  
Note: The [R2] of the remote control handle has the [pause/start] function of this gameplay.  
Due to the problem of movement method, the case in this section does not support the  
Ackerman model. Different models will have different parameter ranges, but the principle  
is the same; take the X3 McLunner as an example. 
Start with one click and the car will start moving after executing the command.
<PI5 needs to open another terminal to enter the same docker container
#Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
roslaunch yahboomcar_laser laser_Warning.launch
Parameters Range Parse
[LaserAngle] [10, 90] Lidar detection angle (left and right angles)
[ResponseDist] [0.0, 8.0] Car response distance
[switch] [False, True] Car movement [Start/Pause]Dynamic debugging parameters
Parameter analysis:
[ang_Kp], [ang_Ki], [ang_Kd]: PID debugging of car angular speed.
In the box in front of [switch], click the value of [switch] to True, and the car will stop. [switch]  
Default is False, the car moves.
Parameter modification
When the parameters are adjusted to the optimal state, the corresponding parameters are  
modified into the file, and no adjustment is required when using again.
According to the optimal parameters of the [rqt_reconfigure] debugging tool, enter the [scripts]  
folder of the [yahboomcar_laser] function package and modify the parameters corresponding to  
the [laser_Warning.py] file, as shown below
[rqt_reconfigure] Modification of the initial value of the debugging toolrosrun rqt_reconfigure rqt_reconfigure
class laserWarning:
     def __init__(self):
         rospy.on_shutdown(self.cancel)
         ... ...
         self.ang_pid = SinglePID(3.0, 0.0, 5.0)
         Server(laserWarningPIDConfig, self.dynamic_reconfigure_callback)
         self.laserAngle = 70
         self.ResponseDist = 0.5
Parameters Analysis Corresponding parameters
name The name of the parameter "linear"
type parameter data type double_t
level a bitmask passed to the callback 0
description A description parameter "linear in robot"
default Initial value for node startup 0.5
min parameter minimum value 0
max parameter maximum value 1.0Enter the [cfg] folder of the [yahboomcar_laser] function package and modify the initial values of  
the parameters corresponding to the [laserWarningPID.cfg] file.
Take the above article as an example to analyze
Note: After modification, you must recompile and update the environment to be effective. 
Node view
gen.add("ang_Kp", double_t, 0, "Kp in PID", 3.0, 0, 10)
gen.add("ang_Ki", double_t, 0, "Ki in PID", 0.0, 0, 10)
gen.add("ang_Kd", double_t, 0, "Kd in PID", 5.0, 0, 10)
gen.add("laserAngle", int_t, 0, "laserAngle", 70, 10, 90)
gen.add("ResponseDist", double_t, 0, "ResponseDist", 0.5, 0, 8)
gen.add("switch", bool_t, 0, "switch in rosbot", False)
gen.add("linear", double_t, 0, "linear in robot", 0.5, 0, 1.0)
cd ~/yahboomcar_ws
catkin_make
source devel/setup.bash
rqt_graph
3.2. Source code analysis  
launch file
-base.launch
laser_Warning.launch
laser_Warning.py source code flow chart:<launch>
     <!-- Start lidar node -->
     <!-- Activate the lidar node -->
     <include file="$(find rplidar_ros)/launch/rplidar.launch"/>
     <!-- Start the car chassis drive node-->
     <!-- Start the car chassis drive node -->
     <include file="$(find yahboomcar_bringup)/launch/yahboomcar.launch"/>
     <!-- Handle control node -->
     <!-- Handle control node -->
     <include file="$(find yahboomcar_ctrl)/launch/yahboom_joy.launch"/>
</launch>
<launch>
     <!-- Launch base.launch file -->
     <!-- Launch the base.launch file -->
     <include file="$(find yahboomcar_laser)/launch/base.launch"/>
     <!-- Start the lidar guard node -->
     <!-- Activate the Lidar guard node -->
     <node name='laser_Warning' pkg="yahboomcar_laser" type="laser_Warning.py" 
required="true" output="screen"/>
</launch>
According to the position of the target, the car automatically rotates to face the target; when the  
target approaches a certain distance, the buzzer alarms.

---

## 3. Open Source CV image processing and drawing text line segments.pdf

3. Open Source CV image processing and drawing
text line segments 
3. Ope n Source CV image processing and drawing text line segments 
3.1. Ope nCV image grayscale processing 
3.2. Ope nCV image binarization processing 
3.3. Ope nCV image edge detection 
3.4. Ope nCV line segment drawing 
3.5. Ope nCV draw rectangle 
3.6. Ope nCV draws a circle 
3.7. Ope nCV draw ellipse 
3.8. Ope nCV draws polygons 
3.9. Ope nCV to draw text 
3.1. OpenCV image grayscale processing  
1. Image grayscale  
The process of converting a color image to a grayscale image is the grayscale processing of the  
image. The color of each pixel in the color image is determined by the three components of R, G,  
and B, and each component can take a value of 0-255, so that a pixel can have more than 16  
million (256 256 256 = 1677256) colors range of changes. The grayscale image is a special color  
image with the same three components of R, G, and B. The variation range of one pixel is 256.  
Therefore, in digital image processing, images of various formats are generally converted into  
grayscale. image to make subsequent images less computationally expensive. The description of  
grayscale images, like color images, still reflects the distribution and characteristics of the overall  
and local chromaticity and highlight levels of the entire image.  
2. Image grayscale processing  
Grayscale processing is the process of converting a color image into a grayscale image. A color  
image is divided into three components, R, G, and B, which respectively display various colors  
such as red, green, and blue. Grayscale is the process of making the R, G, and B components of  
the color equal. Pixels with large grayscale values are brighter (the maximum pixel value is 255,  
which is white), and vice versa (the lowest pixel is 0, which is black).  
The core idea of image grayscale is R = G = B, this value is also called gray value.  
1. Maximum value method: Make the value of R, G, B after transformation equal to the largest  
one of the three values before transformation, namely: R=G=B=max(R, G, B). The grayscale  
image converted by this method is very bright.  
2. Average value method: the value of R, G, B after transformation is the average value of R, G,  
B before transformation. That is: R=G=B=(R+G+B)/3. This method produces a softer grayscale  
image.  
In OpenCV, use cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY) to achieve grayscale processing of  
images  
2. Code and actual effect display
run the program  
3.2. OpenCV image binarization processing  
1. The core idea of binarization  
Set the threshold, which is 0 (black) or 255 (white) above the threshold, so that the image is called  
a black and white image. The threshold can be fixed or adaptive. The adaptive threshold is  
generally the comparison of a point pixel with the average value of the region pixels or the  
weighted sum of Gaussian distribution where this point is in the middle order, and a difference  
value can be set or not set.  
2. The threshold (threshold) function is provided in Python-OpenCV: cv2.threshold (src,  
threshold, maxValue, thresholdType)  
Parameter meaning:  
src: original image  
threshold: current threshold  
maxVal: the maximum threshold, usually 255  
thresholdType: Threshold type, generally has the following values  
enum ThresholdTypes {  THRESH_BINARY = 0, #The  gray value of the pixel point greater than  
the threshold is set to maxValue (for example, the maximum gray value of 8-bit gray value is 255),  
and the gray value of the pixel point whose gray value is less than the threshold value is set to 0.   
THRESH_BINARY_INV = 1, #The  gray value of the pixel greater than the threshold is set to 0, and  
the gray value of the pixel less than the threshold is set to maxValue.  THRESH_TRUNC = 2, #The  python ~/yahboomcar_ws/src/yahboomcar_astra/scripts/opencv/3_1.py
import cv2
import numpy as np
if __name__ == '__main__':
    img = cv2.imread('yahboom.jpg')
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    while True :
        cv2.imshow("frame",img)
        cv2.imshow('gray', gray)
        action = cv2.waitKey(10) & 0xFF
        if action == ord('q') or action == 113:
            break
    img.release()
    cv2.destroyAllWindows()
gray value of the pixel point greater than the threshold is set to 0, and the gray value of the pixel  
less than the threshold is set to maxValue.  THRESH_TOZERO = 3,  # If the gray value of the pixel  
is less than the threshold, no change will be made, and the gray value of the part greater than the  
threshold will all become 0.  THRESH_TOZERO_INV = 4  # If the gray value of the pixel is greater  
than the threshold, no change will be made, and if the gray value of the pixel is less than the  
threshold, all the gray values will become 0.  }  
return value:  
retval: consistent with the parameter thresh  
dst: result image  
Note: Before binarization, we need to grayscale the color image to get a grayscale image.  
3. Code and actual effect display  
run the program
3.3. OpenCV image edge detection  
1. The purpose of image edge detection  python ~/yahboomcar_ws/src/yahboomcar_astra/scripts/opencv/3_2.py
import cv2
import numpy as np
if __name__ == '__main__':
    img = cv2.imread('yahboom.jpg')
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    ret,thresh1=cv2.threshold(gray,180,255,cv2.THRESH_BINARY_INV) 
    while True :
        cv2.imshow("frame",img)
        cv2.imshow('gray', gray)
        cv2.imshow("binary",thresh1)
        action = cv2.waitKey(10) & 0xFF
        if action == ord('q') or action == 113:
            break
    img.release()
    cv2.destroyAllWindows()
In the case of preserving the original image properties, the data size of the image is significantly  
reduced. There are a variety of algorithms for edge detection. Although the Canny algorithm is  
old, it can be said that it is a standard algorithm for edge detection, and it is still widely used in  
research.  
2. Canny edge detection algorithm  
Among the currently commonly used edge detection methods, the Canny edge detection  
algorithm is one of the methods with strict definition that can provide good and reliable  
detection. It has become one of the most popular algorithms for edge detection due to its  
advantages of satisfying the three criteria of edge detection and simple implementation process.  
The Canny edge detection algorithm can be divided into the following 5 steps:  
(1). Use a Gaussian filter to smooth the image and filter out noise  
(2). Calculate the gradient strength and direction of each pixel in the image  
(3). Apply Non-Maximum Suppression to eliminate spurious responses from edge detection  
(4). Apply double-threshold (Double-Threshold) detection to determine real and potential edges  
(5). Edge detection is finally completed by suppressing isolated weak edges  
3. How do we implement it in OpenCV? It's very simple, in three steps  
(1). Image grayscale: gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)  
(2). Gaussian filtering (noise reduction processing) image: GaussianBlur(src, ksize, sigmaX[, dst[,  
sigmaY[, borderType]]]) -> dst  
Parameter meaning:  
src: the input image, usually a grayscale image  
ksize: Gaussian kernel size  
sigmaX : Gaussian kernel standard deviation in X direction  
sigmaY : Gaussian kernel standard deviation in Y direction  
dst: the processed image  
(3). The image processed by the Canny method: edges=cv2.Canny( image, threshold1, threshold2[,  
apertureSize[, L2gradient]])  
Parameter meaning:  
edges: the calculated edge image  
image : The calculated edge image, generally the image obtained after Gaussian processing  
threshold1 : the first threshold during processing  
threshold2 : the second threshold during processing  
apertureSize : the aperture size of the Sobel operator  
L2gradient : The flag for calculating the gradient magnitude of the image. The default value is  
False. If True, the more precise L2 norm is used for the calculation (ie, the sum of the squares of  
the derivatives in the two directions is re-squared), otherwise the L1 norm is used (the absolute  
value of the derivatives in the two directions is directly added).  
4. Code and actual effect display  
run the program
3.4. OpenCV line segment drawing  
1. When using OpenCV to process images, we sometimes need to draw line segments,  
rectangles, etc. on the image.  used in OpenCV  
The cv2.line(dst, pt1, pt2, color, thickness=None, lineType=None, shift=None) function draws line  
segments.  
Parameter meaning:  
dst: output image.  
pt1, pt2: Required parameters. The coordinate points of the line segment, representing the  
starting point and the ending point, respectively  
color: required parameter. Used to set the color of the line segment  
thickness: optional parameter. Used to set the width of the line segment  
lineType: optional parameter. Used to set the type of line segment, optional 8 (8 adjacent  
connecting lines - default), 4 (4 adjacent connecting lines) and cv2.LINE_AA for antialiasing  
2. Code and actual effect display  python ~/yahboomcar_ws/src/yahboomcar_astra/scripts/opencv/3_3.py
import cv2
import numpy as np
if __name__ == '__main__':
    img = cv2.imread('yahboom.jpg')
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    imgG = cv2.GaussianBlur(gray,(3,3),0)
    dst = cv2.Canny(imgG,50,50)
    while True :
        cv2.imshow("frame",img)
        cv2.imshow('gray', gray)
        cv2.imshow("canny",dst)
        action = cv2.waitKey(10) & 0xFF
        if action == ord('q') or action == 113:
            break
    img.release()
    cv2.destroyAllWindows()
run the program
 
3.5. OpenCV draw rectangle  
1. In OpenCV, what is used to draw rectangles is  
cv2.rectangle （ img ， pt1 ， pt2 ， color ， thickness=None ， lineType=None ， shift=None ） 
Parameter meaning:  
img: canvas or carrier image  
pt1, pt2: Required parameters. The vertices of the rectangle represent the vertex and the diagonal  
vertex respectively, that is, the upper left corner and the lower right corner of the rectangle (these  
two vertices can determine a unique rectangle), which can be understood as a diagonal line.  
color: required parameter. Used to set the color of the rectangle  
thickness: optional parameter. Used to set the width of the side of the rectangle. When the value  
is negative, it means to fill the rectangle  
lineType: optional parameter. Used to set the type of line segment, optional 8 (8 adjacent  
connecting lines - default), 4 (4 adjacent connecting lines) and cv2.LINE_AA for antialiasing  
2. Code and effect display  
run the program  python ~/yahboomcar_ws/src/yahboomcar_astra/scripts/opencv/3_4.py
import cv2
import numpy as np
if __name__ == '__main__':
    img = cv2.imread('yahboom.jpg')
    line = cv2.line(img, (50,20), (20,100), (255,0,255), 10)
    while True :
        cv2.imshow("line",line)
        action = cv2.waitKey(10) & 0xFF
        if action == ord('q') or action == 113:
            break
    img.release()
    cv2.destroyAllWindows()
parameter illustrate
cv2.FILLED filling
cv2.LINE_4 4 Connection Types
cv2.LINE_8 8 connection types
cv2.LINE_AA Antialiasing, this parameter will make the lines smoother
3.6. OpenCV draws a circle  
1. In OpenCV, what is used to draw a circle is  
cv2.circle(img, center, radius, color[,thickness[,lineType]])  
Parameter meaning:  
img: painting or vector image cloth  
center: coordinates of the center of the circle, format: (50,50)  
radius: radius  
thickness: Line thickness. Default is 1. If -1 then fill solid  
lineType: Line type. The default is 8, the connection type. Described in the table below  python ~/yahboomcar_ws/src/yahboomcar_astra/scripts/opencv/3_5.py
import cv2
import numpy as np
if __name__ == '__main__':
    img = cv2.imread('yahboom.jpg')
    rect = cv2.rectangle(img, (50,20), (100,100), (255,0,255), 10)
    while True :
        cv2.imshow("line",rect)
        action = cv2.waitKey(10) & 0xFF
        if action == ord('q') or action == 113:
            break
    img.release()
    cv2.destroyAllWindows()
2. Code and actual effect display  
run the program  
3.7. OpenCV draw ellipse  
1. In OpenCV, the ellipse is drawn using  
cv2.ellipse(img, center, axes, angle, StartAngle, endAngle, color[,thickness[,lineType]  
Parameter meaning:  
center: the center point of the ellipse, (x, y)  
axes: refers to the short radius and long radius, (x, y)  
StartAngle: The angle of the starting angle of the arc  
endAngle: the angle of the end angle of the arc  
img, color, thickness, lineType can refer to the description of the circle  
2. Code and actual effect display  
run the program  
 python ~/yahboomcar_ws/src/yahboomcar_astra/scripts/opencv/3_6.py
import cv2
import numpy as np
if __name__ == '__main__':
    img = cv2.imread('yahboom.jpg')
    circle = cv2.circle(img, (80,80), 50, (255,0,255), 10)
    while True :
        cv2.imshow("circle",circle)
        action = cv2.waitKey(10) & 0xFF
        if action == ord('q') or action == 113:
            break
    img.release()
    cv2.destroyAllWindows()
python ~/yahboomcar_ws/src/yahboomcar_astra/scripts/opencv/3_7.py
3.8. OpenCV draws polygons  
1. In OpenCV, what is used to draw polygons is  
cv2.polylines(img,[pts],isClosed, color[,thickness[,lineType]])  
Parameter meaning:  
pts: vertices of the polygon  
isClosed: Whether it is closed.  (True/False)  
Other parameters refer to the drawing parameters of the circle  
2. Code and actual effect display  
run the program
 import cv2
import numpy as np
if __name__ == '__main__':
    img = cv2.imread('yahboom.jpg')
    ellipse = cv2.ellipse(img, (80,80), (20,50),0,0, 360,(255,0,255), 5)
    while True :
        cv2.imshow("ellipse",ellipse)
        action = cv2.waitKey(10) & 0xFF
        if action == ord('q') or action == 113:
            break
    img.release()
    cv2.destroyAllWindows()
python ~/yahboomcar_ws/src/yahboomcar_astra/scripts/opencv/3_8.py
3.9. OpenCV to draw text  
1. In OpenCV, what is used to draw text is  
cv2.putText(img, str, origin, font, size,color,thickness)  
Parameter meaning:  
img: input image  
str: the drawn text  
origin: the upper left corner coordinate (integer), which can be understood as where the text  
starts  
font: font  
size: font size  
color: font color  
thickness: font thickness  
Among them, the font is optional  import cv2
import numpy as np
if __name__ == '__main__':
    img = cv2.imread('yahboom.jpg')
    points = np.array([[120,50], [40,140], [120,70], [110,110], [50,50]], 
np.int32)
    polylines = cv2.polylines(img, [points],True,(255,0,255), 5)
    while True :
        cv2.imshow("polylines",polylines)
        action = cv2.waitKey(10) & 0xFF
        if action == ord('q') or action == 113:
            break
    img.release()
    cv2.destroyAllWindows()
2. Code and actual effect display  
run the program  
 python ~/yahboomcar_ws/src/yahboomcar_astra/scripts/opencv/3_9.py
import cv2
import numpy as np
if __name__ == '__main__':
    img = cv2.imread('yahboom.jpg')
    cv2.putText(img,'This is Yahboom!',(50,50),cv2.FONT_HERSHEY_SIMPLEX,1,
(0,200,0),2)
    while True :
        cv2.imshow("img",img)
        action = cv2.waitKey(10) & 0xFF
        if action == ord('q') or action == 113:
            break
    img.release()
    cv2.destroyAllWindows()

---

## 3. Remote control.pdf

3. Remote control  
3. Remote control 
3.1. PuTTY login 
3.2. SSH 
3.3. jupyter lab 
3.4. VNC 
3.5. WinSCP 
putty ：  https://www.chiark.greenend.org.uk/~sgtatham/putty/latest.html  
VNC ：  https://www.realvnc.com/en/connect/download/viewer/  
WinSCP ：  https://winscp.net/eng/download.php  
Note: Before logging in remotely, you must know the IP of the robot, which can be  
displayed on an external monitor or OLED.  
For example, the following figure: Username [jetson], hostname [yahboom].  
3.1. PuTTY login  
Enter the relevant link at the top, download the installation file, and double-click the [.exe] file to  
install.  
After entering the IP address of the robot, click [open];  
Enter the username [jetson] and password [yahboom], and press Enter to confirm.  
The sign of successful login appears as shown below  
3.2. SSH  
Note: The interface cannot be displayed.  
Log in  
operate under ubuntu system  
1. Enter the following command in the terminal  
2. Then enter [yes], 3) Enter the password [yahboom]  
copy file  
If jetson's IP is 192.168.2.103, username: jetson; virtual machine username: yahboom  
3.3. jupyter lab  
Note: The interface cannot be displayed.  
English input method, enter the following command, press Enter, enter the password [yahboom],  
and click [Log in].  ssh jetson@192.168.2.103 
scp jetson@192.168.2.103:/home/jetson/xxx.tar.gz /home/yahboom/  # Copy files 
from remote to local (file)
scp /home/yahboom/xxx.png jetson@192.168.2.103:/home/jetson/     # from local to 
remote (file) 
scp -r jetson@192.168.2.103:/home/jetson/test /home/yahboom/     # from remote to 
local -r (folder) 
scp -r /home/yahboom/test jetson@192.168.2.103:/home/jetson/     # from local to 
remote (folder) 
http://192.168.2.103:8888 
after login  
You can enter the folder, modify the contents of the file directly, click [Terminal] under [Other],  
enter the terminal, and execute the required command.  
3.4. VNC  
Note: The interface can be displayed.  
Enter the relevant link at the top, download the installation file, and double-click the [.exe] file to  
install.  
The login steps are as shown in the picture. To use the vnc viewer software for VNC connection,  
you first need to query the IP address. What I found here is [192.168.2.103]. After entering the IP  
address, click [Enter] on the keyboard.  
Enter the corresponding VNC user and enter the password [yahboom], select Remember  
password, and click [OK] to enter the VNC interface. The next time the IP remains unchanged, you  
do not need to enter the password again.  
Adjust resolution  
Jetson nano  
Use the command line to adjust as needed, it is only valid for this startup. Open a terminal and  
enter the following command  
raspberry pie  
Modify the [config.txt] file to be permanently valid. Open a terminal and enter the following  
command  
For example, add it at the bottom (set the resolution to 1920x1080)  
For the introduction of configuration parameters, please refer to the blog:  
https://blog.csdn.net/coolwriter/article/details/77719003  
3.5. WinSCP  
Go to the top related link, download the installation file, and double-click the [.exe] file to install.  
Double-click the icon below to open the WinSCP software  
This time the robot IP [192.168.2.93], username [jetson], password [yahboom].  
If the login dialog box does not pop up, click [New Session] in the upper left part, enter the host  
name (H), user name, and password in turn, and click Login.  xrandr --fb 1920x1080 
sudo vim /boot/firmware/config.txt 
hdmi_force_hotplug = 1 
config_hdmi_boost = 4 
hdmi_group = 2 
hdmi_mode = 82 
hdmi_drive = 2 
hdmi_ignore_edid = 0xa5000080 
disable_overscan = 1  
An example of a successful login is as follows  
Transfer files from computer to robot [jetson nano]  
Just select the file (folder) that needs to be transferred and drag it to the right, then the column of  
[/home/jetson],  
Robot 【 jetson nano 】 transfer files to computer  
Just select the file (folder) that needs to be transferred and drag it to the left, the column of [C:], or  
drag it directly to the desktop.  

---

## 3. Robot information release.pdf

3. Robot information release  
According to different models, just set the purchased model in [.bashrc], X1 (normal four-
wheel drive) X3 (Mailun) X3plus (Mailun robotic arm) R2 (Ackerman differential) etc. , this  
section takes X3 as an example
Open the [.bashrc] file
Find the [ROBOT_TYPE] parameter and modify the corresponding model
This section takes X3, Mecanum wheel vehicle as an example. 
3.1. Node subscription and publishing topics  
3.1.1. Function package path:  
Functions that ROSMASTER needs to implement: speed control, speed information feedback,  
battery voltage feedback, buzzer control, and running water light control. (Note: In the version  
with a robotic arm, robotic arm control, robotic arm status feedback and PTZ control also  
need to be implemented )
3.2. View node data  
3.2.1. Start  
3.2.2, View topic list  #Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
#Multiple ros commands require multiple terminals to be executed in the same 
docker container. Please refer to the tutorials in Sections 07/5 and 5.8.
sudo vim .bashrc
export ROBOT_TYPE=X3 # ROBOT_TYPE: X1 X3 X3plus R2 X7
~/yahboomcar_ws/src/yahboomcar_bringup
roslaunch yahboomcar_bringup yahboomcar.launch
rostopic list
rosrun rqt_topic rqt_topic #View topics graphically
3.2.3. Program flow chart  
3.2.4, core code (Mcunamu_driver.py)  
3.2.5. Analysis of three callback functions  Get data (core board -> host computer)
   edition.data = self.car.get_version()
   battery.data = self.car.get_battery_voltage()
   ax, ay, az = self.car.get_accelerometer_data()
   gx, gy, gz = self.car.get_gyroscope_data()
   mx, my, mz = self.car.get_magnetometer_data()
   vx, vy, angular = self.car.get_motion_data()
Publish data (host computer -> host computer)
   self.EdiPublisher.publish(edition)
   self.volPublisher.publish(battery)
   self.imuPublisher.publish(imu) Note: The imu data here is a combination of 
gyroscope and acceleration data.
   self.magPublisher.publish(mag)
   self.velPublisher.publish(twist)
Processing of acquired data (topic receiving data, transmission of data between 
nodes)
   cmd_vel_callback(self, msg)
   RGBLightcallback(self, msg)
   Buzzercallback(self, msg)
Release data (host computer -> core board)
   self.car.set_car_motion(vx, vy, angular)
   self.car.set_colorful_effect(msg.data, 6, parm=1)
   self.car.set_beep(1) or self.car.set_beep(1)
     # Running water light control, server callback function RGBLight control
     '''
     effect=[0, 6], 0: Stop light effect, 1: Running water light, 2: Marquee 
light, 3: Breathing light, 4: Gradient light, 5: Starlight, 6: Battery display
3.2.6. Use the rostopic pub command to send running light control, speed  
control, and buzzer control commands. 
3.2.7. Use the rostopic echo command to view speed information and  
battery voltage      speed=[1, 10], the smaller the value, the faster the speed changes.
     '''
     # Car motion control, subscriber callback function
     '''
      vx = msg.linear.x
      vy = msg.linear.y
      angular = msg.angular.z
      Note: Because this model is a Mecanum wheel, it can move on the y-axis. It 
is not applicable to other models.
      '''
     #Buzzer control, subscriber callback function
     '''
     self.car.set_beep(1): Turn on the buzzer
     self.car.set_beep(0): Turn off the buzzer
     '''
Running water light control
rostopic pub /RGBLight std_msgs/Int32 1 #Turn on the running water light
speed control
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
   x: 0.0
   y: 0.0
   z: 0.0
angular:
   x: 0.0
   y: 0.0
   z: 0.1" #The car moves at an angular speed of 0.1
Buzzer control
rostopic pub /Buzzer std_msgs/Bool true #Turn on the buzzer, send false if turned 
off
speed information
rostopic echo /cmd_vel
battery voltage
rostopic echo /voltage

---

## 3. USB wireless handle + mobile phone screen remote control tutorial.pdf

3. USB wireless handle + mobile phone screen
remote control tutorial 
3. USB wireless handle + mobile phone screen remote control tutorial 
3.1. Ope n steps 
3.1. Open steps  
After booting into the program, first close the large program, you can refer to the second  
section of "05. ROSMASTER Basic Control Tutorial" "2. Turn off the self-starting large  
program".  
Open a terminal, type,  
Open another terminal and enter
#Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker 
tutorial
~/run_docker.sh
roslaunch yahboomcar_bringup bringup.launch #launch chassis + remote control 
cd ~/Rosmaster/rosmaster 
python3 rosmaster_no_control.py 
Refer to the "Mobile APP Remote Control Tutorial" in "03. Mobile Phone Remote Control  
Tutorial", connect the mobile phone to the rosmaster, and then you can see the screen  
transmitted by the rosmaster on the mobile phone screen.  
Click here to enter the control screen  
  Click here for full screen viewing,  
Fix the phone on the handle, press the start button on the handle, and you can control it  
after you hear a beep.  
 
Precautions for using the handle  
When connecting the USB handle receiver, it is recommended to connect it to the  
outermost USB-HUB expansion board instead of directly connecting it to the main  
board or the middle USB-HUB expansion board (X3plus). If it is directly connected to the  
main board or the middle USB-HUB expansion board (X3plus), due to the upper and  
lower aluminum alloy grille, it will seriously interfere with the signal reception of the  
handle.
After plugging and unplugging the handle receiving head, the handle program needs to  
be restarted, otherwise the car will not be able to be controlled.
After starting the handle control program, if the handle cannot control the car, it may be  
caused by the wrong handle control mode. You can press and hold the handle mode  
button for about 15 seconds to switch modes. After the green indicator light is always  
on, press the start button again. If the buzzer sounds, it means the switching is  
successful. If there is no response, you can press and hold the mode button on the  
handle again for 15 seconds.
Jetson series support mode : PC/PCS mode. In PC mode, the POWER MODE indicator  
light is red by default. You can connect the handle receiver to the usb port of the  
computer to connect to the wireless handle. Enter the URL in the browser: https://game
pad-tester.com/ .  Pressing the button URL will display the change of the button value, as  
shown in the following figure:
 
 
Raspberry Pi series support mode : X-BOX mode. In X-BOX mode, the default POWER  
MODE indicator light is green. You can connect the handle receiver to the usb port of  
the computer to connect to the wireless handle. Enter the URL in the browser: https://g
amepad-tester.com/ . Pressing the button URL will display the change of the button  
value, as shown in the following figure:
 

After re-plugging the handle receiver or restarting the motherboard, the handle will  
reset to the factory mode. If it cannot be controlled, you need to switch the mode again  
every time you plug or restart.
In the case of unsuccessful matching, the POWER MODE indicator light will flash red and  
green all the time, and will not light up after a few seconds of sleep.
 

---

## 4. Buzzer whistle .pdf

4. Buzzer whistle  
4. Buzzer whistle 
4.1. Experimental goal 
4.2. Experiment preparation 
4.3. Experimental operation and phenomenon 
4.4. Program source code 
4.1. Experimental goal  
Control the buzzer switch on the expansion board, the whistle time is 100ms, 300ms, 1s, etc.  
 
4.2. Experiment preparation  
The red box in the picture is the buzzer on the expansion board.  
The buzzer on the expansion board is an active buzzer, so it is relatively simple to control, please  
check the following functions.  
Rosmaster_Lib library functions needed to control the buzzer:  
Parameter explanation: on_time=0: turn off, on_time=1: keep ringing, on_time>=10: turn off  
automatically after ringing for xx milliseconds (on_time is a multiple of 10).  
Return value: None.  
 set_beep ( on_time ) 
4.3. Experimental operation and phenomenon  
Check out the course accompanying video.  
 
4.4. Program source code  
Power on the robot, and open the browser of the Jetson Nano or remote computer to enter the  
Jupyter lab editor.  
Reference code path: Rosmaster/Samples/4.beep.ipynb  

---

## 4. FreeRTOS application.pdf

4. FreeRTOS application  
4. FreeRTOS application 
4.1. Purpose of the experiment 
4.2. Configure FreeRTOS information 
4.3. Analysis of the experimental flow chart 
4.4. core code explanation 
4.5. Hardware connection 
4.6. Experimental effect 
4.1. Purpose of the experiment  
Based on the program of "Key Control Buzzer Whistle", the function is imported into the  
FreeRTOS system to run, to detect the state of KEY1 on the expansion board, and to control the  
buzzer to whistle. Press the button once, the buzzer will beep (every 200 milliseconds), press the  
button again, the buzzer will be turned off.  
4.2. Configure FreeRTOS information  
Since each new project needs configuration information, it is more troublesome. Fortunately,  
STM32CubeIDE provides the function of importing .ioc files, which can help us save time.  
1. Import the ioc file from the BEEP project and name it FreeRTOS.  
2. Click Middleware->FreertOS, select CMSIS_V1, click Tasks and Queues, there will be one task  
by default, and then create two new tasks, one of which manages the buzzer and the other  
manages the buttons.  
3. The buzzer task information is shown in the following figure:  
Task Name: The task name.  
Priority: Set the priority.  
Stack Size: heap space, the size can be modified according to the actual situation.  
Entry Function: The task function entity.  
Code Generation Option: Code generation configuration, the default is weak to generate task  
entities, you can choose external not to generate task entities.  
Parameter: The task parameter.  
Allocation: You can choose Dynamic dynamic allocation or Static static allocation.  
Buffer Name: The statically allocated buff name.  
Control Block Name: Statically allocated block name.  
The key task is the same, just with a different name.  
4.3. Analysis of the experimental flow chart  
4.4. core code explanation  
1. Create new buzzer driver library bsp_task.h and bsp_task.c files in BSP. Add the following to  
bsp_task.h:  
Among them, the Task_Entity_LED() function manages the LED lights, the Task_Entity_Beep()  
manages the buzzer, and the Task_Entity_Key() manages the buttons.  

2. Introduce bsp.h into the freertos.c file, find the corresponding entity functions of the three  
tasks, and call the task functions we manually created.  

4.5. Hardware connection  
The LED light, key KEY1 and buzzer in FreeRTOS application are all onboard components and do  
not need to be connected manually.  
4.6. Experimental effect  
After the program is programmed, the LED light flashes every 200 milliseconds, press the button,  
the buzzer beeps (every 200 milliseconds), press the button again, the buzzer turns off.  

---

## 4. Lidar follow.pdf

4. Radar following  
4. Radar following
4.1. How to use
4.2. Source code analysis
Introduction to radar following gameplay:
Set lidar detection angle and distance.
After turning on the car, the car will follow the target closest to the car and keep a certain  
distance.
When there is an obstacle in front of the car, the buzzer keeps sounding and stops retreating  
until there is no obstacle.
The PID can adjust the linear speed and angular speed of the car to make the car follow the  
best effect.
4.1. How to use  
Note: [R2] on the remote control handle has the [pause/start] function for this gameplay.  
There will be slight differences depending on the model, but the principle is the same; take  
the X3 Mai Lun car as an example. 
Start with one click and the car will start moving after executing the command.
<PI5 needs to open another terminal to enter the same docker container
#Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
roslaunch yahboomcar_laser laser_Tracker.launch
Parameters Range Parse
[laserAngle] [10, 90] Lidar detection angle (left and right angles)
[ResponseDist] [0.0, 8.0] Car following distance
[priorityAngle] [10, 50]The car gives priority to the following range (left and right
angles)
[switch][False,
True]The car starts to pauseDynamic debugging parameters
Parameter analysis:
[lin_Kp], [lin_Ki], [lin_Kd]: Car line speed PID debugging.
[ang_Kp], [ang_Ki], [ang_Kd]: PID debugging of car angular speed.
In the box in front of [switch], click the value of [switch] to True, and the car will stop. [switch]  
Default is False, the car moves.
The parameter [priorityAngle] cannot be larger than [laserAngle], otherwise it will be meaningless.
Parameter modification
When the parameters are adjusted to the optimal state, the corresponding parameters are  
modified into the file, and no adjustment is required when using again.
According to the optimal parameters of the [rqt_reconfigure] debugging tool, enter the [scripts]  
folder of the [yahboomcar_laser] function package and modify the parameters corresponding to  
the [laser_Tracker.py] file, as shown belowrosrun rqt_reconfigure rqt_reconfigure
Parameters Analysis Corresponding parameters
name The name of the parameter "lin_Kp"
type parameter data type double_t
level a bitmask passed to the callback 0
description A description parameter "Kp in PID"
default Initial value for node startup 2.0
min parameter minimum value 0
max parameter maximum value 10[rqt_reconfigure] Modification of the initial value of the debugging tool
Enter the [cfg] folder of the [yahboomcar_laser] function package and modify the initial values of  
the parameters corresponding to the [laserTrackerPID.cfg] file.
Take the above article as an example to analyze
Note: After modification, you must recompile and update the environment to be effective. 
Node viewclass laserTracker:
     def __init__(self):
         rospy.on_shutdown(self.cancel)
         ... ...
         self.lin_pid = SinglePID(2.0, 0.0, 2.0)
         self.ang_pid = SinglePID(3.0, 0.0, 5.0)
         self.ResponseDist = rospy.get_param('~targetDist', 1.0)
         Server(laserTrackerPIDConfig, self.dynamic_reconfigure_callback)
         self.laserAngle = 65
         self.priorityAngle = 30 # 40
gen.add("lin_Kp", double_t, 0, "Kp in PID", 2.0, 0, 10)
gen.add("lin_Ki", double_t, 0, "Ki in PID", 0.0, 0, 10)
gen.add("lin_Kd", double_t, 0, "Kd in PID", 2.0, 0, 10)
gen.add("ang_Kp", double_t, 0, "Kp in PID", 3.0, 0, 10)
gen.add("ang_Ki", double_t, 0, "Ki in PID", 0.0, 0, 10)
gen.add("ang_Kd", double_t, 0, "Kd in PID", 5.0, 0, 10)
gen.add("laserAngle", int_t, 0, "laserAngle", 65, 10, 90)
gen.add("ResponseDist", double_t, 0, "ResponseDist", 1.0, 0, 8)
gen.add("priorityAngle", int_t, 0, "priorityAngle", 30, 10, 50)
gen.add("switch", bool_t, 0, "switch in rosbot", False)
gen.add("lin_Kp", double_t, 0, "Kp in PID", 2.0, 0, 10)
cd ~/yahboomcar_ws
catkin_make
source devel/setup.bash
4.2. Source code analysis  
launch file
-base.launch
laser_Avoidance.launch
laser_Tracker.py source code flow chart:rqt_graph
<launch>
     <!-- Start lidar node -->
     <!-- Activate the lidar node -->
     <include file="$(find rplidar_ros)/launch/rplidar.launch"/>
     <!-- Start the car chassis drive node-->
     <!-- Start the car chassis drive node -->
     <include file="$(find yahboomcar_bringup)/launch/yahboomcar.launch"/>
     <!-- Handle control node -->
     <!-- Handle control node -->
     <include file="$(find yahboomcar_ctrl)/launch/yahboom_joy.launch"/>
</launch>
<launch>
     <!-- Launch base.launch file -->
     <!-- Launch the base.launch file -->
     <include file="$(find yahboomcar_laser)/launch/base.launch"/>
     <!-- Start lidar following node -->
     <!-- Activate lidar follow node -->
     <node name='laser_Tracker' pkg="yahboomcar_laser" type="laser_Tracker.py" 
required="true" output="screen"/>
</launch>
According to the location where the target appears, the car moves autonomously to face the  
target and always maintains a certain distance.

---

## 4. Open Source CV image beautification.pdf

4. Open Source CV image beautification  
4. Ope n Source CV image beautification 
4.1. Ope nCV repair image 
4.2. Ope nCV image brightness enhancement 
4.3. Ope nCV image microdermabrasion and whitening 
4.1. OpenCV repair image  
1. Image inpainting is a class of algorithms in computer vision whose goal is to fill areas within  
an image or video. The area is identified using a binary mask, and filling is usually done  
according to the area boundary information that needs to be filled. The most common  
application of image restoration is to restore old scanned photos. It is also used to remove  
small unwanted objects in images.  
2. In OpenCV, dst = cv2.inpaint(src, inpaintMask, inpaintRadius, flags) is provided to repair the  
image,  
Parameter meaning:  
src: source image, which is the image that needs to be repaired  
inpaintMask: Binary mask indicating the pixels to be inpainted.  
dst: result image  
inpaintRadius: Indicates the radius of the repair  
flags : Repair algorithm, mainly INPAINT_NS (Navier-Stokes based method) or INPAINT_TELEA  
(Fast marching based method)  
Navier-Stokes based fixes should be slower and tend to produce more ambiguous results than  
fast marching methods. In practice, we have not found this to be the case.  INPAINT_NS produced  
better results in our tests and was also slightly faster than INPAINT_TELEA.  
3. Code and actual effect display  
(1) First, we first add damage to the intact picture, which can be understood as modifying the pixel  
value of a specific part of it  
run the program
 python ~/yahboomcar_ws/src/yahboomcar_astra/scripts/opencv/4_1_1.py
import cv2
import numpy as np
if __name__ == '__main__':
    img = cv2.imread('yahboom.jpg')
    for i in range(50,100):
        img[i,50] =   (0,0,0)
        img[i,50+1] =   (0,0,0)
        img[i,50-1] =   (0,0,0)
    for i in range(100,150):
After running, a picture will be generated, which is regarded as a damaged picture of the original  
picture.  
(2) repair the photo you just created, first read, then create the mask, and finally use the function  
to repair it  
run the program
         img[150,i] =   (0,0,0)
        img[150,i+1] =   (0,0,0)
        img[150-1,i] =   (0,0,0)
    cv2.imwrite("damaged.jpg",img)
    dam_img = cv2.imread('damaged.jpg')
    while True :
        cv2.imshow("dam_img",dam_img)
        action = cv2.waitKey(10) & 0xFF
        if action == ord('q') or action == 113:
            break
    img.release()
    cv2.destroyAllWindows()
python ~/yahboomcar_ws/src/yahboomcar_astra/scripts/opencv/4_1_2.py
import cv2
import numpy as np
if __name__ == '__main__':
    dam_img = cv2.imread('damaged.jpg')
    imgInfo = dam_img.shape
    height = imgInfo[0]
    width = imgInfo[1]
    paint =   np.zeros((height,width,1),np.uint8)
    for i in range(50,100):
        paint[i,50] = 255
        paint[i,50+1] = 255
        paint[i,50-1] = 255
    for i in range(100,150):
        paint[150,i] = 255
        paint[150+1,i] = 255
        paint[150-1,i] = 255
    dst_img =  cv2.inpaint(dam_img,paint,3,cv2.INPAINT_TELEA)
    while True :
        cv2.imshow("dam_img",dam_img)
        cv2.imshow("paint",paint)
        cv2.imshow("dst",dst_img)
        action = cv2.waitKey(10) & 0xFF
As shown in the figure, the left is before repair, the middle is the mask image, and the right is the  
original image after repair.  
4.2. OpenCV image brightness enhancement  
1. Implementation process: Amplify the three-channel value of each pixel synchronously, while  
keeping the channel value between 0-255. In fact, it is to traverse each pixel, add and  
subtract values to them, and then judge the three channels. Whether rgb is in the range of 0-
255, if it is greater or less than 255 or 0.  
2. Code and actual effect display  
run the program  
         if action == ord('q') or action == 113:
            break
    img.release()
    cv2.destroyAllWindows()
python ~/yahboomcar_ws/src/yahboomcar_astra/scripts/opencv/4_2.py
import cv2
import numpy as np
if __name__ == '__main__':
    img = cv2.imread('yahboom.jpg')
    imgInfo = img.shape
    height = imgInfo[0]
    width = imgInfo[1]
    dst =   np.zeros((height,width,3),np.uint8)
    for i in range(0,height):
        for j in range(0,width):
            (b,g,r) = img[i,j]
            bb = int(b) + 100
            gg = int(g) + 100
            rr = int(r) + 100
            if bb > 255:
                bb = 255
            if gg > 255:
                gg = 255
            if rr > 255:
                rr = 255
            dst[i,j] = (bb,gg,rr)
    while True :
The picture on the left is the original picture, and the picture on the back is the photo after  
increasing the brightness.  
4.3. OpenCV image microdermabrasion and whitening  
1. OpenCV realizes the function of microdermabrasion and whitening of pictures. The principle  
of implementation is basically the same as the principle of "1.20 OpenCV picture brightness  
enhancement", but here we do not need to process the r value, just follow this formula, p =  
p(x)*1.4+ y, where p(x) represents the b channel or g channel, and y represents the value  
that needs to be increased or decreased. Similarly, after adding the value, we need to judge  
the value.  
2. Code and actual effect display  
run the program        cv2.imshow("dst",dst)
        action = cv2.waitKey(10) & 0xFF
        if action == ord('q') or action == 113:
            break
    img.release()
    cv2.destroyAllWindows()
python ~/yahboomcar_ws/src/yahboomcar_astra/scripts/opencv/4_3.py
import cv2
import numpy as np
if __name__ == '__main__':
    img = cv2.imread('yahboom.jpg')
    imgInfo = img.shape
    height = imgInfo[0]
    width = imgInfo[1]
    dst =   np.zeros((height,width,3),np.uint8)
    for i in range(0,height):
        for j in range(0,width):
            (b,g,r) = img[i,j]
            bb = int(b*1.4) + 5
            gg = int(g*1.4) + 5
            if bb > 255:
                bb = 255
            if gg > 255:
                gg = 255
            dst[i,j] = (bb,gg,r)
 
     while True :
        cv2.imshow("origin",img)
        cv2.imshow("dst",dst)
        action = cv2.waitKey(10) & 0xFF
        if action == ord('q') or action == 113:
            break
    img.release()
    cv2.destroyAllWindows()

---

## 4. Robot keyboard control.pdf

[i] or [I] [linear, 0] [u] or [U] [linear, angular]
[,] [-linear, 0] [o] or [O] [linear, - angular]
[j] or [J] [0, angular] [m] or [M] [- linear, - angular]
[l] or [L] [0, - angular] [.] [- linear, angular]
Button Speed change Button Speed change
[q]Linear speed and angular speed
are both increased by 10%[z]Linear speed and angular speed
are both reduced by 10%
[w]Only the linear speed increases
by 10%[x]Only the linear speed decreases
by 10%
[e]Only the angular velocity
increases by 10%[c]Only the angular velocity
decreases by 10%
[t]Line speed X-axis/Y-axis
direction switching[s] Stop keyboard control4. Robot keyboard control  
According to different models, just set the purchased model in [.bashrc], X1 (normal four-
wheel drive) X3 (Mailun) X3plus (Mailun robotic arm) R2 (Ackerman differential) etc. , this  
section takes X3 as an example
Open the [.bashrc] file
Find the [ROBOT_TYPE] parameter and modify the corresponding model
 
4.1. Button description  
4.1.1. Direction control  
4.1.2. Speed control  #Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
sudo vim .bashrc
export ROBOT_TYPE=X3 # ROBOT_TYPE: X1 X3 X3plus R2 X7
4.2. Run the program  
4.2.1. Keyboard control code yahboom_keyboard.py path:  
4.2.2. Run  
<PI5 needs to open another terminal to enter the same docker container
4.2.3. Analysis of yahboom_keyboard.py  
1). Posted topic: cmd_vel
Therefore, you only need to package the speed and publish it through pub.publish(twist). The  
speed subscriber of the chassis can receive the speed data and then drive the car.
2. Mainly used modules
The select module is mainly used for socket communication, seeing changes in file  
descriptions, and completing work in a non-blocking manner.
The termios module provides an interface for IO controlled POSIX calls for tty
The tty module is mainly used to change the mode of the file descriptor fd
3), mobile dictionary and speed dictionary
The mobile dictionary mainly stores characters related to direction control~/yahboomcar_ws/src/yahboom_ctrl/scripts
roslaunch yahboomcar_bringup bringup.launch #robotchassisstart
roslaunch yahboomcar_ctrl yahboom_keyboard.launch #Keyboard control node
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
The speed dictionary mainly stores characters related to speed control
4. Get the current key information
5. Determine whether t/T or s/S is pressedmoveBindings = {
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    ',': (-1, 0),
    '.': (-1, 1),
    'm': (-1, -1),
    'I': (1, 0),
    'O': (1, -1),
    'J': (0, 1),
    'L': (0, -1),
    'U': (1, 1),
    'M': (-1, -1),
}
speedBindings = {
    'Q': (1.1, 1.1),
    'Z': (.9, .9),
    'W': (1.1, 1),
    'X': (.9, 1),
    'E': (1, 1.1),
    'C': (1, .9),
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}
def getKey():
    #tty.setraw(): Change the file descriptor fd mode to raw; fileno(): 
Return an integer file descriptor (fd)
    tty.setraw(sys.stdin.fileno())
    # select(): Directly call the IO interface of the operating system; 
monitor all file handles with the fileno() method
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    # Read a byte input stream
    if rlist: key = sys.stdin.read(1)
    else: key = ''
    #tcsetattr sets the tty attribute of the file descriptor fd from the 
attribute
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
6. Determine whether the string is in the dictionary
6), speed limit
Both angular speed and linear speed have a limit value, and it is impossible to increase  
forever. When starting, the program will first obtain the speed limit value, and when  
increasing the speed, the increased value will be judged and processed.
7), program flow chart
 if key=="t" or key == "T": xspeed_switch = not xspeed_switch
            elif key == "s" or key == "S":
                print ("stop keyboard control: {}".format(not stop))
                stop = not stop
#Whether the key string is in the mobile dictionary
if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
            count = 0
#Whether the key character is in the speed dictionary
 elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                count = 0
linear_limit = rospy.get_param('~linear_speed_limit', 1.0)
angular_limit = rospy.get_param('~angular_speed_limit', 5.0)
Precautions for using the handle  
When connecting the USB handle receiver, it is recommended to connect it to the outermost  
USB-HUB expansion board instead of directly connecting to the motherboard or the middle  
USB-HUB expansion board (X3plus). If it is directly connected to the motherboard or the  
middle USB-HUB expansion board (X3plus), due to the aluminum alloy blocks on the top and  
bottom, it will seriously interfere with the handle signal reception.
After plugging and unplugging the handle receiver, you need to restart the handle program,  
otherwise you will not be able to control the car.
After starting the handle control program, if the handle cannot control the car, it may be  
caused by the wrong handle control mode. You can press and hold the handle mode button  
for about 15 seconds to switch modes. After the green indicator light is always on, press the  
start button again, such as If a buzzer sounds, the switch is successful. If there is no response,  
you can press and hold the mode button on the handle again for 15 seconds.
Jetson series support mode : PC/PCS mode, the POWER MODE indicator light is red by  
default in PC mode. You can connect the handle receiver to the USB port of the computer to  
connect the wireless handle. Enter the URL in the browser: https://gamepad  -tester.com/ .  
Pressing the button URL will display the change of the button value, as shown in the figure  
below:
  
   
 
Raspberry Pi series supported modes : X-BOX mode. In X-BOX mode, the POWER MODE indicator  
light is green by default. You can connect the handle receiver to the USB port of the computer to  
connect the wireless handle. Enter the URL in the browser: https: //gamepad-tester.com/ .  
Pressing the button URL will display the change of the button value, as shown in the figure below:
 
 
After re-plugging or unplugging the handle receiver or restarting the motherboard, the  
handle will be reset to factory mode. If it cannot be controlled, the mode will need to be  
switched again every time it is plugged in, unplugged or restarted.
If matching is not successful, the POWER MODE indicator light will flash in red and green for a  
few seconds and then go into sleep mode.

---

## 4. yolov5 model training.pdf

4 yolov5 model training  
4 yolov5 model training 
4.1 yolov5 introduction 
4.2 folder structure 
4.3 Environmental requirements 
4.4 Use process 
4.5 Custom training data set 
4.5.1 Collecting datasets 
4.5.2 make yaml file 
4.5.3 Modify train.py 
4.5.4 Modify the model configuration file 
4.5.5 Modify detect.py 
4.5.6 Train and predict 
4.1 yolov5 introduction  
In February 2020, Joseph Redmon, the father of YOLO, announced his withdrawal from the field of  
computer vision research, YOLOv4 was released on April 23, 2020, and YOLOv5 was released on  
June 10, 2020.  The developers of YOLOv5 claim that YOLOv5 can achieve fast detection at 140 FPS  
on Tesla P100; while the detection speed of YOLOv4 is 50 FPS. Not only that, but the size of  
YOLOv5 is only 27 MB. And YOLOv4 is 244 MB.  YOLOv5 is nearly 90% smaller than YOLOv4. In  
terms of accuracy, YOLOv5 is comparable to YOLOv4.  
As a result, YOLOv5 is very fast, has a very lightweight model size, and is comparable in accuracy  
to the YOLOv4 benchmark.  
The algorithm performance test chart of the author of Yolov5:  
Video test official case  
cd ~/software/yolov5-5.0
python3 detection_video.py
4.2 folder structure  
. 
├──  data 
│    ├──  argoverse_hd.yaml 
│    ├──  coco128.yaml 
│    ├──  coco.yaml 
│    ├──  garbage 
│    │    ├── Get_garbageData  .py the py            file of the corresponding 
dataset(need more) 
│    │    ├──  image                         # Save the original image(target 
image to be recognized) folder 
│    │    ├──  images                        # Test input image image folder 
│    │    ├──  texture                       # Folder to store background 
images(needs more) 
│    │    └──  train  
│    │        ├──  images                    # The image folder where the dataset 
is stored 
│    │        ├──  labels                    # label folder for storing datasets
 
│    │        └──  labels.cache 
│    ├──  garbage.yaml 
├──    hyp  _.fine tune.yaml 
│    ├──  hyp.scratch.yaml 
│    ├──  images                            # Test input image image folder 
│    │    ├──  bus.jpg 
│    │    └──  zidane.jpg 
│    ├──  scripts 
│    │    ├──  get_argoverse_hd.sh       
│    │    ├──  get_coco.sh                   # Download dataset script 
│    │    └──  get_voc.sh 
│    └──  voc.yaml 
├──  detect # .py                             detect files 
├──  Dockerfile 
├──  hubconf.py 
├──  LICENSE 
├──  models                                # model configuration file 
│    ├──  yolo.py 
│    ├──  yolov5l.yaml 
│    ├──  yolov5m.yaml 
│    ├──  yolov5s.yaml 
│    └──  yolov5x.yaml 
├──  README.md 
├──  requirements # .txt                      Environment requirements 
├──  runs 
│    ├──  detect 
│    │    └──  exp                           # Test output image path folder 
│    │        ├──  bus.jpg 
│    │        └──  zidane.jpg 
│    └──  train 
│        └──  exp 
│            ├──  results.png               # training result image 
│            ├──  results # .txt               training log information 
│            ├──  test_batch0_labels.jpg 
│            ├──  test_batch0_pred.jpg      # prediction image 
│            ├──  test_batch1_labels.jpg 
│            ├──  test_batch1_pred.jpg 
Note: There are verification pictures in the runs/exp file. After training an eTOCh, go and see to  
verify whether the pictures are correct. If they are not correct, adjust them quickly. Otherwise, you  
have to retrain after training all the adjustments.  
label format  
The value of x, y, w, h is the position where the image length and height are pressed [0:1]  
4.3 Environmental requirements  
The factory image is already configured, no need to install  │            ├──  test_batch2_labels.jpg 
├──            ├──  test_batch2_pred.jpg 
│            ├──  train_batch0.jpg          # training picture 
│            ├──  train_batch1.jpg 
│            ├──  train_batch2.jpg 
│            └──  weights 
│                ├──  best.pt               # The best training model(generally 
choose it) 
│                └──  last .pt The               last trained model 
├──  test # .py                               test file 
├──  train # .py                              training file 
├──  tutorial.ipynb                        # tutorial 
├──  utils 
└──  weights                               # The folder where the weight 
files(pre-trained models) are stored 
    ├──  download_weights.sh 
    └──  yolov5s.pt 
label  x y w h  
# pip install -r requirements.txt 
# base ---------------------------------------- 
matplotlib > = 3.2.2 
numpy > = 1.18.5 
opencv - python > = 4.1.2 
Pillow 
PyYAML > = 5.3.1 
scipy > = 1.4.1 
torch > = 1.7.0 
torchvision > = 0.8.1 
tqdm > = 4.41.0 
imgaug 
# logging ------------------------------------- 
tensorboard > = 2.4.1 
# wandb 
# plotting ------------------------------------ 
seaborn > = 0.11.0 
pandas 
# export -------------------------------------- 
# coremltools>=4.1 
Installation example  
4.4 Use process  
After configuring according to the above process  
Under the path of [data/garbage/texture], fill in some background images([more])  
Run the [Get_garbageData.py] file to get the dataset(line 134 can modify the number of  
generated datasets [more])  
Run the [train.py] file to start training  
Run [detect.py] for image prediction  
4.5 Custom training data set  
4.5.1 Collecting datasets  
First go to Baidu to download or other methods, in the [data/garbage/texture] path, fill in some  
background images([more])  
Open the [Get_garbageData.py] file  
Modify the total number of generated datasets and fill in as required.  [More], too few datasets  
will lead to suboptimal training results.  
Run the [Get_garbageData.py] file to get the dataset  
4.5.2 make yaml file  
For example garbage.yaml:  # onnx>=1.8.1 
# scikit-learn==0.19.2  # for coreml quantization 
# extras -------------------------------------- 
thop   # FLOPS computation 
pycocotools > = 2.0   # COCO mAP 
pip install imgaug 
sudo vim Get_garbageData.py 
img_total = 10000 
python Get_garbageData.py 
train : ./garbage/train/images/            # Training set picture path
val: ./garbage/train/images/               # Verification set image path (also 
separate from training set) 
nc : 16                                    # number of categories 
names : [ "Zip_top_can", "Apple_core" ...] # category name 
4.5.3 Modify train.py  
Other places according to your needs.  
4.5.4 Modify the model configuration file  
Modify the second line of the yaml file of the yolov5 neural network, and use which weight file to  
modify the corresponding yaml file.  
Here we are using yolov5s.yaml, so just modify the second line of the models/yolov5s.yaml file  
4.5.5 Modify detect.py  
Similar to the place where the [train.py] file needs to be modified  
Other places according to your needs.  
4.5.6 Train and predict  
After training the model, the final model will be produced in the [runs/train] folder after training.  
Image prediction, modify the model path and input image, predict the image, and the result is in  
the [runs/detect] folder.  parser.add_argument('--weights', type = str, default = './weights/yolov5s.pt', 
help = 'initial weights path') # Line 458: 
parser.add_argument('--data', type = str, default = './garbage/garbage.yaml', 
help = 'data.yaml path') # Line 460: Custom training file 
parser.add_argument('--eTOChs', type = int, default = 300) # Line 462: Customize 
training eTOChs, how many rounds of training 
parser.add_argument('--batch-size', type = int, default = 16, help = 'total 
batch size for all GPUs') # 
parser.add_argument('--img-size', nargs = '+', type = int, default =[ 640, 640 
], help = '[train, test] image sizes') # image size 
parser.add_argument('--device', default = 'cpu', help = 'cuda device, ie 0 or 
0,1,2,3 or cpu') # line 474: select CPU or GPU 
parser.add_argument('--project', default = 'runs/train', help = 'save to 
project/name') # training result output folder 
nc :  16   # number of classes 
parser.add_argument('--weights', nargs = '+', type = str, default = 
'weights/yolov5s.pt', help = 'model.pt path(s)') # Line 151: Pretrained weights 
parser.add_argument('--source', type = str, default = 'data/images', help = 
'source') # file/folder, 0 for webcam # Line 152: Input prediction image 
parser.add_argument('--img-size', type = int, default = 640, help = 'inference 
size(pixels)') # line 153: image size 
parser.add_argument('--device', default = 'cpu', help = 'cuda device, ie 0 or 
0,1,2,3 or cpu') # line 156: select CPU or GPU 
parser.add_argument('--project', default = 'runs/detect', help = 'save results 
to project/name') # Line 165: Prediction results output folder 
python3 train.py 
python3 detect.py 
For real-time video monitoring, the model path needs to be modified.  
python3 detection_video.py 

---

## 5. Robot handle control.pdf

5. Robot handle control  
According to different models, just set the purchased model in [.bashrc], X1 (normal  
four-wheel drive) X3 (Mailun) X3plus (Mailun robotic arm) R2 (Ackerman differential)  
etc. , this section takes X3 as an example
Open the [.bashrc] file
Find the [ROBOT_TYPE] parameter and modify the corresponding model
 
5.1. Operating environment  
Operating system: Ubuntu 18.04 LTS
ROS version: melodic
Equipment: jetson nano/Raspberry Pi, PC, wireless controller (USB receiver)
Controller function code path: ~/yahboomcar _ws/src/yahboomcar_ctrl/scripts
5.2. Install driver  
ROS driver for universal Linux controllers. The Joy package contains Joy_node, a node that  
connects a generic Linux controller to ROS. This node publishes a "/Joy" message containing  
the current state of each button and axis of the controller.
5.3. Usage steps  
5.3.1. Device connection  
First, connect the USB end of the wireless controller to the device (jetson, Raspberry Pi, PC).  
This lesson takes connecting the USB end of the wireless controller to Jetson as an example.  
Connect the mouse, keyboard, and monitor; or you can also connect remotely via VNC.#Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker 
tutorial
~/run_docker.sh
#Multiple ros commands require multiple terminals to be executed in the same 
docker container. Please refer to the tutorials in Sections 07/5 and 5.8.
sudo vim .bashrc
export ROBOT_TYPE=X3 # ROBOT_TYPE: X1 X3 X3plus R2 X7
sudo apt install ros-melodic-joy ros-melodic-joystick-drivers
5.3.2. View device  
Open the terminal and enter the following command. [js0] is displayed. This is the wireless  
controller. In special cases, you can also view the changes in the device list through the two  
states of accessing and not accessing the USB end of the wireless controller. If there is a  
change, the changed device will be the same; otherwise, the connection will be unsuccessful  
or unrecognized.
Note: If a keyboard or mouse or other device is connected to ROSMASTER first, the  
remote control receiver will be recognized as other devices. Therefore, if you need to  
connect a keyboard or mouse, you can connect the remote control receiver first and  
then connect it. into other devices. 
5.3.3. Test handle  
Open the terminal and enter the following command. As shown in the picture, the wireless  
handle has 8 axial inputs and 15 key inputs. You can press the keys individually to test the  
numbers corresponding to the keys.
If jstest is not installed, run the following command:ls /dev/input
sudo jstest /dev/input/js0
5.3.4. Run the handle node  
Open three terminals and enter the following commands in order to view detailed  
information, which is the same as [Test Controller]. Depending on the device (Raspberry Pi,  
Jetson Nano, PC) and the system, the controller status will be different.
5.4. Control the little turtle with the handle  
5.4.1. Start  
1), view nodessudo apt-get install joystick
roscore
rosrun joy joy_node
rostopic echo joy #Print released information
roslaunch yahboomcar_ctrl twist_joy.launch
rostopic list
handle little turtle
On the left joystick Forward
Left joystick down Backward
Right joystick left Turn left
Right joystick right Turn right
2. View speed topic information
3. Correspondence between the handle and the operation of the little turtle
3. View the node graph
【/joy_node 】： Node of controller information
【/twist_joy 】： Nodes controlled by the handle
【/turtlesim_node 】： The node of the little turtle
The node [/joy_node] publishes the topic [/joy] so that the node [/twist_joy] subscribes. After  
processing, it publishes the topic [/turtle1/cmd_vel] so that the node [/turtlesim_node]  
subscribes to drive the movement of the little turtle.

Handle Effect
Left joystick up/down Car forward/backward to go straight
Left joystick left/right The car goes straight left/right
Right joystick left/right Rotate car left/right
Right "1" key Control light strip effect
"2" key on the right End/start other functions
"START" button Control buzzer/end sleep
Press the left joystick Adjust the X/Y axis speed
Press the right joystick Adjust the angular velocity5.5. Handle control ROSMASTER  
Controlling ROSMASTER with a handle is similar to controlling a little turtle with a handle; let  
the handle control node establish contact with the underlying drive node of ROSMASTER,  
change the current state of the handle, send different information to ROSMASTER, and drive  
ROSMASTER to make different reactions. Corresponding to different models of  
ROSMASTER, what we need to control is also different. 
For the ROSMASTER-X3 (Mecanum wheel), what we can control through the handle are the 
buzzer, the light strip, the linear speed of the car's movement (x-axis and y-axis), and  
the angular speed of the car's movement.  
5.5.1. Start handle control  
1. Reference path of handle control yahboom_joy.launch file
2), start
After turning it on, press the "START" button and hear the buzzer sound to start remote  
control. The remote control will enter sleep mode after being turned on for a period of  
time. You need to press the "START" button to end sleep. 
3), remote control effect description
 
4. View the node graph~/yahboomcar_ws/src/yahboomcar_ctrl/launch/
roslaunch yahboomcar_bringup bringup.launch #Start chassis and handle 
control, the launch of handle control is yahboom_joy.launch
rqt_graph
The red box part is the topic communication between the handle control node/yahboom_joy  
and the underlying node/driver_node.
5), program analysis
Underlying control program:
The code path is:
In Mcnamu_driver.py, the subscribers and their respective callback functions for the  
topic information of the buzzer, speed control, and light strip are defined,
Handle control program:
The code path is:
In yahboom_joy.py, define the publisher  of the buzzer, speed control, and light strip topic  
information, and publish these messagesMcnamu_driver.py
~/yahboomcar_ws/src/yahboomcar_bringup/scripts
self.sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback, 
queue_size=100) #Speed control
self.sub_RGBLight = rospy.Subscriber("RGBLight", Int32, 
self.RGBLightcallback, queue_size=100) #Light strip control
self.sub_Buzzer = rospy.Subscriber("Buzzer", Bool, self.Buzzercallback,
queue_size=100) #Buzzer control
def cmd_vel_callback(self, msg) #Speed callback function
def RGBLightcallback(self, msg) #Light strip callback function
def Buzzercallback(self, msg) #Buzzer callback function
yahboom_joy.py
~/yahboomcar_ws/src/yahboomcar_ctrl/scripts
6), code analysis
/joy topic data analysis
Run the joy_node node and view the /joy topic information,
In the printed information, there are two arrays: axes and buttons. The data they store is the  
behavior of the joystick and buttons on the remote control. You can press each button on the  
remote control one by one to correspond to the transformation of the array, and you can  
know what behavior will cause which variable in the array to change its value.
Knowing the changes in the /joy topic data corresponding to the keystroke behavior,  
then in yahboom_joy.py, we only need to make judgments on these values. There are  
also two categories in yahboom_joy.py. The first step is to determine the name of the  
system (the first step is to determine whether the handle receiver is plugged into  
a jetson or a Raspberry Pi/PC (here it refers to a virtual machine)) ,
Value judgment: Take the jetson processor to judge the buzzer data as an example.  
Analysis shows that we use the "START" button to control the buzzer, and it corresponds  
to buttons[11]. When it is pressed At this time, buttons[11] will become 1. At this time,  
we can send the buzzer data,
Other remote control behaviors are analyzed by analogy.
Note: When judging the left and right of the left rocker, only the data of X3  
(Mecanum wheel) and X3 Plus (Mecanum wheel + robotic arm) are valid, because  
of the characteristics of their wheels, they can move laterally. So there is speed on  
the Y axis. self.pub_cmdVel = rospy.Publisher('cmd_vel', Twist, queue_size=10) #Speed 
control
self.pub_RGBLight = rospy.Publisher("RGBLight", Int32, queue_size=10) #Light 
strip control
self.pub_Buzzer = rospy.Publisher("Buzzer", Bool, queue_size=1) #Buzzer 
control
self.pub_cmdVel.publish(twist) #Publish velocity data
self.pub_RGBLight.publish(self.RGBLight_index) #Publish light strip data
self.pub_Buzzer.publish(self.Buzzer_active) #Publish buzzer data
rosrun joy joy_node
rostopic echo joy
if self.user_name == "jetson": self.user_jetson(joy_data)
         else: self.user_pc(joy_data)
 if joy_data.buttons[11] == 1:
 self.Buzzer_active=not self.Buzzer_active
    # print "self.Buzzer_active: ", self.Buzzer_active
    self.pub_Buzzer.publish(self.Buzzer_active)
 ylinear_speed = self.filter_data(joy_data.axes[0]) * self.yspeed_limit 
* self.linear_Gear
  
5.6. Precautions when using the handle  
When connecting the USB handle receiver, it is recommended to connect it to the  
outermost USB-HUB expansion board instead of directly connecting to the motherboard  
or the middle USB-HUB expansion board (X3plus). If it is directly connected to the  
motherboard or the middle USB-HUB expansion board (X3plus), due to the aluminum  
alloy blocks on the top and bottom, it will seriously interfere with the handle signal  
reception.
After plugging and unplugging the handle receiver, you need to restart the handle  
program, otherwise you will not be able to control the car.
After starting the handle control program, if the handle cannot control the car, it may be  
caused by the wrong handle control mode. You can press and hold the handle mode  
button for about 15 seconds to switch modes. After the green indicator light is always  
on, press the start button again, such as If a buzzer sounds, the switch is successful. If  
there is no response, you can press and hold the mode button on the handle again for  
15 seconds.
jetson series support mode: PC/PCS mode
Raspberry Pi series support mode: X-BOX mode
After re-plugging or unplugging the handle receiver or restarting the motherboard, the  
handle will be reset to factory mode. If it cannot be controlled, the mode will need to be  
switched again every time it is plugged in, unplugged or restarted.

---

## 5. Serial communication.pdf

5. Serial communication  
5. Serial commu nication 
5.1. Purpose of the experiment 
5.2. Configuration pin information 
5.3. Analysis of the experimental flow chart 
5.4. core code explanation 
5.5. Hardware connection 
5.6. Experimental effect 
5.1. Purpose of the experiment  
Use the serial port communication of STM32, receive the data of the serial port assistant, and  
return the received data to the serial port, redefine the printf function.  
5.2. Configuration pin information  
Since each new project needs configuration information, it is more troublesome. Fortunately,  
STM32CubeIDE provides the function of importing .ioc files, which can help us save time.  
1.Import the ioc file from the BEEP project and name it Serial.  
Change the mode of serial port 1 to Asynchronous synchronous communication, the baud rate is  
115200, the data width: 8 bits, the test: None, and the stop bit: 1 bit.  
2.Add DMA sending channel  
3. Open serial port 1 interrupt settings.  
5.3. Analysis of the experimental flow chart  
5.4. core code explanation  
1. Create new buzzer driver library bsp_uart.h and bsp_uart.c files in BSP. Add the following to  
bsp_uart.h:  
2. Add the following in bsp_uart.c:  
USART1_Init(): Initialize the related content of the serial port, open the serial port to receive 1  
data.  

USART1_Send_U8(ch): Serial port 1 sends a byte.  
USART1_Send_ArrayU8(BufferPtr,Length): Serial port 1 sends a string of data, BufferPtr is the first  
address of the data, and Length is the length of the data.  ENABLE_UART_DMA is the switch of  
serial port 1 DMA.  
3. Since the serial port 1 receive interrupt is only performed once, it is necessary to call the  
receive data again after receiving the data. In order to facilitate the test, the serial port is  
called in an interrupt to send data. In practical applications, data should not be sent in an  
interrupt. It is time-consuming to send data through the serial port, which may cause  
problems such as packet loss or even serial port errors.  
4. Redefine printf to use serial port 1 to send data.  

5. In BSP initialization, call USART1_Init() function to request to receive data.  
6. After the button is pressed, add the printf() function to print information through serial port  
1. 
5.5. Hardware connection  
The expansion board needs to be connected to the USB port of the computer using a micro-USB  
data cable. The connection diagram is as follows:  
5.6. Experimental effect  
After programming the program, the LED light flashes once every 200 milliseconds. Connect the  
expansion board to the computer through the micro-USB data cable and open the serial port  
assistant (the specific parameters are shown in the figure below). The buzzer will sound every  
time the button is pressed. After 50 milliseconds, you can see that the serial port assistant will  
display press:xx, and each time you press the button xx will automatically increase by 1. The serial  
port assistant sends the character a, and the expansion board will automatically return the  
character a. Since the above uses to send data in an interrupt, you cannot send too many  
characters at a time, otherwise characters will be lost or even serial port errors will occur.  

---

## 5. Static IP and hotspot mode.pdf

5. Static IP and hotspot mode  
5. Static IP and hotspot mode 
5.1. Static IP 
5.2. Hotspot mode 
This section uses jetson nano as an example.  
5.1. Static IP  
First, click the WiFi icon in the upper right corner and the following frame will appear, click [Edit  
Connections...] at the bottom.  
Double-click the connected Wi-Fi, here is [Yahboom].  

In the [Wi-Fi] directory, select [Client] for [Mode].  
In the [IPv4 Settings] directory, click the [Add] icon, enter the IP as shown below, and finally click  
[save] to save.  
Modify the .bashrc file and enter the command  
Set ROS_IP to the IP modified in the previous step, as shown in the following figure. Note; if you  
do not connect this Wi-Fi, be sure to comment out the modified line (just add # in front).  
When we newly open the terminal, [binary operator expected] appears, don't bother, it does not  
affect the use.  
sudo vim ~/.bashrc 
5.2. Hotspot mode  
First, click the WiFi icon in the upper right corner and the following frame will appear, click [Edit  
Connections...] at the bottom.  
The frame as shown below will pop up, click [+] to select [Wi-Fi] mode, and click [Create...].  
In the [Wi-Fi] directory, add [yah] in the [SSID] column, and select [Hotspot] in the [Mode] column.  
In the [Wi-Fi Security] directory, select [WPA & WPA2 Personal] in the [Security] column, and enter  
the password in the [Password] column.  
In the [IPv4 Settings] directory, click the [Add] icon, and enter the IP as shown below.  
In the [IPv4 Settings] directory, select [Ignore] in the [Method] column, and finally click [Save] to  
save.  
In the [Wi-Fi] mode, our newly created WIFI appears.  
At this point, the new WIFI has been successfully created, and the next step is to connect the new  
WIFI. Follow the steps as shown below.  
In the pop-up dialog box [Connections], select the newly created WIFI [Wi-Fi connections 1], and  
click [Connect].  
 

---

## 5. Visual tracking autopilot.pdf

5. Astra autonomous driving  
5. Astra autonomous driving
5.1. Introduction
5.1.1, HSV introduction
5.1.2, HSV hexagonal pyramid
5.2. Ope ration steps
5.2.1. Start
5.2.2. Identification
5.2.3. Color calibration
5.2.4. Tracking driving
Function package: ~/yahboomcar_ws/src/yahboomcar_linefollw
5.1. Introduction  
The Yahboom mobile robot's depth camera is capable of autonomous driving. It can recognize  
multiple colors at any time, independently store the currently recognized colors, and follow the  
detected and recognized colors. During the tracking process, it can also achieve real-time obstacle  
avoidance.
The Yahboom mobile robot can also realize the function of real-time control of HSV. By adjusting  
the high and low thresholds of HSV, it filters out interfering colors, so that the tracking route can  
be ideally identified in complex environments. If the color selection effect is not ideal, If so, you  
need to move the car to different environments and calibrate it at this time, so that it can  
recognize the colors we need in complex environments.
5.1.1, HSV introduction  
HSV (Hue, Saturation, Value) is a color space created by A. R. Smith in 1978 based on the intuitive  
characteristics of color, also known as the Hexcone Model.
The parameters of color in this model are: hue (H), saturation (S), and lightness (V).
H: 0 — 180
S: 0 — 255
V: 0 — 255
Here some reds are classified into the purple range:
5.1.2, HSV hexagonal pyramid  
Hue H
Represents color information, that is, the position of the spectral color. This parameter is  
represented by an angle, with a value ranging from 0° to 360°, starting from red and counting in  
counterclockwise direction. Red is 0°, green is 120°, and blue is 240°. Their complementary colors  
are: yellow is 60°, cyan is 180°, and purple is 300°.
Saturation S
Saturation S is expressed as the ratio between the purity of the selected color and the maximum  
purity of that color. When S=0, there is only grayscale. 120 degrees apart. Complimentary colors  
are 180 degrees apart. A color can be thought of as the result of mixing a certain spectral color  
with white. The greater the proportion of spectral colors, the closer the color is to spectral colors,  
and the higher the saturation of the color. The saturation is high and the color is deep and vivid.  
The white light component of the spectral color is 0, and the saturation reaches the highest level.  
Usually the value range is 0% ~ 100%. The larger the value, the more saturated the color.
LightnessV
Brightness represents the brightness of a color. For light source color, the brightness value is  
related to the brightness of the luminous body; for object color, this value is related to the  
transmittance or reflectance of the object. Usually the value range is 0% (black) to 100% (white).  
One thing to note: there is no direct relationship between it and light intensity.
The three-dimensional representation of the HSV model evolves from the RGB cube. If you  
imagine looking from the white vertices of the RGB along the diagonal of the cube to the black  
vertices, you can see the hexagonal shape of the cube. The hexagonal borders represent color, the  
horizontal axis represents purity, and lightness is measured along the vertical axis.
5.2. Operation steps  
5.2.1. Start  
Note: [R2] on the remote control handle has the [pause/start] function for this gameplay.  
Different models will have different parameter ranges, but the principle is the same; take  
the X3 McLunner as an example. 
There are two starting methods, choose one. The demonstration case is method 2; before starting,  
place the robot to the starting position so that the depth camera faces downward as much as  
possible.
method one
robot side
Method 2
Can be controlled remotely for easy operation.
robot side
virtual machine
VideoSwitch parameter: whether to use the camera function package to start.
img_flip parameter: whether to flip the screen horizontally, the default is false.
Set parameters according to needs, or modify the launch file directly, so there is no need to attach  
parameters when starting.
5.2.2. Identification  
After startup, the system defaults to [Target Detection Mode], as shown below on the left:
#Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
roslaunch yahboomcar_linefollw follow_line.launch VideoSwitch:=true 
img_flip:=false
roslaunch yahboomcar_linefollw follow_line.launch VideoSwitch:=false 
img_flip:=false
roslaunch yahboomcar_linefollw line.launch
Keyboard key control:
【r 】 : Color selection mode, you can use the mouse to select the area of the color to be  
recognized (cannot exceed the area range).
【i 】 : Target detection mode. The color image on the left (Color) and the binary image on the right  
(Binary).
【q 】 : Exit the program.
【Spacebar 】 : Follow the track.
In the color selection mode, you can use the mouse to select the area of the color to be recognized  
(cannot exceed the area range), as shown in the figure below, release it to start recognition.
5.2.3. Color calibration  
<PI5 needs to open another terminal to enter the same docker container
Dynamic parameter debugging tool
Set the mode to [Target Detection Mode] and start the dynamic parameter debugging tool.
Select the [LineDetect] node. Generally, you only need to adjust [Hmin], [Smin], [Vmin], and  
[Hmax]. These four parameters can be well identified. The slide bar is always in a dragging state  
and data will not be transferred to the system until it is released; you can also select a row and  
then slide the mouse wheel.
Parameter analysis:rosrun rqt_reconfigure rqt_reconfigure
[Kp], [Ki], [Kd]: PID control during car driving.
[scale]: PID scaling.
[linear]: Car running speed; range [0, 1.0], unit: meters; set as required.
[LaserAngle]: Lidar effective angle; range [0, 180], unit: degree; set as required.
[ResponseDist]: lidar response distance; range [0.15, 8.0], unit: meters; set as required.
Parameter modification
When the parameters are adjusted to the optimal state, the corresponding parameters are  
modified into the file, and no adjustment is required when using again.
According to the optimal parameters of the [rqt_reconfigure] debugging tool, enter the [scripts]  
folder of the [yahboomcar_linefollw] function package and modify the parameters corresponding  
to the [follow_line.py] file, as shown below
[rqt_reconfigure] Modification of the initial value of the debugging tool
Enter the [cfg] folder of the [yahboomcar_linefollw] function package and modify the initial values  
of the parameters corresponding to the [LineDetectPID.cfg] file. The color [HSV] adjustment  
parameters do not need to be modified. The system will automatically generate the  
[LineFollowHSV.text] file, which will be automatically read when the system starts.class LineDetect:
     def __init__(self):
         rospy.on_shutdown(self.cancel)
         rospy.init_node("LineDetect", anonymous=False)
         ... ...
         self.scale = 1000
         self.FollowLinePID = (60, 0, 20)
         self.linear = 0.4
         self.LaserAngle = 30
         self.ResponseDist = 0.55
         self.PID_init()
         ... ...
gen.add("Hmin", int_t, 0, "Hmin in HSV", 0, 0, 180)
gen.add("Smin", int_t, 0, "Smin in HSV", 85, 0, 255)
gen.add("Vmin", int_t, 0, "Vmin in HSV", 126, 0, 255)
gen.add("Hmax", int_t, 0, "Hmax in HSV", 9, 0, 180)
gen.add("Smax", int_t, 0, "Smax in HSV", 253, 0, 255)
gen.add("Vmax", int_t, 0, "Vmax in HSV", 255, 0, 255)
gen.add("scale", int_t, 0, "scale", 1000, 0, 1000)
gen.add("Kp", int_t, 0, "Kp in PID", 60, 0, 100)
gen.add("Ki", int_t, 0, "Ki in PID", 0, 0, 100)
gen.add("Kd", int_t, 0, "Kd in PID", 20, 0, 100)
gen.add("linear", double_t, 0, "linear", 0.4, 0, 1.0)
gen.add("LaserAngle", int_t, 0, "LaserAngle", 30, 10, 90)
gen.add("ResponseDist", double_t, 0, "ResponseDist", 0.55, 0, 8)
exit(gen.generate(PACKAGE, "LineDetect", "LineDetectPID"))
gen.add("Kp", int_t, 0, "Kp in PID", 60, 0, 100)
Parameters Analysis Corresponding parameters
name The name of the parameter "Kp"
type parameter data type int_t
level a bitmask passed to the callback 0
description A description parameter "Kp in PID"
default Initial value for node startup 60
min parameter minimum value 0
max parameter maximum value 10.0Take the above article as an example to analyze
Note: After modification, you must recompile and update the environment to be effective. 
5.2.4. Tracking driving  
After identifying that there is no problem, click the [space bar] on the keyboard to execute the  
tracking program.
Node view
 
【LineDetect 】 Node analysiscd ~/yahboomcar_ws
catkin_make
source devel/setup.bash
rqt_graph
Subscribe to lidar
Subscribe to images
Subscription handle
Publish speed information
Post buzzer messages

---

## 5. Yolov5+tensorrt acceleration (jetson).pdf

5 Yolov5+tensorrt acceleration(jetson)  
5 Yolov5+tensorrt acceleration(jetson) 
5.1 Introduction 
5.2 Use 
5.3 tensorrt deployment process 
5.3.1 Generate .wts file 
5.3.2 Generate .engine file 
5.3.3 Test 
5.4 ROS deployment 
tensorrt source code: https://github.com/wang-xinyu/tensorrtx  
Official installation tutorial: https://docs.nvidia.com/deeplearning/tensorrt/install-
guide/index.html  
Official tutorial: https://docs.nvidia.com/deeplearning/tensorrt/developer-guide/index.html  
5.1 Introduction  
TensorRT is a high-performance deep learning inference(Inference) optimizer that provides low-
latency, high-throughput deployment inference for deep learning applications.  TensorRT can be  
used to accelerate inference in hyperscale data centers, embedded platforms, or autonomous  
driving platforms.  TensorRT can now support almost all deep learning frameworks such as  
TensorFlow, Caffe, Mxnet, Pytorch, etc. The combination of TensorRT and NVIDIA GPU can  
perform fast and efficient deployment inference in almost all frameworks.  
Can TensorRT accelerate the model?  
can!According to official documents, using TensorRT, it can provide 10X or even 100X acceleration  
in CPU or GPU mode.  TensorRT provides 20X speedup.  
TensorRT is only responsible for the inference process of the model and optimizes the trained  
model. Generally, TensorRT is not used to train the model.  tensorRT is just an inference  
optimizer. After your network is trained, you can directly drop the training model file into  
tensorRT without relying on the deep learning framework(Caffe, TensorFlow, etc.), as follows:  
It can be considered that tensorRT is a deep learning framework with only forward propagation.  
This framework can parse the network models of Caffe and TensorFlow, and then map them one  
by one with the corresponding layers in tensorRT, and convert all the models of other frameworks  
into tensorRT. Then in tensorRT, you can implement optimization strategies for NVIDIA's own  
GPUs and accelerate deployment.  
5.2 Use  
[device] parameter: the main control device of the robot, for example: nano4G, nx, etc.  
[display] parameter: whether to enable the visual interface.  
Support real-time monitoring of web pages, such as:  roslaunch  yahboomcar_yolov5  yolodetect.launch  device:=nano4G  display:=true 
View node information  
Print detection information  
print as follows  
frame_id: Identifying name.  
scores: Identify scores.  
ptx, pty: the coordinates of the upper left corner of the recognition box.  
distw, disth: The width and height of the recognition box.  
centerx, centery: Identify the center.  
5.3 tensorrt deployment process  
5.3.1 Generate .wts file  
For example: yolov5s.wts  
Copy tensorrtx_yolov5_jetson/gen_wts.py and [yolov5-5.0/weights/yolov5s.pt] weight file(or the  
weight file downloaded by Baidu, take [yolov5s.pt] as an example)) to [~/software/yolov5-5.0 ]  
folder, and execute gen_wts.py in this directory to generate the .wts file.  
Copy the generated [yolov5s.wts] file to the tensorrtx_yolov5_jetson directory, and compile  
tensorrtx in this directory  192.168.2.89:8080 
rqt_graph 
rostopic echo/DetectMsg 
data :  
  -  
    frame_id :  "tv" 
    stamp :  
      secs :  1646123012 
      nsecs :  116803169 
    scores :  0.754247903824 
    ptx :  106.302101135 
    pty :  121.952651978 
    distw : 144.548706055 
    disth : 90.0989227295 
    centerx :  72.2743530273 
    centery :  45.0494613647 
python3 gen_wts.py -w yolov5s.pt 
mkdir build && cd build && cmake .. 
Change CLASS_NUM in yololayer.h to yours. Because the official data set is coco, the default is 80.  
Execute makeFile. (make once every time it is modified to CLASS_NUM)  
5.3.2 Generate .engine file  
Generate .engine file(I use yolov5s, so end with s)  
If you customize depth_multiple and width_multiple during training, just write:  
The P6 model of yolov5 was also updated in tensorrtx 5.0  
5.3.3 Test  
Test it with the pictures that come with it(there are two pictures in the samples)  
You can test it with a python file, you need to create a yaml file.  
For example: coco.yaml  
test 
5.4 ROS deployment  
Copy the generated [yolov5s.engine] and [libmyplugins.so] files to the [yahboomcar_yolov5]  
function package [param/nano4G] folder, for example:  make -j4 
sudo ./yolov5 -s ../yolov5s.wts yolov5s.engine s 
sudo ./yolov5 -s ../yolov5.wts yolov5.engine c 0.33 0.5 
sudo ./yolov5 -s ../yolov5.wts yolov5.engine s6 
sudo ./yolov5 -d yolov5s.engine ../samples 
PLUGIN_LIBRARY : libmyplugins.so              # plugin library 
engine_file_path : yolov5s.engine             # engine file path 
CONF_THRESH : 0.5                             # CONF threshold 
IOU_THRESHOLD : 0.4                           # IOU threshold 
categories : [ "person", "bicycle", ... ...] # Identify categories 
cd .. 
python3 yolov5.py 
yahboomcar_yolov5 
    ├──  CMakeLists.txt 
    ├──  launch 
    │    └──  yolodetect.launch 
    ├──  package.xml 
    ├──  param 
    │    ├──  coco.yaml 
When using, follow the operation steps of [5.2].  
write yaml file  
For example: coco.yaml  
If the file name changes, you need to modify the [yolov5.py] file and open it in the file path  
As follows  
The parameters of [file_yaml], [PLUGIN_LIBRARY], and [engine_file_path] should correspond one-
to-one with the used names.      │    ├──  nano4G 
    │    │    ├──  libmyplugins.so 
    │    │    └──  yolov5s.engine 
    │    └──  nx 
    │        ├──  libmyplugins.so 
    │        └──  yolov5s.engine 
    └──  scripts 
        └──  yolov5.py 
PLUGIN_LIBRARY : libmyplugins.so              # plugin library 
engine_file_path : yolov5s.engine             # engine file path 
CONF_THRESH : 0.5                             # CONF threshold 
IOU_THRESHOLD : 0.4                           # IOU threshold 
categories : [ "person", "bicycle", ... ...] # Identify categories 
sudo vim ~/yahboomcar_ws/src/yahboomcar_yolov5/scripts/yolov5.py 
class  YoloDetect : 
     def __init__(self): 
        rospy.on_shutdown(self.cancel) 
        rospy.init_node("YoloDetect", anonymous = False) 
        self.pTime  =  self.cTime  =  0 
        device  =  rospy.get_param("~device", "nano4G") 
        param_  =  rospkg.RosPack(). get_path("yahboomcar_yolov5")  +  '/param/' 
 
        file_yaml  =  param_  +  'coco.yaml' 
        PLUGIN_LIBRARY  =  param_  +  device  +  "/libmyplugins.so" 
        engine_file_path  =  param_  +  device  +  "/yolov5s.engine" 
        self.yolov5_wrapper  =  YoLov5TRT(file_yaml, PLUGIN_LIBRARY, 
engine_file_path) 
        self.pub_image  =  rospy.Publisher('Detect/image_msg', Image_Msg, 
queue_size = 10) 
        self.pub_msg  =  rospy.Publisher('DetectMsg', TargetArray, queue_size = 
10) 

---

## 5.Patrol game.pdf

5. Patrol gameplay  
5. Patrol gameplay
5.1. How to use
5.1.1. Start
5.1.2. Parameter modification
5.1.3. Patrol function
5.2. Source code analysis
5.1. How to use  
Note: [R2] on the remote control handle has the [pause/start] function for this gameplay.  
This section takes the X3 Mai Lun car as an example. 
Depending on the environment, the parameters will be different; this function requires patient  
debugging to achieve good results.
5.1.1. Start  
One-click startup (robot side)
Started successfully, print log
At this time, after the above three parts are started successfully, the dynamic parameter  
debugging tool is started on the virtual machine side.
<PI5 needs to open another terminal to enter the same docker container#Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
roslaunch yahboomcar_bringup patrol.launch
# Log of successful patrol function startup
Bring up rqt_reconfigure to control the Robot.
# Log of successful startup of the underlying driver
First IMU message received.
# Log of successful lidar startup
RPLIDAR S/N: 6A97EDF9C7E29BD1A7E39EF2FA44431B
[ INFO] [1631677752.206795121]: Firmware Ver: 1.29
[ INFO] [1631677752.208026726]: Hardware Rev: 7
[ INFO] [1631677752.210976099]: RPLidar health status : 0
[ INFO] [1631677752.808115075]: current scan mode: Sensitivity, max_distance: 
12.0 m, Point number: 7.9K, angle_compensate: 2
Parameters Range Parse
[SetLoop] [False, True] Loop patrol, default False.
[Linear] [0.0, 1.0] The running linear speed of the car
[Angular] [0.0, 5.0] The running angular speed of the car
[Length] [0.0, 2.0] The straight-line running distance of the car
[Angle] [0.0, 360.0] Rotation angle of the car
Parameter analysis:rosrun rqt_reconfigure rqt_reconfigure
Parameters Range Parse
[LineScaling] [0.0, 2.0] Straight line distance scaling ratio, default 0.9
[RotationScaling] [0.0, 2.0] Rotation angle scaling, default 1.0
[LineTolerance] [0.0, 1.0] Allowable straight line distance error
[RotationTolerance] [0.0, 5.0] Allowed rotation angle error
[ResponseDist] [0.0, 8.0]If there are obstacles within the response distance,
the car will stop moving;
Remove the obstacles and the car will continue to
complete the patrol task.
[LaserAngle] [10, 180] Lidar detection angle (left and right angles)
[Command]Default
[Square]Patrol mode: [LengthTest, AngleTest, Triangle,
Square, Parallelogram, Circle]
[Switch] [False, True] Patrol function [Start/Pause]
When debugging parameters, mainly debug [RotationScaling]. It can be debugged to [1.0, 1.1, 1.2,  
1.3, 1.4], etc. according to the actual situation. Observe the effect. The circle is generally 0.9.
1), [LengthTest]: Run the test command straight, adjust the [LineScaling] and [LineTolerance]  
parameters so that the actual running distance of the car is close to the value [Length].
[LineScaling] The smaller the parameter, the greater the straight distance. The smaller the  
[LineTolerance] parameter is, the greater the front and rear vibration will be. Just debug multiple  
times and find the best data. Errors will always exist.
2), [AngleTest]: Rotation test command, adjust the [RotationScaling] and [RotationTolerance]  
parameters so that the true rotation distance of the car is close to the value [Angle].
[RotationScaling] The smaller the parameter, the larger the rotation angle. The smaller the  
[RotationTolerance] parameter is, the greater the left and right vibration will be. Just debug  
multiple times and find the best data. Errors will always exist.
After debugging [1)] and [2)], [LineScaling] and [LineTolerance], [RotationScaling] and  
[RotationTolerance] generally do not need to be adjusted.
5.1.2. Parameter modification  
When the parameters are adjusted to the optimal state, the corresponding parameters are  
modified into the file, and no adjustment is required when using again.
According to the optimal parameters of the [rqt_reconfigure] debugging tool, enter the [scripts]  
folder of the [yahboomcar_bringup] function package and modify the parameters corresponding  
to the [patrol.py] file, as shown below
class YahboomCarPatrol():
     def __init__(self):
         ... ...
         self.SetLoop = False
         self.Linear = 0.5
         self.Angular = 1.0
Parameters Analysis Corresponding parameters
name The name of the parameter "Linear"
type parameter data type double_t
level a bitmask passed to the callback 0
description A description parameter "Linear in robot"
default Initial value for node startup 0.5
min parameter minimum value 0
max parameter maximum value 1.0[rqt_reconfigure] Modification of the initial value of the debugging tool
Enter the [cfg] folder of the [yahboomcar_bringup] function package and modify the initial value of  
the corresponding parameter in the [PatrolParam.cfg] file.
Take the above article as an example to analyze
Note: After modification, you must recompile and update the environment to be effective.          self.Length = 1.0
         self.Angle = 360.0
         self.LineScaling = 1.1
         self.RotationScaling = 0.75
         self.LineTolerance = 0.1
         self.RotationTolerance = 0.3
         self.ResponseDist = 0.6
         self.LaserAngle = 20
         self.Command = "finish"
         self.circle_adjust = rospy.get_param('~circle_adjust', 2.0)
gen.add("SetLoop", bool_t, 0, "SetLoop", False)
gen.add("Linear", double_t, 0, "Linear in robot", 0.5, 0, 1.0)
gen.add("Angular", double_t, 0, "Angular in robot", 1.0, 0, 5.0)
gen.add("Length", double_t, 0, "Length in limit", 1.0, 0, 2.0)
gen.add("Angle", double_t, 0, "Angle in limit", 360.0, 0, 360.0)
gen.add("LineScaling", double_t, 0, "Line Scaling", 1.1, 0, 2.0)
gen.add("RotationScaling", double_t, 0, "Rotation Scaling", 0.75, 0, 2.0)
gen.add("LineTolerance", double_t, 0, "LineTolerance", 0.1, 0, 3.0)
gen.add("RotationTolerance", double_t, 0, "RotationTolerance", 0.3, 0, 5.0)
gen.add("ResponseDist", double_t, 0, "ResponseDist in limit", 0.6, 0.0, 8.0)
gen.add("LaserAngle", int_t, 0, "LaserAngle in limit", 30, 10, 180)
gen.add("Linear", double_t, 0, "Linear in robot", 0.5, 0, 1.0)
cd ~/yahboomcar_ws
catkin_make
source devel/setup.bash
5.1.3. Patrol function  
After successful startup, select the trajectory [Triangle, Square, Parallelogram, Circle] to be  
executed for patrol in [Command].
The parameter [Length] can be adjusted according to needs. For example, the default value is  
1.0, the command is Square, and the trajectory of the car is a square with a side length of 1.0.
When adjusting the parameter [Linear], please note that the greater the speed, the greater  
the inertia, and the smaller the accuracy.
Parameter [LaserAngle]: For example: the angle is 30°. At this time, the system only analyzes  
30° on the left and right of the car (0° in front of the car)
Parameter [Switch]: After setting, click the box behind [Switch] to start patrolling. By default, it  
is executed once. After the execution is completed, the check mark in the box disappears  
automatically.
If you need to patrol in a loop, click the box behind [SetLoop] to continue patrolling, and the  
error will accumulate to become larger and larger.
Node view
5.2. Source code analysis  
launch file
patrol.launch
circle_adjust parameter: It is a circle patrol parameter, which is a proportional factor for  
adjusting the circle radius.
patrol.py source code flow chart:rqt_graph
<launch>
     <!-- Start the underlying driver -->
     <include file="$(find yahboomcar_bringup)/launch/bringup.launch"/>
     <!-- Start lidar Start lidar -->
     <include file="$(find rplidar_ros)/launch/rplidar.launch"/>
     <!-- Start patrol node Start patrol node -->
     <node pkg="yahboomcar_bringup" type="patrol.py" name="YahboomCarPatrol" 
required="true" output="screen">
         <param name="circle_adjust" type="double" value="2.0"/>
     </node>
</launch>

---

## 6、yolov4-tiny_en.pdf

6.yolov4-tiny  
6.yolov4-tiny
6.1. Introduction
6.2. Use
6.3. Folder structure
6.4. Environmental requirements
6.5. Customized training data set
6.5.1. Create data set
6.5.2. Add weight file
6.5.3. Create label file
6.5.4. Modify the train.py file
6.5.5. Model detection
yolov4-tiny official website: https://github.com/AlexeyAB/darknet
Source code: https://github.com/bubbliiiiing/yolov4-tiny-tf2
6.1. Introduction  
Release time node
2020.04: YOLOv4 officially released
2020.06: YOLOv4-Tiny officially released
YOLOv4-Tiny performance on COCO: 40.2% AP50, 371 FPS (GTX 1080 Ti)  Whether it is AP or FPS  
performance, it is a huge improvement compared to YOLOv3-Tiny, Pelee, and CSP. As shown  
below:
Comparison of YOLOv4 and YOLOv4-Tiny detection results, source network
YOLOv4 test results
 
YOLOv4-Tiny test resultsDone! Loaded 162 layers from weights-file
data/dog.jpg: Predicted in 27.039000 milli-seconds.
Bicycle: 92%
dog: 98%
truck: 92%
Potted plants: 33%
 
We can see that the detection accuracy of Yolov4-tiny has declined, but Yolov4-tiny has obvious  
advantages in terms of time consumption: Yolov4-tiny detection takes only 2.6 milliseconds, while  
Yolov4 detection takes 27 milliseconds, which is faster. More than 10 times!
6.2. Use  
[display] parameter: whether to enable the visual interface.
Supports real-time monitoring of web pages, such as:
View node informationDone! Loaded 38 layers from weights-file
data/dog.jpg: Predicted in 2.609000 milli-seconds.
Bicycle: 29%
dog: 72%
truck: 82%
cars: 46%
#Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
roslaunch yahboomcar_yolov4_tiny yolodetect.launch display:=true
192.168.2.89:8080
Print detection information
Print as follows
frame_id: identification name.
scores: recognition scores.
ptx, pty: Coordinates of the upper left corner of the identification box.
distw, disth: the width and height of the identification box.
centerx, centery: identify the center.
6.3. Folder structure  rqt_graph
rostopic echo /DetectMsg
data:
   -
     frame_id: "person"
     stamp:
       secs: 1646128857
       nsecs: 530594825
     scores: 0.609634816647
     ptx: 109.685585022
     pty: -2.94450759888
     distw: 374.364135742
     Disth: 236.672561646
     centerx: 187.182067871
     centery: 118.336280823
yolov4-tiny-tf2
├── font # Store font package
│ └── Block_Simplified.TTF
├── garbage_data # Data set
│ ├── GetData.py # Get the data set code
│ ├── image # Target source file
│ ├── JPEGImages # Data set pictures (as many as possible)
│ ├── texture # Background image (as many as possible)
│ └── train.txt # Label file corresponding to the data set image
├── img # Store test images
│ └── 1.jpg
├── logs # Store test logs and final training model last1.h5.
├── model_data #Storage pre-trained model (weight file)
│ ├── coco.txt
│ ├── garbage.h5
│ ├── garbage.txt # Custom label file (corresponding to the target source file)
│ ├── yolo_anchors.txt
The concept of anchor box was introduced in the YOLO-v2 version, which greatly increased the  
performance of target detection. The essence of anchor is the reverse of the SPP (spatial pyramid  
pooling) idea, and what does SPP itself do? It is to combine different sizes The input is resized to  
become the output of the same size, so the reverse of SPP is to reverse the output of the same  
size to get the input of different sizes.
6.4. Environmental requirements  
The factory image is already configured and no installation is required.
Installation example
6.5. Customized training data set  
6.5.1. Create data set  
Method 1: Take some photos first, use the annotation tool to mark the targets on each photo,  
create a new [train.txt] file under the [garbage_data] folder, and write the target information in a  
specific format.
Method 2: Put background images (as many as possible) in the [garbage_data/texture] folder,  
modify the [GetData.py] code as needed, and execute [GetData.py] to generate a data set (as  
many as possible).
The names of the pictures and label files must correspond. The label format in the [train.txt] file is  
as follows:
Take method 2 as an example.│ ├── yolov4_tiny_weights_coco.h5 # Weight file
│ └── yolov4_tiny_weights_voc.h5 # Weight file
├── predict_img.py # Image detection code
├── predict_video.py # Video detection code
├── README.md
├── train.py # Training model code
├── utils # library files
└── yolo_nets # Network structure library file
tensorflow-gpu==2.2.0
lxml
matplotlib
pandas
Pillow
scikit-learn
seaborn
tqd
imgaug
pip install imgaug
./garbage_data/JPEGImages/0.jpg 113,163,293,298,9 # Image path y, x, y + w, x + h 
,label
Open the [GetData.py] file
Modify the total number of generated data sets and fill it in as needed. [More], too few data sets  
will lead to unsatisfactory training results.
Run the [GetData.py] file to obtain the data set
6.5.2. Add weight file  
There are good weight files (pre-trained models) [yolov4_tiny_weights_coco.h5] and  
[yolov4_tiny_weights_voc.h5] provided under the [model_data] file. Choose one of the two, and  
recommend coco’s weight file.
If you need the latest weight file, just search it on Baidu and download it.
6.5.3. Create label file  
Be careful not to use Chinese tags and there should be no spaces in the folder!
For example: garbage.txt
6.5.4. Modify the train.py file  
Modify according to your own needs by referring to the comments.sudo vim GetData.py
img_total=10000
python GetData.py
Zip_top_can
Old_school_bag
Newspaper
Book
Toilet_paper
... ...
# label position
annotation_path = 'garbage_data/train.txt'
# Get the location of classes and anchor
classes_path = 'model_data/garbage.txt'
anchors_path = 'model_data/yolo_anchors.txt'
# Location of pre-trained model
weights_path = 'model_data/yolov4_tiny_weights_coco.h5'
# Get classes and anchor
class_names = get_classes(classes_path)
anchors = get_anchors(anchors_path)
#How many categories are there in total?
num_classes = len(class_names)
num_anchors = len(anchors)
# The location where the trained model is saved
log_dir = 'logs/'
# Enter the image size. If the video memory is large, 608x608 can be used.
Follow the above process, and after the operation is completed, directly run the [train.py] file for  
training.
6.5.5. Model detection  
Modify the yolov4-tiny-tf2/utils/yolo.py file
Image detection
During this period, you need to manually enter the images that need to be detected, as shown  
below:input_shape = (416,416)
#Initial epoch value
Init_epoch = 0
# Freeze the epoch value of training
Freeze_epoch = 50
# The size of Batch_size indicates how much data is fed each time. If there is 
OOM or insufficient video memory, please adjust it smaller.
batch_size = 16
# Maximum learning rate
learning_rate_base = 1e-3
#Total epoch value
Epoch = 100
python3train.py
class YOLO(object):
     _defaults = {
         # Used to detect and train the model path.
         "model_path": 'model_data/garbage.h5',
         # yolo model parameter anchors path
         "anchors_path": 'model_data/yolo_anchors.txt',
         # Custom label file path
         "classes_path": 'model_data/garbage.txt',
         "score" : 0.5,
         "iou" : 0.3,
         "eager" : False,
         # The default is 416x416 (image size)
         "model_image_size" : (416, 416)
     }
... ...
#Font package path
self.font_path = 'font/Block_Simplified.TTF'
python3 predict_img.py
Video detection
python3 predict_video.py

---

## 6. AR vision.pdf

6. AR vision  
6. AR vision 
6.1. Overview 
6.2. How to use 
6.3. Source code analysis 
6.3.1. Algorithm principle 
6.3.2. core code 
Feature pack: ~/yahboomcar_ws/src/yahboomcar_visual 
This section can be run on a virtual machine or on the mainboard of the car. Let's take jetson as  
an example. 
6.1. Overview  
Augmented Reality, referred to as "AR", is a technology that ingeniously integrates virtual  
information with the real world. It widely uses multimedia, 3D modeling, real-time tracking and  
registration, intelligent interaction, sensing and other technologies. The method is to simulate and  
simulate virtual information such as text, images, three-dimensional models, music, and videos  
generated by the computer, and then apply it to the real world. The two kinds of information  
complement each other, thereby realizing the "enhancement" of the real world. 
The AR system has three prominent features: (1) the information integration of the real world and  
the virtual world; (2) real-time interactivity; (3) the addition of positioning virtual objects in the  
three-dimensional scale space. 
Augmented reality technology includes multimedia, three-dimensional modeling, real-time video  
display and control, multi-sensor fusion, real-time tracking and registration, scene fusion and  
other new technologies and new means. 
6.2. How to use  
When using the AR case, you must have the internal parameters of the camera, otherwise it  
will not work.  The internal parameter file is in the same directory as the code (under the AR  
folder of the function package); different cameras correspond to different internal parameters. 
The internal parameter calibration can be quickly calibrated with a checkerboard. The specific  
method can be seen in the lesson [Astra Camera Calibration]. 
Start the monocular camera 
<PI5 needs to open another terminal and enter the same docker container#Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
roslaunch usb_cam usb_cam-test.launch 
Start the calibration node 
After calibration, move the [calibrationdata.tar.gz] file to the [home] directory. 
After decompression, open [ost.yaml] in the folder, find the camera internal parameter matrix and  
distortion coefficient and modify it to the corresponding location of the [astra.yaml] file, just  
modify the contents of two [data]. For example: the following. 
A total of 12 effects. 
start command rosrun camera_calibration cameracalibrator.py image:=/usb_cam/image_raw 
camera:=/usb_cam --size 9x6 --square 0.02 
sudo mv /tmp/calibrationdata.tar.gz ~ 
camera_matrix :  !! opencv - matrix 
  rows :  3 
  cols :  3 
  dt :  d 
  data : [ 615.50506 ,    0.      ,  365.84388 , 
           0. , 623.69024 , 238.778 , 
           0. , 0. , 1. ] 
distortion_model :  plumb_bob 
distortion_coefficients :  !! opencv - matrix 
  rows :  1 
  cols :  5 
  dt :  d 
  data : [ 0.166417 ,  - 0.160106 ,  - 0.008776 ,  0.025459 ,  0.000000 ] 
[ "Triangle" ,  "Rectangle" ,  "Parallelogram" , "WindMill" , "TableTennisTable" 
,  "Ball" ,  "Arrow" ,  "Knife" ,  "Desk" , 
"Bench" ,  "Stickman" ,  "ParallelBars" ] 
display parameter: True; the image window is displayed locally; Fasle, it is not displayed. 
flip parameter: whether to switch the screen horizontally, the default is OK. 
Set parameters according to your needs, you can also directly modify the launch file, and you  
don't need to attach parameters when starting. When the screen is not turned on, you can use the  
network monitoring method to view 
1. When the screen is displayed (that is, display is true), press the [q] key to exit, and the [f] key  
to switch between different effects. You can also use the command line to switch. 
Use the [f] or [F] key to switch between different effects. roslaunch yahboomcar_visual simple_AR.launch display:=true flip:=false
Open the IP of the device: 8080 
2. When the screen is not displayed (ie display is false), the effect can only be switched through  
the command line 

6.3. Source code analysis  
6.3.1. Algorithm principle  
Find object poses from 3D-2D point correspondences using the RANSAC scheme. 
The RanSaC algorithm (Random Sampling Consistency) was originally a classic algorithm for data  
processing. Its function is to extract specific components in objects in the presence of a large  
amount of noise. The following figure is an illustration of the effect of the RanSaC algorithm. There  
are some points in the figure that obviously satisfy a straight line, and another group of points is  
pure noise. The goal is to find the equation of the line in the presence of a lot of noise, where the  
amount of noisy data is 3 times that of the line. 
If this effect cannot be obtained by the least squares method, the straight line will be a little higher  
than the straight line in the figure. 
The basic assumptions of RANSAC are:  (1) The data consists of "inside points", for example: the  
distribution of the data can be explained by some model parameters;  (2) "outlier points" are data  
that cannot fit the model;  (3) Except Data other than this is noise.  The causes of outliers are:  
extreme values of noise; wrong measurement methods; wrong assumptions about the data.   
RANSAC also makes the following assumptions: given a set of (usually small) intra-office points,  
there is a process by which the model parameters can be estimated; and the model can explain or  
apply to intra-office points. 
Yes Noinput image
Is the cor ner o f each pictur e
Find cor ner subpix els output image
Comput ed object attitude
Output image points and J acobian matrix
Draw and output the image6.3.2. core code  
Design Flow: 
launch file 
python main function 
key function 
https://docs.opencv.org/3.0-
alpha/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html 
findChessboardCorners() < launch > 
    < arg  name = "flip"  default = "False" /> 
    < arg  name = "display"  default = "False" /> 
    < node  name = "simple_AR"  pkg = "yahboomcar_visual"  type = "simple_AR.py"  
output = "screen"  args = "$(arg display)" > 
        < param  name = "flip"  type = "bool"  value = "$(arg flip)" /> 
        < remap  from = "/simpleAR/camera"  to = "/simpleAR/camera" /> 
    </ node > 
    <!-- web_video_server --> 
    < node  pkg = "web_video_server"  type = "web_video_server"  name = 
"web_video_server"  output = "screen" /> 
</ launch > 
    def  process ( self ,  img ): 
        if  self . flip  ==  'True' :  img  =  cv . flip ( img ,  1 ) 
        gray  =  cv . cvtColor ( img ,  cv . COLOR_BGR2GRAY ) 
        # Find the corners of each image 
        retval ,  corners  =  cv . findChessboardCorners ( 
            gray ,  self . patternSize ,  None , 
            flags = cv . CALIB_CB_ADAPTIVE_THRESH  +  cv . 
CALIB_CB_NORMALIZE_IMAGE  +  cv . CALIB_CB_FAST_CHECK ) 
        # Find corner subpixels 
        if  retval : 
            corners  =  cv . cornerSubPix ( 
                gray ,  corners , ( 11 ,  11 ), ( - 1 ,  - 1 ), 
                ( cv . TERM_CRITERIA_EPS  +  cv . TERM_CRITERIA_MAX_ITER ,  30 , 
 0.001 )) 
            # Calculate object pose solvePnPRansac 
            retval ,  rvec ,  tvec ,  inliers  =  cv . solvePnPRansac ( 
                self . objectPoints ,  corners ,  self . cameraMatrix ,  self . 
distCoeffs ) 
            # output image points and jacobian matrix 
            image_Points ,  jacobian  =  cv . projectPoints ( 
                self . __axis ,  rvec ,  tvec ,  self . cameraMatrix ,  self . 
distCoeffs , ) 
            # draw the image 
            img  =  self . draw ( img ,  corners ,  image_Points ) 
        return  img 
def  findChessboardCorners ( image ,  patternSize ,  corners = None ,  flags = 
None ):  
    ''' 
Find image corners 
:param image: Input the original checkerboard image. The image must be an 8-bit 
grayscale or colormap. 
cornerSubPix() 
We need to use cornerSubPix() to further optimize the detected corners, so that the accuracy of  
the corners can reach sub-pixel level. 
solvePnPRansac() :param patternSize: (w,h), the number of interior corners of each row and column 
on the board.  w = the number of black and white blocks on a row of the board - 
1, h = the number of black and white blocks on a column of the board - 1. 
For example: 10x6 chessboard, then (w,h)=(9,5) 
:param corners: array, output array of detected corners. 
:param flags: int, different operation flags, can be 0 or a combination of the 
following values: 
CALIB_CB_ADAPTIVE_THRESH Converts the image to black and white using adaptive 
thresholding instead of using a fixed threshold. 
    CALIB_CB_NORMALIZE_IMAGE Use histogram equalization to equalize the image 
before binarizing it with fixed threshold or adaptive thresholding. 
    CALIB_CB_FILTER_QUADS uses additional criteria (such as contour area, 
perimeter, square shape) to filter out false quads extracted during the contour 
retrieval stage. 
    CALIB_CB_FAST_CHECK Runs a quick check mechanism on the image to find the 
corners of the board, and returns a quick reminder if no corners are found. 
    Calls in degenerate conditions can be greatly accelerated when the 
checkerboard is not observed. 
    :return: retval, corners 
''' 
    pass 
def  cornerSubPix ( image ,  corners ,  winSize ,  zeroZone ,  criteria ): 
    ''' 
Subpixel corner detection function 
:param image: input image 
:param corners: pixel corners (both as input and output) 
:param winSize: area size is NXN; N=(winSize*2+1) 
:param zeroZone: Similar to winSize, but always has a smaller range, Size(-1,-1) 
means ignore 
:param criteria: criteria to stop optimization 
:return: subpixel corner 
''' 
    pass 
def  solvePnPRansac ( objectPoints ,  imagePoints ,  cameraMatrix ,  distCoeffs , 
 
                   rvec = None ,  tvec = None ,  useExtrinsicGuess = None , 
 iterationsCount = None ,  
                   reprojectionError = None ,  confidence = None ,  inliers = 
None ,  flags = None ): 
    ''' 
    Calculate object pose 
:param objectPoints: list of object points 
:param imagePoints: list of corner points 
:param cameraMatrix: camera matrix 
:param distCoeffs: Distortion coefficients 
    :param rvec: 
Find object poses from 3D-2D point correspondences using the RANSAC scheme. This function  
estimates the object pose given a set of object points, their corresponding image projections, and  
the camera matrix and distortion coefficients. This function finds a pose that minimizes the re-
projection error, the re-observation error, that is, the observed pixel point projection imagePoints  
and the object projection ( projectPoints （）) sum of squared distances between objectPoints.   
The use of RANSAC can avoid the effect of outliers on the results. 
projectPoints()     :param tvec: 
    :param useExtrinsicGuess: 
    :param iterationsCount: 
:param reprojectionError: 
    :param confidence: 
    :param inliers: 
    :param flags: 
    :return: retval, rvec, tvec, inliers 
''' 
    pass 
def  projectPoints ( objectPoints ,  rvec ,  tvec ,  cameraMatrix ,  distCoeffs , 
 imagePoints = None ,  jacobian = None ,  aspectRatio = None ):  
    ''' 
Output image points and Jacobian matrix 
    :param objectPoints:  
:param rvec: rotation vector 
:param tvec: translation vector 
:param cameraMatrix: camera matrix 
:param distCoeffs: Distortion coefficients 
    :param imagePoints:  
    :param jacobian:  
param aspectRatio: 
    :return: imagePoints, jacobian 
''' 
    pass 

---

## 6. Bind the device ID.pdf

6. Bind the device ID  
6. Bind the device ID 
6.1. Device view comma nd 
6.2. Establish port mapping relationship 
6.2.1. Device binding 
6.2.2. Introduction to rule file syntax 
6.3. Verify View 
6.4. Binding the USB port 
When the robot uses two or more USB serial devices, the corresponding relationship between the  
device name and the device is not fixed, but is allocated according to the order in which the  
devices are connected to the system. Inserting one device first and then another device can  
determine the relationship between the device and the device name, but it is very troublesome to  
plug and unplug the device every time the system starts. The serial port can be mapped to a fixed  
device name. No matter what the insertion order is, the device will be mapped to the new device  
name. We only need to use the new device name to read and write the device.  
6.1. Device view command  
View camera device parameters  
Enter the following command in the terminal to check the correspondence between the pixel size  
of the camera and the frame rate.  
v4l2-ctl --list-formats-ext 
Device ID view  
As can be seen from the figure below, the ID number of each device, Astra has the official file for  
binding the device, the handle generally does not need to be bound, and the main binding is PCB  
and radar.  lsusb 
Device number view  
ll /dev/ 
6.2. Establish port mapping relationship  
6.2.1. Device binding  
Astra binding  
There is a create_udev_rules file in the scripts folder under the astra_camera function package,  
which is automatically bound by running the file. Run the command as follows  
Go to the rules.d directory  
You can find the 56-orbbec-usb.rules file, which is the Astra camera device binding file.  ./create_udev_rules 
cd  /etc/udev/rules.d/ 
PCB and Radar Bonding  
Go to the rules.d directory  
Create a new rplidar.rules file  
Open the rplidar.rules file  
write the following  
Exit to make the rules take effect  
Replug the USB device and you're done.  
6.2.2. Introduction to rule file syntax  
Parse  
From [6.1], it is easy to see that the device number of the PCB is [ttyUSB0], and the ID number is  
[1a86, 7523], which is fixed. 0, 1, 2, 3, 4, ...] are all bound to [myserial]; the same is true for radar  
device [ttyUSB1]; the same is true for other devices that need to be bound.  
Note: When taking an alias, do not take some device names that already exist in the  
system, otherwise it will fail.  cd  /etc/udev/rules.d/ 
sudo touch rplidar.rules 
sudo chmod 777 rplidar.rules 
sudo vim rplidar.rules 
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", 
MODE:="0777", SYMLINK+="myserial" 
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", 
MODE:="0777", SYMLINK+="rplidar" 
sudo udevadm trigger 
sudo service udev reload 
sudo service udev restart 
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", 
MODE:="0777", SYMLINK+="myserial" 
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", 
MODE:="0777", SYMLINK+="rplidar" 
KERNEL           # The device name that matches the event 
ATTR{filename}   # Match the sysfs attribute of the event device. 
idVendor         # Vendor ID 
idProduct        # product number 
SYMLINK          # Generate symbolic links for device files under /dev/. Just 
give this device an alias. 
MODE             # Set permissions for the device. 
6.3. Verify View  
Device number view  
PCB 
laser  
6.4. Binding the USB port  
The above situations are all different ID numbers. If the ID numbers of the radar and the PCB are  
the same, or there are two or more PCBs (radars) with the same ID, the above binding will be  
confused.  
Then, we need to bind the USB port. After binding, the cannot be changed at will  , and each  
device can only be linked to a fixed  USB port.  
Binding method, take [ttyUSB0] as an example, check the port of the device at this time  ll /dev/ 
What we need is to modify in the rules file  
 udevadm info --attribute-walk --name=/dev/ttyUSB0 |grep KERNELS 
# KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", 
MODE:="0777", SYMLINK+="myserial"    # before modification 
KERNELS == "1-2.1.3" , ATTRS { idVendor }== "1a86" , ATTRS { idProduct }== "7523" 
, MODE := "0777" , SYMLINK += "myserial"      # After modification 

---

## 6. Pure visual 2D mapping and navigation.pdf

6. Pure visual 2D mapping navigation  
6. Pure visual 2D mapping navigation
6.1. Introduction
6.2. Use
6.2.1. Map construction
6.2.2. Controlling the robot
6.2.3. Map saving
6.2.4. Navigation
6.3. Topics and services
6.4. Configuration parameters
6.5, TF transformation
depthimage_to_laserscan: http://wiki.ros.org/depthimage_to_laserscan
depthimage_to_laserscan source code: https://github.com/ros-perception/depthimage_to_lasersca
n
6.1. Introduction  
depthimage_to_laserscan takes a depth image (float encoded meters or preferably uint16  
encoded millimeters for OpenNI devices) and generates a 2D laser scan based on the provided  
parameters. depthimage_to_laserscan uses delayed subscription, which does not subscribe to  
image or camera information until a user scans.
The depthimage_to_laserscan function package converts depth images into lidar data, and its  
mapping and navigation functions are the same as lidar. Note: The scanning range of the depth  
camera is not 360°.
6.2. Use  
Note: Pure depth mapping navigation in this section does not work well and is not  
recommended. 
Note: When building a map, the slower the speed, the better the effect (note that the  
rotation speed should be slower). If the speed is too fast, the effect will be poor. 
According to different models, you only need to set the purchased model in [.bashrc], X1 (normal  
four-wheel drive) X3 (Mailun) Take X3 as an example
Open the [.bashrc] file
Find the [ROBOT_TYPE] parameters and modify the corresponding car model#Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
sudo vim .bashrc
export ROBOT_TYPE=X3 # ROBOT_TYPE: X1 X3 X3plus R2 X7
6.2.1. Map construction  
Start command (robot side)
<PI5 needs to open another terminal to enter the same docker container
Mapping command (robot side)
[use_rviz] parameter: whether to enable rviz visualization.
[map_type] parameter: Set the mapping algorithm [gmapping].
Turn on the visual interface (virtual machine side)roslaunch yahboomcar_nav astrapro_bringup.launch
roslaunch yahboomcar_nav yahboomcar_map.launch use_rviz:=false map_type:=gmapping
roslaunch yahboomcar_nav view_vision_mapping.launch
6.2.2. Controlling the robot  
Keyboard controls robot movement
Control the robot movement with the handle
There may be some scattered points during the mapping process. If the mapping environment is  
well closed, relatively regular, and the movement is slow, the scattering phenomenon will be much  
smaller.
6.2.3. Map saving  
The map will be saved to the ~/yahboomcar_ws/src/yahboomcar_nav/maps/ folder, a pgm image  
and a yaml file.
map.yaml
Parameter analysis:rosrun teleop_twist_keyboard teleop_twist_keyboard.py # System integration
roslaunch yahboomcar_ctrl yahboom_keyboard.launch # Custom
rosrun map_server map_saver -f ~/yahboomcar_ws/src/yahboomcar_nav/maps/my_map # 
The first way
bash ~/yahboomcar_ws/src/yahboomcar_nav/maps/map.sh # The second way
image: map.pgm
resolution: 0.05
origin: [-15.4,-12.2,0.0]
Negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
image: The path of the map file, which can be an absolute path or a relative path.
resolution: resolution of the map, meters/pixel
Origin: 2D pose (x, y, yaw) in the lower left corner of the map. The yaw here is rotated  
counterclockwise (yaw=0 means no rotation). Many parts of the current system ignore the  
yaw value.
negate: whether to reverse the meaning of white/black and free/occupied (the interpretation  
of the threshold is not affected)
occupied_thresh: Pixels with an occupation probability greater than this threshold will be  
considered fully occupied.
free_thresh: Pixels with occupancy probability less than this threshold will be considered  
completely free.
6.2.4. Navigation  
Start command (robot side)
<PI5 needs to open another terminal to enter the same docker container
Navigation commands (robot side)
[use_rviz] parameter: whether to enable rviz visualization.
[map_type] parameter: Set the mapping algorithm [gmapping].
Turn on the visual interface (virtual machine side)
1. Single point navigationroslaunch yahboomcar_nav astrapro_bringup.launch
roslaunch yahboomcar_nav yahboomcar_navigation.launch use_rviz:=false map:=house
roslaunch yahboomcar_nav view_navigate.launch
Subscription
topicType Description
image sensor_msgs/ImageInput image. This can be in floating point or
raw uint16 format. For OpenNI devices,
uint16 is the native representation and is
more efficient to handle. This is usually
/camera/depth/image_raw. If your image is
distorted, you should remap this theme to
image_rect. OpenNI cameras typically have
very little distortion, so correction can be
skipped for this application.
camera_info sensor_msgs/CameraInfoCamera information for the associated
image.
Post Topic Type Description
scan sensor_msgs/LaserScanOutput laser scan. and will output a range
array containing NAN and +-INF.Use the [2D Pose Estimate] of the [rviz] tool to set the initial pose until the position of the car  
in the simulation is consistent with the position of the actual car.
Click [2D Nav Goal] of the [rviz] tool, and then select a target point on the map where there  
are no obstacles. Release the mouse to start navigation. Only one target point can be  
selected, and it will stop when it is reached.
2. Multi-point navigation
Same as the first step of single-point navigation, first set the initial pose of the car.
Click [Publish Point] of the [rviz] tool, and then select the target point on the map where there  
are no obstacles. Release the mouse to start navigation. You can click [Publish Point] again,  
and then select the point, and the robot will click on it. Cruising between points.
When using the [2D Pose Estimate] tool of the [rviz] tool to set the initial pose of the car, the  
multi-point navigation function is automatically canceled.
6.3. Topics and services  
Node view
rqt_graph
Parameters Type Default value Description
scan_height int 1 pixelNumber of rows of pixels used to
generate the laser scan. For each
column, the scan returns the
minimum value of the vertically
centered pixel in the image.
scan_time double 1/30.0Hz (0.033s)Scan interval time (seconds).
Typically, a rate of 1.0/frame. This
value is not easily calculated from
consecutive messages and is
therefore set correctly by the user.
range_min double 0.45mThe minimum range returned in
meters. Ranges smaller than this
value will be output as -Inf.
range_max double 10.0mThe maximum range returned in
meters. Ranges larger than this
value will be output as +Inf.
output_frame_id str camera_depth_frameLaser scanning frame id. For point
clouds from Z-forward "optical"
frames, this value should be set to
the corresponding frames in X-
forward and Z-up
6.4. Configuration parameters  
Required TF
transformationDescription
laser-->base_linkThe transformation between the laser radar coordinate system and
the base coordinate system is generally published by
robot_state_publisher or static_transform_publisher
base_link-->odomTransformation between the map coordinate system and the robot
odometer coordinate system, estimating the robot's pose in the map
Released TF
TransformDescription
map-->odomTransformation between the map coordinate system and the robot's
odometry coordinate system, estimating the robot's pose in the map6.5, TF transformation  
View tf tree
rosrun rqt_tf_tree rqt_tf_tree

---

## 6. RGB colorful light bar special effects display.pdf

6. RGB colorful light bar special effects display  
6. RGB colorful light bar special effects display 
6.1. Experimental objectives 
6.2. Experiment preparation 
6.3. Experimental effect 
6.4. Program source code 
6.1. Experimental objectives  
Control the RGB colorful light bar to display different special effects, manually control the color of  
the RGB light bar, and set the color of a single RGB light.  
 
6.2. Experiment preparation  
The position of the red box in the picture below is the interface of the RGB colorful light bar. The  
interface has anti-reverse connection function, and there is no need to worry about reverse  
connection during the connection process.  
RGB dazzling lights support the color of a single light, as well as control the color of all lights.  
The Rosmaster_Lib library functions required to control the RGB colorful light bar effects are as  
follows:  
Parameter explanation: RGB programmable light strip special effects display.  
effect=[0, 6], 0: stop light effect, 1: running water light, 2: marquee light, 3: breathing light, 4:  
gradient light, 5: starlight, 6: battery display  set_colorful_effect ( effect ,  speed = 255 ,  parm = 255 ) 
speed=[1, 10], the smaller the value, the faster the speed changes.  
parm, optional, as an additional parameter. Usage 1: Enter [0, 6] for the breathing light effect to  
modify the color of the breathing light.  
Return value: None.  
Parameter explanation: RGB programmable light strip control, which can be controlled  
individually or as a whole. Before the control, you need to stop the RGB light effects.  
led_id=[0, 16], control the corresponding number of RGB lights; led_id=0xFF, control all lights.  
red,green,blue=[0, 255], indicating the color RGB value.  
Return value: None.  
 
6.3. Experimental effect  
Check out the course accompanying video.  
 
6.4. Program source code  
Power on the Rosmaster robot, and open the browser of the Jetson Nano or remote computer to  
enter the Jupyter lab editor.  
Reference code path: Rosmaster/Samples/6.rgb_effect.ipynb  set_colorful_lamps ( led_id ,  red ,  green ,  blue ) 

---

## 6. Robot state estimation.pdf

6 Robot state estimation  
6 Robot state estimation 
6.1 Start 
6.1.1 Code reference path 
6.1.1 Start 
6.1.1 View tf tree and node graph 
6.1.2 launch file analysis 
6.1.3 imu_filter_madgwick 
6.1.4 robot_localization 
According to different models, you only need to set the purchased model in [.bashrc], X1  
(ordinary four-wheel drive) X3 (Mike wheel) X3plus (Mike wheel mechanical arm) R2  
(Ackerman differential) and so on. Section takes X3 as an example  
Open the [.bashrc] file  
Find the [ROBOT_TYPE] parameter and modify the corresponding model  
 
6.1 Start  
6.1.1 Code reference path  
6.1.1 Start  #Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
#Multiple ros commands require multiple terminals to be executed in the same 
docker container. Please refer to the tutorials in Sections 07/5 and 5.8.
sudo vim .bashrc 
export ROBOT_TYPE=X3    # ROBOT_TYPE: X1 X3 X3plus R2 X7 
~/yahboomcar_ws/src/yahboomcar_bringup/launch/bringup.launch 
roslaunch yahboomcar_bringup bringup.launch use_rviz:=true 
6.1.1 View tf tree and node graph  
1. view the TF tree 
2. view the node graph rosrun rqt_tf_tree rqt_tf_tree 
rosrun rqt_graph rqt_graph 
6.1.2 launch file analysis  
In the bringup.launch file, there are several important nodes 
1. /drive_node 
This node mainly publishes /imu/imu_raw and /vel_raw data, terminal input 
As can be seen, 
The /driver_node node publishes the /imu/imu_raw topic data to the /imu_filter_madgwick  
node, which filters and fuses the imu data; 
The /driver_node node publishes the /vel_raw topic data to the /odometry_publisher node.  
After the latter is integrated, the /odom_raw data is published; 
2. /odometry_publisher 
This node mainly receives the vel_raw topic data sent by the /driver_node node, publishes the  
/odom_raw topic data to /ekf_localization, and publishes the /tf topic data to other nodes, terminal  
input, rosnode info /driver_node  
rosnode info /odometry_publisher 
3. /imu_filter_madgwick 
This node mainly filters and fuses imu data, and then publishes the processed /imu/imu_data  
topic data to the /ekf_localization node to receive and publish /tf data to other nodes, terminal  
input, 
4. /ekf_localization 
This node mainly integrates imu data and odom data and publishes tf data, terminal input, rosnode info /imu_filter_madgwick 
rosnode info /ekf_localization 
/ topic name type Parse
Subscribed /imu/data_raw sensor_msgs/ImuMessages of calibrated
IMU data, including
angular velocity and linear
acceleration
Subscribed /imu/mag sensor_msgs/MagneticField[Optional] Magnetometer,
will be affected by
magnetic fields
Published /imu/data sensor_msgs/Imu Fused Imu information.
Next, we describe these nodes in detail. 
6.1.3 imu_filter_madgwick  
1 Introduction 
IMU refers to a six-axis sensor that includes a gyroscope and an accelerometer.  MARG refers to a  
nine-axis sensor that adds a magnetometer to the IMU. 
Madgwick is an Orientation Filter that filters and fuses raw data from IMU devices. It fuses angular  
velocity, acceleration, and (optional) magnetometer readings from generic IMU devices into  
orientation quaternions, and publishes the fused data on the IMU topic, regardless of the overall  
IMU integration process. 
2. topic 
3. parameters IMU  =  gyroscope  +  accelerometer 
MARG (Magnetic, Angular  Rate, and  Gravity)  =  gyroscope  +  accelerometer  +  
magnetometer 
parameter name type Defaults Parse
~gain double 0.1The gain of the filter. Higher values
result in faster convergence but more
noise. The lower the value, the slower
the convergence, but the smoother
the signal. Range: 0.0 to 1.0
~zeta double 0.0Gyro drift gain (about rad/s). Range:
-1.0 to 1.0
~ mag_bias_x double 0.0Magnetometer bias (hard iron
corrected) x-component. Range: -10.0
to 10.0
~mag_bias_y double 0.0Magnetometer bias (hard iron
correction) y component. Range: -10.0
to 10.0
~ mag_bias_z double 0.0Magnetometer bias (hard iron
correction) z component. Range: -10.0
to 10.0
~orientation_stddev double 0.0The standard deviation of the
orientation estimate. Range: 0.0 to 1.0
~world_frame string "nwu"World frame indicating direction (see
REP-145). The old default was "nwu"
(northwest up). New deployments
should use "enu". Valid values: "nwu",
"enu", "ned".
~ use_mag bool trueWhether to use magnetic field data in
data fusion.
~use_magnetic_field_msg bool falseIf set to true, Then subscribe /imu and
/mag topics as
sensor_msgs/MagneticField; If set to
false (deprecated) Then use
geometry_msgs/Vector3Stamped
~fixed_frame string odomParent coordinate system to use in
publishing
~publish_tf bool falseWhether to publish the TF transform
representing the direction of the IMU
as the pose of the IMU; Use a fixed
coordinate system as the parent
coordinate system, The input imu
information is used as the sub-
coordinate system
parameter name type Defaults Parse
~reverse_tf bool falseIf set to true, the transformation from
the imu coordinate system to the fixed
coordinate system is published, not
the other way around.
~constant_dt double 0.0The dt to use; if 0.0 (default) the dt
dynamic value is calculated from the
message start position.
~publish_debug_topics bool falseIf set to true, two debug topics are
published.
~stateless bool falseIf set to true, the filtered orientation is
not published. Instead, a stateless
estimate of orientation is published
based only on the latest accelerometer
(and optional magnetometer)
readings. for debugging.
~remove_gravity_vector bool falseIf set to true, the gravity vector is
subtracted from the acceleration field
in the published IMU message.
/ topic name type Parse
Subscribed /imu/data sensor_msgs/Imu Filtered imu information
Subscribed /odom_raw nav_msgs/Odometry Odometer Information
Published /odom nav_msgs/Odometry Fusion odometer information
Published /tf tf2_msgs/TFMessage Coordinate system information6.1.4 robot_localization  
1 Introduction 
robot_localizationis a collection of state estimation nodes, each of which is an implementation  
of a nonlinear state estimator for a robot moving in 3D space. It includes two state estimation  
nodes ekf_localization_nodeand ukf_localization_node. in addition, 
robot_localizationsupply navsat_transform_node, it helps to integrate GPS data. 
ekf_localization_nodeis Extended Kalman Filter . It uses an omnidirectional motion model to  
predict states in time and uses sensed sensor data to correct the predicted estimates. 
ukf_localization_nodeis unscented Kalman filter . It uses a carefully chosen set of sigma points  
to project the state through the same motion model used in the EKF, and then uses these  
projected sigma points to recover the state estimate and covariance. This eliminates the use of the  
Jacobian matrix and makes the filter more stable. However, with ekf_localization_nodeIt is also  
more computationally heavy in comparison. 
2. topic 
3. parameters 
frequency: The true frequency (in Hz) at which the filter produces state estimates. Note: The  
filter does not start computing until it has received at least one message from one of  
the inputs.  
[sensor] : For each sensor, the user needs to define this parameter according to the message  
type. Each parameter name is indexed from 0 (e.g. odom0, odom1, etc.) and must be defined  
in order (e.g. don't use pose0 and pose2 if pose1 is not already defined). The value of each  
parameter is the topic name for that sensor. 
[sensor]_differential: For each sensor message defined above that contains pose information,  
the user can specify whether the pose variables should be integrated differentially. If the  
given value is set to true, then for the measurement taken at time t from the relevant sensor,  
we will first subtract the measurement at time t-1 and then convert the resulting value to  
velocity. 
[sensor]_relative: If this parameter is set to true, any measurements from this sensor will be  
fused relative to the first measurement received from this sensor. This is useful, for example,  
if you want the state estimate to always start at (0,0,0) and the roll, pitch and yaw angle  
values to be (0,0,0). 
two_d_mode: Set this to true if your robot operates in a flat environment and can ignore  
small changes in the ground (as reported by the IMU). It fuses all 3D variables (Z, roll, pitch  
and their respective velocity and acceleration) into a value of 0. This ensures that the  
covariances of these values don't explode, while ensuring that your robot's state estimate  
remains fixed on the XY plane. 
odom0_config ：  [false, false, false, false, false, false, true, true, false, false, false, true, false,  
false, false] 
The order of Boolean values is: [[X],[Y],[Z],[roll],[pitch],[yaw],[X ' ],[Y ' ],[Z ' ],[roll '],[pitch '],[yaw  
'],[X ''],[Y ''],[Z '']]. The user must specify which variables of these messages should be fused  
into the final state estimate. odom0: /odom_raw 
imu0:/imu/data 
~odomN_differential 
~imuN_differential 
~poseN_differential 
~odomN_relative 
~imuN_relative 
~poseN_relative 
4. the published transformation 
If the user's world_frameparameter is set to odom_framevalue, the conversion from 
odom_frameThe coordinate system given by the parameter is published to base_link_frameThe 
coordinate system given by the parameter. If the user's world_frameparameter is set to 
map_framevalue, the conversion from map_frameThe coordinate system given by the parameter  
is published to odom_frameThe coordinate system given by the parameter. 
For example, we set to publish the transformation from the coordinate system given by the  
[odom_frame] parameter to the coordinate system given by the [base_link_frame] parameter. 
 odom_frame: odom 
base_link_frame: base_footprint 
world_frame: odom 

---

## 6. SBUS model aircraft remote control.pdf

6. SBUS model aircraft remote control  
6. SBUS model aircraft remote control 
6.1. Purpose of the experiment 
6.2. Configuration pin information 
6.3. Analysis of the experimental flow chart 
6.4. core code explanation 
6.5. Hardware connection 
6.6. Experimental effect 
6.1. Purpose of the experiment  
Use the serial communication of STM32 to analyze the SBUS protocol data transmitted by the  
remote control transmitter of the model aircraft, and print the value of each channel.  
6.2. Configuration pin information  
1. Import the ioc file from the Serial project and name it SBUS.  
According to the schematic diagram, SBUS is connected to the RX pin of serial port 2, only  
receiving but not sending.  
2. Change the mode of serial port 2 to Asynchronous synchronous communication, the baud  
rate is 100000, the data width: 9 bits, the test: Even, the stop bit: 2 bits. Serial port 2 only  
uses the receive function, so Data Direction can choose Receive and Transmit or Receive  
Only.  
3. Open the serial port 2 interrupt settings.  
6.3. Analysis of the experimental flow chart  
6.4. core code explanation  
1. Add the following in bsp_uart.c:  
USART1_Init(): Initialize the serial port related content, open serial port 1 and serial port 2 to  
receive 1 data.  
2. In the serial port interrupt callback, judge whether serial port 2 data is received, and at the  
same time distinguish whether serial port 1 or serial port 2 has received the data.  
3. Create new bsp_sbus.h and bsp_sbus.c files to manage sbus data analysis content. Create  
the following in bsp_sbus.h:  
Among them, SBUS_ALL_CHANNELS controls the number of channels parsed. By default, only  
eight channels are displayed. If full channel display is required, modify it to 1.  
4. SBUS_Reveive(data) receives the data of the serial port as a buffer. If it conforms to the  
communication protocol of SBUS, it will update a frame of data to the sbus_data array.  
5. Analyze the data in sbus_data according to the SBUS communication protocol.  
6. The SBUS_Handle() function is called cyclically in Bsp_Loop(), and the parsed data of each  
channel is printed out through serial port 1.  
6.5. Hardware connection  
Because SBUS communication needs to connect the SBUS receiver to the SBUS interface on the  
expansion board, S is connected to the signal, V is connected to the positive pole of the power  
supply, and G is connected to the ground. Therefore, you need to prepare your own model  
aircraft remote control and SBUS receiver, pair them in advance and turn on the power switch.  
6.6. Experimental effect  
After programming the program, the LED light flashes every 200 milliseconds. After connecting  
the expansion board to the computer through the micro-USB data cable, open the serial port  
assistant (the specific parameters are shown in the figure below), and you can see that the model  
aircraft remote control has been printed on the serial port assistant. The data of each channel of  
the controller, when we manually toggle the joystick or button of the model aircraft remote  
controller, the data will change accordingly.  

---

## 6.gmapping mapping algorithm.pdf

6. gmapping mapping algorithm  
6. gmapping mapping algorithm
6.1. Introduction
6.2. Use
6.2.1. Start
6.2.2. Controlling the robot
6.2.3. Map saving
6.2. Topics and services
6.3. Configuration parameters
6.4, TF transformation
Gmapping: http://wiki.ros.org/gmapping/
map_server: https://wiki.ros.org/map_server
6.1. Introduction  
gmapping is only applicable to points where the number of two-dimensional laser points in a  
single frame is less than 1440. If the number of laser points in a single frame is greater than  
1440, then problems such as [[mapping-4] process has died] will occur.
Gmapping is a commonly used open source SLAM algorithm based on the filtered SLAM  
framework.
Gmapping is based on the RBpf particle filter algorithm, which separates the real-time  
positioning and mapping processes. Positioning is performed first and then mapping is  
performed.
Gmapping has made two major improvements on the RBpf algorithm: improved proposal  
distribution and selective resampling.
Advantages: Gmapping can construct indoor maps in real time. The amount of calculation  
required to construct small scene maps is small and the accuracy is high.
Disadvantages:  As the scene grows, the number of particles required increases because each  
particle carries a map, so the amount of memory and computation required when building a large  
map increases. Therefore it is not suitable for building large scene maps. And there is no loop  
detection, so the map may be misaligned when the loop is closed. Although increasing the  
number of particles can close the map, it comes at the expense of increased calculations and  
memory.
6.2. Use  
Note: When building a map, the slower the speed, the better the effect (note that the  
rotation speed should be slower). If the speed is too fast, the effect will be poor. 
According to different models, you only need to set the purchased model in [.bashrc], X1 (normal  
four-wheel drive) X3 (Mailun) Take X3 as an example
Open the [.bashrc] file
Find the [ROBOT_TYPE] parameters and modify the corresponding car model
6.2.1. Start  
Start the command (robot side). For the convenience of operation, this section takes [mono + laser  
+ yahboomcar] as an example.
<PI5 needs to open another terminal to enter the same docker container#Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
sudo vim .bashrc
export ROBOT_TYPE=X3 # ROBOT_TYPE: X1 X3 X3plus R2 X7
roslaunch yahboomcar_nav laser_bringup.launch # laser + yahboomcar
roslaunch yahboomcar_nav laser_usb_bringup.launch # mono + laser + yahboomcar
roslaunch yahboomcar_nav laser_astrapro_bringup.launch # Astra + laser + 
yahboomcar
Mapping command (robot side)
[use_rviz] parameter: whether to enable rviz visualization.
[map_type] parameter: Set the mapping algorithm [gmapping].
Turn on the visual interface (virtual machine side)
roslaunch yahboomcar_nav yahboomcar_map.launch use_rviz:=false map_type:=gmapping
roslaunch yahboomcar_nav view_map.launch
The gap at the back of the robot is due to the obstruction caused by the installation position of the  
display screen, so a certain range of radar data is blocked. The shielding range can be adjusted, or  
it can not be blocked according to the actual situation. For specific operations, see [01. Radar Basic  
Course].
6.2.2. Controlling the robot  
Keyboard controls robot movement
Control the robot movement with the handle
Make the robot cover the area to be mapped and the map should be as closed as possible.
There may be some scattered points during the mapping process. If the mapping environment is  
well closed, relatively regular, and the movement is slow, the scattering phenomenon will be much  
smaller.
6.2.3. Map saving  
The map will be saved to the ~/yahboomcar_ws/src/yahboomcar_nav/maps/ folder, a pgm image  
and a yaml file.
map.yaml
Parameter analysis:
image: The path of the map file, which can be an absolute path or a relative path.
resolution: resolution of the map, meters/pixel
Origin: 2D pose (x, y, yaw) in the lower left corner of the map. The yaw here is rotated  
counterclockwise (yaw=0 means no rotation). Many parts of the current system ignore the  
yaw value.
negate: whether to reverse the meaning of white/black and free/occupied (the interpretation  
of the threshold is not affected)
occupied_thresh: Pixels with an occupation probability greater than this threshold will be  
considered fully occupied.
free_thresh: Pixels with occupancy probability less than this threshold will be considered  
completely free.rosrun teleop_twist_keyboard teleop_twist_keyboard.py # System integration
roslaunch yahboomcar_ctrl yahboom_keyboard.launch # Custom
rosrun map_server map_saver -f ~/yahboomcar_ws/src/yahboomcar_nav/maps/my_map # 
The first way
bash ~/yahboomcar_ws/src/yahboomcar_nav/maps/map.sh # The second way
image: map.pgm
resolution: 0.05
origin: [-15.4,-12.2,0.0]
Negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
Subscription
topicType Description
tf tf/tfMessageUsed for conversion between lidar
coordinate system, base coordinate
system, and odometer coordinate system
scan sensor_msgs/LaserScan Lidar scan data
Post Topic Type Description
map_metadata nav_msgs/MapMetaData Publish map Metadata
map nav_msgs/OccupancyGrid Publish map raster data
~entropy std_msgs/Float64Publish an estimate of the entropy of the
robot pose distribution
Service Type Description
dynamic_map nav_msgs/GetMap Get map data
Parameters TypeDefault
valueDescription
~throttle_scans int 1Each time this number of frames of
laser data is received, only one
frame of data is processed. By
default, each frame of data is
processed6.2. Topics and services  
Node view
6.3. Configuration parameters  rqt_graph
Parameters TypeDefault
valueDescription
~base_frame string "base_link" Robot base coordinate system
~map_frame string "map" Map coordinate system
~odom_frame string "odom" Odometer coordinate system
~map_update_interval float 5.0Map update frequency, the lower
the value, the greater the
computational load
~maxUrange float 80.0The maximum range that the laser
can detect
~sigma float 0.05standard deviation of endpoint
matches
~kernelSize int 1 Search in the corresponding kernel
~lstep float 0.05Optimization step size during
translation
~astep float 0.05Optimization step size during
rotation
~iterations int 5Number of iterations to scan for
matches
~lsigma float 0.075Laser standard deviation of
likelihood calculation
~ogain float 3.0Used to smooth the resampling
effect during likelihood calculation
~lskip int 0 Number of beams skipped per scan
~minimumScore float 0The minimum value of scan
matching results. When using a
laser scanner with a limited range
(e.g. 5m), jumping in large open
spaces can be avoided
~srr float 0.1Translation function (rho/rho),
odometry error during translation
~srt float 0.2rotation function (rho/theta),
odometry error in translation
~str float 0.1Translation function (theta/rho),
odometry error when rotating
~stt float 0.2Rotation function (theta/theta),
odometry error when rotating
Parameters TypeDefault
valueDescription
~linearUpdate float 1.0Process the laser scanning data
every time the robot translates this
distance
~angularUpdate float 0.5The robot processes laser scanning
data every time it rotates this
distance
~temporalUpdate float -1.0Process a scan if the latest scan is
processed slower than the update.
Turn off time-based updates when
this value is negative
~resampleThreshold float 0.5 Neff-based resampling threshold
~particles int 30 Number of particles in the filter
~xmin float -100.0Initial minimum size of map in x
direction
~ymin float -100.0The initial minimum size of the map
in the y direction
~xmax float 100.0The initial maximum size of the
map in the x direction
~ymax float 100.0The initial maximum size of the
map in the y direction
~delta float 0.05 map resolution
~llsamplerange float 0.01Translation sampling distance for
likelihood calculation
~llsamplestep float 0.01translation sampling step for
likelihood calculation
~lasamplerange float 0.005Rotated sampling distance for
likelihood calculation
~lasamplestep float 0.005Rotation sampling step size for
likelihood calculation
~transform_publish_period float 0.05TF transform publishing time
interval
~occ_threh float 0.25Threshold for raster map
occupancy
~maxRange(float) float - The maximum range of the sensor
Required TF
transformationDescription
laser-->base_linkThe transformation between the laser radar coordinate system and
the base coordinate system is generally published by
robot_state_publisher or static_transform_publisher
base_link-->odomTransformation between the map coordinate system and the robot
odometer coordinate system, estimating the robot's pose in the map
Released TF
TransformDescription
map-->odomTransformation between the map coordinate system and the robot's
odometry coordinate system, estimating the robot's pose in the map6.4, TF transformation  
View tf tree
rosrun rqt_tf_tree rqt_tf_tree

---

## 7. AR QR code.pdf

7. AR QR code  
7. AR QR code 
7.1. Overview 
7.2. Create ARTag 
7.2.1. install software package 
7.2.2. Create AR QR code 
7.3, ARTag identification 
7.3.1. Start the identification instance 
7.3.2. launch file analysis 
7.3.3. ar_track_alvar node 
7.3.4. View node graph 
7.3.5. View tf tree 
7.3.6. View output information 
7.1. Overview  
wiki: http://wiki.ros.org/ar_track_alvar/ 
Source code: https://github.com/ros-perception/ar_track_alvar.git  
Feature pack location: ~/yahboomcar_ws/src/yahboomcar_visual 
ARTag (AR tag, AR means "augmented reality") is a fiducial marker system, which can be  
understood as a reference for other objects. It looks similar to a two-dimensional code, but its  
encoding system and two-dimensional code are still very large. The difference is mostly used in  
camera calibration, robot positioning, augmented reality (AR) and other applications. One of the  
most important functions is to identify the pose relationship between the object and the camera.  
An ARTag can be attached to the object, or an ARTag label can be attached to a plane to calibrate  
the camera. After the camera recognizes the ARTag, the position and pose of the tag in the camera  
coordinates can be calculated. 
ar_track_alvar has 4 main functions: 
Generate AR tags of different sizes, resolutions and data/ID encodings. 
Recognize and track the pose of a single AR tag, optionally integrating kinect depth data  
(when kinect is available) for better pose estimation. 
Recognize and track poses in "clusters" consisting of multiple labels. This allows for more  
stable pose estimation, robustness to occlusion, and tracking of polygonal objects. 
The spatial relationship between tags in a bundle is automatically calculated using camera  
images so that users do not have to manually measure and enter tag positions in an XML file  
to use the bundle feature. 
Alvar is newer and more advanced than ARToolkit, which has been the basis for several other ROS  
AR tagging packages.  Alvar features adaptive thresholding to handle various lighting conditions,  
optical flow-based tracking for more stable pose estimation, and an improved label recognition  
method that does not slow down significantly as the number of labels increases. 
7.2. Create ARTag  
7.2.1. install software package  
ar_track_alvar is an open source marker library that provides examples of pr2+kinect. The first use  
case of this package is to recognize and track the pose of (potentially) multiple AR tags, each of  
which is considered individually. 
7.2.2. Create AR QR code  
Generate multiple labels in a row on an image #Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
sudo apt install ros-melodic-ar-track-alvar 
rosrun ar_track_alvar createMarker 
You can enter [ID] and location information here, and enter [-1] to end. One or more can be  
generated, and the layout can be designed by yourself. 
generate a single number 
Command+parameters directly generate digital pictures; e.g. 
11: The number is the QR code of 11.  -s: Specifies the image size.  5: 5x5 picture.  33: The number  
is the QR code of 33. 
7.3, ARTag identification  
Note: When starting the camera, you need to load the camera calibration file, otherwise it  
will not be recognized.  
7.3.1. Start the identification instance  
The open_rviz parameter is turned on by default. rosrun ar_track_alvar createMarker 11 
rosrun ar_track_alvar createMarker -s 5 33  
roslaunch yahboomcar_visual ar_track.launch open_rviz:=true 
In rviz, you need to set the corresponding camera topic name. 
Image_Topic: The camera topic is [/usb_cam/image_raw]. 
Marker: The display component of rviz, different squares display the location of the AR QR  
code. 
TF: The display component of rviz, used to display the coordinate system of AR QR code. 
Camera: The display component of rviz, which displays the camera screen. 
world: world coordinate system. 
usb_cam: Camera coordinate system. 
7.3.2. launch file analysis  
< launch > 
    <!-- Set camDevice parameters, the default is USBCam --> 
    < arg  name = "open_rviz"  default = "true" /> 
    < arg  name = "marker_size"  default = "5.0" /> 
    < arg  name = "max_new_marker_error"  default = "0.08" /> 
    < arg  name = "max_track_error"  default = "0.2" /> 
    <!-- Set camera image topic, camera internal reference topic, camera frame --
> 
    < arg  name = "cam_image_topic"  default = "/usb_cam/image_raw" /> 
    < arg  name = "cam_info_topic"  default = "/usb_cam/camera_info" /> 
    < arg  name = "output_frame"  default = "/usb_cam" /> 
    <!-- start camera node --> 
    < include  file = "$(find usb_cam)/launch/usb_cam-test.launch" /> 
    <!-- Set the correspondence between the camera coordinate system and the 
world coordinate system--> 
topic name type of data
/camera_info ( sensor_msgs/CameraInfo )
/image_raw ( sensor_msgs/Image )
topic name type of data
/visualization_marker ( visualization_msgs/Marker )Node parameters: 
marker_size (double) : The width (in centimeters) of one side of the border of the black  
square marker. 
max_new_marker_error (double): Threshold to determine when a new marker can be  
detected under uncertainty. 
max_track_error (double) : A threshold that determines how many tracking errors can be  
observed before markers disappear. 
camera_image (string) : Provides the image topic name used to detect AR tags. This can be  
monochrome or color, but should be an uncorrected image since correction is done in this  
package. 
camera_info (string) : Subject name that provides camera calibration parameters to correct  
images. 
output_frame (string) : Publish the coordinate position of the AR label in the camera  
coordinate system. 
7.3.3. ar_track_alvar node  
Subscribed topic  
Published Topics      < node  pkg = "tf"  type = "static_transform_publisher"  name = 
"world_to_cam"  args = "0 0 0.5 0 1.57 0 world usb_cam 10" /> 
    <!-- Start AR recognition node--> 
    < node  name = "ar_track_alvar"  pkg = "ar_track_alvar"  type = 
"individualMarkersNoKinect"  respawn = "false"  output = "screen" > 
        < param  name = "marker_size"  type = "double"  value = "$(arg 
marker_size)" /> 
        < param  name = "max_new_marker_error"  type = "double"  value = "$(arg 
max_new_marker_error)" /> 
        < param  name = "max_track_error"  type = "double"  value = "$(arg 
max_track_error)" /> 
        < param  name = "output_frame"  type = "string"  value = "$(arg 
output_frame)" /> 
        < remap  from = "camera_image"  to = "$(arg cam_image_topic)" /> 
        < remap  from = "camera_info"  to = "$(arg cam_info_topic)" /> 
    </ node > 
    <!-- start rviz --> 
    < node  pkg = "rviz"  type = "rviz"  name = "rviz"  args = "-d $(find 
yahboomcar_visual)/rviz/ar_track.rviz"  if = "$(arg open_rviz)" /> 
</ launch > 
topic name type of data
/ar_pose_marker ( ar_track_alvar/AlvarMarkers )
Provided tf Transforms  
A single QR code: camera coordinate system →  AR tag coordinate system 
Multiple QR Codes: Provides a transformation from the camera coordinate system to each AR  
marker coordinate system (named ar_marker_x), where x is the marker's ID number. 
7.3.4. View node graph  
7.3.5. View tf tree  rqt_graph 
rosrun rqt_tf_tree rqt_tf_tree 
Through rviz, we can intuitively see the relative position of the QR code and the camera. The  
camera and world coordinate system are set by themselves. 
7.3.6. View output information  
The display is as follows: rostopic echo / ar_pose_marker 
    header:  
      seq: 0 
      stamp:  
        secs: 1630584915 
        nsecs: 196221070 
      frame_id: "/usb_cam" 
    id: 3 
    confidence: 0 
    pose:  
      header:  
        seq: 0 
        stamp:  
          secs: 0 
          nsecs:         0 
        frame_id: '' 
frame_id: the coordinate system name of the camera 
id: the recognized number is 3 
pose: pose of the QR code 
position: the position of the QR code coordinate system relative to the camera coordinate  
system 
orientation: the orientation of the QR code coordinate system relative to the camera  
coordinate system       pose:  
        position:  
          x: 0.0249847882514 
y: 0.0290736736336 
with: 0.218054183012 
        orientation:  
          x: 0.682039034537 
y: 0.681265739969 
with: -0.156112715404 
in: 0.215240718735 

---

## 7. CAN bus communication.pdf

7. CAN bus communication  
7. CAN bus commu nication 
7.1. Purpose of the experiment 
7.2. Configuration pin information 
7.3. Analysis of the experimental flow chart 
7.4. core code explanation 
7.5. Hardware connection 
7.6. Experimental effect 
7.1. Purpose of the experiment  
Using the CAN communication of STM32, using the loopback mode, the key control sends CAN  
data, interrupts the received CAN data and prints it out through the serial port assistant.  
7.2. Configuration pin information  
Since each new project needs configuration information, it is more troublesome. Fortunately,  
STM32CubeIDE provides the function of importing .ioc files, which can help us save time.  
1. Import the ioc file from the Serial project and name it CAN.  
Find CAN in Connectivity and tick Activated to enable the CAN peripheral.  
2. According to the schematic diagram, the pins connected to the CAN bus are PB8 and PB9,  
while the default CAN bus pins are PA11 and PA12, so it is necessary to manually modify the  
CAN bus pins to PB8 and PB9.  

3. Set the parameters of the CAN peripheral, here we set the baud rate to 1000kbps and the  
mode to Loopback.  

Since this is only used to test communication, choose Loopback mode (data is sent and received  
automatically); if you need to connect a third-party CAN device, please choose Normal mode (data  
receiving/sending independent).  
 
4. Turn on the CAN RX0 interrupt in the interrupt setting. If the interrupt is not turned on, the  
data cannot be received.  
7.3. Analysis of the experimental flow chart  

7.4. core code explanation  
1. Create new buzzer driver library bsp_can.h and bsp_can.c files in BSP. Add the following to  
bsp_can.h:  
2. Add the following in bsp_can.c:  
Can_Init(): Initialize the CAN peripheral related content, set the CAN receive filter, and open the  
CAN bus communication.  
3. In order to test the sending data, the new Can_Test_Send() function sends the data through  
CAN and prints it to the serial port assistant. If you need to modify the sent data, you can  
modify the TxData array before sending.  
4. The CAN receive interrupt callback function prints the received CAN data through the serial  
port. The name of this function cannot be modified, otherwise the function cannot be called.  
5. In BSP initialization, call the Can_Init() function to initialize the CAN peripherals.  
6. After the button is pressed, the function of sending CAN data is added.  
7.5. Hardware connection  
Since the loopback mode is used, the CAN interface may not be connected to external devices.  
7.6. Experimental effect  
After programming the program, the LED light flashes once every 200 milliseconds. After  
connecting the expansion board to the computer through the micro-USB data cable, open the  
serial port assistant (the specific parameters are shown in the figure below), and the buzzer will  
sound every time the button is pressed. 50 milliseconds, you can see that the serial port assistant  
will display the data sent by CAN and the data received by CAN.  
 

---

## 7. Control motor.pdf

7. Control motor  
7. Control motor 
7.1. Experimental goal 
7.2. Experiment preparation 
7.3. Experimental effect 
7.4. Program source code 
7.1. Experimental goal  
Control the forward and reverse rotation of the motor on the Rosmaster, and control the speed of  
the motor by controlling the PWM duty cycle of the motor.  
 
7.2. Experiment preparation  
The red squares in the picture below are the motor MOTOR 1, MOTOR 2, MOTOR 3, and MOTOR  
4. The motor interface has an anti-reverse connection function, which can be connected to the  
motor using Rosmaster's motor cable. Here you need to pay attention to connecting the motors  
according to different models. Here is an example of a Mecanum wheel car. MOTOR 1 is  
connected to the left front motor of the car, MOTOR 2 is connected to the left rear motor of the  
car, and MOTOR 3 is connected to the right front motor of the car. MOTOR 4 is connected to the  
right rear motor of the dolly.  
Rosmaster_Lib library functions required to control the Rosmaster motor:  
set_motor ( speed_1 ,  speed_2 ,  speed_3 ,  speed_4 ) 
Parameter explanation: Control the motor PWM pulse, thereby controlling the motor speed. This  
function does not use the encoder speed function.  
speed_X=[-100, 100], a positive number means forward rotation, and a negative number means  
backward rotation.  
Return value: None.  
 
7.3. Experimental effect  
Check out the course accompanying video.  
Since this function controls the rotation of the motor by modifying the PWM duty cycle, it can only  
be used to test whether the motor is working normally, and this function is rarely used in actual  
use. 
 
7.4. Program source code  
Power on the Rosmaster robot, and open the browser of the Jetson Nano or remote computer to  
enter the Jupyter lab editor.  
Reference code path: Rosmaster/Samples/7.motor.ipynb  

---

## 7. Robot calibration.pdf

7 Robot calibration  
7 Robot calibration 
7.1 imu school associate 
7.1.1 Calibration steps 
7.1.2 Use the calibrated imu data 
7.2 Linear speed calibration 
7.2.1 Preparation 
7.2.2 Start 
7.3. Angular velocity calibration 
7.3.1. Preparation 
7.3.2 Start 
Note: The parameters have been calibrated before the product leaves the factory, and  
generally do not need to be calibrated. If you feel that the control robot has a large  
deviation, you need to calibrate [imu], [linear velocity], and [angular velocity]; when  
calibrating, place the robot in advance and do not move the robot.  
It should be noted that this lesson is applicable to the ROSMASTER X3 and X3Plus for the  
Mecanum wheel models.  X1 is an ordinary four-wheel drive model, and R2 is an Ackerman  
model. Due to the difference in the principle of motion direction control, this lesson is not  
applicable.  
7.1 imu school associate  
ROSMASTER has already calibrated the imu before leaving the factory, and the user does  
not need to calibrate. The following tutorial refers to the procedure that should be started  
if the imu needs to be calibrated.  
7.1.1 Calibration steps  
Note: When calibrating, make sure the robot is still.  
1. start 
As shown in the figure below, press Enter to calibrate the data in the X+, X-, Y+, Y-, Z+, Z- directions  
in turn. After the calibration, it will be automatically saved to the specified folder. #Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
#Multiple ros commands require multiple terminals to be executed in the same 
docker container. Please refer to the tutorials in Sections 07/5 and 5.8.
roslaunch yahboomcar_bringup calibrate_imu.launch  
When shipped from the factory, the calibration data is stored in, 
7.1.2 Use the calibrated imu data  
The above command is to use the self-calibrated imu data when starting the chassis drive. 
7.2 Linear speed calibration  
7.2.1 Preparation  
1. Measure the distance of 1 meter with a meter ruler and make a mark. 
2. put the trolley at the starting point. 
3. Modify the parameters linear_scale_x  and linear_scale_y  to 1.0. 
7.2.2 Start  
1. terminal input 
2. Click the square on the right of [start_test] to start moving the distance of [test_distance]. At  
this time, observe whether the car really moves [test_distance]. If not adjust the parameter  
[odom_linear_scale_correction], put the car back to the starting point to continue the test. ~/yahboomcar_ws/src/yahboomcar_bringup/param/imu_calib.yaml 
roslaunch yahboomcar_bringup bringup_calib.launch  
roslaunch yahboomcar_bringup bringup.launch             # chassis control 
roslaunch yahboomcar_bringup calibrate_linear.launch    # linear speed 
calibration 
rosrun rqt_reconfigure rqt_reconfigure                  # Enable  dynamic 
parameter adjustment 
Note: The car defaults to calibrate the linear speed of the X-axis direction at the beginning.  
If you want to calibrate the Y-axis, click [direction] to switch the direction and then click  
[start_test] to start calibrating the linear speed of the Y-axis.  
test_distance: test distance. It should not be too large, the default is one meter. 
speed: Test line speed. The higher the speed, the greater the inertia. 
tolerance: The error in reaching the target. If the error is too small, it will shake at the target  
position, otherwise, the error of reaching the target point will be very large. 
odom_linear_scale_correction: odometer scaling. 
start_test: start the test. 
direction: the direction of the linear velocity, the default is the X axis. 
After the test, remember the value of [odom_linear_scale_correction] and modify it to the values  
of the parameters linear_scale_x and linear_scale_y in bringup.launch. 
2. calibrate_linear.py program flow chart 
7.3. Angular velocity calibration  
7.3.1. Preparation  
1. Put the trolley in a position where it is easy to rotate the angle. 
2. Modify the parameter angular_scale  to 1.0. 
7.3.2 Start  
1. terminal input 
roslaunch yahboomcar_bringup bringup.launch             # chassis control 
roslaunch yahboomcar_bringup calibrate_angular.launch   # angular velocity 
calibration 
rosrun rqt_reconfigure rqt_reconfigure                  # Enable dynamic 
parameter adjustment 
2. Click the square on the right side of [start_test] to start moving the distance of [test_angle]. At  
this time, observe whether the car really turns [test_angle]. If not adjust the parameter  
[odom_angule_scale_correction], put the car back to the starting point to continue the test. 
test_angle: test distance. It should not be too large, the default is 360°. 
speed: Test angular velocity. The higher the speed, the greater the inertia. 
tolerance: The error in reaching the target. If the error is too small, it will shake at the target  
position, otherwise, the error of reaching the target point will be very large. 
odom_angule_scale_correction: odometer scaling. 
start_test: start the test. 
After the test, remember the value of [odom_angule_scale_correction] and modify it to the value  
of the parameter [angular_scale] in bringup.launch. 
3. calibrate_angular.py program flow chart 
 

---

## 7. RTAB-Map 3D mapping navigation.pdf

7. RTAB-Map 3D mapping navigation  
7. RTAB-Map 3D mapping navigation
7.1. Introduction
7.2. Map construction and use
7.2.1. Start
7.2.2. Map construction
7.3. Navigation and obstacle avoidance
7.3.1. Single-point navigation
7.3.2. Multi-point navigation
7.3.3. Parameter configuration
7.4, node rtabma p
7.4.1. Subscribe to topics
7.4.2. Publishing topics
7.4.3, Service
7.4.4. Parameters
7.4.5, tf conversion
7.5, node rtabma pviz
7.5.1. Subscribe to topics
7.5.2. Parameter configuration
7.5.3. Required tf conversion
wiki: http://wiki.ros.org/rtabmap_ros
7.1. Introduction  
This package is a ROS function package for RTAB Map, an RGB-D SLAM method based on a global  
loop closure detector with real-time constraints. This package can be used to generate 3D point  
clouds of environments and create 2D occupancy raster maps for navigation.
As can be seen from the above figure, Monte Carlo positioning amcl is not required. RTAB Map has  
its own positioning function; if used, it will cause repeated positioning and positioning failure.  
When using RTAB Map to navigate the core framework, the initialized map is provided by RTAB  
Map, not map_server.
7.2. Map construction and use  
Note: When building a map, the slower the speed, the better the effect (note that the  
rotation speed should be slower). If the speed is too fast, the effect will be poor. 
According to different models, you only need to set the purchased model in [.bashrc], X1 (normal  
four-wheel drive) X3 (Mailun) Take X3 as an example
Open the [.bashrc] file
Find the [ROBOT_TYPE] parameters and modify the corresponding car model
7.2.1. Start  
Start the underlying driver command (robot side)
<PI5 needs to open another terminal to enter the same docker container
Command to start mapping or navigation (robot side)
use_rviz parameter: whether to open rviz.#Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
sudo vim .bashrc
export ROBOT_TYPE=X3 # ROBOT_TYPE: X1 X3 X3plus R2 X7
roslaunch yahboomcar_nav laser_astrapro_bringup.launch
roslaunch yahboomcar_nav yahboomcar_rtabmap.launch use_rviz:=False
Start visualization (virtual machine)
Keyboard control node (virtual machine)
7.2.2. Map construction  
After starting according to the above method, choose any method to control the mapping (handle  
control is recommended); the slower the speed when constructing the map, the better the effect  
(especially the angular speed); the robot will cover the area to be mapped and the map will be as  
closed as possible.
Joystick control
Keyboard control
When the map construction is completed, directly [ctrl+c] exit the map construction node, and the  
system will automatically save the map. The default saving path of the map is [~/.ros/rtabmap.db].
View tf treeroslaunch yahboomcar_nav view_rtabmap.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py # System integration
roslaunch yahboomcar_ctrl yahboom_keyboard.launch # yahboomcar customization
rosrun rqt_tf_tree rqt_tf_tree
Node view
As can be seen from the above figure, the information that the [rtabmap] node needs to subscribe  
to: radar data, camera data, and tf data.
7.3. Navigation and obstacle avoidance  
Note: [R2] on the remote control handle has the function of canceling the target point. 
Start the underlying driver command (robot side)
Command to start mapping or navigation (robot side)rqt_graph
roslaunch yahboomcar_nav laser_astrapro_bringup.launch
roslaunch yahboomcar_nav yahboomcar_rtabmap_nav.launch use_rviz:=False
use_rviz parameter: whether to open rviz.
Start visualization (virtual machine)
Keyboard control node (virtual machine)
When the navigation mode is turned on, the system automatically loads the 2D raster map. It  
cannot directly load the 3D map and needs to be loaded manually.
Load the three-dimensional map (1, 2, 3), 4 is to add the rviz debugging tool.
roslaunch yahboomcar_nav view_rtabmap_nav.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py # System integration
roslaunch yahboomcar_ctrl yahboom_keyboard.launch # yahboomcar customization
At this time, you can manually add [MarkerArray] to facilitate multi-point navigation and  
observation, and adjust [rviz] display parameters according to needs, such as the size of lidar  
points.
7.3.1. Single-point navigation  
Use the [2D Pose Estimate] of the [rviz] tool to set the initial pose until the position of the car  
in the simulation is consistent with the position of the actual car.
Click [2D Nav Goal] of the [rviz] tool, and then select a target point on the map where there  
are no obstacles. Release the mouse to start navigation. Only one target point can be  
selected, and it will stop when it is reached.
7.3.2. Multi-point navigation  
Same as the first step of single-point navigation, first set the initial pose of the car.
Click [Publish Point] of the [rviz] tool, and then select the target point on the map where there  
are no obstacles. Release the mouse to start navigation. You can click [Publish Point] again,  
and then select the point, and the robot will click on it. Cruising between points.
When using the [2D Pose Estimate] tool of the [rviz] tool to set the initial pose of the car, the  
multi-point navigation function is automatically canceled.
7.3.3. Parameter configuration  
After starting the navigation function, open the dynamic parameter adjustment tool, adjust  
according to your own needs, and observe the robot's motion status until the effect is optimal.  
Record the current parameters and modify them to the corresponding  
dwa_local_planner_params.yaml file under the yahboomcar_nav function package.
rosrun rqt_reconfigure rqt_reconfigure
Looking at the yahboomcar_navigation.launch file, we can see that the navigation parameters are  
modified in the move_base.launch file under the yahboomcar_rtabmap_nav function package.
Find the move_base.launch file and open the example file as follows. It can be modified and  
replaced according to your needs. At this time, [DWA Planner] is selected and the [DWA] file is  
loaded.<launch>
     <!-- Whether to open rviz || Whether to open rviz -->
     <arg name="use_rviz" default="false"/>
     <!-- MarkerArray node-->
     <node name='send_mark' pkg="yahboomcar_nav" type="send_mark.py"/>
     <!-- Mobile APP node -->
     <include file="$(find yahboomcar_nav)/launch/library/app.launch"/>
     <!-- Navigation core component move_base -->
     <include file="$(find yahboomcar_nav)/launch/library/move_base.launch"/>
     <!-- rtabmap navigation -->
     <include file="$(find yahboomcar_nav)/launch/library/rtabmap_nav.launch"/>
     <!-- RVIZ -->
     <include file="$(find yahboomcar_nav)/launch/view/view_rtabmap_nav.launch" 
if="$(arg use_rviz)"/>
</launch>
<launch>
     <arg name="robot_type" value="$(env ROBOT_TYPE)" doc="robot_type 
[X1,X3,X3plus,R2,X7]"/>
Note: When using the DWA planner, the difference between an omnidirectional car and a  
differential car lies in whether the speed in the Y direction is 0. There are clear comments in  
it, which can be modified according to the actual situation. 
Enter the dwa_local_planner_params.yaml file under the yahboomcar_nav function package. Some  
parameters are as follows:     <!-- Arguments -->
     <arg name="move_forward_only" default="false"/>
     <!-- move_base -->
     <node pkg="move_base" type="move_base" respawn="false" name="move_base" 
output="screen">
         <rosparam file="$(find 
yahboomcar_nav)/param/common/global_costmap_params.yaml" command="load"/>
         <rosparam file="$(find 
yahboomcar_nav)/param/common/local_costmap_params.yaml" command="load"/>
         <rosparam file="$(find 
yahboomcar_nav)/param/common/move_base_params.yaml" command="load"/>
         <rosparam file="$(find 
yahboomcar_nav)/param/common/costmap_common_params_$(arg robot_type).yaml" 
command="load"
             ns="global_costmap"/>
         <rosparam file="$(find 
yahboomcar_nav)/param/common/costmap_common_params_$(arg robot_type).yaml" 
command="load"
             ns="local_costmap"/>
         <rosparam file="$(find 
yahboomcar_nav)/param/common/dwa_local_planner_params_$(arg robot_type).yaml" 
command="load"/>
         <param name="base_local_planner" type="string" 
value="dwa_local_planner/DWAPlannerROS" if="$(eval arg('robot_type') == 'X3')"/>
         <!-- <param name="base_local_planner" type="string" 
value="teb_local_planner/TebLocalPlannerROS"/>--><param 
name="DWAPlannerROS/min_vel_x" value="0.0" if="$(arg move_forward_only)"/>
         <remap from="cmd_vel" to="cmd_vel"/>
         <remap from="odom" to="odom"/>
     </node>
</launch>
DWAPlannerROS:
   # Robot Configuration Parameters
   # Absolute value of maximum linear velocity in x direction, unit: 
meters/second
   # The maximum y velocity for the robot in m/s
   max_vel_x: 0.6
   # The absolute value of the minimum linear velocity in the x direction, a 
negative number means it can be retreated, unit: meters/second
   # The minimum x velocity for the robot in m/s, negative for backwards motion.
   min_vel_x: -0.6
   # Absolute value of the maximum linear velocity in the y direction, unit: 
meters/second. The differential robot is 0
   # The maximum y velocity for the robot in m/s
   max_vel_y: 0.3
   # The absolute value of the minimum linear velocity in the y direction, unit: 
meters/second. The differential robot is 0
Name Type Parse
odom nav_msgs/OdometryOdometry. If the parameter
subscribe_depth or
subscribe_stereo is true; and
odom_frame_id is not set, this is a
required parameter.
rgb/image sensor_msgs/Image RGB/monocular image.
rgb/camera_info sensor_msgs/CameraInfo RGB camera parameters.
depth/image sensor_msgs/Image Depth image.
scan sensor_msgs/LaserScan Single line laser.
scan_cloud sensor_msgs/PointCloud2 Laser scanning point cloud stream.
left/image_rect sensor_msgs/Image Left eye correction image.
left/camera_info sensor_msgs/CameraInfo Left eye camera parameters.
right/image_rect sensor_msgs/Image Right eye correction image.
right/camera_info sensor_msgs/CameraInfo Right eye camera parameters.
goal geometry_msgs/PoseStampedPlan a path to achieve this goal
using the current online map.
rgbd_image rtabmap_ros/RGBDImageRGB-D sync image, only if
subscribe_rgbd is true.Other parameter files can be opened, combined with comments and courseware, and modified  
according to your own needs.
7.4, node rtabmap  
This is the master node for this package. It is a wrapper around the RTAB mapping core library.  
Here, when loop closure is detected, the map is incrementally built and optimized. The node's  
online output is this map, which contains the latest data added to the map. The default location of  
the RTAB map database is [.ros/rtabmap.db], and the workspace is also set to [.ros].
7.4.1. Subscribe to topics     # The minimum y velocity for the robot in m/s
   min_vel_y: -0.3
... ...
   #The ultimate acceleration of the robot in the x direction, the unit is 
meters/sec^2
   # The x acceleration limit of the robot in meters/sec^2
   acc_lim_x: 10.0
   # The ultimate acceleration of the robot in the y direction, which is 0 for 
differential robots
   # The y acceleration limit of the robot in meters/sec^2
   acc_lim_y: 10.0
... ..
Name Type Parse
info rtabmap_ros/Info rtabmap information.
mapData rtabmap_ros/MapDatartabmap's graph and latest
node data.
mapGraph rtabmap_ros/MapGraph rtabmap’s graph
grid_map nav_msgs/OccupancyGridMap occupancy grid generated
by laser scanning.
proj_map nav_msgs/OccupancyGridDeprecated, use /grid_map
instead of
Grid/FromDepth=true
cloud_map sensor_msgs/PointCloud2A 3D point cloud generated
from a local raster.
cloud_obstacles sensor_msgs/PointCloud2Generate a 3D point cloud of
obstacles from a local mesh.
cloud_ground sensor_msgs/PointCloud2A 3D ground point cloud
generated from a local raster.
scan_map sensor_msgs/PointCloud23D point cloud generated by
2D scan or 3D scan.
labels visualization_msgs/MarkerArrayConvenient way to display
graph labels in RVIZ.
global_path nav_msgs/PathThe planning pose of the
global path. Published only
once per planned path.
local_path nav_msgs/PathPlan the future local pose
corresponding to the global
path. Published every time the
map is updated.
goal_reached std_msgs/BoolPlan status message whether
the goal was successfully
achieved.
goal_out geometry_msgs/PoseStampedPlan the current metric goal
sent from rtabmap's topology
planner. For example, you can
connect to move_base via
move_base_simple/goal.
octomap_full octomap_msgs/OctomapGet octomap. Only available if
rtabmap_ros is built with
octomap.7.4.2. Publishing topics  
Name Type Parse
octomap_binary octomap_msgs/OctomapGet octomap. Only available if
rtabmap_ros is built with
octomap.
octomap_occupied_space sensor_msgs/PointCloud2Point cloud of octomap
occupied space (obstacles and
ground). Only available if
rtabmap_ros is built with
octomap.
octomap_obstacles sensor_msgs/PointCloud2Point cloud of obstacles on
octomap. Only available if
rtabmap_ros is built with
octomap.
octomap_ground sensor_msgs/PointCloud2Point cloud of octomap. Only
available if rtabmap_ros is built
with octomap.
octomap_empty_space sensor_msgs/PointCloud2The empty point cloud of
octomap. Only available if
rtabmap_ros is built with
octomap.
octomap_grid nav_msgs/OccupancyGridProject an octomap into a 2D
occupancy grid map. Only
available if rtabmap_ros is built
with octomap.
Name Type Parse
get_map rtabmap_ros/GetMapCall this service to get a standard
2D occupancy grid.
get_map_data rtabmap_ros/GetMap Call this service to get map data.
publish_map rtabmap_ros/PublishMapCall this service to publish map
data.
list_labels rtabmap_ros/ListLabelsGet the current labels of the
graph.
update_parameters std_srvs/EmptyThe node will be updated with the
current parameters of the
rosparam server.
reset std_srvs/Empty Delete the map.
pause std_srvs/Empty Pause mapping.
resume std_srvs/Empty Resume mapping.7.4.3, Service  
Name Type Parse
trigger_new_map std_srvs/Empty Will start a new map.
backup std_srvs/EmptyBack up the database to
"database_path.back" (default
~/.ros/rtabmap.db.back).
set_mode_localization std_srvs/Empty Set pure localization mode.
set_mode_mapping std_srvs/Empty Set mapping mode.
set_label rtabmap_ros/SetLabelSet the label to the latest node or
the specified node.
set_goal rtabmap_ros/SetGoal Plan and set topology goals.
octomap_full octomap_msgs/GetOctomapGet octomap. Only available when
rtabmap_ros is built with octomap
octomap_binary octomap_msgs/GetOctomapGet octomap. Only available when
rtabmap_ros is built with octomap
name type default value parse
subscribe_depth bool true Subscribe to depth image
subscribe_scan bool false Subscribe to lidar data
subscribe_scan_cloud bool falseSubscribe to laser 3D point
cloud
subscribe_stereo bool false Subscribe to stereo images
subscribe_rgbd bool falseSubscribe to rgbd_image
topic
frame_id string base_linkThe frame to connect to the
mobile base.
map_frame_id string mapThe coordinate system
attached to the map.
odom_frame_id string ‘ ’The coordinate system
attached to the odometer.
odom_tf_linear_variance double 0.001When using
odom_frame_id, the first 3
values of the diagonal of the
6x6 covariance matrix are
set to this value.7.4.4. Parameters  
name type default value parse
odom_tf_angular_variance double 0.001When using
odom_frame_id, the last 3
values of the 6x6 covariance
matrix diagonal are set to
this value
queue_size int 10Message queue size per
synchronization topic.
publish_tf bool truePublish TF from /map to
/odom.
tf_delay double 0.05  
tf_prefix string ‘ ‘The prefix to be added to
the generated tf.
wait_for_transform bool trueThe wait for the transform
while the tf transform is still
unavailable (the maximum
wait time for the transform
is seconds).
wait_for_transform_duration double 0.1The waiting time of
wait_for_transform.
config_path string ‘ ’Path to the configuration
file containing RTAB
mapping parameters.
Parameters set in the
startup file will override
parameters in the
configuration file.
database_path string .ros/rtabmap.dbThe path of the rtabmap
database.
gen_scan bool falseGenerate a laser scan from
a depth image (using the
middle horizontal line of the
depth image). Not
generated if subscribe_scan
or subscribe_scan_cloud is
true.
gen_scan_max_depth double 4.0The maximum depth of the
generated laser scan.
name type default value parse
approx_sync bool falseSynchronize using the
approximate time of the
input message. If false, note
that the odometry input
must have exactly the same
timestamp as the input
image
rgbd_cameras int 1Number of RGB-D cameras
to use (when
subscribe_rgbd is true).
Currently, up to 4 cameras
can be synced
simultaneously.
use_action_for_goal bool falseuse actionlib sends the
metric target to move_base.
odom_sensor_sync bool falseAdjust the image and scan
pose relative to the
odometry pose for each
node added to the graph.
gen_depth bool falseGenerate a depth image
from the scanned cloud
projection to the RGB
camera, taking into account
the displacement of the
RGB camera based on
odometry and lidar frames.
gen_depth_decimation int 1Reduce the image size of
the received camera
information (create a
smaller depth image)
gen_depth_fill_holes_size int 0Fill empty pixels to this size.
Interpolates values from
adjacent depth values. 0
means disabled.
gen_depth_fill_iterations double 0.1Maximum depth error to
interpolate (m).
gen_depth_fill_holes_error int 1Number of iterations to fill
holes.
map_filter_radius double 0.0Load data for only one node
in the filter radius (using the
latest data) up to the filter
angle (map filter angle).
name type default value parse
map_filter_angle double 30.0The angle to use when
filtering nodes before
creating the map. Reference
map_filter_radius
map_cleanup bool trueIf there are no map cloud
maps, raster maps, or
project maps subscribed to,
clear the corresponding
data.
latch bool trueIf true, the last message
posted on the map topic will
be saved.
map_always_update bool trueAlways update the
occupancy raster map
map_empty_ray_tracing bool truePerform ray tracing to fill
the unknown space of
invalid 2D scan rays
(assuming invalid rays are
infinite). Only used if
map_always_update is also
true.
7.4.5, tf conversion  
what is needed:
base_link →  sensor
odom →  base_link
which provided:
map →  odom
7.5, node rtabmapviz  
This node starts the visual interface of RTAB-Map. It is a wrapper for the RTAB-MapGUI library. Its  
purpose is the same as rviz, but with specific options for RTAB-Map.
Name Type Parse
odom nav_msgs/OdometryOdometry. Required if the parameters
subscribe_depth or subscribe_stereo are
true and odom_frame_id is not set.
rgb/image sensor_msgs/ImageRGB/monocular image. If the parameter
subscribe_stereo is true, this option is
not required (left/image_rect is used
instead).
rgb/camera_info sensor_msgs/CameraInfoRGB camera metadata. If the parameter
subscribe_stereo is true, this option is
not required (left/camera_info is used
instead).
depth/image sensor_msgs/ImageRegister depth image. Required if
parameter subscribe_depth is true.
scan sensor_msgs/LaserScanLaser scan stream. Required if
parameter subscribe_scan is true.
scan_cloud sensor_msgs/PointCloud2Laser scan stream. Required if
parameter subscribe_scan_cloud is true.
left/image_rect sensor_msgs/ImageLeft eye correction image. Required if
parameter subscribe_stereo is true.
left/camera_info sensor_msgs/CameraInfoLeft eye camera parameters. Required if
parameter subscribe_stereo is true.
right/image_rect sensor_msgs/ImageRight corrected image. Required if
parameter subscribe_stereo is true.
7.5.1. Subscribe to topics  
Name Type Parse
right/camera_info sensor_msgs/CameraInfoRight eye camera parameters. Required
if parameter subscribe_stereo is true.
odom_info rtabmap_ros/OdomInfoRequired if the parameter
subscribe_odom_info is true.
info rtabmap_ros/Info Statistical information of rtabmap.
mapData rtabmap_ros/MapData rtabmap’s chart and latest node data.
rgbd_image rtabmap_ros/RGBDImageRGB-D synchronized image, only when
subscribe_rgbd is true.
name typedefault
valueparse
subscribe_depth bool false Subscribe to depth image
subscribe_scan bool false Subscribe to lidar data
subscribe_scan_cloud bool falseSubscribe to the laser scanning point
cloud.
subscribe_stereo bool false Subscribe to stereo images.
subscribe_odom_info bool false Subscribe to odom information messages.
subscribe_rgbd bool false Subscribe to rgbd_image topic.
frame_id string base_linkThe coordinate system connected to the
mobile base.
odom_frame_id string ‘ ’The coordinate system of the odometer. If
empty, rtabmapviz will subscribe to the
odom topic to get odometry. If set, gets
the odometer from tf.
tf_prefix string ‘ ‘ The prefix to be added to the generated tf.
wait_for_transform bool falseWait for a transform (up to 1 second)
when the tf transform is still not available.
queue_size int 10Message queue size per synchronization
topic.
rgbd_cameras int 1Number of RGB-D cameras to use (when
subscribe_rgbd is true). Currently, up to 4
cameras can be synced simultaneously.7.5.2. Parameter configuration  
7.5.3. Required tf conversion  
base_link →  sensor coordinate system
odom →  base_link
map →  odom

---

## 7. Web page real-time monitoring.pdf

7. Web page real-time monitoring  
7. Web page real-time monitoring 
7.1. Environment Construction 
7.2. modify the launch file 
7.3. effect demonstration 
7.1. Environment Construction  
First make sure that the USB camera link is correct, enter the following command to check that the  
USB device number exists and is video0 
If the execute permission is not enough, you need to add the execute permission #Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to the ROS/07.Docker 
tutorial
~/run_docker.sh
sudo apt-get install ros-melodic-async-web-server-cpp ros-melodic-web-video-
server ros-melodic-usb-cam 
ll  /dev 
7.2. modify the launch file  
Keep hitting the keyboard [d] to delete everything. Click [i] on the keyboard to enter the edit  
mode, and write the following content into it. 
Click the [ESC] key twice (or multiple times), click the keyboard [shift plus;], enter [wq], and click  
the [Enter] key. 
7.3. effect demonstration  
open terminal, start 
<PI5 needs to open another terminal and enter the same docker containersudo chmod 777 /dev/video* 
sudo vim /opt/ros/melodic/share/usb_cam/launch/usb_cam-test.launch 
< launch > 
    < arg  name = "open_view"  default = "false" /> 
    < node  name = "usb_cam"  pkg = "usb_cam"  type = "usb_cam_node"  output = 
"screen" > 
        < param  name = "video_device"  value = "/dev/video0" /> 
        < param  name = "image_width"  value = "640" /> 
        < param  name = "image_height"  value = "480" /> 
        < param  name = "pixel_format"  value = "yuyv" /> 
        < param  name = "camera_frame_id"  value = "usb_cam" /> 
        < param  name = "io_method"  value = "mmap" /> 
    </ node > 
    <!-- web_video_server --> 
    < node  pkg = "web_video_server"  type = "web_video_server"  name = 
"web_video_server"  output = "screen" /> 
    <!-- image_view --> 
    < group  if = "$(arg open_view)" > 
        < node  name = "image_view"  pkg = "image_view"  type = "image_view"  
respawn = "false"  output = "screen" > 
            < remap  from = "image"  to = "/usb_cam/image_raw" /> 
            < param  name = "autosize"  value = "true" /> 
        </ node > 
    </ group > 
</ launch > 
roslaunch usb_cam usb_cam-test.launch  
Start web_video_server
View in local web browser 
View other devices (must be under the same LAN, 192.168.2.93 is the IP address of the  
master) 
Note: It is recommended to use Google Chrome or mobile QQ browser, other browsers may not  
be able to open the image 
Click [image_raw] to view the camera image in real time, and click [Snapshot] to display only one  
frame of image. rosrun web_video_server web_video_server
http://localhost:8080/ 
http://192.168.2.93:8080/ 

---

## 7.hector mapping algorithm.pdf

7. Hector mapping algorithm  
7. Hector mapping algorithm
7.1. Introduction
7.2. Use
7.2.1. Start
7.2.2. Controlling the robot
7.2.3. Map saving
7.3. Topics and services
7.4. Configuration parameters
7.5, TF transformation
hector_slam: http://wiki.ros.org/hector_slam
hector_slam/Tutorials: http://wiki.ros.org/hector_slam/Tutorials/SettingUpForYourRobot
hector_mapping: http://wiki.ros.org/hector_mapping
map_server: https://wiki.ros.org/map_server
7.1. Introduction  
Features: hector_slam does not need to subscribe to odometry/odom messages. It uses the  
Gauss-Newton method and directly uses lidar to estimate odometry information. However, when  
the robot is fast, slipping will occur, causing deviations in the mapping effect and placing high  
demands on sensors. When building a map, set the car's rotation speed as low as possible.
There is no method to use the odom coordinate system, taken from Wiki.
7.2. Use  
Note: When building a map, the slower the speed, the better the effect (note that the  
rotation speed should be slower). If the speed is too fast, the effect will be poor. 
According to different models, you only need to set the purchased model in [.bashrc], X1 (normal  
four-wheel drive) X3 (Mailun) Take X3 as an example
Open the [.bashrc] file#Raspberry Pi 5 master needs to enter docker first, please perform this step
#If running the script into docker fails, please refer to ROS/07, Docker tutorial
~/run_docker.sh
sudo vim .bashrc
Find the [ROBOT_TYPE] parameters and modify the corresponding car model
7.2.1. Start  
Start the command (robot side). For the convenience of operation, this section takes [mono + laser  
+ yahboomcar] as an example.
Mapping command (robot side)
<PI5 needs to open another terminal to enter the same docker container
[use_rviz] parameter: whether to enable rviz visualization.
[map_type] parameter: Set the mapping algorithm [hector].
Turn on the visual interface (virtual machine side)export ROBOT_TYPE=X3 # ROBOT_TYPE: X1 X3 X3plus R2 X7
roslaunch yahboomcar_nav laser_bringup.launch # laser + yahboomcar
roslaunch yahboomcar_nav laser_usb_bringup.launch # mono + laser + yahboomcar
roslaunch yahboomcar_nav laser_astrapro_bringup.launch # Astra + laser + 
yahboomcar
roslaunch yahboomcar_nav yahboomcar_map.launch use_rviz:=false map_type:=hector
roslaunch yahboomcar_nav view_map.launch
The gap at the back of the robot is due to the obstruction caused by the installation position of the  
display screen, so a certain range of radar data is blocked. The shielding range can be adjusted, or  
it can not be blocked according to the actual situation. For specific operations, see [01. Radar Basic  
Course].
7.2.2. Controlling the robot  
Keyboard controls robot movement
Control the robot movement with the handle
Make the robot cover the area to be mapped and the map should be as closed as possible.
There may be some scattered points during the mapping process. If the mapping environment is  
well closed, relatively regular, and the movement is slow, the scattering phenomenon will be much  
smaller.
7.2.3. Map saving  
The map will be saved to the ~/yahboomcar_ws/src/yahboomcar_nav/maps/ folder, a pgm image  
and a yaml file.
map.yamlrosrun teleop_twist_keyboard teleop_twist_keyboard.py # System integration
roslaunch yahboomcar_ctrl yahboom_keyboard.launch # Custom
rosrun map_server map_saver -f ~/yahboomcar_ws/src/yahboomcar_nav/maps/my_map # 
The first way
bash ~/yahboomcar_ws/src/yahboomcar_nav/maps/map.sh # The second way
Topic Subscription Type Description
scan sensor_msgs/LaserScanDepth data of lidar
scan
syscommand std_msgs/StringSystem command. If
the string equals
"reset", the map and
robot pose are reset
to the initial state
Topic Post Type Description
map_metadata nav_msgs/MapMetaDataPublish map
Metadata
map nav_msgs/OccupancyGridPublish map raster
data
slam_out_pose geometry_msgs/PoseStampedCovariance-free
robot pose
estimation
poseupdategeometry_msgs
/PoseWithCovarianceStampedRobot pose
estimation with
Gaussian
uncertainty
estimationParameter analysis:
image: The path of the map file, which can be an absolute path or a relative path.
resolution: resolution of the map, meters/pixel
Origin: 2D pose (x, y, yaw) in the lower left corner of the map. The yaw here is rotated  
counterclockwise (yaw=0 means no rotation). Many parts of the current system ignore the  
yaw value.
negate: whether to reverse the meaning of white/black and free/occupied (the interpretation  
of the threshold is not affected)
occupied_thresh: Pixels with an occupation probability greater than this threshold will be  
considered fully occupied.
free_thresh: Pixels with occupancy probability less than this threshold will be considered  
completely free.
7.3. Topics and services  image: map.pgm
resolution: 0.05
origin: [-15.4,-12.2,0.0]
Negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
Topic Subscription Type Description
Service Type Description
dynamic_map nav_msgs/GetMap Get map data
reset_map std_srvs/TriggerCall this service to
reset the map, and
hector will create a
brand new map
from scratch. Note
that this will not
restart the robot's
pose, it will restart
from the last
recorded pose.
pause_mapping std_srvs/SetBoolCall this service to
stop/start
processing laser
scans.
restart_mapping_with_new_pose hector_mapping/ResetMappingCall this service to
reset the map,
robot's pose, and
resume the map (if
paused)
Node view
rqt_graph
Parameters Type Default value Description
~base_frame String "base_link"Robot base
coordinate system,
used for positioning
and laser scanning
data
transformation
~map_frame String "map"The coordinate
system of the map
~odom_frame string "odom"Odo meter
coordinate system
(essentially the
coordinate system
pointed to by map)
~map_resolution Double 0.025(m)Map resolution,
edge length of grid
cells
~map_size Int 1024 The size of the map
~map_start_x double 0.5/The position of the
origin of map [0.0,
1.0] on the x-axis
relative to the grid
map
~map_start_y double 0.5/The position of the
origin of /map [0.0,
1.0] on the y-axis
relative to the grid
map
~map_update_distance_thresh double 0.4(m)The threshold for
map update, which
is calculated from
the first update on
the map until the
straight distance
reaches this
parameter value
and is updated
again
~map_update_angle_thresh double 0.9(rad)The threshold for
map update,
starting from the
first update on the
map and updating
again after the
rotation reaches
this parameter
value7.4. Configuration parameters  
Parameters Type Default value Description
~map_pub_period double 2.0 Map release cycle
~map_multi_res_levels int 3Map multi-
resolution grid
levels
~update_factor_free double 0.4Used to update the
map of free cells,
the range is [0.0,
1.0]
~update_factor_occupied double 0.9Used to update the
map of occupied
units, the range is
[0.0, 1.0]
~laser_min_dist double 0.4(m)The minimum
distance of laser
scanning points.
Scanning points
smaller than this
value will be
ignored
~laser_max_dist double 30.0(m)The maximum
distance of laser
scanning points.
Scanning points
smaller than this
value will be
ignored
~laser_z_min_value double -1.0(m)Minimum height
relative to the lidar,
scan points below
this value will be
ignored
~laser_z_max_value double 1.0(m)The maximum
height relative to
the lidar, scan
points above this
value will be
ignored
~pub_map_odom_transform bool trueWhether to publish
the coordinate
transformation
between map and
odom
~output_timing bool falseProcess the output
timing information
of each laser scan
through ROS_INFO
~scan_subscribe_queue_size int 5Queue size for scan
subscribers
Parameters Type Default value Description
~pub_map_scanmatch_transform bool trueWhether to publish
the coordinate
transformation
between
scanmatcher and
map
~tf_map_scanmatch_transform_frame_name String "scanmatcher_frame"The coordinate
system name of
scanmatcher
Required TF
transformationDescription
laser-->base_linkUsually a fixed value, the transformation between the lidar
coordinate system and the base coordinate system, generally
published by robot_state_publisher or static_transform_publisher
Released TF
TransformDescription
map-->odomThe current estimate of the robot pose within the map frame (only
provided if parameter "pub_map_odom_transform" is true).7.5, TF transformation  
View tf tree
rosrun rqt_tf_tree rqt_tf_tree

---

## 8. Expansion tutorial.pdf

8. Expansion tutorial  
8. Expansion tutorial 
8.1. Problems 
8.2. Solutions 
8.1. Problems  
After using a TF larger than the image memory to burn the image, a part of the free memory  
cannot be used, resulting in an error of insufficient space, or failure to run a large project.  
Note: This tutorial is only for users who burn the image by themselves. If there is a factory  
image in the sd, you can skip this tutorial.  The expansion method of U disk and SD card is  
the same. This section takes SD card as an example.  
8.2. Solutions  
Install the expansion software and use the expansion software to expand the capacity.  
Open the software  
Right click 【 /dev/mmcblk0p1 】 ->Resize/Move  sudo apt install gparted 
Pull the box on the right to the top until the gray area becomes completely white->Resize  
Click √  -> Apply at the bottom of the function bar  
Expansion complete!  
Use the command to perform query verification in the terminal  
Verify that the expansion is successful, and the expansion information of the 32G card is as  
follows  df -h 

---

## 8. RGB colorful light bar .pdf

8. RGB colorful light bar  
8. RGB colorful light bar 
8.1. Purpose of the experiment 
8.2. Configuration pin information 
8.3. Analysis of the experimental flow chart 
8.4. core code explanation 
8.5. Hardware connection 
8.6. Experimental effect 
8.1. Purpose of the experiment  
Use the SPI communication of STM32 to simulate the communication protocol of the WS2812B  
module, and drive the RGB light bar to display the effect.  
8.2. Configuration pin information  
1. Import the ioc file from the Beep project and name it RGB_Strip.  
According to the schematic diagram, the pin connected to the RGB light bar is PB5.  The driving  
method of the RGB light bar can use the timer PWM output or SPI output. The PB5 pin supports  
redefinition as the timer PWM or SPI output. Considering the subsequent timer conflict, the SPI  
communication method is used to drive the RGB light bar.  
2. Set the mode of SPI3 to Transmit Only Master, so that the PB5 pin will be automatically set to  
SPI3_MOSI.  
3. Modify the parameters of data sent by SPI3, refer to the following pictures for specific  
parameters.  
4. Add DMA settings for SPI3_TX.  
 
8.3. Analysis of the experimental flow chart  

8.4. core code explanation  
1. Create new bsp_rgb.h and bsp_rgb.c, and add the following content to bsp_rgb.h:  
Among them, the ws2812_t structure is used to store the cache data of the light bar.  
2. Create the following content in the bsp_rgb.c file: According to the communication protocol  
of WS2812, use the three bits of SPI to simulate one bit of WS2812.  When the value of the  
SPI output three bits is 0b110, it corresponds to the high level of WS2812, and when the SPI  
output three bits are 0b100, it corresponds to the low level of WS2812.  
3. The WS2812_Set_Color_One() function sets the color value of a single RGB lamp, index=[0,  
MAX_RGB-1], RGB=[0x00000000, 0x00FFFFFF], the arrangement of RGB values: red is at the  
front, green in the middle, and blue at the end.  
4. RGB_Set_Color(index, r, g, b) and RGB_Set_Color_U32(index, color) set the color value of the  
RGB light bar, index=[0, MAX_RGB-1] controls the color of the corresponding light beads, and  
index=255 controls the color of all light beads.  
5. RGB_Clear() clears the RGB light cache.  
6. The RGB_Update() function refreshes the color of the RGB light bar.  After changing the color  
of the RGB lamp, you must call the RGB_Update() function to refresh the display, otherwise  
the product will not work.  
7. Send data using SPI3 DMA.  
8.5. Hardware connection  
Since the RGB light bar needs to be connected to the position of the RGB light bar on the  
expansion board, the interface has been set to prevent reverse connection, and you can find the  
interface and insert it in the correct direction.  
8.6. Experimental effect  
After the program is programmed, the LED light flashes once every 200 milliseconds, and the RGB  
light bar will display red, green, and blue in sequence, switching a color every 500 milliseconds.  

---

## 9.1 Jetson Nano write sytem.pdf

9.1 Jetson Nano write sytem  
9.1 Jetson Nano write sytem
1. Prepare to install 
2. Burn the U disk system 
3. Burn EMMC boot 
Note: 1. Jetson Nano has multiple versions, JETSON NANO B01 official version and JETSON NANO  
B01 SUB version, both versions can be used with the robot, this product is factory standard as  
JETSON NANO B01 SUB version, before leaving the factory The robot image system has been  
configured. Generally, users do not need to follow this tutorial to flash the image system.  
2.Different versions have different flashing methods. Among them, the JETSON NANO B01 official  
version uses the TF card to flash the image method, and the JETSON NANO B01 SUB version  
needs to burn the U disk image + flash the EMMC boot method.  
3.The method of burning the U disk system is the same as the process of burning the TF card  
system. The following content takes burning the U disk image as an example.
1. Prepare to install  
1.Prepare a win10 computer and a USB flash drive (32G or larger is recommended). This step of  
burning the USB flash drive does not require the participation of Jetson Nano.  
2.Download the mirror  
Find the factory image in the data, and download the corresponding image compressed package  
file according to different motherboards.  
System default username: jetson, password: yahboom  
3.Format SD card  
Use SDFormatter to format the U disk, and be careful not to select the wrong Drive here,  
otherwise it will cause unnecessary trouble. If the U disk has already programmed the system,  
there may be an error in the first format, just execute it again.  
2. Burn the U disk system  
1.Unzip the downloaded system compressed file to get the img image file  
2.Insert the U disk into the computer USB port  
3.Unzip and run the Win32DiskImager tool  
4.Select the img (image) file in the software, select the drive letter of the U disk under "Device",  
then select "Write"  and then start burning the system. According to the speed of your U disk, the  
burning process is fast or slow. .  

5.A completion dialog box will pop up after the burning is completed, indicating that the  
installation is complete. If it is unsuccessful, please close the software such as the firewall, and  
reinsert the U disk for burning. Please note that after the installation, the U disk is divided into  
multiple partitions under the Windows system and cannot be clicked to enter. This is a normal  
phenomenon, because the disk partition under Linux cannot be seen under win!  
So far, the Jetson Nano has been programmed successfully. After the burning is successful, the  
system may prompt to format the partition because the partition cannot be recognized. Do not  
format at this time! Do not format! Do not format! Click Cancel, then eject the USB drive.  
3. Burn EMMC boot  
This step is not required for the JETSON NANO B01 official version, this step is dedicated to the  
JETSON NANO B01 SUB version. Under normal circumstances, the JETSON NANO B01 SUB  
motherboard of the trolley has already burned the EMMC boot file. Unless the EMMC boot is  
cleared by the burning operation, there is no need to re-burn the EMMC boot.  
1. Prepare Jetson nano motherboard, jumper caps, display screen, mouse and keyboard, etc.  
2. Let the jetson Nano enter the system REC flashing mode.  
Connect the jumper caps to the FC REC and GND pins, that is, to the second and third pins of the  
carrier board below the core board, as shown in the image below:  
 
3. Connect the line, connect the HDMI display, mouse and keyboard to the Jetson Nano, then  
plug in the power supply, and finally insert the microUSB data cable. Since the jumper cap  
has been connected to the FC REC and GND pins in the previous step, it will automatically  
enter the REC flashing mode after power on.  
 
Under normal circumstances, the following window will pop up after inserting the microUSB data  
cable. Note here that you need to set the device to connect to the virtual machine to use the  
virtual machine.  
 
4. Please transfer the Jetson_Boot_USB.tar.gz file in the data to the Ubuntu 18.04 system, and  
open the terminal to run the decompression command.  
tar xzvf Jetson_Boot_USB.tar.gz 
5. After unzipping, go to the Jetson_Boot_USB folder, then  
 
6. Run the command to burn the EMMC boot file.  
 
7. Finally, wait for the file to be burned into the EMMC, and it will prompt "The target t210ref  
has been flashed successfully.  
Reset the board to boot from internal eMMC.”  
 cd Jetson_Boot_USB/ 
ls 
sudo ./flash.sh -r jetson-nano-devkit-emmc mmcblk0p1 
If an error message appears, please confirm whether the Jetson Nano is connected normally,  
enter the flashing mode, and reconnect to burn.  
After the burning is completed, please unplug the jumper cap of the Jetson Nnao, then insert the  
U disk, and power on again.  
Note: If you are using the virtual machine provided in the Jetson data, and the Jetson_Boot_USB  
file is already included, you do not need to transfer it to the system again.  
Virtual machine username: yahboom  
Password: yahboom  

---

## 9.4 Raspberry Pi write system.pdf

9.4 Raspberry Pi write system  
9.4 Raspberry Pi write system
1. Prepare to install 
2. Burn the system image 
1. Prepare to install  
Note: The TF supporting the ROSMASTER robot has an image already burned in the factory. You  
can directly insert the TF card into the robot for use. Generally, there is no need to burn the image  
system according to this tutorial!  !  !  
1. Prepare a win10 system computer, card reader and TF card (recommended 32G or larger)  
2. Download the mirror  
Find the factory image in the data, and download the corresponding image compressed package  
file according to different motherboards.  
System default username: pi, password: yahboom  
3. Format SD card  
Use SDFormatter to format the TF card. Be careful not to select the wrong Drive here, otherwise it  
will cause unnecessary trouble. If the TF card has already been programmed with the system,  
there may be an error in the first format, just execute it again.  
 
2. Burn the system image  
1. Unzip the downloaded system compressed file to get the img image file  
2. Insert the TF card into the card reader, and then insert the card reader into the computer  
USB port  
3. Unzip and run the Win32DiskImager tool  
4. Select the img (image) file in the software, select the drive letter under "Device", then select  
"Write"  and then start burning the system. According to the speed of your TF card and card  
reader, the burning process is fast slow.  
 
5. A completion dialog box will pop up after the burning is completed, indicating that the  
installation is complete. If it is unsuccessful, please close the software such as the firewall  
and start the burning again. Please note that after the installation, the TF card is divided into  
multiple partitions under the windows system and cannot be clicked to enter. This is a  
normal phenomenon, because the disk partition under linux cannot be seen under win!  
So far, the Raspberry Pi system has been successfully programmed. After the burning is  
successful, the system may prompt to format the partition because the partition cannot be  
recognized. Do not format at this time! Do not format! Do not format! Click Cancel, then eject the  
TF card.  

---



## 6. Commands Cheatsheet
- `roslaunch <pkg> <file.launch>`: Start launch file
- `rostopic echo /topic`: View messages
- `rostopic list`: List ROS topics
- `rosrun <pkg> <node>`: Run node
- Mapping start: `roslaunch cartographer_ros demo_backpack_2d.launch`

## 7. Glossary
- **ROS**: Robot Operating System
- **LiDAR**: Laser distance sensor
- **URDF**: Robot model file
- **TEB**: Timed Elastic Band path planner
- **SLAM**: Mapping + localization
