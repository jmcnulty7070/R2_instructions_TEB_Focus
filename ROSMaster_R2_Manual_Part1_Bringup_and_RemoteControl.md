
# Part 1 — Bringup & Remote Control (Rosmaster R2 User Manual)

## Introduction
This section explains how to start the robot (“bringup”) and how to control it manually using the keyboard, a wireless controller, or the mobile app.

---

# 1. Starting the Robot (Bringup)

The bringup process loads:
- Motor drivers  
- LiDAR  
- Camera system  
- TF transforms  
- Base controller  

Start bringup:

```
roslaunch yahboomcar_bringup bringup.launch
```

This command initializes the robot so that all functions (movement, mapping, navigation) work.

---

# 2. Remote Control Options

## 2.1 Keyboard Control
After bringup:

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

Key commands:
- W – forward  
- S – backward  
- A – turn left  
- D – turn right  
- X – stop  

---

## 2.2 Wireless Game Controller
Plug in the USB dongle.

Start joystick:

```
rosrun joy joy_node
```

Convert joystick -> robot commands:

```
rosrun teleop_twist_joy teleop_twist_joy.py
```

---

## 2.3 Mobile App Control
1. Join the robot’s WiFi (`ROSMASTER-XXXX`).  
2. Open the MakerControl app.  
3. Enter the robot’s IP address.  
4. Use the virtual joysticks to drive.

---

# 3. System Health Checks

Useful commands:
```
rostopic list
rosnode list
roswtf
```

End of Part 1.
