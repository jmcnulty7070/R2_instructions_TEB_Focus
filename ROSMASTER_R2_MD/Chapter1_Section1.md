# Chapter 1 — Section 1  
## Multi-Machine Handle Control (8th Grade Level)

### 1. Overview
This section explains how to connect multiple robots to one master computer, sync their time, and control them using a single joystick (handle).

---

## 1.1 Multi-Machine Configuration
Robots must:
- Be on the same network (Wi-Fi or wired).
- Use the same ROS master.
- Have synchronized time.

Master = the main computer running `roscore` (usually your virtual machine).  
Slaves = each robot that listens to the master.

---

## 1.1.1 Communication Settings

### Step 1 — Check master IP
Run on **virtual machine**:

```
ifconfig
```

Example IP:
```
192.168.2.106
```

---

### Step 2 — Set ROS master for each robot
Edit `.bashrc` on every robot:

```
sudo vim ~/.bashrc
```

Add:

```
export ROS_MASTER_URI=http://192.168.2.106:11311
```

Reload:

```
source ~/.bashrc
```

---

## 1.1.2 Time Synchronization (NTP)

### Step 1 — Install NTP everywhere

```
sudo apt install ntp ntpdate
```

Check time:

```
date -R
```

Set manually if needed:

```
sudo date -s "2020-01-01 01:01:01"
```

---

### Step 2 — Configure master NTP

```
sudo vim /etc/ntp.conf
```

Add:

```
ntpd pool ntp.ubuntu.com
```

Restart:

```
sudo /etc/init.d/ntp restart
```

---

### Step 3 — Configure each robot’s NTP client

```
sudo vim /etc/ntp.conf
```

Add:

```
server 192.168.2.106
```

Force sync:

```
sudo /etc/init.d/ntp stop
sudo ntpdate 192.168.2.106
sudo /etc/init.d/ntp restart
```

---

## 1.2 Multi-Machine Handle (Joystick) Control

After:
- Setting ROS master,
- Syncing time,
- Connecting the robots,

You can control one or many robots with one joystick.

---

## 1.2.1 Start robot bringup

Example for **robot1**:

```
roslaunch yahboomcar_multi laser_bringup_multi.launch ns:=robot1
```

For USB camera:

```
roslaunch yahboomcar_multi laser_usb_bringup_multi.launch ns:=robot1
```

For Astra Pro camera:

```
roslaunch yahboomcar_multi laser_astrapro_bringup_multi.launch ns:=robot1
```

Do the same for robot2 but change:

```
ns:=robot2
```

---

## 1.2.2 Turn on joystick control

### Control **multiple robots** with one joystick:

```
roslaunch yahboomcar_multi joy_multi.launch
```

### Control **one robot**:

```
roslaunch yahboomcar_multi joy_each.launch ns:=robot1
```

---

## 1.3 How Handle Control Works

### Controlling multiple robots
- One `joy_node` reads joystick data.
- Each robot has its own `yahboom_joy` node.
- Moving joystick forward moves **all robots**.

### Controlling one robot
Only one `yahboom_joy` node is active.

---

## ✔ Summary
- All robots connect to same ROS master.
- NTP sync ensures timestamps match.
- Bring up each robot under its own namespace.
- Use:
  - `joy_multi.launch` → one joystick → many robots  
  - `joy_each.launch` → joystick → one robot

