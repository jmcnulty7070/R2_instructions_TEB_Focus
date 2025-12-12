# ROSMASTER R2 (ROS1) — Run Everything on the Jetson Xavier NX (No VM)

This guide rewrites the Yahboom “Robot terminal + VM terminal” tutorials into **one machine**: your **Jetson Xavier NX** runs:

- Chassis bringup + teleop
- LiDAR / depth camera drivers
- Mapping + navigation
- **RViz and all visualization locally** (no remote VM)

> Assumptions: ROS1 image is already installed on the NX, and you have a GUI session (HDMI monitor or VNC) so RViz can open.

---

## Table of Contents

1. One-time setup (single-machine ROS networking)
2. Quick start: “does my NX support RViz?”
3. Core bringup (chassis + handle/joystick)
4. LiDAR mapping (gmapping)
5. LiDAR navigation (map + goals in RViz)
6. Astra Pro Plus / depth camera mapping + navigation (if you use it)
7. Multi-point goals (recording goals from RViz)
8. Voice multi-point navigation (optional)
9. ORB-SLAM2 + Octomap (optional)
10. Common problems (RViz, TF, map not showing)
11. Switching back to VM visualization (optional)

---

## 1) One-time setup (single-machine ROS networking)

On the NX, you want ROS to point to **itself** (not a VM IP).

Edit your `~/.bashrc`:

```bash
gedit ~/.bashrc
```

Add (or replace older ROS networking lines with):

```bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
# (If your setup uses ROS_IP instead of ROS_HOSTNAME, use this instead)
# export ROS_IP=localhost
```

Reload:

```bash
source ~/.bashrc
```

Verify:

```bash
echo $ROS_MASTER_URI
echo $ROS_HOSTNAME
echo $ROS_IP
```

---

## 2) Quick start: confirm RViz can open on the NX

On the NX desktop:

```bash
echo $DISPLAY
rviz
```

- If RViz opens → good.
- If RViz fails with “cannot open display” → you’re in SSH-only / headless mode. Use HDMI monitor or VNC.

---

## 3) Core bringup (chassis + handle/joystick)

Start the robot chassis driver:

```bash
roslaunch yahboomcar_bringup bringup.launch
```

Quick checks:

```bash
rostopic list
rostopic echo /cmd_vel
rostopic echo /odom
rostopic echo /tf
```

---

## 4) LiDAR mapping (2D mapping with gmapping)

### 4.1 Start LiDAR + base

Terminal 1:

```bash
roslaunch yahboomcar_nav laser_bringup.launch
```

### 4.2 Start mapping

Terminal 2:

```bash
roslaunch yahboomcar_nav yahboomcar_map.launch use_rviz:=false map_type:=gmapping
```

### 4.3 Open RViz mapping view (this is what Yahboom calls “VM side”)

Terminal 3:

```bash
roslaunch yahboomcar_nav view_lidar_mapping.launch
```

### 4.4 Drive slowly to build the map

Use your handle/joystick teleop, or:

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### 4.5 Save the map

Pick a name like `my_map`:

```bash
mkdir -p ~/maps
rosrun map_server map_saver -f ~/maps/my_map
```

You should get:

- `~/maps/my_map.pgm`
- `~/maps/my_map.yaml`

---

## 5) LiDAR navigation (drive on your saved map)

Terminal 1:

```bash
roslaunch yahboomcar_nav laser_bringup.launch
```

Terminal 2 (use your map name):

```bash
roslaunch yahboomcar_nav yahboomcar_navigation.launch use_rviz:=false map:=my_map
```

Terminal 3 (RViz view):

```bash
roslaunch yahboomcar_nav view_navigate.launch
```

In RViz:

1. Click **2D Pose Estimate** and click/drag on the map to align the robot.
2. Click **2D Nav Goal** and click on the map to send a goal.

---

## 6) Astra Pro Plus (depth camera) mapping + navigation (optional)

If you’re using an Orbbec Astra Pro Plus, Yahboom tutorials often provide a bringup launch for it.

### 6.1 Depth camera mapping

Terminal 1:

```bash
roslaunch yahboomcar_nav astrapro_bringup.launch
```

Terminal 2:

```bash
roslaunch yahboomcar_nav yahboomcar_map.launch use_rviz:=false map_type:=gmapping
```

Terminal 3 (RViz view for vision mapping):

```bash
roslaunch yahboomcar_nav view_vision_mapping.launch
```

Save the map the same way:

```bash
mkdir -p ~/maps
rosrun map_server map_saver -f ~/maps/visual_map
```

### 6.2 Depth camera navigation

Terminal 1:

```bash
roslaunch yahboomcar_nav astrapro_bringup.launch
```

Terminal 2:

```bash
roslaunch yahboomcar_nav yahboomcar_navigation.launch use_rviz:=false map:=visual_map
```

Terminal 3:

```bash
roslaunch yahboomcar_nav view_navigate.launch
```

---

## 7) Multi-point goals (recording goal poses from RViz)

Run navigation as in Section 5, then in another terminal:

```bash
rostopic echo /move_base_simple/goal
```

Each time you click **2D Nav Goal** in RViz, you’ll see a pose printed in the terminal.
Copy those values into your waypoint / goal script (Yahboom often uses a Python script for this).

---

## 8) Voice multi-point navigation (optional)

Typical flow:

Terminal 1:

```bash
roslaunch yahboomcar_nav laser_bringup.launch
```

Terminal 2:

```bash
roslaunch yahboomcar_nav yahboomcar_navigation.launch use_rviz:=false map:=my_map
```

Terminal 3:

```bash
roslaunch yahboomcar_nav view_navigate.launch
```

Terminal 4 (example script path used in Yahboom packages):

```bash
python ~/yahboomcar_ws/src/yahboomcar_voice_ctrl/scripts/voice_Ctrl_send_mark.py
```

---

## 9) ORB-SLAM2 + Octomap (optional 3D mapping)

If you use Yahboom’s ORB-SLAM2 + Octomap launch, you can start RViz locally on NX by setting `use_rviz:=true`.

```bash
roslaunch yahboomcar_slam robot_orb_octomap.launch frame_id:=odom use_rviz:=true
```

If their lesson uses a separate “view” launch file, run it on the NX too (in another terminal).

---

## 10) Common problems (the stuff that usually causes “it only works on the VM”)

### RViz opens but shows nothing
- Check that topics exist:

```bash
rostopic list
```

- Check TF is alive:

```bash
rosrun tf view_frames
evince frames.pdf
```

- Check common topics:

```bash
rostopic echo /tf
rostopic echo /scan
rostopic echo /map
```

### “No transform from map to base_link”
Usually means you’re missing a localization/mapping node, or your base driver isn’t publishing odometry.

- Make sure bringup is running (Section 3)
- Make sure mapping or navigation launch is running (Sections 4–5)

### Map is rotated / robot drives “wrong direction”
- In RViz, redo **2D Pose Estimate** carefully.
- Confirm your laser is mounted forward and your `base_link` axes match the physical robot.

### Jetson is slow / RViz is laggy
- Reduce RViz displays (turn off point clouds you don’t need).
- Use a simpler view launch (LiDAR-only if possible).

---

## 11) Switching back to VM visualization (optional)

If you decide later to run RViz on a PC/VM again:

- NX runs `roscore` and all robot nodes.
- PC/VM connects to the NX ROS master.

Example IPs:

- NX: `192.168.68.51`
- VM: `192.168.68.100`

### NX `~/.bashrc`

```bash
export ROS_MASTER_URI=http://192.168.68.51:11311
export ROS_HOSTNAME=192.168.68.51
```

### VM `~/.bashrc`

```bash
export ROS_MASTER_URI=http://192.168.68.51:11311
export ROS_HOSTNAME=192.168.68.100
```

Test:

```bash
# On NX
roscore

# On VM
rviz
```

---

## Done

**Rule to remember:** any command Yahboom labels “virtual machine terminal” is just **a new terminal on your NX** as long as RViz can open on the NX desktop.
