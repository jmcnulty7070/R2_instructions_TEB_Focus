
# README – Rosmaster R2 Racing (6th-Grade Level)
**How to: Build a Map → Add Waypoints → Race the Track → Use Handle Control**

This guide shows you, in simple steps, how to:

1. Start the robot
2. Build a map with Cartographer
3. Save the map
4. Load the map for navigation
5. Create racing waypoints (checkpoints)
6. Make the robot drive laps
7. Take over at any time with the wireless handle

Everything here is written at a **6th-grade reading level**.

---

# 1. Start the Robot

Open a terminal and run:

```
source /opt/ros/noetic/setup.bash
source ~/yahboomcar_ws/devel/setup.bash
roslaunch yahboomcar_bringup bringup.launch
```

This turns on the robot’s motors and sensors.

---

# 2. Start the LiDAR

In a second terminal:

```
source /opt/ros/noetic/setup.bash
source ~/yahboomcar_ws/devel/setup.bash
roslaunch rplidar_ros rplidar.launch
```

The LiDAR gives the robot a “scan” of the room.

---

# 3. Build a Map (Cartographer)

In a third terminal:

```
source /opt/ros/noetic/setup.bash
source ~/yahboomcar_ws/devel/setup.bash
roslaunch yahboomcar_nav yahboomcar_map.launch use_rviz:=false map_type=cartographer
```

Now **drive the robot slowly** around the room to draw the map.

You can use:
- Keyboard
- Mobile App
- Wireless Handle

Go slow and try to move in smooth loops.

---

# 4. Save the Map

In a fourth terminal:

```
source /opt/ros/noetic/setup.bash
source ~/yahboomcar_ws/devel/setup.bash
mkdir -p ~/maps
rosrun map_server map_saver -f ~/maps/r2_cartographer_map
```

This makes two files:
- `r2_cartographer_map.yaml`
- `r2_cartographer_map.pgm`

These are your saved map.

---

# 5. Restart for Navigation

Stop all previous terminals (press **CTRL+C**), then restart:

### Bringup:
```
roslaunch yahboomcar_bringup bringup.launch
```

### LiDAR:
```
roslaunch rplidar_ros rplidar.launch
```

### Navigation (loads your saved map):
```
roslaunch yahboomcar_nav yahboomcar_navigation.launch use_rviz:=false map:=r2_cartographer_map
```

### RViz to view the map:
```
roslaunch yahboomcar_nav view_navigate.launch
```

---

# 6. Set the Robot’s Starting Spot (AMCL)

In RViz:

1. Click **“2D Pose Estimate.”**
2. Click on the map where your robot really is.
3. Drag to show the direction it is facing.

The robot now knows its location.

---

# 7. Send a Test Goal

In RViz:

1. Click **“2D Nav Goal.”**
2. Click anywhere on the map.

The robot should drive there using the TEB planner.

If that worked — you’re ready for racing!

---

# 8. Create Waypoints for Racing (6th-Grade Level)

Waypoints are like **checkpoints** on a race track.

The robot will drive to each checkpoint in order, then repeat.

---

## 8.1 Open the Waypoints File

Open:

```
~/yahboomcar_ws/src/yahboomcar_nav/config/race_waypoints.yaml
```

You will see something like:

```yaml
waypoints:
  - name: wp1
    frame_id: "map"
    pose:
      position: {x: 0.0, y: 0.0}
      orientation: {z: 0.0, w: 1.0}
```

You will fill this file with real checkpoint numbers.

---

## 8.2 Get Real Numbers from RViz

1. In RViz, click **“2D Nav Goal.”**
2. Click on the map where you want checkpoint #1.
3. Now open a terminal and type:

```
rostopic echo /move_base_simple/goal
```

Every time you click, you will see something like:

```
position:
  x: 2.15
  y: -1.00
orientation:
  z: 0.12
  w: 0.94
```

These numbers show where the checkpoint is.

---

## 8.3 Put the Numbers Into the File

Go back to the file and replace the numbers:

```yaml
waypoints:
  - name: wp1
    frame_id: "map"
    pose:
      position: {x: 2.15, y: -1.00}
      orientation: {z: 0.12, w: 0.94}
```

Make more checkpoints:

```yaml
  - name: wp2
    frame_id: "map"
    pose:
      position: {x: 3.0, y: 0.5}
      orientation: {z: 0.0, w: 1.0}
```

Make a loop around your room.

---

# 9. Start the Racing Loop

Now run:

```
roslaunch yahboomcar_nav race_loop.launch
```

The robot will:

1. Read your waypoint list
2. Drive to waypoint 1
3. Then waypoint 2
4. Then waypoint 3
5. Then go back to waypoint 1
6. Keep doing laps forever

The robot uses **TEB** to drive smoothly like a mini racecar.

---

# 10. Use the Wireless Handle Anytime

Start handle control:

```
rosrun joy joy_node
rosrun teleop_twist_joy teleop_twist_joy.py
```

You can now take control anytime.
This is your **safety override**.

If something goes wrong — just steer or stop.

---

# 11. Quick Tips

- Start slow
- Use open spaces
- Keep people away from the robot
- Increase speed later in the TEB config
- Always have a finger on the emergency stop switch

---

# 12. You Are Done!

You now know how to:

- Make a map
- Save a map
- Load a map
- Create waypoints
- Drive laps using TEB
- Take control with the wireless handle

Your Rosmaster R2 now works like a small **F1TENTH racing robot**.
