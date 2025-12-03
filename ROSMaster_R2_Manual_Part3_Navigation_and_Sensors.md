
# Part 3 — Navigation & Sensors

This section explains how the robot localizes itself and drives to goals, and how its sensors work.

---

# 1. Load Your Saved Map

```
roslaunch yahboomcar_nav yahboomcar_navigation.launch use_rviz:=false map:=my_map
```

Replace `my_map` with your map name.

---

# 2. Start RViz Viewer

```
roslaunch yahboomcar_nav view_navigate.launch
```

In RViz:
- Click **2D Nav Goal**
- Choose your destination on the map

The robot will compute a path and follow it.

---

# 3. Waypoint Navigation (Multi-Goal Paths)

You can send multiple waypoints for the robot to follow.

Use RViz "Publish Point" or waypoint plugins.

The robot will:
1. Drive to waypoint 1  
2. Continue to waypoint 2  
3. Repeat until done  

---

# 4. Sensor Overview

## 4.1 LiDAR

Start:

```
roslaunch rplidar_ros rplidar.launch
```

Topic:
```
/scan
```

Used for:
- Mapping
- Navigation
- Obstacle detection

---

## 4.2 Depth Camera (Astra / D435)

Start:

```
roslaunch astra_camera astra.launch
```

Main topics:
- `/camera/color/image_raw`
- `/camera/depth/image_raw`

Used for:
- Object detection  
- Visual navigation  
- ML autopilot  

---

# 5. Localization (AMCL Summary)

AMCL uses:
- The map  
- LiDAR data  
- Odometry  

It maintains a “cloud” of particles to estimate robot position.

Important topics:
- `/amcl_pose`
- `/particlecloud`
- `/map`
- `/tf`

End of Part 3.
