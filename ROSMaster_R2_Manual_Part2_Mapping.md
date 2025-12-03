
# Part 2 — Mapping (Cartographer SLAM)

Mapping lets your robot create a 2D map of the environment using a LiDAR.

---

# 1. Start Required Sensors

Start bringup:

```
roslaunch yahboomcar_bringup bringup.launch
```

Start LiDAR:

```
roslaunch rplidar_ros rplidar.launch
```

---

# 2. Start Cartographer SLAM

```
roslaunch yahboomcar_nav yahboomcar_map.launch use_rviz:=false map_type:=cartographer
```

The robot builds a map using its LiDAR as it moves.

---

# 3. View Mapping in RViz
Open the visualization tool:

```
rviz
```

Add:
- Map  
- LaserScan  
- TF  

Walk or drive the robot slowly around the room. Slow rotation improves accuracy.

---

# 4. Save Your Map

```
rosrun map_server map_saver -f my_map
```

This creates:
- `my_map.yaml`  
- `my_map.pgm`  

Store this map so you can use it for navigation later.

---

# 5. Tips for Better Mapping
- Move *slowly*  
- Avoid shiny walls  
- Keep LiDAR level  
- Create loop closures (walk the perimeter twice)

End of Part 2.
