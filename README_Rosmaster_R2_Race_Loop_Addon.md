
# Rosmaster R2 – Racing Loop Add-on (Cartographer + TEB + Waypoints + Handle Safety)

This add-on is designed to sit **on top of** your existing Yahboom `yahboomcar_nav` package
and give you:

- A **racing loop** using waypoints
- Continuous laps using **TEB** as the local planner
- A clear workflow:
  1. Make a map with **Cartographer**
  2. Save the map
  3. Load the map with navigation (TEB)
  4. Run **race_loop.launch** to drive repeated laps
  5. Use your **USB wireless handle** as a safety override

> Everything here assumes ROS1 on a Jetson (Nano / Xavier NX / Orin) with the Yahboom Rosmaster R2 stack.

---

## 1. Where to Copy These Files

This ZIP contains an overlay folder called:

- `yahboomcar_nav_race_addon/`

Inside it you have:

- `launch/race_loop.launch`
- `config/race_waypoints.yaml`
- `config/teb_local_planner_params_race.yaml`
- `scripts/waypoint_race.py`

You should **merge** these into your existing `yahboomcar_nav` package, typically at:

```bash
cd ~/yahboomcar_ws/src
cp -r rosmaster_r2_race_loop_addon/yahboomcar_nav_race_addon/* yahboomcar_nav/
```

Then rebuild your workspace:

```bash
cd ~/yahboomcar_ws
catkin_make
source devel/setup.bash
```

Now your `yahboomcar_nav` package has:

- A **new launch file**: `race_loop.launch`
- A **new script**: `scripts/waypoint_race.py`
- A **sample waypoint file**: `config/race_waypoints.yaml`
- An **example TEB tuning file**: `config/teb_local_planner_params_race.yaml`

---

## 2. Overall Workflow (High Level)

1. Start the robot and LiDAR
2. Run Cartographer to build a 2D map
3. Save the map to disk
4. Restart with navigation using the saved map + TEB
5. Run `race_loop.launch` to loop through waypoints
6. Use wireless handle teleop as override

The next sections give all commands in order.

---

## 3. Step 1 – Bringup and LiDAR

### 3.1 Terminal 1 – Bringup

```bash
source /opt/ros/noetic/setup.bash
source ~/yahboomcar_ws/devel/setup.bash

roslaunch yahboomcar_bringup bringup.launch
```

This connects to the expansion board MCU and enables motors/servos.

### 3.2 Terminal 2 – LiDAR

```bash
source /opt/ros/noetic/setup.bash
source ~/yahboomcar_ws/devel/setup.bash

roslaunch rplidar_ros rplidar.launch
```

LiDAR should now publish on `/scan`.

Leave both terminals running.

---

## 4. Step 2 – Run Cartographer and Build a Map

### 4.1 Terminal 3 – Cartographer Mapping

```bash
source /opt/ros/noetic/setup.bash
source ~/yahboomcar_ws/devel/setup.bash

roslaunch yahboomcar_nav yahboomcar_map.launch use_rviz:=false map_type:=cartographer
```

- `map_type:=cartographer` selects Cartographer for SLAM.

### 4.2 Optional: RViz on PC

On your PC (with ROS network configured):

```bash
roslaunch yahboomcar_nav view_navigate.launch
```

In RViz:

- Fixed Frame: `map`
- Add displays: `Map`, `LaserScan`, `TF`

### 4.3 Drive Around to Build the Map

- Use:
  - Keyboard teleop, or
  - USB wireless handle, or
  - Mobile app
- Move **slowly** and cover the whole drivable area.
- Try to make loops (start and finish in similar positions).

---

## 5. Step 3 – Save the Map

Open **Terminal 4** on the Jetson:

```bash
source /opt/ros/noetic/setup.bash
source ~/yahboomcar_ws/devel/setup.bash

mkdir -p ~/maps
rosrun map_server map_saver -f ~/maps/r2_cartographer_map
```

You should now have:

- `~/maps/r2_cartographer_map.pgm`
- `~/maps/r2_cartographer_map.yaml`

You can **stop** Cartographer and LiDAR after saving:

- `Ctrl+C` in Terminal 3 (Cartographer)
- `Ctrl+C` in Terminal 2 (LiDAR)

You may also stop bringup if you’d like a clean restart.

---

## 6. Step 4 – Restart with Navigation (TEB + Saved Map)

### 6.1 Terminal 1 – Bringup Again

```bash
source /opt/ros/noetic/setup.bash
source ~/yahboomcar_ws/devel/setup.bash

roslaunch yahboomcar_bringup bringup.launch
```

### 6.2 Terminal 2 – LiDAR Again

```bash
source /opt/ros/noetic/setup.bash
source ~/yahboomcar_ws/devel/setup.bash

roslaunch rplidar_ros rplidar.launch
```

### 6.3 Terminal 3 – Navigation with Map

Depending on how `yahboomcar_navigation.launch` is written, you’ll either pass the **base name** or a path.

Try this form first (maps in default nav folder):

```bash
roslaunch yahboomcar_nav yahboomcar_navigation.launch use_rviz:=false map:=r2_cartographer_map
```

If your launch expects a full path:

```bash
roslaunch yahboomcar_nav yahboomcar_navigation.launch use_rviz:=false map:=/home/robot/maps/r2_cartographer_map
```

Adjust `/home/robot/maps` to match your actual home path.

### 6.4 RViz Navigation View

On your PC or on the Jetson (if it has a screen):

```bash
roslaunch yahboomcar_nav view_navigate.launch
```

In RViz, you should now see:

- The **static map**
- The **robot model**
- Live laser scans

### 6.5 Set Initial Pose (AMCL)

In RViz:

1. Click **“2D Pose Estimate”**
2. Click where the robot physically is on the map
3. Drag to indicate robot heading
4. Drive a bit to see if the estimate tracks reality

### 6.6 Test a Single Nav Goal

In RViz:

1. Click **“2D Nav Goal”**
2. Click a target location in the map
3. Robot should:
   - Plan a global path
   - Follow it using TEB
   - Avoid obstacles

If this works, navigation is functioning properly.

---

## 7. Step 5 – TEB Racing-Style Settings (Optional but Recommended)

You have an example TEB config here:

- `config/teb_local_planner_params_race.yaml`

You can **merge** these parameters into your existing TEB YAML or use them as a reference.

Key racing-style changes:

```yaml
TebLocalPlannerROS:
  max_vel_x: 1.5
  max_vel_theta: 4.0
  acc_lim_x: 2.0
  acc_lim_theta: 3.0
  min_turning_radius: 0.10

  weight_optimaltime: 100.0
  weight_kinematics_forward_drive: 50.0
  weight_acc_limit: 1.0
```

- Increase `max_vel_x` carefully (speed).
- Increase `max_vel_theta` for quicker turns.
- Decrease `min_turning_radius` only if your car hardware can handle tighter turns.
- `weight_optimaltime` encourages faster paths.

After editing your TEB config, **restart navigation** to apply changes.

---

## 8. Step 6 – Configure Waypoints for the Racing Loop

You have a sample file:

- `config/race_waypoints.yaml`

Edit it to match your room/track. Each waypoint has:

- A `name`
- A `frame_id` (usually `"map"`)
- A `pose` with `position` (x, y) and `orientation` (z, w)

Example (already in the file):

```yaml
waypoints:
  - name: wp1
    frame_id: "map"
    pose:
      position: {x: 0.0, y: 0.0}
      orientation: {z: 0.0, w: 1.0}
```

Use `rostopic echo /move_base_simple/goal` while clicking 2D Nav Goals in RViz to grab real poses and paste them into `race_waypoints.yaml`.

---

## 9. Step 7 – Run the Racing Loop

Once:

- Bringup is running
- LiDAR is running
- Navigation (with TEB and your map) is running
- AMCL pose is set and stable

You can start the race loop:

```bash
source /opt/ros/noetic/setup.bash
source ~/yahboomcar_ws/devel/setup.bash

roslaunch yahboomcar_nav race_loop.launch
```

This will:

- Start `waypoint_race.py`
- Continually publish waypoints from `race_waypoints.yaml` to `/move_base_simple/goal`
- Cause the robot to drive from waypoint 1 → 2 → 3 → ... → N → 1 → … in a loop
- Use TEB to generate smooth, racing-style motion between points

Stop the loop with `Ctrl+C` in the race_loop terminal.

---

## 10. Step 8 – Handle Control as Safety Override

In another terminal, you can run:

```bash
source /opt/ros/noetic/setup.bash
source ~/yahboomcar_ws/devel/setup.bash

rosrun joy joy_node
rosrun teleop_twist_joy teleop_twist_joy.py
```

This will:

- Read your **USB wireless handle** inputs
- Turn them into `/cmd_vel` commands

Depending on how your stack is set up, you can:

- Use a **mode switch** to choose between autonomous nav and manual teleop
- Or use a multiplexer (like `twist_mux`) that prioritizes manual commands in emergencies

Always test at **low speed first**.

---

## 11. Safety Notes & Tips

- Start with small loops in an open area
- Slowly increase `max_vel_x` as you gain confidence
- Confirm obstacle avoidance by placing a box on the track and watching TEB steer around it
- Keep your hand near the power switch or emergency stop
- Keep a terminal ready to send:
  ```bash
  rostopic pub /cmd_vel geometry_msgs/Twist "linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0" -r 10
  ```
  if you ever need to forcibly stop motion by flooding zero velocity

---

## 12. Summary

This add-on gives you:

- A clear **Cartographer → Navigation → TEB racing** pipeline
- A **waypoint-based racing loop** with simple YAML configuration
- A **Python node** that can be extended (lap timing, logging, etc.)
- A straightforward way to keep the **handle** as an override

You can now use the Rosmaster R2 like a tiny **F1TENTH-style research platform**.

Happy racing!
