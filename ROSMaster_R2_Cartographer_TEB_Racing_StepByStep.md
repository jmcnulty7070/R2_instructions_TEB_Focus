
# Rosmaster R2 – Step‑by‑Step: Make a Map with Cartographer and Race It with TEB (Using Handle Control)

This guide walks you through **one complete workflow** on the Rosmaster R2:

1. Start the robot and sensors  
2. Make a 2D map with **Cartographer**  
3. Save the map  
4. Load the map and run **navigation** using **TEB** as the local planner  
5. Add a simple **racing loop** (waypoints)  
6. Use the **USB wireless handle** to take over whenever you want  

Everything is written at an 8th‑grade reading level. No Raspberry Pi or Docker commands are used here; this assumes a **Jetson**‑based setup (Nano / Xavier NX / Orin).

---

## 0. Assumptions and Folder Layout

We assume:

- Your robot uses the Yahboom Rosmaster R2 ROS1 stack.  
- Your catkin workspace is: `~/yahboomcar_ws`  
- The main navigation package is: `yahboomcar_nav`  
- You can open a terminal on the Jetson (directly or via SSH).  

If your paths differ, adjust the commands accordingly.

---

## 1. Start the Robot (Bringup) and LiDAR

Open **Terminal 1** on the Jetson.

### 1.1 Source ROS and Workspace

```bash
source /opt/ros/noetic/setup.bash
source ~/yahboomcar_ws/devel/setup.bash
```

(Use `melodic` instead of `noetic` if that’s your ROS version.)

### 1.2 Start the Robot Drivers (Bringup)

```bash
roslaunch yahboomcar_bringup bringup.launch
```

This should:

- Connect to the expansion board MCU  
- Enable motor and servo control  
- Start basic TF (robot coordinate frames)  

Leave this terminal running.

### 1.3 Start the LiDAR

Open **Terminal 2**:

```bash
source /opt/ros/noetic/setup.bash
source ~/yahboomcar_ws/devel/setup.bash

roslaunch rplidar_ros rplidar.launch
```

Your LiDAR should now publish on `/scan` and be visible in RViz later.

Leave this terminal running as well.

---

## 2. Start Cartographer and Build a Map

Cartographer uses LiDAR (and optionally IMU/odom) to create a 2D map.

### 2.1 Start Cartographer Mapping

Open **Terminal 3**:

```bash
source /opt/ros/noetic/setup.bash
source ~/yahboomcar_ws/devel/setup.bash

roslaunch yahboomcar_nav yahboomcar_map.launch use_rviz:=false map_type:=cartographer
```

- `map_type:=cartographer` tells the launch file to use Cartographer instead of other SLAM backends.  
- `use_rviz:=false` means it won’t auto‑start RViz (you can start RViz separately on another machine if you want a GUI).

### 2.2 Optional: View Mapping in RViz (on a PC)

On your **development PC** (not on the Jetson), make sure:

- `ROS_MASTER_URI` points to the Jetson  
- `ROS_IP` or `ROS_HOSTNAME` are set correctly  

Then run:

```bash
roslaunch yahboomcar_nav view_navigate.launch
```

In RViz:

- Set **Fixed Frame** to `map`  
- Add displays for:
  - `Map`
  - `LaserScan` (topic `/scan`)
  - `TF`

You should see the map gradually appear as the robot moves.

### 2.3 Drive the Robot to Create the Map

While Cartographer is running:

- Use **handle control**, **keyboard teleop**, or **mobile app** to move the robot.  
- Drive **slowly**. Important tips:
  - Keep rotation speed low.  
  - Try to walk the robot around the edges of the room / hallway.  
  - Make closed loops if possible (start and end at similar positions).  

The slower and smoother you move, the better the map will look.

---

## 3. Save the Cartographer Map

When you’re happy with the map shape, you need to **save** it so it can be used later for navigation.

Open **Terminal 4** on the Jetson:

```bash
source /opt/ros/noetic/setup.bash
source ~/yahboomcar_ws/devel/setup.bash

rosrun map_server map_saver -f ~/maps/r2_cartographer_map
```

This will create two files (in `~/maps`):

- `r2_cartographer_map.pgm` – the image of the map  
- `r2_cartographer_map.yaml` – the metadata and resolution info  

You can choose any folder you like; `~/maps` is just a clean place to store map files.

> ✅ At this point, you have built and saved a Cartographer 2D map.

You can now **stop** the mapping processes:

- Stop Cartographer (`Ctrl+C` in Terminal 3)  
- Stop LiDAR (`Ctrl+C` in Terminal 2)  

Leave Terminal 1 (bringup) running if you wish, or stop and restart everything in the next step.

---

## 4. Load the Map and Run Navigation (with TEB)

Now you’ll load the saved map and run navigation using `yahboomcar_navigation.launch`.  
In most Yahboom configs, **TEB** is already set as the local planner inside the navigation stack.

### 4.1 Start Drivers and LiDAR Again

Open a **fresh Terminal 1** (or reuse the old one):

```bash
source /opt/ros/noetic/setup.bash
source ~/yahboomcar_ws/devel/setup.bash

roslaunch yahboomcar_bringup bringup.launch
```

Open **Terminal 2**:

```bash
source /opt/ros/noetic/setup.bash
source ~/yahboomcar_ws/devel/setup.bash

roslaunch rplidar_ros rplidar.launch
```

### 4.2 Start Navigation with the Saved Map

Open **Terminal 3**:

```bash
source /opt/ros/noetic/setup.bash
source ~/yahboomcar_ws/devel/setup.bash

roslaunch yahboomcar_nav yahboomcar_navigation.launch use_rviz:=false map:=/home/robot/maps/r2_cartographer_map
```

Adjust the path (`/home/robot/maps/...`) to match where you saved your map.  
`map:=` expects the **base name** (without `.yaml` extension) or the launch file may directly accept the path depending on its configuration. If one form doesn’t work, try:

```bash
map:=r2_cartographer_map
```

with the map files placed in the default maps directory used by `yahboomcar_nav` (often something like `~/yahboomcar_ws/src/yahboomcar_nav/maps`).

### 4.3 Start the RViz Navigation View

On your PC (or directly on the Jetson if it has a screen), run:

```bash
roslaunch yahboomcar_nav view_navigate.launch
```

In RViz, you should see:

- The **static map** (your Cartographer map)
- Robot model at some location on the map
- Laser scans overlaid on the map

### 4.4 Set the Initial Pose (AMCL)

In RViz:

1. Click the **“2D Pose Estimate”** tool.  
2. Click on the map where the robot is roughly located.  
3. Drag to set the robot’s facing direction.  

You should see a cloud of AMCL particles around the robot.  
Drive the robot slightly and ensure the pose estimate follows reality.

### 4.5 Send a Navigation Goal

Still in RViz:

1. Click **“2D Nav Goal”**.  
2. Click a target point on the map (e.g., across the room).  
3. The robot should:
   - Plan a path (global path)  
   - Follow it using the local planner (TEB), avoiding obstacles  

If it can reach the goal smoothly, your **Cartographer + TEB navigation** pipeline is working.

---

## 5. Enable High‑Speed TEB «Racing» Behavior

To make the robot behave more like an F1TENTH car, you tune the TEB parameters.

> ⚠️ Do this carefully. Start with modest speed increases and always leave plenty of space.

### 5.1 Locate the TEB Config File

Look under your `yahboomcar_nav` package, often in a `config` folder, for a file such as:

- `teb_local_planner_params.yaml`  
- or a navigation config YAML that contains a `TebLocalPlannerROS` section.

Example path (adjust if needed):

```bash
cd ~/yahboomcar_ws/src/yahboomcar_nav/config
ls
```

Open the TEB YAML in an editor:

```bash
nano teb_local_planner_params.yaml
```

### 5.2 Example TEB Racing‑Style Parameters

In the `TebLocalPlannerROS:` section, you may find parameters like:

```yaml
TebLocalPlannerROS:
  max_vel_x: 0.8
  max_vel_x_backwards: 0.0
  max_vel_theta: 1.0
  acc_lim_x: 1.0
  acc_lim_theta: 1.0
  min_turning_radius: 0.2
  wheelbase: 0.25
```

For more “racing” behavior (small robot in a clear indoor track), you can **gradually** increase:

```yaml
  max_vel_x: 1.5              # forward speed
  max_vel_theta: 4.0          # turning speed
  acc_lim_x: 2.0              # stronger acceleration
  acc_lim_theta: 3.0
  min_turning_radius: 0.10    # tighter turns if your car allows it
```

Also, consider these weights:

```yaml
  weight_optimaltime: 100.0
  weight_kinematics_forward_drive: 50.0
  weight_acc_limit: 1.0
```

- Higher `weight_optimaltime` makes the planner care more about reaching the goal quickly.  
- `weight_kinematics_forward_drive` encourages forward motion rather than reversing.  

Save the file and re‑launch navigation to apply changes.

> ✅ Now your TEB planner is tuned toward smoother, faster, racing‑style motion.

---

## 6. Create a Simple Racing Loop with Waypoints

You can create a loop of waypoints around the map and have the robot drive them in order, using TEB for the local path.

There are many ways to implement this; here is a simple pattern using a YAML file and a small ROS node.

### 6.1 Record Waypoints in RViz

1. Start navigation as in **Step 4**.  
2. Use RViz (or a small script) to capture several poses on the map that define your loop.  
3. For each point:
   - Use `2D Nav Goal` to move the robot there once  
   - Echo `/move_base_simple/goal` to see its pose:

```bash
rostopic echo /move_base_simple/goal
```

You’ll see output like:

```yaml
pose:
  pose:
    position:
      x: 2.15
      y: -5.02
    orientation:
      z: 0.72
      w: 0.68
```

4. Copy the pose values (x, y, z, w) for each waypoint.

### 6.2 Create a Waypoint YAML File

Create a file, for example:

```bash
mkdir -p ~/yahboomcar_ws/src/yahboomcar_nav/config
nano ~/yahboomcar_ws/src/yahboomcar_nav/config/race_waypoints.yaml
```

Example content:

```yaml
waypoints:
  - name: wp1
    frame_id: "map"
    pose:
      position: {x: 0.0, y: 0.0}
      orientation: {z: 0.0, w: 1.0}

  - name: wp2
    frame_id: "map"
    pose:
      position: {x: 2.0, y: 0.5}
      orientation: {z: 0.0, w: 1.0}

  - name: wp3
    frame_id: "map"
    pose:
      position: {x: 2.0, y: -1.0}
      orientation: {z: 0.0, w: 1.0}

  - name: wp4
    frame_id: "map"
    pose:
      position: {x: 0.0, y: -1.5}
      orientation: {z: 0.0, w: 1.0}
```

Adjust these positions/orientations to match your room; make them form a loop.

### 6.3 Minimal Waypoint Follower Node

Create a simple Python node, e.g.:

```bash
nano ~/yahboomcar_ws/src/yahboomcar_nav/scripts/waypoint_race.py
```

Paste this minimal example:

```python
#!/usr/bin/env python3
import rospy
import yaml
from geometry_msgs.msg import PoseStamped

def load_waypoints(path):
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    return data['waypoints']

if __name__ == "__main__":
    rospy.init_node("waypoint_race")
    pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

    yaml_path = rospy.get_param("~waypoints_file", "")
    loop = rospy.get_param("~loop", True)
    wait_time = rospy.get_param("~wait_time", 1.0)

    if not yaml_path:
        rospy.logerr("No waypoints_file parameter provided")
        exit(1)

    waypoints = load_waypoints(yaml_path)
    rospy.loginfo("Loaded %d waypoints", len(waypoints))

    rate = rospy.Rate(0.1)  # slow start

    while not rospy.is_shutdown():
        for wp in waypoints:
            msg = PoseStamped()
            msg.header.frame_id = wp.get("frame_id", "map")
            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x = wp["pose"]["position"]["x"]
            msg.pose.position.y = wp["pose"]["position"]["y"]
            msg.pose.orientation.z = wp["pose"]["orientation"]["z"]
            msg.pose.orientation.w = wp["pose"]["orientation"]["w"]
            pub.publish(msg)
            rospy.loginfo("Sent waypoint: %s", wp.get("name", "noname"))
            rospy.sleep(wait_time)
        if not loop:
            break
        rate.sleep()
```

Make it executable:

```bash
chmod +x ~/yahboomcar_ws/src/yahboomcar_nav/scripts/waypoint_race.py
```

### 6.4 Launch the Racing Loop

Create a launch file, for example:

```bash
nano ~/yahboomcar_ws/src/yahboomcar_nav/launch/race_loop.launch
```

Content:

```xml
<launch>
  <!-- Assume navigation (move_base + TEB) is already running -->
  <node pkg="yahboomcar_nav" type="waypoint_race.py" name="waypoint_race" output="screen">
    <param name="waypoints_file" value="$(find yahboomcar_nav)/config/race_waypoints.yaml" />
    <param name="loop" value="true" />
    <param name="wait_time" value="1.0" />
  </node>
</launch>
```

Now, with navigation already running (Step 4), start the loop:

```bash
roslaunch yahboomcar_nav race_loop.launch
```

The car should:

- Drive from waypoint 1 → 2 → 3 → 4 → back to 1  
- Use TEB to generate smooth, obstacle‑avoiding, racing‑style paths between them  
- Continue looping until you stop the node (`Ctrl+C`)  

> ✅ You now have a basic “racing loop” like a mini F1TENTH track.

---

## 7. Using the USB Wireless Handle to Take Over

You can always use the **USB wireless handle** as a safety/override control.

### 7.1 Starting Handle‑Based Teleop (Example Pattern)

In a separate terminal (with ROS and workspace sourced):

```bash
rosrun joy joy_node
rosrun teleop_twist_joy teleop_twist_joy.py
```

- `joy_node` reads the handle inputs  
- `teleop_twist_joy` converts joystick inputs into `/cmd_vel`

Make sure:

- The navigation stack is configured so that manual `/cmd_vel` can override or you have a way (such as a multiplexer or a mode switch) to switch between **autonomous** and **manual**.

If your Yahboom stack includes a **mode switch** via a button or RC channel, follow that tutorial to:

- Map a handle button to enter **manual mode**
- Map another to return to **navigation mode**

### 7.2 Safety Tips

- Always test at **low speed** first (`max_vel_x` small).  
- Keep a clear area in front of the car.  
- Keep your hand near the power switch or emergency stop.  
- If the robot behaves strangely, stop all navigation nodes and verify TF, map, and LiDAR data.

---

## 8. Summary of the Full Pipeline

From start to finish, the workflow is:

1. **Bringup + LiDAR**  
2. **Run Cartographer** and drive around slowly to build a map  
3. **Save the map** with `map_saver`  
4. **Restart bringup + LiDAR + navigation** and load your saved map  
5. **Use TEB** as the local planner (tuned for smoother, faster motion)  
6. **Create a loop of waypoints** and run `race_loop.launch`  
7. Use the **USB wireless handle** as an override or manual mode controller  

This gives you a **complete mapping → navigation → racing** pipeline on your Rosmaster R2.
