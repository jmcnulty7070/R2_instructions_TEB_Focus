
# Part 4 — Voice Control, Vision, Machine Learning, and TEB Racing

---

# 1. Voice Control

Start navigation:

```
roslaunch yahboomcar_nav yahboomcar_navigation.launch use_rviz:=false map:=my_map
```

Run voice target sender:

```
python ~/yahboomcar_ws/src/yahboomcar_voice_ctrl/scripts/voice_Ctrl_send_mark.py
```

Then say:
- “Hello Xiaoya”
- “Navigate to position 1”
- “Navigate to position 2”

---

# 2. Vision (OpenCV Basics)

Example script:

```python
import cv2

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow("View", gray)
    if cv2.waitKey(1) == 27:
        break
```

You can expand this to:
- Line tracking  
- Object detection  
- Depth processing  
- Lane following  

---

# 3. Machine Learning Autopilot

## 3.1 Collect Training Data
```
rosbag record -O training /camera/color/image_raw /cmd_vel
```

## 3.2 Label Images
Use CVAT or LabelImg.

## 3.3 Train Model
```
python train.py --img 640 --batch 16 --epochs 40 --data data.yaml
```

## 3.4 Deploy on Jetson (TensorRT recommended)

## 3.5 Autopilot Node
Your model outputs `/cmd_vel` based on camera input.

---

# 4. TEB Racing Guide (F1TENTH-Style)

TEB = Timed Elastic Band planner  
Used for:
- Smooth cornering  
- High-speed following  
- Racing lines  

Start TEB navigation:

```
roslaunch yahboomcar_navigation teb_navigation.launch
```

---

## 4.1 TEB Racing Tunings

### Faster Speed
```
max_vel_x = 1.5
max_vel_theta = 4.0
```

### Sharper Turns
```
min_turning_radius = 0.10
```

### Smoother Paths
```
weight_optimaltime = 100
weight_kinematics_forward_drive = 50
```

---

## 4.2 Racing Loop

Create a loop of 8–20 waypoints.

Launch loop navigation:

```
roslaunch yahboomcar_navigation race_loop.launch
```

Robot drives like a mini F1TENTH car:
- Smooth curves  
- Continuous laps  
- Obstacle avoidance  

---

End of Part 4.
