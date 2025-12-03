#!/usr/bin/env python3
import rospy
import yaml
from geometry_msgs.msg import PoseStamped

def load_waypoints(path):
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    if not data or 'waypoints' not in data:
        rospy.logerr("No 'waypoints' key found in %s", path)
        return []
    return data['waypoints']

if __name__ == "__main__":
    rospy.init_node("waypoint_race")
    pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

    yaml_path = rospy.get_param("~waypoints_file", "")
    loop = rospy.get_param("~loop", True)
    wait_time = rospy.get_param("~wait_time", 1.0)

    if not yaml_path:
        rospy.logerr("No waypoints_file parameter provided")
        rospy.signal_shutdown("Missing waypoints_file")
    else:
        rospy.loginfo("Using waypoints file: %s", yaml_path)

    waypoints = load_waypoints(yaml_path)
    if not waypoints:
        rospy.signal_shutdown("No waypoints loaded")
    rospy.loginfo("Loaded %d waypoints", len(waypoints))

    rate = rospy.Rate(0.1)

    while not rospy.is_shutdown():
        for wp in waypoints:
            if rospy.is_shutdown():
                break
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
