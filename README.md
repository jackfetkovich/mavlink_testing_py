## 🚀 Proof of Concept: MAVLink → ROS 2

This proof of concept validates that MAVLink messages received by the script are published on a ROS 2 topic:
/mavtopic

Test Method 1: Jetson Nano / Raspberry Pi (ROS 2 Humble)
  Setup (2 terminals) 
  
$source /opt/ros/humble/setup.bash

Terminal 1 — Run script

$python3 barebones_mavlink_ros2.py

Terminal 2 — Monitor topic

$ros2 topic list

$ros2 topic echo /mavtopic 

(Raw Idea)Test Method 2: Local (WSL with ROS 2 Humble) - Run QGC via a dummy python script for mavlink messages 
  Setup (2 terminals) 
  
$source /opt/ros/humble/setup.bash

Terminal 1 — Run script

$python3 barebones_mavlink_ros2.py

Terminal 2 — Monitor topic

$ros2 topic list

$ros2 topic echo /mavtopic 

Success Criteria
Script runs without errors
/mavtopic appears in topic list
ros2 topic echo /mavtopic outputs all the mavlink commands getting published on mavtopic.
Messages are visible:
ros2 topic echo /mavtopic
