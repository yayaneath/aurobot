# Aurobot

This repository contains URDF, meshes and MoveIt files used for working with AUROVA's lab custom robot.

## Requirements

* Ubuntu 16.04
* ROS Kinetic

## Description

AUROVA's AUROBOT consists of a Mitsubishi PA10 robot arm on a custom stand and with the posibility of attaching a Shadow Dexterous Hand or an Allegro Hand to the end-effector. 

All commands are performed from a Linux computer with Ubuntu 16.04 and ROS Kinetic. There is computer with Windows XP which acts as a communication gate between the Linux computer and the controller.

## Current configuration

The current configuration features a Shadow Dexterous Hand with 3 BioTac SP sensors mounted on the thumb, first and middle fingers.

The system is prepared to obtain tactile data from grasps performed on objects located on a table. Computer vision using an Intel RealSense D415 depth camera is used to obtain the target's location.

## Usage

All the necessary steps to run the current configuration are described below.

*WARNING: When sending commands to the ROBOT, the EMERGENCY STOP button must always be at a reachable location.*

1. [WORLD] Turn on robot arm controller, Windows PC and Ubuntu PC.

2. [LINUX] Open terminal and setup terminal layout:
```
terminator --layout=shadow-tact
```

3. [LINUX] Group all in Tab 0 and broadcast to Tab 0.
```
cd catkin_ws/
source devel/setup.bash
```
End broadcast.

4. [LINUX] Run command in Tab 0:
```
roscore
```

5. [LINUX] Run command in Tab 0:
```
roslaunch aurobot_right_shadow_moveit_config demo.launch
```
Wait for RViz to be up and running. Check initial robot position. The robot must be initialized in the 'home' position with the arm pointing up.

6. [WINDOWS] Run PA10 server.

7. [LINUX] Run command in Tab 0:
```
rosrun aurobot_utils move_right_pa10
```
This will synchronize the real robot with the position displayed in RViz. From now on, any movement commanded to the robot using MoveIt will also be transmitted to the real robot.

When the real robot position and the RViz visualization are synchronized the message 'WE KEEP SAME POSITION' will be displayed.

8. [LINUX] Move robot to 'ready' position from RViz.
Initial state: 'current'.
Goal state: 'ready'.
Update, plan, check and execute.

9. [WORLD] Connect Shadow Hand contoller power adapter.
Turning on the controller will produce an oscillating motion (vibration) in the joints of the hand.

10. [LINUX] Group all in Tab 1 and broadcast to Tab 1.
```
cd shadow_ws/
export ROS_MASTER_URI=http://localhost:11312
source devel/setup.bash
```
End broadcast.

11. [LINUX] Run command in Tab 1:
```
roscore -p 11312
```
Shadow Hand needs a different ROS master node (in a different port) to work correctly. Communication between both ROS master nodes must be estabilished.

12. [LINUX] Run command in Tab 0:
```
rosrun master_discovery_fkie master_discovery
```

13. [LINUX] Run command in Tab 1:
```
rosrun master_discovery_fkie master_discovery
```
Both ROS master nodes are in communication now.

14. [LINUX] Run command in Tab 1:
```
roslaunch sr_ethercat_hand_config sr_rhand.launch moveit:="false"
```
This will connect to the Shadow Hand and reset controllers, producing a vibration.

15. [LINUX] Run command in Tab 1:
```
rosrun master_sync_fkie master_sync _~sync_topics:=['/rh_trajectory_controller/command']
```

16. [LINUX] Run command in Tab 0:
```
rosrun aurobot_utils move_right_shadow
```
This will cause the hand to go to the position showed in RViz, causing it to open up.
The hand is ready to receive commands from MoveIt.

17. [WORLD] Connect RealSense camera to high-speed USB 3.0 connector on the back of the Linux computer.

18. [LINUX] Run command in Tab 0:
```
realsense-viewer
```
Connect 'stereo' and 'RGB' camera streams to load custom parameters and close.

19. [LINUX] Run command in Tab 0:
```
cd ../rsense_ws/
source devel/setup.bash
roslaunch realsense2_camera rs_rgbd.launch initial_reset:=true
```
Camera is ready to be used.

20. [LINUX] Run command in Tab 0:
```
rosrun aurobot_utils realsense_to_world _topic:=/camera/depth_registered/points _link:=/top_camera_link
```
This step is optional, it will show the point cloud in RViz.

*The robot is ready now.*