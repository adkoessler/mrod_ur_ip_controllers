# MROD UR IP controllers

Controllers for Robotic Manipulation of Deformable Objects (MROD) to be used with Institut Pascal's UR robots.

Controllers use the *VelocityJointInterface* of UR robots that is part of *Universal_Robots_ROS_Driver*: https://github.com/UniversalRobots/Universal_Robots_ROS_Driver

Controllers use the tools of *ROS Control*: http://wiki.ros.org/ros_control

Controllers use the tools of *KDL*: http://wiki.ros.org/kdl

## Velocity-based Shape Controller
(vel_based_shape_controller)

### Description

The controller implements shape control of linear elastic objects. The input is a desired array of node coordinates to be published on topic "desired_shape".
It uses KDL solvers (notably inverse kinematics solver) in order to turn the desired twist into a target joint velocity sent to the interface.

### Topics

#### desired_shape
The controller subscribes to this topic to get the goal node coordinates to be reached.

Name: "/vel_based_shape_controller/desired_shape"

Type: std_msgs/Float64MultiArray.

#### vision_shape
The controller subscribes to this topic to get the real node coordinates measured by vision.

Name: "/vel_based_shape_controller/vision_shape"

Type: std_msgs/Float64MultiArray.

#### x_master
The controller publishes on this topic to get the current pose of the end-effector

Type: geometry_msgs::Pose (with realtime_tools)

#### x_dot_master
The controller publishes on this topic to get the current twist of the end-effector

Type: geometry_msgs::Twist (with realtime_tools)



## Velocity-based Cartesian Velocity Controller
(vel_based_cartesian_velocity_controller)

### Description

The controller implements a cartesian velocity control. The input is a desired twist value to be published on topic "cmd_vel".

It uses KDL solvers (notably inverse kinematics solver) in order to turn this desired twist into a target joint velocity sent to the interface.

### Topics

#### cmd_vel
The controller subscribes to this topic to get the desired value of cartesian velocity specified by the user

Type: geometry_msgs::Twist

#### x_master
The controller publishes on this topic to get the current pose of the end-effector

Type: geometry_msgs::Pose (with realtime_tools)

#### x_dot_master
The controller publishes on this topic to get the current twist of the end-effector

Type: geometry_msgs::Twist (with realtime_tools)
