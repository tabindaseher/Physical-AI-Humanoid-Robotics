---
sidebar_position: 2
title: "Appendix B: Gazebo Setup"
---

# Appendix B: Gazebo Setup

## Introduction to Gazebo

Gazebo is a 3D simulation environment for robotics that provides realistic physics simulation, high-quality graphics, and convenient programmatic interfaces.

## Installation

### Ubuntu Installation
```bash
# Set up the Gazebo package repository
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

# Update packages and install Gazebo
sudo apt-get update
sudo apt-get install gazebo libgazebo-dev
```

### ROS 2 Integration
```bash
# Install Gazebo ROS packages
sudo apt install ros-humble-gazebo-ros-pkgs
```

## Basic Gazebo Usage

### Starting Gazebo
```bash
# Launch Gazebo with GUI
gazebo

# Launch with a specific world file
gazebo my_world.world

# Launch without GUI (server only)
gzserver my_world.world
```

## Creating Robot Models

### URDF to SDF Conversion
Gazebo uses SDF (Simulation Description Format), but can read URDF (Unified Robot Description Format) models.

**Basic URDF Robot:**
```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
</robot>
```

### Adding Gazebo-Specific Tags
```xml
<robot name="gazebo_robot">
  <link name="chassis">
    <!-- Link definition as above -->
  </link>

  <!-- Gazebo-specific plugin -->
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <ros>
        <namespace>robot</namespace>
        <remapping>cmd_vel:=cmd_vel</remapping>
        <remapping>odom:=odom</remapping>
      </ros>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.4</wheel_separation>
      <wheel_diameter>0.2</wheel_diameter>
    </plugin>
  </gazebo>
</robot>
```

## Common Sensors in Gazebo

### Camera Sensor
```xml
<sensor name="camera" type="camera">
  <camera name="head">
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.1</near>
      <far>10</far>
    </clip>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
</sensor>
```

### LIDAR Sensor
```xml
<sensor name="laser" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>10</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <always_on>1</always_on>
  <visualize>true</visualize>
  <update_rate>10</update_rate>
</sensor>
```

## Launching with ROS 2

### Launch File Example
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch Gazebo Server
    gazebo_server = Node(
        package='gazebo_ros',
        executable='gzserver',
        name='gazebo_server',
        parameters=[{'use_sim_time': True}]
    )
    
    # Launch Gazebo Client
    gazebo_client = Node(
        package='gazebo_ros',
        executable='gzclient',
        name='gazebo_client'
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot',
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo_server,
        gazebo_client,
        spawn_robot
    ])
```

## Physics Configuration

### World File Example
```xml
<sdf version="1.7">
  <world name="default">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    
    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Sun -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Your robot or models -->
    <model name="my_robot">
      <!-- Model definition -->
    </model>
  </world>
</sdf>
```

## Gazebo ROS Control

### ROS 2 Control Integration
```xml
<gazebo>
  <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
    <parameters>$(find my_robot_control)/config/my_robot_control.yaml</parameters>
  </plugin>
</gazebo>
```

### Control Configuration
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

joint_state_broadcaster:
  type: joint_state_broadcaster/JointStateBroadcaster

velocity_controller:
  type: diff_drive_controller/DiffDriveController

diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]
    wheel_separation: 0.4
    wheel_radius: 0.1
    publish_rate: 50.0
```

## Troubleshooting

### Common Issues and Solutions
1. **Gazebo not starting**: Check graphics drivers and X11 forwarding
2. **Robot falling through ground**: Verify inertial properties in URDF
3. **Sensors not publishing**: Check ROS topic connections
4. **Performance issues**: Reduce physics update rate or simplify models

### Environment Variables
```bash
# Set Gazebo resources path
export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:$GAZEBO_RESOURCE_PATH

# Set models path
export GAZEBO_MODEL_PATH=/path/to/models:$GAZEBO_MODEL_PATH

# Set plugin path
export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:$GAZEBO_PLUGIN_PATH
```