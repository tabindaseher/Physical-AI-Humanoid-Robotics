---
sidebar_position: 32
title: 'Appendix A: ROS2 Cheatsheet'
---

# Appendix A: ROS2 Cheatsheet

## Common ROS2 Commands

### Package Management
- `ros2 pkg create <package_name>` - Create a new package
- `colcon build` - Build all packages
- `source install/setup.bash` - Source the workspace

### Node Management
- `ros2 run <package_name> <executable_name>` - Run a node
- `ros2 node list` - List all active nodes
- `ros2 node info <node_name>` - Get information about a node

### Topic Management
- `ros2 topic list` - List all topics
- `ros2 topic echo <topic_name>` - Print messages from a topic
- `ros2 topic pub <topic_name> <msg_type> <args>` - Publish to a topic

### Service Management
- `ros2 service list` - List all services
- `ros2 service call <service_name> <srv_type> <args>` - Call a service

### Parameter Management
- `ros2 param list` - List parameters for a node
- `ros2 param get <node_name> <param_name>` - Get parameter value
- `ros2 param set <node_name> <param_name> <value>` - Set parameter value

## Common Message Types

- `std_msgs/msg/String` - String message
- `std_msgs/msg/Int32` - Integer message
- `sensor_msgs/msg/JointState` - Joint state information
- `geometry_msgs/msg/Twist` - Velocity commands
- `nav_msgs/msg/Odometry` - Odometry information