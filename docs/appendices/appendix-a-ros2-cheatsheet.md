---
sidebar_position: 1
title: "Appendix A: ROS2 Cheatsheet"
---

# Appendix A: ROS 2 Cheatsheet

## Core Concepts

### Nodes
Nodes are the fundamental execution units in ROS 2 that perform computation.

**Creating a Node:**
```cpp
// C++ Example
#include "rclcpp/rclcpp.hpp"

class MinimalNode : public rclcpp::Node
{
public:
    MinimalNode() : Node("minimal_node") {}
};
```

```python
# Python Example
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
```

### Topics and Messages
Topics are named buses over which nodes exchange messages.

**Publisher:**
```cpp
// C++ Publisher
auto publisher = this->create_publisher<std_msgs::msg::String>("topic", 10);
```

```python
# Python Publisher
self.publisher_ = self.create_publisher(String, 'topic', 10)
```

**Subscriber:**
```cpp
// C++ Subscriber
auto subscription = this->create_subscription<std_msgs::msg::String>(
    "topic", 10, 
    [this](const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
);
```

```python
# Python Subscriber
def listener_callback(self, msg):
    self.get_logger().info('I heard: "%s"' % msg.data)

self.subscription = self.create_subscription(
    String,
    'topic',
    self.listener_callback,
    10)
```

## Common Commands

### Package Management
```bash
# Create a new package
ros2 pkg create --build-type ament_cmake my_package

# Build all packages
colcon build

# Build specific package
colcon build --packages-select my_package

# Source the workspace
source install/setup.bash
```

### Command Line Tools
```bash
# List topics
ros2 topic list

# Echo a topic
ros2 topic echo /topic_name std_msgs/msg/String

# Publish to a topic
ros2 topic pub /topic_name std_msgs/msg/String "data: 'Hello'"

# List services
ros2 service list

# Call a service
ros2 service call /service_name example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"

# List nodes
ros2 node list

# Get information about a node
ros2 node info /node_name
```

## Launch Files

### Python Launch Files
```python
# launch/my_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='my_node',
            parameters=[
                {'param1': 'value1'},
                {'param2': 42}
            ]
        )
    ])
```

## Parameters
```cpp
// C++ Parameter Declaration
this->declare_parameter("my_param", "default_value");
auto param_value = this->get_parameter("my_param").as_string();
```

```python
# Python Parameter Declaration
self.declare_parameter('my_param', 'default_value')
param_value = self.get_parameter('my_param').value
```

## Actions
```python
# Python Action Client
from rclpy.action import ActionClient
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self, Fibonacci, 'fibonacci')
```

## Quality of Service (QoS)
```cpp
// C++ QoS Settings
rclcpp::QoS qos_profile(10);
qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

auto publisher = this->create_publisher<std_msgs::msg::String>(
    "topic", qos_profile);
```

## TF2 (Transform Library)
```cpp
// C++ TF2 Broadcaster
#include "tf2_ros/transform_broadcaster.h"
geometry_msgs::msg::TransformStamped t;
t.header.stamp = this->get_clock()->now();
t.header.frame_id = "world";
t.child_frame_id = "robot";
// Set transform values...
tf_broadcaster_->sendTransform(t);
```

## Common Build System (CMakeLists.txt)
```cmake
cmake_minimum_required(VERSION 3.8)
project(my_package)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp std_msgs)

install(TARGETS my_node DESTINATION lib/${PROJECT_NAME})

ament_package()
```

## Package.xml Template
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_package</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```