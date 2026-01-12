---
sidebar_position: 2
title: Python-based ROS 2 Agents
---

# Python-based ROS 2 Agents: Using rclpy for Intelligent Systems

## Introduction

Python has become one of the most popular languages for robotics development due to its simplicity, extensive libraries, and strong support for AI and machine learning applications. The `rclpy` library provides Python bindings for ROS 2, enabling developers to create sophisticated robotic agents that can perceive, reason, and act in their environment.

This chapter explores the implementation of intelligent agents using Python and ROS 2, covering the fundamentals of the rclpy library, best practices for agent design, and advanced patterns for creating responsive and intelligent robotic systems.

## Understanding rclpy Fundamentals

### rclpy Architecture

The `rclpy` library provides a Python interface to the ROS 2 client library (rcl). It abstracts the underlying DDS middleware and provides a consistent API for creating ROS 2 nodes, publishers, subscribers, services, and actions.

#### Core Components

1. **Node**: The basic unit of computation in ROS 2
2. **Executor**: Manages callbacks and node execution
3. **Publisher**: Sends messages to topics
4. **Subscriber**: Receives messages from topics
5. **Client**: Calls services
6. **Service**: Provides services
7. **Action Client**: Sends action goals
8. **Action Server**: Handles action goals

### Basic Node Structure

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating Intelligent Agents with rclpy

### Agent Architecture Patterns

#### Reactive Agents

Reactive agents respond directly to environmental stimuli without maintaining internal state:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ReactiveAvoidanceAgent(Node):
    def __init__(self):
        super().__init__('reactive_avoidance_agent')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.safe_distance = 1.0  # meters

    def laser_callback(self, msg):
        # Find minimum distance in front of robot
        min_distance = min(msg.ranges)

        cmd = Twist()
        if min_distance < self.safe_distance:
            # Stop and turn
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
        else:
            # Move forward
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0

        self.publisher.publish(cmd)
```

#### Deliberative Agents

Deliberative agents maintain internal state and plan actions based on goals:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from action_msgs.msg import GoalStatus

class DeliberativeNavigationAgent(Node):
    def __init__(self):
        super().__init__('deliberative_navigation_agent')
        self.current_goal = None
        self.goal_status = GoalStatus.STATUS_UNKNOWN
        self.path_publisher = self.create_publisher(Path, 'plan', 10)

    def set_goal(self, x, y):
        """Set a new navigation goal"""
        goal_msg = PoseStamped()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        # Additional logic for path planning would go here
        self.current_goal = goal_msg
```

#### Hybrid Agents

Hybrid agents combine reactive and deliberative behaviors:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from enum import Enum

class AgentState(Enum):
    IDLE = 1
    NAVIGATING = 2
    AVOIDING = 3
    EMERGENCY_STOP = 4

class HybridNavigationAgent(Node):
    def __init__(self):
        super().__init__('hybrid_navigation_agent')
        self.subscription = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state = AgentState.IDLE
        self.intended_velocity = Twist()

    def laser_callback(self, msg):
        # Emergency stop if obstacle too close
        if min(msg.ranges) < 0.3:
            self.state = AgentState.EMERGENCY_STOP
            cmd = Twist()
        elif min(msg.ranges) < 1.0:
            # Reactive obstacle avoidance
            self.state = AgentState.AVOIDING
            cmd = self.avoid_obstacle(msg)
        else:
            # Continue with intended navigation
            self.state = AgentState.NAVIGATING
            cmd = self.intended_velocity

        self.publisher.publish(cmd)

    def avoid_obstacle(self, scan_msg):
        """Reactive obstacle avoidance behavior"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5  # Turn away from obstacle
        return cmd
```

## Advanced rclpy Patterns

### Asynchronous Programming

ROS 2 supports asynchronous programming patterns for complex agents:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import asyncio
from std_msgs.msg import String

class AsyncAgent(Node):
    def __init__(self):
        super().__init__('async_agent')
        self.publisher = self.create_publisher(String, 'async_topic', 10)

    async def async_publisher(self):
        """Asynchronous publisher"""
        i = 0
        while rclpy.ok():
            msg = String()
            msg.data = f'Async message {i}'
            self.publisher.publish(msg)
            self.get_logger().info(f'Published: {msg.data}')
            i += 1
            await asyncio.sleep(1.0)  # Non-blocking sleep
```

### Parameter Management

Agents can use parameters for configuration:

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class ConfigurableAgent(Node):
    def __init__(self):
        super().__init__('configurable_agent')

        # Declare parameters with default values
        self.declare_parameter('max_linear_velocity', 0.5)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('safety_distance', 0.5)

        # Access parameters
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.safety_distance = self.get_parameter('safety_distance').value

    def parameter_callback(self, params):
        """Handle parameter changes"""
        for param in params:
            if param.name == 'max_linear_velocity':
                self.max_linear_vel = param.value
        return SetParametersResult(successful=True)
```

### Lifecycle Management

Complex agents may need lifecycle management:

```python
import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from lifecycle_msgs.msg import Transition

class LifecycleAgent(LifecycleNode):
    def __init__(self):
        super().__init__('lifecycle_agent')

    def on_configure(self, state):
        """Called when configuring the node"""
        self.get_logger().info(f'Configuring node, current state: {state}')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        """Called when activating the node"""
        self.get_logger().info(f'Activating node, current state: {state}')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        """Called when deactivating the node"""
        self.get_logger().info(f'Deactivating node, current state: {state}')
        return TransitionCallbackReturn.SUCCESS
```

## Integration with AI Libraries

### Machine Learning Integration

ROS 2 agents can integrate with popular ML libraries:

```python
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tensorflow as tf  # or pytorch, etc.

class MLPerceptionAgent(Node):
    def __init__(self):
        super().__init__('ml_perception_agent')
        self.subscription = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()

        # Load pre-trained model
        self.model = tf.keras.models.load_model('path/to/model')

    def image_callback(self, msg):
        """Process image with ML model"""
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Preprocess image for model
        input_tensor = np.expand_dims(cv_image, axis=0) / 255.0

        # Run inference
        predictions = self.model.predict(input_tensor)

        # Process results and publish
        # (Implementation depends on specific use case)
```

### Planning and Reasoning

Agents can incorporate planning and reasoning capabilities:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import OccupancyGrid
import numpy as np
from scipy.spatial import KDTree

class PlanningAgent(Node):
    def __init__(self):
        super().__init__('planning_agent')
        self.subscription = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10)
        self.map_data = None
        self.resolution = None
        self.origin = None

    def map_callback(self, msg):
        """Store map data for planning"""
        self.map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin

    def plan_path(self, start, goal):
        """Simple path planning algorithm"""
        if self.map_data is None:
            return None

        # Convert to grid coordinates
        start_grid = self.world_to_grid(start)
        goal_grid = self.world_to_grid(goal)

        # Simple A* or Dijkstra implementation
        # (Simplified for example)
        path = self.a_star_search(start_grid, goal_grid)

        return path

    def world_to_grid(self, pose):
        """Convert world coordinates to grid coordinates"""
        grid_x = int((pose.position.x - self.origin.position.x) / self.resolution)
        grid_y = int((pose.position.y - self.origin.position.y) / self.resolution)
        return (grid_x, grid_y)
```

## Best Practices for Python-based Agents

### Performance Optimization

1. **Threading**: Use appropriate threading models for performance
2. **Memory Management**: Be mindful of memory usage in long-running nodes
3. **Message Efficiency**: Use efficient message serialization
4. **Callback Design**: Keep callbacks lightweight and responsive

### Error Handling and Robustness

1. **Graceful Degradation**: Agents should continue operating when possible
2. **Timeout Handling**: Implement timeouts for blocking operations
3. **Exception Handling**: Properly handle exceptions to prevent crashes
4. **State Recovery**: Implement state recovery mechanisms

### Testing and Validation

1. **Unit Testing**: Test individual components
2. **Integration Testing**: Test component interactions
3. **Simulation Testing**: Test in simulated environments
4. **Real-world Validation**: Validate in real environments

## Multi-Agent Systems

Python-based ROS 2 agents can coordinate with each other:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile

class CoordinatorAgent(Node):
    def __init__(self):
        super().__init__('coordinator_agent')

        # Publishers for different agents
        self.agent1_pub = self.create_publisher(String, 'agent1_commands', 10)
        self.agent2_pub = self.create_publisher(String, 'agent2_commands', 10)

        # Subscribers for agent status
        self.agent1_status = self.create_subscription(
            String, 'agent1_status', self.agent1_status_callback, 10)
        self.agent2_status = self.create_subscription(
            String, 'agent2_status', self.agent2_status_callback, 10)

        self.agents_status = {'agent1': 'idle', 'agent2': 'idle'}

    def coordinate_agents(self):
        """Coordinate multiple agents for a task"""
        if self.agents_status['agent1'] == 'idle' and self.agents_status['agent2'] == 'idle':
            # Assign tasks to agents
            cmd1 = String()
            cmd1.data = 'navigate_to_location_1'
            cmd2 = String()
            cmd2.data = 'navigate_to_location_2'

            self.agent1_pub.publish(cmd1)
            self.agent2_pub.publish(cmd2)
```

## Learning Objectives

After studying this chapter, students should be able to:

1. Implement ROS 2 nodes using the rclpy library
2. Design different types of intelligent agents (reactive, deliberative, hybrid)
3. Integrate machine learning models with ROS 2 agents
4. Apply advanced patterns like parameter management and lifecycle management
5. Create multi-agent coordination systems
6. Implement proper error handling and testing strategies

## Prerequisites

- Basic Python programming skills
- Understanding of object-oriented programming
- Familiarity with ROS 2 concepts
- Elementary knowledge of AI and machine learning concepts

## References

1. Quigley, M., Gerkey, B., & Smart, W. D. (2022). *Programming Robots with ROS: A Practical Introduction to the Robot Operating System*. O'Reilly Media.
2. Macenski, S. (2022). *ROS2 in Action*. Available online through ROS community resources.
3. Choset, H., Lynch, K. M., Hutchinson, S., Kantor, G., Burgard, W., Kavraki, L. E., & Thrun, S. (2005). *Principles of Robot Motion: Theory, Algorithms, and Implementations*. MIT Press.
4. Russell, S., & Norvig, P. (2020). *Artificial Intelligence: A Modern Approach* (4th ed.). Pearson.

## Exercises

1. Implement a reactive obstacle avoidance agent using laser scan data
2. Create a deliberative navigation agent with path planning capabilities
3. Design a hybrid agent that combines multiple behaviors
4. Integrate a machine learning model for object detection in a ROS 2 node
5. Implement a multi-agent coordination system for cooperative tasks
6. Create a parameterized agent that can be reconfigured at runtime