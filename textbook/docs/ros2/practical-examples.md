---
sidebar_position: 7
title: Practical Examples and Exercises
---

# Practical Examples and Exercises: ROS 2 Integration for Humanoid Robotics

## Introduction

This chapter provides practical, hands-on examples and exercises to reinforce the concepts covered in previous chapters. The examples are designed to bridge the gap between theoretical knowledge and practical implementation, focusing on real-world scenarios that humanoid robotics developers commonly encounter.

Each example includes step-by-step instructions, complete code implementations, and exercises that build upon the concepts learned. The exercises are designed to progressively increase in complexity, allowing students to develop comprehensive skills in ROS 2 integration for humanoid robots.

## Example 1: Simple Navigation Node

### Problem Statement
Create a ROS 2 node that subscribes to laser scan data and publishes velocity commands to navigate safely around obstacles.

### Implementation

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class SimpleNavigationNode(Node):
    def __init__(self):
        super().__init__('simple_navigation')

        # Create publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create subscriber for laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        # Parameters
        self.safe_distance = 0.5  # meters
        self.linear_speed = 0.3   # m/s
        self.angular_speed = 0.5  # rad/s

        self.get_logger().info('Simple Navigation Node initialized')

    def scan_callback(self, msg):
        """Process laser scan data and generate navigation commands"""
        # Find minimum distance in front of robot
        min_distance = min(msg.ranges)

        # Create twist message
        cmd = Twist()

        if min_distance < self.safe_distance:
            # Obstacle detected - turn
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed
            self.get_logger().info(f'Obstacle detected! Distance: {min_distance:.2f}m, turning...')
        else:
            # Clear path - move forward
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
            self.get_logger().info(f'Clear path. Moving forward at {self.linear_speed} m/s')

        # Publish command
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Navigation node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Launch File

Create a launch file to run the navigation node:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simple_navigation',
            executable='simple_navigation',
            name='simple_navigation',
            parameters=[
                {'safe_distance': 0.5},
                {'linear_speed': 0.3},
                {'angular_speed': 0.5}
            ],
            output='screen'
        )
    ])
```

### Exercise 1.1: Enhanced Obstacle Avoidance
Modify the simple navigation node to implement a more sophisticated obstacle avoidance algorithm that:
1. Looks at a wider range of laser readings (not just the minimum)
2. Implements proportional control for smoother turning
3. Adds hysteresis to prevent oscillation near obstacle boundaries

### Exercise 1.2: Parameter Configuration
Extend the node to accept parameters for:
- Safe distance threshold
- Linear and angular speed limits
- Turning behavior (aggressive vs. conservative)
- Minimum turning radius

## Example 2: Action-Based Navigation

### Problem Statement
Implement a navigation system using ROS 2 actions that allows external clients to send navigation goals and receive feedback and results.

### Implementation

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
import threading
import math

class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_action_server')

        # Create action server
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, 'current_pose', self.pose_callback, 10
        )

        # Robot state
        self.current_pose = None
        self.navigation_active = False
        self.nav_thread = None

        self.get_logger().info('Navigation Action Server initialized')

    def goal_callback(self, goal_request):
        """Accept or reject navigation goal"""
        self.get_logger().info(f'Received navigation goal: {goal_request.pose.pose}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject cancel request"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def pose_callback(self, msg):
        """Update current robot pose"""
        self.current_pose = msg.pose

    def execute_callback(self, goal_handle):
        """Execute navigation goal"""
        self.get_logger().info('Executing navigation goal')

        # Set navigation active
        self.navigation_active = True

        # Extract goal position
        goal_x = goal_handle.request.pose.pose.position.x
        goal_y = goal_handle.request.pose.pose.position.y

        # Navigation loop
        feedback_msg = NavigateToPose.Feedback()
        result = NavigateToPose.Result()

        while self.navigation_active:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                result.result = result.CANCELED
                return result

            if self.current_pose is None:
                continue

            # Calculate distance to goal
            current_x = self.current_pose.position.x
            current_y = self.current_pose.position.y
            distance = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)

            # Check if reached
            if distance < 0.2:  # 20cm tolerance
                self.stop_robot()
                goal_handle.succeed()
                result.result = result.SUCCEEDED
                self.get_logger().info('Navigation succeeded')
                return result

            # Generate command to move toward goal
            cmd = Twist()
            cmd.linear.x = min(0.3, distance)  # Proportional speed
            cmd.angular.z = self.calculate_angular_command(goal_x, goal_y)

            # Publish command
            self.cmd_vel_pub.publish(cmd)

            # Publish feedback
            feedback_msg.current_pose = PoseStamped()
            feedback_msg.distance_remaining = distance
            goal_handle.publish_feedback(feedback_msg)

            # Sleep to control loop rate
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))

        result.result = result.CANCELED
        return result

    def calculate_angular_command(self, goal_x, goal_y):
        """Calculate angular command to face goal"""
        if self.current_pose is None:
            return 0.0

        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        # Calculate angle to goal
        angle_to_goal = math.atan2(goal_y - current_y, goal_x - current_x)

        # Get current orientation (simplified)
        current_yaw = 0.0  # In real implementation, use orientation from pose

        # Calculate angular error
        angular_error = angle_to_goal - current_yaw

        # Normalize to [-pi, pi]
        while angular_error > math.pi:
            angular_error -= 2 * math.pi
        while angular_error < -math.pi:
            angular_error += 2 * math.pi

        return angular_error * 0.5  # Proportional control

    def stop_robot(self):
        """Stop robot movement"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        self.navigation_active = False

def main(args=None):
    rclpy.init(args=args)

    node = NavigationActionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Navigation server stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Client Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

    def send_goal(self, x, y):
        """Send navigation goal"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self._action_client.wait_for_server()
        self.get_logger().info(f'Sending navigation goal to ({x}, {y})')

        # Send goal and wait for result
        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle feedback"""
        self.get_logger().info(
            f'Distance remaining: {feedback_msg.feedback.distance_remaining:.2f}m'
        )

    def result_callback(self, future):
        """Handle result"""
        result = future.result().result
        if result.result == result.SUCCEEDED:
            self.get_logger().info('Navigation succeeded!')
        else:
            self.get_logger().info('Navigation failed')

def main(args=None):
    rclpy.init(args=args)
    client = NavigationClient()

    # Send a navigation goal
    client.send_goal(2.0, 2.0)

    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Exercise 2.1: Advanced Navigation with Obstacle Avoidance
Enhance the navigation action server to:
1. Integrate laser scan data for obstacle avoidance during navigation
2. Implement a local planner that can navigate around obstacles while still heading toward the goal
3. Add recovery behaviors for when the robot gets stuck

### Exercise 2.2: Multi-Goal Navigation
Extend the navigation system to:
1. Accept a sequence of goals
2. Execute them in order
3. Provide overall progress feedback
4. Handle cancellation of the entire sequence

## Example 3: Sensor Integration and State Estimation

### Problem Statement
Create a ROS 2 node that integrates data from multiple sensors (IMU, encoders, camera) to estimate robot state using a Kalman filter.

### Implementation

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
from scipy.linalg import block_diag

class StateEstimationNode(Node):
    def __init__(self):
        super().__init__('state_estimation')

        # State vector: [x, y, theta, vx, vy, omega]
        self.state = np.zeros(6)
        self.covariance = np.eye(6) * 0.1  # Initial uncertainty

        # Process noise matrix
        self.Q = np.eye(6) * 0.01

        # Measurement noise matrices
        self.R_imu = np.diag([0.01, 0.01, 0.01])  # orientation noise
        self.R_odom = np.diag([0.1, 0.1, 0.05])   # position noise
        self.R_twist = np.diag([0.05, 0.05, 0.01]) # velocity noise

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )

        # Publisher
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'estimated_pose', 10
        )

        # Timer for prediction update
        self.timer = self.create_timer(0.02, self.prediction_step)  # 50Hz

        self.get_logger().info('State Estimation Node initialized')

    def prediction_step(self):
        """Prediction step of Kalman filter"""
        dt = 0.02  # 50Hz update rate

        # State transition matrix (constant velocity model)
        F = np.eye(6)
        F[0, 3] = dt  # x = x + vx*dt
        F[1, 4] = dt  # y = y + vy*dt
        F[2, 5] = dt  # theta = theta + omega*dt

        # Control input matrix (simplified)
        B = np.zeros((6, 3))  # No direct control input

        # Predict state
        self.state = F @ self.state

        # Predict covariance
        self.covariance = F @ self.covariance @ F.T + self.Q

        # Publish estimated pose
        self.publish_estimated_pose()

    def imu_callback(self, msg):
        """Handle IMU measurements"""
        # Extract orientation from IMU
        orientation = msg.orientation
        theta = self.quaternion_to_yaw(orientation)

        # Measurement vector [theta]
        z = np.array([theta])

        # Measurement matrix for orientation
        H = np.zeros((1, 6))
        H[0, 2] = 1.0  # theta is third state variable

        # Innovation
        y = z - H @ self.state
        y = self.normalize_angle(y[0])  # Normalize angle difference
        y = np.array([y])

        # Innovation covariance
        S = H @ self.covariance @ H.T + self.R_imu[0:1, 0:1]

        # Kalman gain
        K = self.covariance @ H.T @ np.linalg.inv(S)

        # Update state
        self.state = self.state + K @ y

        # Update covariance
        I_KH = np.eye(6) - K @ H
        self.covariance = I_KH @ self.covariance

    def odom_callback(self, msg):
        """Handle odometry measurements"""
        # Extract position and velocity from odometry
        position = msg.pose.pose.position
        twist = msg.twist.twist

        # Measurement vector [x, y, vx, vy]
        z = np.array([
            position.x,
            position.y,
            twist.linear.x,
            twist.linear.y
        ])

        # Measurement matrix for position and velocity
        H = np.zeros((4, 6))
        H[0, 0] = 1.0  # x position
        H[1, 1] = 1.0  # y position
        H[2, 3] = 1.0  # x velocity
        H[3, 4] = 1.0  # y velocity

        # Innovation
        y = z - H @ self.state

        # Innovation covariance
        S = H @ self.covariance @ H.T + block_diag(self.R_odom[0:2, 0:2], self.R_twist[0:2, 0:2])

        # Kalman gain
        K = self.covariance @ H.T @ np.linalg.inv(S)

        # Update state
        self.state = self.state + K @ y

        # Update covariance
        I_KH = np.eye(6) - K @ H
        self.covariance = I_KH @ self.covariance

    def joint_state_callback(self, msg):
        """Handle joint state measurements for odometry"""
        # This would integrate wheel encoder data
        # For simplicity, we'll use the odometry callback instead
        pass

    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def publish_estimated_pose(self):
        """Publish estimated pose with covariance"""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        # Fill pose
        msg.pose.pose.position.x = self.state[0]
        msg.pose.pose.position.y = self.state[1]
        msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        from tf_transformations import quaternion_from_euler
        quat = quaternion_from_euler(0, 0, self.state[2])
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]

        # Fill covariance (flattened 6x6 matrix)
        for i in range(6):
            for j in range(6):
                msg.pose.covariance[i*6 + j] = self.covariance[i, j]

        self.pose_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StateEstimationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('State estimation node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Exercise 3.1: Extended Kalman Filter
Modify the state estimation node to implement an Extended Kalman Filter (EKF) that can handle:
1. Non-linear motion models
2. Non-linear measurement models
3. More complex state representations (e.g., including acceleration)

### Exercise 3.2: Particle Filter Implementation
Implement an alternative state estimation approach using a particle filter that:
1. Handles multi-modal distributions
2. Works better with non-Gaussian noise
3. Can represent uncertainty in complex environments

## Example 4: Multi-Node Coordination

### Problem Statement
Create a system of coordinated ROS 2 nodes that implement a complete humanoid robot control system with separate nodes for perception, planning, and control.

### Implementation

```python
# perception_node.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # Subscribers for sensor data
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10
        )
        self.camera_sub = self.create_subscription(
            Image, 'camera/image_raw', self.camera_callback, 10
        )

        # Publishers for processed data
        self.obstacle_pub = self.create_publisher(
            PointStamped, 'obstacles', 10
        )
        self.object_pub = self.create_publisher(
            String, 'detected_objects', 10
        )

        self.get_logger().info('Perception Node initialized')

    def laser_callback(self, msg):
        """Process laser scan to detect obstacles"""
        # Simple obstacle detection
        min_distance = min(msg.ranges)

        if min_distance < 1.0:  # Obstacle within 1m
            # Publish obstacle location (simplified)
            obstacle_msg = PointStamped()
            obstacle_msg.header.stamp = self.get_clock().now().to_msg()
            obstacle_msg.header.frame_id = 'base_link'

            # Find angle of closest obstacle
            min_idx = msg.ranges.index(min_distance)
            angle = msg.angle_min + min_idx * msg.angle_increment

            obstacle_msg.point.x = min_distance * math.cos(angle)
            obstacle_msg.point.y = min_distance * math.sin(angle)
            obstacle_msg.point.z = 0.0

            self.obstacle_pub.publish(obstacle_msg)

    def camera_callback(self, msg):
        """Process camera image to detect objects"""
        # In a real implementation, this would use computer vision
        # For now, we'll simulate object detection
        detected_objects = ['person', 'chair', 'table']  # Simulated detection

        for obj in detected_objects:
            obj_msg = String()
            obj_msg.data = obj
            self.object_pub.publish(obj_msg)

# planning_node.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import String
from visualization_msgs.msg import Marker

class PlanningNode(Node):
    def __init__(self):
        super().__init__('planning_node')

        # Subscribers
        self.obstacle_sub = self.create_subscription(
            PointStamped, 'obstacles', self.obstacle_callback, 10
        )
        self.object_sub = self.create_subscription(
            String, 'detected_objects', self.object_callback, 10
        )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.viz_pub = self.create_publisher(Marker, 'planning_viz', 10)

        # Planning state
        self.obstacles = []
        self.objects = []

        self.get_logger().info('Planning Node initialized')

    def obstacle_callback(self, msg):
        """Handle obstacle detection"""
        self.obstacles.append({
            'position': (msg.point.x, msg.point.y),
            'timestamp': msg.header.stamp
        })

        # Plan avoidance behavior
        self.plan_avoidance()

    def object_callback(self, msg):
        """Handle object detection"""
        self.objects.append({
            'name': msg.data,
            'timestamp': self.get_clock().now().to_msg()
        })

    def plan_avoidance(self):
        """Plan obstacle avoidance"""
        if not self.obstacles:
            return

        # Simple avoidance planning
        cmd = Twist()

        # Get most recent obstacle
        latest_obstacle = self.obstacles[-1]
        obs_x, obs_y = latest_obstacle['position']
        distance = math.sqrt(obs_x**2 + obs_y**2)

        if distance < 0.5:  # Very close - stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5 if obs_y > 0 else -0.5  # Turn away
        elif distance < 1.0:  # Close - slow down and turn
            cmd.linear.x = 0.1
            cmd.angular.z = 0.3 if obs_y > 0 else -0.3
        else:  # Far - continue normally
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

        # Visualize plan
        self.visualize_plan(cmd)

    def visualize_plan(self, cmd):
        """Visualize the planned motion"""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'base_link'
        marker.ns = 'planned_motion'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Set arrow start and end points
        marker.points = []
        start_point = Point()
        start_point.x = 0.0
        start_point.y = 0.0
        start_point.z = 0.0
        marker.points.append(start_point)

        end_point = Point()
        end_point.x = cmd.linear.x * 0.5  # Scale for visualization
        end_point.y = cmd.angular.z * 0.1  # Scale for visualization
        end_point.z = 0.0
        marker.points.append(end_point)

        marker.scale.x = 0.05  # Arrow shaft diameter
        marker.scale.y = 0.1   # Arrow head diameter
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.viz_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Planning node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Exercise 4.1: Behavior Tree Integration
Implement a behavior tree for the planning node that:
1. Defines different behaviors (explore, avoid obstacles, approach objects)
2. Switches between behaviors based on sensor input
3. Implements fallback behaviors when primary plans fail

### Exercise 4.2: Multi-Robot Coordination
Extend the system to coordinate multiple robots by:
1. Adding communication between robots
2. Implementing a leader-follower pattern
3. Creating a formation control system

## Exercise Solutions and Best Practices

### Common Patterns

#### 1. Parameter Management
```python
# Always use parameters for configurable values
class ConfigurableNode(Node):
    def __init__(self):
        super().__init__('configurable_node')

        # Declare parameters with defaults
        self.declare_parameter('safety_distance', 0.5)
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.5)

        # Access parameters
        self.safety_distance = self.get_parameter('safety_distance').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
```

#### 2. Error Handling
```python
# Robust error handling
def safe_divide(self, numerator, denominator):
    """Safely divide with error handling"""
    try:
        if abs(denominator) < 1e-6:
            self.get_logger().warn('Division by near-zero value')
            return 0.0
        return numerator / denominator
    except ZeroDivisionError:
        self.get_logger().error('Division by zero')
        return 0.0
    except Exception as e:
        self.get_logger().error(f'Unexpected error in division: {e}')
        return 0.0
```

#### 3. Threading Considerations
```python
import threading

class ThreadSafeNode(Node):
    def __init__(self):
        super().__init__('thread_safe_node')
        self.lock = threading.Lock()
        self.shared_data = {'value': 0}

    def update_shared_data(self, new_value):
        """Thread-safe data update"""
        with self.lock:
            self.shared_data['value'] = new_value
```

## Project: Complete Humanoid Robot System

### Problem Statement
Design and implement a complete humanoid robot system that integrates:
1. Sensor processing (LiDAR, camera, IMU)
2. State estimation and localization
3. Path planning and navigation
4. Control and actuation
5. High-level task execution

### Requirements
- Use ROS 2 with appropriate message types
- Implement at least 3 different node types
- Include parameter configuration
- Add visualization capabilities
- Implement safety features
- Provide logging and diagnostics

### Evaluation Criteria
1. **Modularity**: Components should be independent and reusable
2. **Robustness**: System should handle sensor failures gracefully
3. **Performance**: Real-time constraints should be met
4. **Safety**: Emergency stop and protection mechanisms required
5. **Documentation**: Code should be well-documented

## Learning Objectives

After completing these examples and exercises, students should be able to:

1. Create complete ROS 2 nodes with proper initialization and lifecycle management
2. Implement different communication patterns (topics, services, actions)
3. Integrate multiple sensors for robust state estimation
4. Design coordinated multi-node systems
5. Apply best practices for parameter management and error handling
6. Implement safety and diagnostic features

## Prerequisites

- Basic understanding of ROS 2 concepts
- Python programming skills
- Familiarity with robotics concepts
- Understanding of control systems

## References

1. Quigley, M., Gerkey, B., & Smart, W. D. (2022). *Programming Robots with ROS: A Practical Introduction to the Robot Operating System*. O'Reilly Media.
2. Open Robotics. (2023). *ROS 2 Documentation*. Available: https://docs.ros.org/
3. Siciliano, B., & Khatib, O. (2016). *Springer Handbook of Robotics*. Springer. Chapter on Robot Software.
4. Macenski, S. (2022). *Effective ROS 2: Patterns, Tools, and Tips for Building Reliable Robotic Systems*. Available online.

## Self-Assessment Questions

1. How would you modify the navigation example to handle dynamic obstacles?
2. What are the advantages and disadvantages of using actions vs. topics for navigation?
3. How would you implement a Kalman filter for a 6-DOF humanoid robot?
4. What safety considerations would you add to the multi-node system?
5. How would you extend the system to work with a real humanoid robot?