---
sidebar_position: 3
title: "Appendix C: Isaac ROS Tutorials"
---

# Appendix C: Isaac ROS Tutorials

## Overview of Isaac ROS

Isaac ROS is NVIDIA's collection of hardware-accelerated packages that bring the power of NVIDIA's computing platforms to the Robot Operating System (ROS 2). These packages provide GPU-accelerated perception, navigation, and manipulation capabilities for robotics applications.

## Installation

### Prerequisites
- NVIDIA GPU with CUDA support (Compute Capability 6.0+)
- Ubuntu 20.04 or 22.04
- ROS 2 Humble Hawksbill
- NVIDIA Container Toolkit

### Setup
```bash
# Install Isaac ROS Developer Tools
sudo apt update
sudo apt install nvidia-isaa-ros-dev-tools

# Or install specific packages individually
sudo apt install nvidia-isaac-ros-perceptor
sudo apt install nvidia-isaac-ros-isaac-ros-nav2
```

## Getting Started with Isaac ROS Packages

### Isaac ROS Apriltag

Apriltag detection accelerated on GPU for precise localization.

```cpp
#include <rclcpp/rclcpp.hpp>
#include <isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp>

class ApriltagNode : public rclcpp::Node
{
public:
    ApriltagNode() : Node("apriltag_node")
    {
        subscription_ = this->create_subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>(
            "tag_detections", 10,
            std::bind(&ApriltagNode::detection_callback, this, std::placeholders::_1));
    }

private:
    void detection_callback(const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Detected %zu tags", msg->detections.size());
        
        for (const auto& detection : msg->detections) {
            RCLCPP_INFO(this->get_logger(), 
                "Tag ID: %d, Position: (%.2f, %.2f, %.2f)",
                detection.id, 
                detection.pose.pose.position.x,
                detection.pose.pose.position.y,
                detection.pose.pose.position.z);
        }
    }
    
    rclcpp::Subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>::SharedPtr subscription_;
};
```

### Isaac ROS Stereo Image Processing

Real-time stereo processing accelerated on GPU.

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage

class StereoProcessorNode(Node):
    def __init__(self):
        super().__init__('stereo_processor')
        
        # Subscribe to stereo image topics
        self.left_sub = self.create_subscription(
            Image, '/camera/left/image_rect', self.left_callback, 10)
        self.right_sub = self.create_subscription(
            Image, '/camera/right/image_rect', self.right_callback, 10)
            
        # Publish disparity map
        self.disp_pub = self.create_publisher(
            DisparityImage, '/disparity_map', 10)
    
    def left_callback(self, msg):
        # Process left image
        self.get_logger().info(f"Received left image: {msg.width}x{msg.height}")
    
    def right_callback(self, msg):
        # Process right image
        self.get_logger().info(f"Received right image: {msg.width}x{msg.height}")
```

### Isaac ROS Visual Simultaneous Localization and Mapping (VSLAM)

```cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class VSLAMNode : public rclcpp::Node
{
public:
    VSLAMNode() : Node("vslam_node")
    {
        // Subscribe to camera feed
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image", 10,
            std::bind(&VSLAMNode::image_callback, this, std::placeholders::_1));
            
        // Publish odometry
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        
        // Publish pose
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Process visual SLAM with Isaac ROS VSLAM
        // This would interface with Isaac ROS VSLAM package
        RCLCPP_INFO(this->get_logger(), 
            "Processing frame for VSLAM: %dx%d", 
            msg->width, msg->height);
            
        // Publish odometry result
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = this->get_clock()->now();
        odom_msg.header.frame_id = "map";
        odom_publisher_->publish(odom_msg);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
};
```

## Isaac ROS Navigation

GPU-accelerated navigation stack for mobile robots.

### Basic Navigation Node
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class IsaacNavigationNode(Node):
    def __init__(self):
        super().__init__('isaac_navigation_node')
        
        # Create action client for navigation
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')
        
        # Create publisher for goal poses
        self.goal_publisher = self.create_publisher(
            PoseStamped, 'goal_pose', 10)
    
    def send_goal(self, x, y, z, w=1.0):
        """Send navigation goal to Isaac ROS Navigation"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = z
        goal_msg.pose.pose.orientation.w = w
        
        # Wait for action server
        self.nav_client.wait_for_server()
        
        # Send goal
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
            
        self.get_logger().info('Goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        self.get_logger().info(f'Navigation completed: {result}')
```

## Isaac ROS Object Detection

### YOLO-based Object Detection with Isaac ROS
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Header

class IsaacObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('isaac_object_detection_node')
        
        # Subscribe to camera feed
        self.image_subscription = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        
        # Publish detections
        self.detection_publisher = self.create_publisher(
            Detection2DArray, 'detections', 10)
    
    def image_callback(self, msg: Image):
        """
        Process image and detect objects using Isaac ROS
        This is a simplified example - real implementation would interface
        with Isaac ROS object detection packages
        """
        # Simulate object detection results
        detection_array = Detection2DArray()
        detection_array.header = Header()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = msg.header.frame_id
        
        # Example detection (in reality, this would come from Isaac ROS AI model)
        if True:  # Condition would be based on actual detection
            detection = Detection2D()
            detection.header = detection_array.header
            
            # Bounding box (normalized coordinates)
            bbox = BoundingBox2D()
            bbox.center.x = 0.5
            bbox.center.y = 0.5
            bbox.size_x = 0.2
            bbox.size_y = 0.3
            detection.bbox = bbox
            
            # Class and confidence
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = "person"
            hypothesis.score = 0.95
            detection.results.append(hypothesis)
            
            detection_array.detections.append(detection)
        
        # Publish detections
        self.detection_publisher.publish(detection_array)
        self.get_logger().info(f'Published {len(detection_array.detections)} detections')
```

## Isaac ROS with Isaac Sim

### Connecting Isaac ROS to Isaac Sim
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import Twist
import numpy as np

class IsaacSimBridgeNode(Node):
    def __init__(self):
        super().__init__('isaac_sim_bridge')
        
        # Publishers for Isaac Sim
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers from Isaac Sim
        self.rgb_subscription = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.rgb_callback, 10)
            
        self.depth_subscription = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10)
            
        self.camera_info_subscription = self.create_subscription(
            CameraInfo, '/camera/rgb/camera_info', self.camera_info_callback, 10)
    
    def rgb_callback(self, msg: Image):
        """Process RGB camera data from Isaac Sim"""
        # Convert ROS Image to format usable by Isaac ROS packages
        # Process with Isaac ROS perception algorithms
        self.get_logger().info(f'Received RGB image: {msg.width}x{msg.height}')
    
    def depth_callback(self, msg: Image):
        """Process depth camera data from Isaac Sim"""
        # Process depth data with Isaac ROS algorithms
        self.get_logger().info(f'Received depth image: {msg.width}x{msg.height}')
    
    def camera_info_callback(self, msg: CameraInfo):
        """Process camera calibration data"""
        # Use calibration data for Isaac ROS stereo or monocular processing
        self.get_logger().info(f'Camera calibration received')
    
    def send_velocity_command(self, linear_x: float, angular_z: float):
        """Send velocity command to robot in Isaac Sim"""
        cmd_msg = Twist()
        cmd_msg.linear.x = linear_x
        cmd_msg.angular.z = angular_z
        self.cmd_vel_publisher.publish(cmd_msg)
```

## Launch Files for Isaac ROS

### Isaac ROS Stereo Example Launch
```python
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Isaac ROS Stereo Disparity container
    stereo_disparity_container = ComposableNodeContainer(
        name='stereo_disparity_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_stereo_image_proc',
                plugin='isaac_ros::stereo_image_proc::DisparityNode',
                name='disparity_node',
                parameters=[{
                    'approximate_sync': True,
                    'use_system_default_qos': True
                }],
                remappings=[
                    ('left/image_rect', '/camera/left/image_rect'),
                    ('right/image_rect', '/camera/right/image_rect'),
                    ('left/camera_info', '/camera/left/camera_info'),
                    ('right/camera_info', '/camera/right/camera_info'),
                ]
            )
        ]
    )

    return LaunchDescription([stereo_disparity_container])
```

### Isaac ROS AprilTag Example Launch
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'image_width',
            default_value='640',
            description='Width of input images'),
        DeclareLaunchArgument(
            'image_height', 
            default_value='480',
            description='Height of input images'),
    ]

    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')

    apriltag_container = ComposableNodeContainer(
        name='apriltag_container',
        namespace='apriltag',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_apriltag',
                plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
                name='apriltag_node',
                parameters=[{
                    'size': 0.32,  # Tag size in meters
                    'max_tags': 16,
                    'family': 'tag36h11',
                }],
                remappings=[
                    ('image', '/image_rect'),
                    ('camera_info', '/camera_info'),
                ]
            )
        ],
        output='screen'
    )

    return LaunchDescription(
        launch_args + [apriltag_container]
    )
```

## TensorRT Integration with Isaac ROS

### Using TensorRT for Optimized Inference
```python
import rclpy
from rclpy.node import Node
import torch
import tensorrt as trt
import pycuda.driver as cuda
import numpy as np

class TensorRTInferenceNode(Node):
    def __init__(self):
        super().__init__('tensorrt_inference_node')
        
        # Initialize TensorRT engine
        self.engine = self.load_tensorrt_engine('/path/to/model.plan')
        self.context = self.engine.create_execution_context()
        
        # Allocate buffers
        self.inputs, self.outputs, self.bindings, self.stream = self.allocate_buffers()
        
    def load_tensorrt_engine(self, engine_path):
        """Load a pre-built TensorRT engine"""
        with open(engine_path, 'rb') as f:
            engine_data = f.read()
        
        runtime = trt.Runtime(trt.Logger(trt.Logger.WARNING))
        engine = runtime.deserialize_cuda_engine(engine_data)
        return engine
    
    def allocate_buffers(self):
        """Allocate buffers for TensorRT engine"""
        inputs = []
        outputs = []
        bindings = []
        stream = cuda.Stream()
        
        for idx in range(self.engine.num_bindings):
            print(f"Binding {idx}: {self.engine.get_binding_name(idx)}")
            print(f"Binding {idx} shape: {self.engine.get_binding_shape(idx)}")
            
            binding_shape = self.engine.get_binding_shape(idx)
            size = trt.volume(binding_shape) * self.engine.max_batch_size * np.dtype(np.float32).itemsize
            
            # Allocate host and device buffers
            host_mem = cuda.pagelocked_empty(size, dtype=np.float32)
            device_mem = cuda.mem_alloc(host_mem.nbytes)
            
            bindings.append(int(device_mem))
            
            if self.engine.binding_is_input(idx):
                inputs.append({'host': host_mem, 'device': device_mem})
            else:
                outputs.append({'host': host_mem, 'device': device_mem})
        
        return inputs, outputs, bindings, stream
    
    def do_inference(self, input_data):
        """Perform TensorRT inference"""
        # Copy input data to GPU
        np.copyto(self.inputs[0]['host'], input_data.ravel())
        cuda.memcpy_htod_async(self.inputs[0]['device'], self.inputs[0]['host'], self.stream)
        
        # Execute inference
        self.context.execute_async_v2(bindings=self.bindings, stream_handle=self.stream.handle)
        
        # Copy output data back to CPU
        cuda.memcpy_dtoh_async(self.outputs[0]['host'], self.outputs[0]['device'], self.stream)
        self.stream.synchronize()
        
        return self.outputs[0]['host'].copy()
```

## Best Practices with Isaac ROS

### Performance Optimization
1. **Use appropriate QoS settings** for real-time performance
2. **Minimize data copying** between host and device memory
3. **Batch operations** where possible to maximize GPU utilization
4. **Profile your application** to identify bottlenecks

### Safety Considerations
1. **Implement safety checks** before executing actions
2. **Validate sensor data** before using it for navigation
3. **Monitor GPU utilization** and temperature
4. **Implement fallback strategies** when acceleration is not available

### Resource Management
1. **Manage GPU memory** carefully to avoid out-of-memory errors
2. **Use appropriate precision** (FP16 vs FP32) based on application needs
3. **Monitor power consumption** for mobile robots
4. **Implement proper cleanup** of GPU resources