---
sidebar_position: 1
title: ROS 2 Architecture
---

# ROS 2 Architecture: The Robotic Nervous System

## Introduction

Robot Operating System 2 (ROS 2) serves as the nervous system for modern robotic applications, providing a flexible framework for communication between different software components. Unlike its predecessor, ROS 2 addresses the needs of production robotics with improved real-time capabilities, security features, and robust communication mechanisms.

This chapter explores the core architectural concepts of ROS 2, including its communication patterns, middleware implementation, and the fundamental building blocks that enable complex robotic systems to function as integrated wholes.

## Evolution from ROS 1 to ROS 2

### Limitations of ROS 1

ROS 1, while revolutionary for robotics research, had several limitations that made it unsuitable for production systems:

1. **Single Master Architecture**: Centralized master node created a single point of failure
2. **Lack of Real-time Support**: No deterministic communication for time-critical applications
3. **Security Limitations**: No built-in authentication or encryption mechanisms
4. **Limited Multi-robot Support**: Difficult to coordinate multiple robots reliably
5. **Middleware Limitations**: Tied to a specific communication system (TCPROS/UDPROS)

### ROS 2 Solutions

ROS 2 addresses these limitations through:

1. **Distributed Architecture**: No single master node; uses peer-to-peer communication
2. **Real-time Support**: Implements real-time communication patterns and quality of service (QoS) policies
3. **Security Features**: Built-in authentication, encryption, and access control
4. **Improved Multi-robot Support**: Better coordination and communication between multiple robots
5. **Middleware Abstraction**: Uses Data Distribution Service (DDS) for flexible communication

## Core Communication Concepts

### Nodes: The Fundamental Units

In ROS 2, a **node** is a process that performs computation. Nodes are the basic building blocks of a ROS 2 system and are designed to be modular and reusable. Each node typically handles a specific task or function within the robotic system.

#### Node Characteristics

- **Modularity**: Each node performs a specific function
- **Communication**: Nodes communicate through topics, services, and actions
- **Isolation**: Failure of one node doesn't necessarily affect others
- **Configurability**: Nodes can be configured with parameters
- **Lifecycle**: Nodes can have explicit lifecycle management

#### Creating Nodes

Nodes in ROS 2 are implemented as classes that inherit from the `rclcpp::Node` (C++) or `rclpy.Node` (Python) base class. This provides access to ROS 2's communication infrastructure.

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        # Node initialization code here
```

### Topics: Publish-Subscribe Communication

**Topics** implement a publish-subscribe communication pattern where publishers send messages to a topic and subscribers receive messages from the topic. This is an asynchronous, decoupled communication method ideal for sensor data and continuous streams.

#### Topic Characteristics

- **Asynchronous**: Publishers and subscribers don't need to be synchronized
- **Decoupled**: Publishers don't know about subscribers and vice versa
- **Broadcast**: One publisher can send to multiple subscribers
- **Type Safety**: Messages must conform to defined message types

#### Quality of Service (QoS) in Topics

ROS 2 provides QoS policies for topics to handle different communication requirements:

- **Reliability**: Best effort vs. reliable delivery
- **Durability**: Volatile vs. transient local (replay last message to new subscribers)
- **History**: Keep last N messages vs. keep all messages
- **Deadline**: Maximum time between messages
- **Liveliness**: How to detect if a publisher is alive

### Services: Request-Response Communication

**Services** implement a request-response communication pattern where a client sends a request and waits for a response from a server. This is synchronous communication suitable for operations that require a specific response.

#### Service Characteristics

- **Synchronous**: Client waits for response
- **Request-Response**: One request, one response pattern
- **Stateless**: Each request is independent
- **Blocking**: Client is blocked until response is received

#### Service Implementation

Services are defined using `.srv` files that specify the request and response message types. The service server handles requests and returns responses, while the client sends requests and receives responses.

### Actions: Goal-Feedback-Result Communication

**Actions** provide a communication pattern for long-running tasks that require feedback during execution. They combine the benefits of services and topics, providing goal specification, continuous feedback, and final results.

#### Action Characteristics

- **Long-running**: Designed for tasks that take significant time
- **Feedback**: Continuous feedback during execution
- **Cancelation**: Ability to cancel ongoing actions
- **Preemption**: Ability to replace current goal with new one

#### Action Implementation

Actions are defined using `.action` files that specify the goal, feedback, and result message types. The action server handles goals, provides feedback, and returns results, while the action client sends goals and receives feedback and results.

## Middleware Abstraction: Data Distribution Service (DDS)

### DDS Overview

ROS 2 uses DDS as its underlying middleware, providing a standardized communication infrastructure. DDS stands for Data Distribution Service and is defined by the Object Management Group (OMG).

### Key DDS Concepts

- **Data-Centricity**: Focus on data rather than communication endpoints
- **Discovery**: Automatic discovery of participants in the system
- **Quality of Service**: Rich set of policies for different communication needs
- **Reliability**: Built-in mechanisms for reliable data delivery
- **Real-time Support**: Deterministic communication patterns

### DDS Implementations

ROS 2 supports multiple DDS implementations:
- **Fast DDS** (formerly Fast RTPS): Default implementation
- **Cyclone DDS**: Lightweight implementation
- **RTI Connext DDS**: Commercial implementation
- **OpenSplice DDS**: Open-source implementation

## ROS 2 Ecosystem Components

### rclcpp and rclpy

The **ROS Client Libraries** (rcl) provide the interface between ROS 2 and programming languages:

- **rclcpp**: C++ client library
- **rclpy**: Python client library

These libraries abstract the underlying DDS implementation and provide a consistent API across languages.

### rmw: ROS Middleware Interface

The **ROS Middleware Interface** (rmw) provides an abstraction layer between ROS 2 and different DDS implementations, allowing ROS 2 to work with various middleware solutions without changing application code.

### ros2cli: Command-Line Tools

ROS 2 provides extensive command-line tools for system introspection and management:
- `ros2 run`: Run nodes
- `ros2 topic`: Topic-related commands
- `ros2 service`: Service-related commands
- `ros2 action`: Action-related commands
- `ros2 node`: Node-related commands
- `ros2 param`: Parameter-related commands

## Communication Patterns in Practice

### Sensor Data Pipeline

A typical sensor data pipeline uses topics for continuous data streaming:

```
Sensor Driver Node → Topic → Processing Node
```

The sensor driver publishes raw sensor data to a topic, and processing nodes subscribe to receive and process the data.

### Control Command Pattern

Control commands often use services for immediate responses:

```
Controller Node → Service Request → Actuator Node
Controller Node ← Service Response ← Actuator Node
```

### Navigation System Pattern

A navigation system might use all three communication patterns:

- **Topics**: For continuous sensor data (LIDAR, cameras, IMU)
- **Services**: For immediate commands (pause, resume, clear costmaps)
- **Actions**: For navigation goals (move to location with feedback)

## Best Practices for ROS 2 Architecture

### Design Principles

1. **Single Responsibility**: Each node should have a single, well-defined purpose
2. **Loose Coupling**: Minimize dependencies between nodes
3. **High Cohesion**: Group related functionality within nodes
4. **Consistent Interfaces**: Use standard message types when possible
5. **Error Handling**: Implement robust error handling and recovery

### Performance Considerations

1. **Message Efficiency**: Use appropriate message types and compression
2. **QoS Selection**: Choose appropriate QoS policies for each use case
3. **Threading**: Use appropriate threading models for performance
4. **Resource Management**: Manage memory and computational resources efficiently

### Security Considerations

1. **Authentication**: Use ROS 2's security features for production systems
2. **Encryption**: Enable encryption for sensitive data
3. **Access Control**: Implement appropriate access control policies
4. **Network Security**: Secure network communications

## Learning Objectives

After studying this chapter, students should be able to:

1. Explain the fundamental differences between ROS 1 and ROS 2 architectures
2. Identify and describe the three primary communication patterns in ROS 2
3. Implement nodes using both rclcpp and rclpy client libraries
4. Apply appropriate Quality of Service policies for different communication needs
5. Design robotic systems using proper ROS 2 architectural patterns
6. Evaluate the trade-offs between different communication patterns for specific applications

## Prerequisites

- Basic understanding of robotics concepts
- Familiarity with programming concepts
- Elementary knowledge of network communication
- Understanding of object-oriented programming

## References

1. Macenski, S., & Pham, Q. (2022). *Effective ROS 2: Patterns, Tools, and Tips for Building Reliable Robotic Systems*. Effective Robotics Programming Series.
2. Quigley, M., Gerkey, B., & Smart, W. D. (2022). *Programming Robots with ROS: A Practical Introduction to the Robot Operating System*. O'Reilly Media.
3. Object Management Group. (2015). *DDS (Data Distribution Service) for Real-Time Systems, 3rd Edition*. OMG Standard.
4. Colomé, A., & Torras, C. (2017). Joint-space control of redundant robots without acceleration inversion. *IEEE Robotics and Automation Letters*, 2(2), 719-726.

## Exercises

1. Design a ROS 2 system architecture for a mobile manipulator robot
2. Implement a simple publisher-subscriber example with different QoS policies
3. Create a service server and client for robot state queries
4. Design an action server for a robot navigation task
5. Analyze the communication patterns used in a real-world ROS 2 application