---
sidebar_position: 4
title: "Chapter 2 Exercises"
---

# Chapter 2: Exercises

## Exercise 2.1: Node Communication

**Difficulty Level**: Basic  
**Time Required**: 45 minutes  
**Learning Objective**: Remember & Apply

Create two ROS 2 nodes: one that publishes simulated sensor data and another that subscribes to this data and logs the values. Implement appropriate error handling and logging.

**Instructions:**
1. Create a publisher node that generates simulated sensor data (e.g., temperature readings, distance measurements)
2. Create a subscriber node that receives this data and logs it with timestamps
3. Implement error handling for network issues and invalid data
4. Use ROS 2 logging macros for proper logging
5. Test the communication between nodes

**Submission Requirements:**
- Complete code for both nodes
- Package.xml file with proper dependencies
- CMakeLists.txt with correct build configuration
- Brief report on testing results

---

## Exercise 2.2: Service Implementation

**Difficulty Level**: Intermediate  
**Time Required**: 60 minutes  
**Learning Objective**: Apply & Analyze

Implement a ROS 2 service that calculates the distance between two points in 2D space. Create both the service server and a client node that makes requests to the service.

**Instructions:**
1. Define a custom service message for 2D point coordinates
2. Implement a service server that calculates Euclidean distance
3. Create a client node that sends requests to the server
4. Test the service with multiple coordinate pairs
5. Analyze the performance and reliability of the service

**Key Components to Implement:**
- Custom service definition (srv file)
- Service server implementation
- Service client implementation
- Proper error handling and validation
- Performance analysis

**Submission Requirements:**
- Service definition file (.srv)
- Server node implementation
- Client node implementation
- Performance analysis report
- Testing with multiple scenarios

---

## Exercise 2.3: QoS Policy Experimentation

**Difficulty Level**: Intermediate  
**Time Required**: 75 minutes  
**Learning Objective**: Analyze & Evaluate

Modify the publisher-subscriber example to experiment with different Quality of Service policies. Compare the effects of reliable vs. best-effort delivery and different history policies.

**Instructions:**
1. Create a publisher node with configurable QoS policies
2. Create a subscriber node with appropriate matching QoS settings
3. Test with different combinations of:
   - Reliability (reliable vs. best-effort)
   - Durability (volatile vs. transient_local)
   - History (keep_last N vs. keep_all)
   - Depth (message queue size)
4. Measure and compare performance metrics
5. Document findings and recommendations

**Metrics to Measure:**
- Message delivery rate
- Latency distribution
- Memory usage
- CPU utilization
- Network bandwidth

**Submission Requirements:**
- Modified publisher and subscriber code
- Configuration files for different QoS settings
- Performance measurement data
- Comparison analysis with recommendations

---

## Exercise 2.4: Robot Control Node

**Difficulty Level**: Advanced  
**Time Required**: 90 minutes  
**Learning Objective**: Apply & Create

Create a ROS 2 node that subscribes to a LIDAR topic and publishes velocity commands to avoid obstacles. Implement the control logic using a simple proportional controller.

**Instructions:**
1. Subscribe to a simulated LIDAR topic (sensor_msgs/LaserScan)
2. Process LIDAR data to detect obstacles in different directions
3. Implement a proportional control algorithm for obstacle avoidance
4. Publish velocity commands (geometry_msgs/Twist) to control the robot
5. Implement safety mechanisms to prevent collisions
6. Test with various obstacle configurations

**Control Algorithm Requirements:**
- Forward movement when no obstacles detected
- Angular velocity for obstacle avoidance
- Emergency stopping when too close to obstacles
- Priority for different obstacle directions

**Submission Requirements:**
- Complete robot control node implementation
- Launch file to start the node
- Configuration parameters for tuning
- Test results with different obstacle scenarios
- Analysis of control effectiveness

---

## Exercise 2.5: Launch File Creation

**Difficulty Level**: Intermediate  
**Time Required**: 60 minutes  
**Learning Objective**: Create & Apply

Create a launch file that starts multiple nodes simultaneously: a sensor simulator, a processing node, and a visualization node. Use parameters to configure each node.

**Instructions:**
1. Create a sensor simulator node that publishes mock sensor data
2. Create a processing node that transforms the sensor data
3. Create a visualization node that displays the processed data
4. Write a launch file using Python launch system
5. Use parameters to configure each node's behavior
6. Test the complete system with the launch file

**Launch System Components:**
- Python launch description file
- Parameter configuration
- Node startup ordering
- Conditional node launching
- Event handling

**Submission Requirements:**
- Sensor simulator node
- Processing node
- Visualization node
- Python launch file
- Configuration parameters
- Testing documentation

---

## Exercise 2.6: Custom Message Definition

**Difficulty Level**: Basic  
**Time Required**: 40 minutes  
**Learning Objective**: Remember & Apply

Define and implement custom message types for a robotic application. Create publisher and subscriber nodes that use these custom messages.

**Instructions:**
1. Define a custom message for robot status (battery level, operational state, etc.)
2. Define a custom message for robot commands (movement, operation mode, etc.)
3. Create a publisher node that sends robot status messages
4. Create a subscriber node that receives and processes robot commands
5. Test the custom message communication

**Message Definition Requirements:**
- At least 3 fields in each custom message
- Appropriate data types for each field
- Proper packaging in msg directory
- Correct build configuration

**Submission Requirements:**
- Custom message definition files
- Publisher node implementation
- Subscriber node implementation
- Package configuration files
- Test results

---

## Exercise 2.7: TF System Implementation

**Difficulty Level**: Advanced  
**Time Required**: 120 minutes  
**Learning Objective**: Apply & Create

Implement a complete TF (Transform) system for a mobile robot with multiple sensors. Create frames for the robot base, sensors, and navigation goals.

**Instructions:**
1. Create static transforms for robot sensor frames
2. Implement dynamic transforms for robot movement
3. Create a transform broadcaster for robot base movement
4. Implement a transform listener that queries transforms
5. Test coordinate frame transformations
6. Visualize the TF tree

**TF Requirements:**
- Base link frame for robot
- Sensor frames (camera, LIDAR, IMU)
- Map and odom frames
- Proper transform broadcasting
- Transform querying and usage

**Submission Requirements:**
- Transform broadcaster implementation
- Transform listener implementation
- TF tree configuration
- Visualization of the TF system
- Test results and analysis

---

## Self-Assessment Checklist

After completing these exercises, you should:

- [ ] Understand the fundamental ROS 2 architecture and components
- [ ] Be able to create nodes that communicate using topics, services, and actions
- [ ] Know how to configure Quality of Service policies for different requirements
- [ ] Understand the package structure and build system in ROS 2
- [ ] Be able to create custom messages and services
- [ ] Understand the TF system for coordinate transformations
- [ ] Know how to use launch files for system orchestration
- [ ] Understand security and real-time considerations in ROS 2

## Solutions Guide (Instructor Access)

### Exercise 2.1 Sample Structure
- Node initialization with proper error handling
- Publisher with appropriate queue size
- Subscriber with logging and validation
- Proper resource cleanup

### Exercise 2.2 Expected Service Definition
```
# Request
float64 x1
float64 y1
float64 x2
float64 y2
---
# Response
float64 distance
bool success
string error_message
```

### Exercise 2.3 QoS Analysis
- Reliable delivery for critical commands
- Best-effort for sensor data
- History policy based on data importance
- Depth settings for memory optimization