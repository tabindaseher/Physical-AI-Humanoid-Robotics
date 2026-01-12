---
sidebar_position: 3
title: URDF for Humanoid Robots
---

# URDF for Humanoid Robots: Robot Description and Modeling

## Introduction

Unified Robot Description Format (URDF) is an XML-based format used in ROS to describe robot models. For humanoid robots, URDF becomes particularly important as it defines the complex kinematic structure, physical properties, and visual representation of the robot. This chapter explores the intricacies of creating URDF models for humanoid robots, covering the fundamental concepts, best practices, and advanced techniques for accurate robot representation.

## URDF Fundamentals

### URDF Overview

URDF (Unified Robot Description Format) is an XML format that describes robots in terms of their links, joints, and the connections between them. It provides a complete description of a robot's physical and visual properties, enabling simulation, visualization, and kinematic analysis.

### Key URDF Concepts

1. **Links**: Rigid bodies with mass, visual, and collision properties
2. **Joints**: Connections between links that define motion constraints
3. **Materials**: Visual appearance properties
4. **Transmissions**: Mapping between actuators and joints
5. **Gazebo Extensions**: Simulation-specific properties

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Links -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="joint_name" type="revolute">
    <parent link="base_link"/>
    <child link="child_link"/>
    <origin xyz="0.1 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="child_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>
</robot>
```

## Humanoid Robot Anatomy in URDF

### Typical Humanoid Structure

A humanoid robot typically consists of the following components:

1. **Torso**: Central body with head, arms, and legs attached
2. **Head**: Contains sensors (cameras, IMU) and possibly neck joints
3. **Arms**: Shoulders, elbows, wrists, and hands
4. **Legs**: Hips, knees, ankles, and feet
5. **End Effectors**: Hands for manipulation

### Common Joint Types in Humanoid Robots

1. **Revolute**: Rotational joint with limited range (e.g., elbow, knee)
2. **Continuous**: Rotational joint without limits (e.g., shoulder)
3. **Prismatic**: Linear sliding joint (rare in humanoids)
4. **Fixed**: No movement (e.g., attaching sensors)

## Detailed URDF Components

### Links

Links represent rigid bodies in the robot structure. Each link contains:

#### Visual Elements
```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="1 1 1"/>
    <!-- or <cylinder radius="0.1" length="0.5"/> -->
    <!-- or <sphere radius="0.1"/> -->
    <!-- or <mesh filename="package://path/to/mesh.stl"/> -->
  </geometry>
  <material name="blue">
    <color rgba="0 0 1 1"/>
  </material>
</visual>
```

#### Collision Elements
```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="1 1 1"/>
    <!-- Simpler geometry for collision detection -->
  </geometry>
</collision>
```

#### Inertial Elements
```xml
<inertial>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <mass value="1.0"/>
  <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
</inertial>
```

### Joints

Joints define the connection between links and their motion constraints:

#### Joint Types
- `revolute`: Rotational with limits
- `continuous`: Rotational without limits
- `prismatic`: Linear sliding
- `fixed`: No movement
- `floating`: 6 DOF
- `planar`: Planar motion

#### Joint Definition
```xml
<joint name="shoulder_joint" type="revolute">
  <parent link="torso"/>
  <child link="upper_arm"/>
  <origin xyz="0.1 0.2 0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100.0" velocity="1.0"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

## Complete Humanoid URDF Example

Here's a simplified example of a humanoid robot URDF:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.78" upper="0.78" effort="10.0" velocity="1.0"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0.1 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20.0" velocity="1.0"/>
  </joint>

  <!-- Right Arm (similar structure) -->
  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="right_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_upper_arm"/>
    <origin xyz="0.15 -0.1 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20.0" velocity="1.0"/>
  </joint>

  <!-- Left Leg -->
  <link name="left_upper_leg">
    <visual>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.2"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_leg"/>
    <origin xyz="-0.1 0.1 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="30.0" velocity="1.0"/>
  </joint>

  <!-- Right Leg -->
  <link name="right_upper_leg">
    <visual>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.2"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <joint name="right_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_upper_leg"/>
    <origin xyz="-0.1 -0.1 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="30.0" velocity="1.0"/>
  </joint>
</robot>
```

## Advanced URDF Features for Humanoids

### Materials and Colors

Materials define the visual appearance of robot links:

```xml
<material name="black">
  <color rgba="0.0 0.0 0.0 1.0"/>
</material>

<material name="blue">
  <color rgba="0.0 0.0 0.8 1.0"/>
</material>

<material name="green">
  <color rgba="0.0 0.8 0.0 1.0"/>
</material>

<material name="grey">
  <color rgba="0.5 0.5 0.5 1.0"/>
</material>

<material name="orange">
  <color rgba="1.0 0.423529411765 0.0392156862745 1.0"/>
</material>

<material name="brown">
  <color rgba="0.870588235294 0.811764705882 0.764705882353 1.0"/>
</material>

<material name="red">
  <color rgba="0.8 0.0 0.0 1.0"/>
</material>

<material name="white">
  <color rgba="1.0 1.0 1.0 1.0"/>
</material>
```

### Transmissions

Transmissions define the mapping between actuators and joints:

```xml
<transmission name="left_shoulder_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_shoulder_joint">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_shoulder_motor">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

### Gazebo Extensions

Gazebo-specific extensions for simulation:

```xml
<gazebo reference="base_link">
  <material>Gazebo/Grey</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
  <self_collide>true</self_collide>
</gazebo>

<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/simple_humanoid</robotNamespace>
  </plugin>
</gazebo>
```

## Best Practices for Humanoid URDF

### Link Design

1. **Realistic Dimensions**: Use actual robot dimensions for accurate simulation
2. **Appropriate Shapes**: Use simple geometric shapes for collision detection
3. **Mass Distribution**: Accurately model mass and inertia properties
4. **Visual vs. Collision**: Use detailed meshes for visualization, simple shapes for collision

### Joint Design

1. **Correct Joint Types**: Use appropriate joint types for each connection
2. **Realistic Limits**: Set joint limits based on physical constraints
3. **Proper Axes**: Define rotation/translation axes correctly
4. **Damping and Friction**: Include realistic physical properties

### Organization

1. **Modular Structure**: Organize URDF files logically
2. **Xacro Usage**: Use Xacro for parameterization and modularity
3. **File Structure**: Separate URDF into multiple files for complex robots
4. **Documentation**: Comment complex URDF sections

## Xacro for Complex Humanoid Models

Xacro (XML Macros) allows parameterization and modularization:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_with_xacro">
  <!-- Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="torso_length" value="0.5" />
  <xacro:property name="arm_length" value="0.3" />
  <xacro:property name="leg_length" value="0.4" />

  <!-- Macro for arm -->
  <xacro:macro name="arm" params="side parent xyz rpy">
    <link name="${side}_upper_arm">
      <visual>
        <geometry>
          <cylinder radius="0.05" length="${arm_length}"/>
        </geometry>
        <origin xyz="0 0 ${arm_length/2}" rpy="0 0 0"/>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="0.05" length="${arm_length}"/>
        </geometry>
        <origin xyz="0 0 ${arm_length/2}" rpy="0 0 0"/>
      </collision>
      <inertial>
        <mass value="0.8"/>
        <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
      </inertial>
    </link>

    <joint name="${side}_shoulder_joint" type="revolute">
      <parent link="${parent}"/>
      <child link="${side}_upper_arm"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="20.0" velocity="1.0"/>
    </joint>
  </xacro:macro>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 ${torso_length}"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 ${torso_length}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Use macros to create arms -->
  <xacro:arm side="left" parent="base_link" xyz="0.15 0.1 0.1" rpy="0 0 0"/>
  <xacro:arm side="right" parent="base_link" xyz="0.15 -0.1 0.1" rpy="0 0 0"/>
</robot>
```

## Validation and Testing

### URDF Validation

1. **XML Validation**: Ensure proper XML syntax
2. **Kinematic Validation**: Check for proper joint connections
3. **Mass Properties**: Verify realistic mass and inertia values
4. **Collision Detection**: Test collision properties in simulation

### Tools for URDF Development

1. **RViz**: Visualize robot model
2. **Gazebo**: Test in physics simulation
3. **CheckURDF**: Validate URDF syntax
4. **Robot State Publisher**: Publish joint states for visualization

## Common Issues and Troubleshooting

### Kinematic Issues

- **Disconnected Links**: Ensure all links are connected via joints
- **Incorrect Axes**: Verify joint axes are correctly defined
- **Joint Limits**: Set appropriate limits based on physical constraints

### Simulation Issues

- **Inertia Problems**: Incorrect inertia values can cause simulation instability
- **Collision Issues**: Poor collision geometry can cause unexpected behavior
- **Mass Distribution**: Unrealistic mass values affect physics simulation

### Visualization Issues

- **Mesh Loading**: Ensure mesh files are in correct location
- **Material Issues**: Check material definitions and references
- **Scale Problems**: Verify units are consistent throughout the model

## Learning Objectives

After studying this chapter, students should be able to:

1. Create complete URDF models for humanoid robots
2. Define appropriate links, joints, and their properties
3. Use Xacro for parameterized and modular robot descriptions
4. Apply best practices for mass, collision, and visual properties
5. Validate URDF models for simulation and visualization
6. Troubleshoot common URDF issues

## Prerequisites

- Basic understanding of XML syntax
- Knowledge of robot kinematics
- Understanding of ROS concepts
- Basic knowledge of physics (mass, inertia, collision)

## References

1. *ROS URDF Documentation*. (2023). Robot Operating System. Available: http://wiki.ros.org/urdf
2. *Xacro Documentation*. (2023). Robot Operating System. Available: http://wiki.ros.org/xacro
3. Siciliano, B., & Khatib, O. (2016). *Springer Handbook of Robotics*. Springer. Chapter on Robot Modeling.
4. Corke, P. (2017). *Robotics, Vision and Control: Fundamental Algorithms In MATLAB*. Springer. Chapter on Robot Modeling.

## Exercises

1. Create a URDF model for a simple humanoid robot with at least 12 degrees of freedom
2. Use Xacro to parameterize the robot model and create multiple variants
3. Add Gazebo extensions to your URDF for physics simulation
4. Validate your URDF using check_urdf and visualize in RViz
5. Implement a complete humanoid model with arms, legs, and head
6. Add sensors to your URDF model (cameras, IMU, etc.)