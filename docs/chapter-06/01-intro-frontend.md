---
sidebar_position: 1
title: "Chapter 6: Frontend - Humanoid Robot Development"
---

# Chapter 6: Frontend - Humanoid Robot Development

## Learning Objectives

By the end of this chapter, you should be able to:

**Remember**: List the key components and design principles of humanoid robot development

**Understand**: Explain the biomechanics, dynamics, and control challenges of humanoid robots

**Apply**: Design and implement control systems for humanoid robot locomotion and manipulation

**Analyze**: Evaluate the gait stability and balance systems in humanoid robots

**Evaluate**: Compare different humanoid robot platforms and their capabilities

**Create**: Develop a complete humanoid robot subsystem with perception, control, and safety

## 6.1 Humanoid Robot Design Principles

Humanoid robots are designed to mimic human form and function, which provides unique advantages and challenges in robotics. The human-like form factor enables interaction with human environments and tools, but also requires sophisticated control systems to replicate human capabilities.

### Anthropomorphic Design Considerations

Humanoid robots incorporate human-like features that serve both functional and social purposes:

- **Bipedal Locomotion**: Two-legged walking capability similar to humans
- **Upper Limb Manipulation**: Arms and hands designed for dexterous manipulation
- **Sensory Systems**: Vision, hearing, and tactile systems positioned similarly to humans
- **Communication Interface**: Face and body language for human-robot interaction

### Degrees of Freedom and Mobility

The number and arrangement of joints significantly impact a humanoid robot's capabilities:

```python
class HumanoidRobotConfig:
    def __init__(self):
        # Define joint configuration similar to human body
        self.left_leg = {
            'hip_yaw_pitch': {'range': (-45, 45), 'type': 'revolute'},
            'hip_roll': {'range': (-20, 45), 'type': 'revolute'},
            'hip_pitch': {'range': (-120, 30), 'type': 'revolute'},
            'knee': {'range': (0, 135), 'type': 'revolute'},
            'ankle_pitch': {'range': (-45, 45), 'type': 'revolute'},
            'ankle_roll': {'range': (-20, 20), 'type': 'revolute'}
        }
        
        self.right_leg = self._mirror_joints(self.left_leg)
        
        self.left_arm = {
            'shoulder_pitch': {'range': (-120, 120), 'type': 'revolute'},
            'shoulder_roll': {'range': (-90, 30), 'type': 'revolute'},
            'shoulder_yaw': {'range': (-120, 120), 'type': 'revolute'},
            'elbow': {'range': (0, 160), 'type': 'revolute'},
            'wrist_yaw': {'range': (-120, 120), 'type': 'revolute'},
            'wrist_pitch': {'range': (-90, 90), 'type': 'revolute'},
            'wrist_roll': {'range': (-120, 120), 'type': 'revolute'}
        }
        
        self.right_arm = self._mirror_joints(self.left_arm)
        
        self.torso = {
            'waist_yaw': {'range': (-60, 60), 'type': 'revolute'},
            'waist_pitch': {'range': (-30, 30), 'type': 'revolute'},
            'waist_roll': {'range': (-30, 30), 'type': 'revolute'}
        }
        
        self.head = {
            'neck_yaw': {'range': (-120, 120), 'type': 'revolute'},
            'neck_pitch': {'range': (-30, 60), 'type': 'revolute'},
            'neck_roll': {'range': (-30, 30), 'type': 'revolute'}
        }
    
    def _mirror_joints(self, joints):
        """Create mirror configuration for opposite side"""
        mirrored = {}
        for name, props in joints.items():
            mirrored[name] = props.copy()
        return mirrored
    
    def get_total_dof(self):
        """Calculate total degrees of freedom"""
        total = 0
        for limb in [self.left_leg, self.right_leg, self.left_arm, self.right_arm, 
                     self.torso, self.head]:
            total += len(limb)
        return total

# Example usage
robot_config = HumanoidRobotConfig()
print(f"Total DOF: {robot_config.get_total_dof()}")
```

### Actuator Selection and Placement

Selecting appropriate actuators for humanoid robots requires balancing power, precision, and safety:

- **Servo Motors**: High precision, good for manipulation tasks
- **Series Elastic Actuators (SEA)**: Compliant actuation for safe interaction
- **Pneumatic Muscles**: Human-like compliance and lightweight design
- **Hydraulic Actuators**: High power-to-weight ratio for heavy lifting

### Structural Materials and Fabrication

Humanoid robots require materials that balance strength, weight, and safety:

- **Aluminum Alloys**: Lightweight with good strength-to-weight ratio
- **Carbon Fiber**: Extremely lightweight with high strength
- **Engineering Plastics**: Cost-effective for non-critical components
- **Titanium**: High strength applications requiring corrosion resistance

## 6.2 Locomotion and Gait Control

### Bipedal Walking Mechanics

Bipedal walking presents unique challenges due to the need for dynamic balance:

- **Single Support Phase**: One foot in contact with ground
- **Double Support Phase**: Both feet in contact with ground
- **Swing Phase**: Leg moves forward without ground contact
- **Balance Control**: Maintaining center of mass within support polygon

```python
import numpy as np
from scipy import integrate
import matplotlib.pyplot as plt

class BipedalWalker:
    def __init__(self, mass=70, leg_length=0.9, gravity=9.81):
        self.mass = mass  # kg
        self.leg_length = leg_length  # m
        self.gravity = gravity  # m/s^2
        
        # Walking parameters
        self.step_length = 0.3  # m
        self.step_height = 0.1  # m
        self.gait_cycle_time = 1.0  # s
        
        # State variables
        self.x = 0.0  # x position
        self.y = 0.0  # y position (lateral movement)
        self.z = leg_length  # z position (height)
        self.omega = 0.0  # angular velocity
        
        # Control parameters
        self.com_height = 0.85  # Center of mass height
        self.zmp_x = 0.0  # Zero Moment Point x-coordinate
        self.zmp_y = 0.0  # Zero Moment Point y-coordinate
        
    def inverted_pendulum_model(self, state, t):
        """
        Inverted pendulum model for bipedal walking
        state = [x, y, z, dx, dy, dz, theta_x, theta_y, omega_x, omega_y]
        """
        x, y, z, dx, dy, dz, theta_x, theta_y, omega_x, omega_y = state
        
        # Simplified inverted pendulum dynamics
        x_dd = self.gravity / (z - self.com_height) * theta_x
        y_dd = self.gravity / (z - self.com_height) * theta_y
        
        # Angular velocity derivatives
        theta_x_d = omega_x
        theta_y_d = omega_y
        
        return [dx, dy, dz, x_dd, y_dd, 0, theta_x_d, theta_y_d, 0, 0]
    
    def compute_zmp(self, x, y, z, dx, dy, dz):
        """
        Compute Zero Moment Point (ZMP) for balance control
        """
        # ZMP calculation based on center of mass and ground reaction forces
        zmp_x = x - (z - self.com_height) / self.gravity * dx
        zmp_y = y - (z - self.com_height) / self.gravity * dy
        
        return zmp_x, zmp_y
    
    def generate_com_trajectory(self, start_pos, goal_pos, duration):
        """
        Generate Center of Mass trajectory for walking
        """
        # Simple 5th order polynomial trajectory
        t = np.linspace(0, duration, int(duration * 100))  # 100 Hz sampling
        
        # Generate trajectory for each axis
        x_traj = np.linspace(start_pos[0], goal_pos[0], len(t))
        y_traj = np.full_like(t, self.com_height)  # Keep CoM at constant height
        z_traj = np.full_like(t, self.com_height)  # Constant height
        
        # Add small lateral movement for balance
        y_traj += 0.05 * np.sin(2 * np.pi * t / duration)
        
        return t, np.column_stack([x_traj, y_traj, z_traj])
    
    def step_pattern_generator(self, step_length, step_height, step_time):
        """
        Generate step pattern for walking gait
        """
        t = np.linspace(0, step_time, int(step_time * 100))
        
        # Swing leg trajectory
        x_swing = step_length * (1 - np.cos(np.pi * t / step_time)) / 2
        z_swing = step_height * np.sin(np.pi * t / step_time)
        
        # Support leg stays relatively constant but with small adjustments
        x_support = np.full_like(t, step_length / 2)  # Average position
        z_support = np.full_like(t, 0)  # Ground level
        
        return {
            'swing_leg': np.column_stack([x_swing, np.zeros_like(x_swing), z_swing]),
            'support_leg': np.column_stack([x_support, np.zeros_like(x_support), z_support])
        }

class WalkingController:
    def __init__(self):
        self.biped_model = BipedalWalker()
        self.zmp_reference = np.array([0.0, 0.0])
        self.com_reference = np.array([0.0, 0.0, 0.85])
        
        # Controller gains
        self.k_p = 10.0  # Proportional gain
        self.k_d = 2.0   # Derivative gain
        self.k_i = 1.0   # Integral gain
        
    def zmp_controller(self, current_zmp, dt):
        """ZMP-based balance controller"""
        error = self.zmp_reference - current_zmp
        self.integral_error += error * dt
        
        # PID control for balance
        control_output = self.k_p * error + self.k_i * self.integral_error
        return control_output
    
    def compute_foot_placement(self, com_state, target_velocity):
        """Compute optimal foot placement for balance"""
        # Simple model for foot placement based on inverted pendulum
        # Calculate where to place foot to maintain balance
        com_x, com_y, com_z = com_state
        com_x_dot, com_y_dot = target_velocity[:2]
        
        # Time to foot placement
        support_time = 0.8  # seconds
        
        # Predict where CoM will be
        predicted_x = com_x + com_x_dot * support_time
        predicted_y = com_y + com_y_dot * support_time
        
        # Add safety margin
        foot_x = predicted_x + 0.1  # Small forward margin
        foot_y = com_y + 0.0  # Stay in line with CoM y
        
        return np.array([foot_x, foot_y, 0.0])
```

### Zero Moment Point (ZMP) Control

ZMP control is fundamental to humanoid balance during walking:

```python
class ZMPController:
    def __init__(self, robot_mass, com_height, gravity=9.81):
        self.mass = robot_mass
        self.com_height = com_height
        self.gravity = gravity
        self.integral_error = np.zeros(2)
        
        # Controller parameters
        self.kp = 100.0  # Proportional gain
        self.kd = 10.0   # Derivative gain 
        self.ki = 1.0    # Integral gain
        
    def compute_zmp(self, com_pos, com_vel, com_acc, cop_pos):
        """
        Compute Zero Moment Point based on CoM and CoP information
        """
        # ZMP_x = CoM_x - (CoM_z - h) / g * CoM_acc_x
        # ZMP_y = CoM_y - (CoM_z - h) / g * CoM_acc_y
        
        zmp = np.array([
            com_pos[0] - (com_pos[2] - self.com_height) / self.gravity * com_acc[0],
            com_pos[1] - (com_pos[2] - self.com_height) / self.gravity * com_acc[1]
        ])
        
        return zmp
    
    def balance_control(self, current_zmp, reference_zmp, dt):
        """
        Compute control forces to maintain balance
        """
        # Calculate error
        error = reference_zmp - current_zmp
        
        # Update integral term
        self.integral_error += error * dt
        
        # Compute control output (PID)
        control_output = (
            self.kp * error + 
            self.kd * (error - self.previous_error) / dt + 
            self.ki * self.integral_error
        )
        
        self.previous_error = error.copy()
        
        return control_output
    
    def update_reference_zmp(self, com_pos, target_pos, stiffness=1.0):
        """
        Update reference ZMP based on desired center of mass position
        """
        # Simple spring-mass tracking of desired position
        pos_error = target_pos - com_pos[:2]
        reference_zmp = target_pos - stiffness * pos_error
        
        # Keep reference within support polygon
        reference_zmp = self.constrain_to_support_polygon(reference_zmp)
        
        return reference_zmp
    
    def constrain_to_support_polygon(self, zmp):
        """
        Constrain ZMP to be within the convex hull of foot support points
        """
        # For a simple case with rectangular feet
        foot_width = 0.1  # meters
        foot_length = 0.15  # meters
        
        # This would be more complex in a real implementation
        constrained_zmp = np.clip(zmp, [-foot_width/2, -foot_length/2], 
                                [foot_width/2, foot_length/2])
        
        return constrained_zmp
```

### Gait Pattern Generation

Creating stable walking patterns requires sophisticated trajectory planning:

```python
class GaitPatternGenerator:
    def __init__(self, step_length=0.3, step_height=0.1, step_time=0.8):
        self.step_length = step_length
        self.step_height = step_height
        self.step_time = step_time
        
        # Walking parameters
        self.stride_length = 2 * step_length  # Distance between foot placements
        self.walking_speed = self.stride_length / step_time  # m/s
        
    def generate_walk_trajectory(self, num_steps, start_pos=np.array([0, 0, 0])):
        """
        Generate complete walking trajectory for specified number of steps
        """
        trajectory = []
        time_points = []
        
        current_pos = start_pos.copy()
        
        for step in range(num_steps):
            # Generate single step trajectory
            step_trajectory, step_times = self.generate_single_step(
                current_pos, step % 2  # Alternate between left/right foot
            )
            
            # Update current position for next step
            current_pos[0] += self.stride_length / 2  # Move forward by half a stride
            
            trajectory.extend(step_trajectory)
            time_points.extend(step_times)
        
        return np.array(trajectory), np.array(time_points)
    
    def generate_single_step(self, start_pos, foot_index):
        """
        Generate trajectory for a single step (either left or right foot)
        foot_index: 0 for left foot, 1 for right foot
        """
        t = np.linspace(0, self.step_time, int(self.step_time * 100))  # 100 Hz
        
        # Swing foot trajectory (parabolic for smooth movement)
        x_swing = start_pos[0] + self.step_length * (1 - np.cos(np.pi * t / self.step_time)) / 2
        y_swing = start_pos[1] + (-1)**foot_index * 0.15  # Offset for left/right foot
        z_swing = start_pos[2] + self.step_height * np.sin(np.pi * t / self.step_time)
        
        # Support foot trajectory (minimal movement)
        x_support = start_pos[0] * np.ones_like(t)
        y_support = start_pos[1] + (-1)**(1-foot_index) * 0.15  # Opposite foot position
        z_support = start_pos[2] * np.ones_like(t)  # Ground level
        
        step_trajectory = []
        for i in range(len(t)):
            if foot_index == 0:  # Left foot is swing foot
                step_trajectory.append([
                    x_swing[i], y_swing[i], z_swing[i],  # Left foot
                    x_support[i], y_support[i], z_support[i]  # Right foot
                ])
            else:  # Right foot is swing foot
                step_trajectory.append([
                    x_support[i], y_support[i], z_support[i],  # Left foot
                    x_swing[i], y_swing[i], z_swing[i]  # Right foot
                ])
        
        return step_trajectory, t.tolist()
    
    def adjust_for_obstacles(self, trajectory, obstacle_positions):
        """
        Adjust gait pattern to avoid obstacles
        """
        # This would implement obstacle avoidance during walking
        # For now, a simplified version
        adjusted_trajectory = trajectory.copy()
        
        for i, pos in enumerate(adjusted_trajectory):
            for obs_pos in obstacle_positions:
                if self.distance_to_obstacle(pos[:2], obs_pos[:2]) < 0.3:  # 30cm clearance
                    # Adjust step to go around obstacle
                    adjusted_trajectory[i][0] += 0.1  # Move slightly to the side
                    adjusted_trajectory[i][3] += 0.1  # Move support foot too
        
        return adjusted_trajectory
    
    def distance_to_obstacle(self, point1, point2):
        """Calculate distance between two 2D points"""
        return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
```

## 6.3 Manipulation and Dexterity

### Anthropomorphic Hands and Fingers

Creating dexterous hands remains one of the greatest challenges in humanoid robotics:

- **Underactuation**: Using fewer actuators than degrees of freedom
- **Tendon-driven systems**: Mimicking human muscle-tendon systems
- **Compliant mechanisms**: Built-in compliance for safe interaction
- **Tactile sensing**: Feedback for grasp stability and object properties

### Grasp Planning and Execution

```python
import open3d as o3d
from scipy.spatial.distance import cdist
import numpy as np

class GraspPlanner:
    def __init__(self):
        # Grasp primitive types
        self.grasp_types = [
            'precision_pinch', 
            'power_grasp', 
            'spherical_grasp',
            'circular_grasp',
            'lateral_grasp'
        ]
        
        # Hand kinematics (simplified)
        self.finger_lengths = [0.04, 0.035, 0.03]  # Three segments per finger
        self.hand_width = 0.08  # Maximum hand opening
        self.finger_count = 5
        
    def find_grasp_points(self, object_mesh):
        """
        Find potential grasp points on object surface
        """
        # Sample points on object surface
        surface_points = self.sample_surface_points(object_mesh)
        
        # Analyze object geometry to find suitable grasp locations
        grasp_candidates = []
        
        for point in surface_points:
            # Check for suitable grasp direction at this point
            normal = self.estimate_surface_normal(object_mesh, point)
            
            # Calculate approach direction (orthogonal to normal)
            approach_dirs = self.generate_approach_directions(normal)
            
            for approach_dir in approach_dirs:
                if self.is_stable_grasp(object_mesh, point, approach_dir):
                    grasp_candidates.append({
                        'position': point,
                        'normal': normal,
                        'approach': approach_dir,
                        'quality': self.evaluate_grasp_quality(object_mesh, point, approach_dir)
                    })
        
        # Sort candidates by quality score
        grasp_candidates.sort(key=lambda x: x['quality'], reverse=True)
        
        return grasp_candidates
    
    def sample_surface_points(self, mesh, num_points=100):
        """Sample points on mesh surface"""
        pcd = mesh.sample_points_uniformly(number_of_points=num_points)
        return np.asarray(pcd.points)
    
    def estimate_surface_normal(self, mesh, point):
        """Estimate surface normal at a point"""
        # In practice, this would use mesh normals or point cloud analysis
        # For this example, we'll return a dummy normal
        return np.array([0, 0, 1])  # Default upward normal
    
    def generate_approach_directions(self, normal):
        """Generate possible approach directions orthogonal to surface normal"""
        # Generate directions perpendicular to surface normal
        # This is a simplified implementation
        directions = []
        
        # Create rotation matrix to get orthogonal directions
        z_axis = normal / np.linalg.norm(normal)
        
        # Find one orthogonal vector
        if abs(z_axis[2]) < 0.9:
            x_axis = np.cross(z_axis, [0, 0, 1])
        else:
            x_axis = np.cross(z_axis, [1, 0, 0])
        
        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        
        # Generate multiple approach directions
        for angle in np.linspace(0, 2*np.pi, 8):
            direction = np.cos(angle) * x_axis + np.sin(angle) * y_axis
            directions.append(direction)
        
        return directions
    
    def is_stable_grasp(self, object_mesh, point, approach_dir):
        """Check if a grasp is geometrically stable"""
        # Check if approach direction allows for a stable grip
        # This would involve checking for adequate contact areas
        # and force closure properties
        
        # Simplified check: ensure approach direction doesn't intersect
        # with object in a way that prevents grasping
        grasp_length = 0.03  # Typical grasp depth
        grasp_end = point - grasp_length * approach_dir / np.linalg.norm(approach_dir)
        
        # Check if line from point to grasp_end intersects object
        # (indicating grasp would be blocked)
        intersection = self.check_line_mesh_intersection(
            point, grasp_end, object_mesh
        )
        
        return not intersection
    
    def check_line_mesh_intersection(self, start, end, mesh):
        """Check if line intersects with mesh"""
        # Simplified intersection check
        # In practice, this would use ray-casting or geometric algorithms
        return False
    
    def evaluate_grasp_quality(self, object_mesh, point, approach_dir):
        """Evaluate the quality of a potential grasp"""
        # Consider factors like:
        # - Surface curvature at contact point
        # - Object size relative to hand
        # - Approach accessibility
        # - Stability of grasp
        
        quality = 0.0
        
        # Curvature consideration (flatter surfaces are better for grasping)
        curvature = self.estimate_curvature(object_mesh, point)
        quality += max(0, 1.0 - abs(curvature) * 10)  # Lower curvature = better
        
        # Size consideration (object should fit in hand)
        object_size = self.estimate_object_size(object_mesh)
        if object_size < self.hand_width:
            quality += 0.3
        else:
            quality += max(0, 0.3 * (self.hand_width / object_size))
        
        # Approach accessibility
        accessibility = self.evaluate_approach_accessibility(point, approach_dir)
        quality += accessibility * 0.4
        
        return min(quality, 1.0)
    
    def estimate_curvature(self, mesh, point):
        """Estimate surface curvature at a point"""
        # Simplified curvature estimation
        return 0.0  # For now, assume flat surface
    
    def estimate_object_size(self, mesh):
        """Estimate characteristic size of object"""
        bounds = mesh.get_axis_aligned_bounding_box()
        dims = bounds.get_extent()
        return max(dims)
    
    def evaluate_approach_accessibility(self, point, approach_dir):
        """Evaluate how accessible the approach is"""
        # Consider obstacles, robot configuration, etc.
        return 1.0  # Assume fully accessible for this example
    
    def plan_hand_trajectory(self, grasp_pose, pre_grasp_offset=0.05):
        """
        Plan trajectory to move hand to grasp position
        """
        # Calculate pre-grasp pose (approach from safe distance)
        pre_grasp_pose = grasp_pose.copy()
        pre_grasp_pose[:3] += pre_grasp_offset * grasp_pose[3:6]  # Move along approach vector
        
        # Plan trajectory through joint space
        trajectory = self.interpolate_trajectory(pre_grasp_pose, grasp_pose)
        
        return trajectory
    
    def interpolate_trajectory(self, start_pose, end_pose, steps=50):
        """Interpolate between two poses"""
        trajectory = []
        
        for i in range(steps + 1):
            t = i / steps
            pose = (1 - t) * start_pose + t * end_pose
            trajectory.append(pose)
        
        return trajectory

class HandController:
    def __init__(self):
        self.finger_positions = np.zeros(5)  # 5 fingers
        self.finger_velocities = np.zeros(5)
        self.finger_forces = np.zeros(5)
        
        # Grasp parameters
        self.grasp_force_limits = [10, 10, 10, 10, 10]  # N
        self.finger_range = [0, 90]  # degrees
        
    def execute_grasp(self, grasp_type, force_multiplier=1.0):
        """Execute a specific type of grasp"""
        if grasp_type == 'precision_pinch':
            # Move thumb and index finger together
            self.finger_positions[0] = 60  # Thumb
            self.finger_positions[1] = 60  # Index finger
            # Keep other fingers in neutral position
            self.finger_positions[2:] = 10  # Other fingers slightly curled
            
        elif grasp_type == 'power_grasp':
            # Close all fingers for power grasp
            self.finger_positions = np.full(5, 80)  # Close all fingers
            
        elif grasp_type == 'cylindrical_grasp':
            # Grasp cylindrical objects
            self.finger_positions[0] = 70  # Thumb wraps around
            self.finger_positions[1:] = 60  # Other fingers close on opposite side
            
        # Apply force proportional to grasp type requirements
        self.finger_forces = self.calculate_grasp_forces(grasp_type, force_multiplier)
        
        return self.finger_positions.copy(), self.finger_forces.copy()
    
    def calculate_grasp_forces(self, grasp_type, force_multiplier):
        """Calculate appropriate grasp forces for different grasp types"""
        base_forces = {
            'precision_pinch': [5, 5, 1, 1, 1],
            'power_grasp': [8, 8, 8, 8, 8],
            'cylindrical_grasp': [6, 8, 8, 8, 8],
            'spherical_grasp': [7, 7, 7, 7, 7],
            'lateral_grasp': [10, 2, 1, 1, 1]
        }
        
        forces = np.array(base_forces.get(grasp_type, [5, 5, 5, 5, 5]))
        forces *= force_multiplier
        
        # Ensure forces are within limits
        forces = np.clip(forces, 0, self.grasp_force_limits)
        
        return forces
    
    def adjust_grasp(self, tactile_feedback):
        """
        Adjust grasp based on tactile feedback
        tactile_feedback: array with pressure info from tactile sensors
        """
        # Simple adjustment algorithm
        if np.any(tactile_feedback > 0.8 * self.grasp_force_limits):
            # Too much force detected, reduce slightly
            self.finger_forces *= 0.95
        elif np.any(tactile_feedback < 0.2 * self.grasp_force_limits):
            # Not enough force for secure grasp, increase slightly
            self.finger_forces *= 1.02
            # Limit maximum force
            self.finger_forces = np.clip(self.finger_forces, 0, self.grasp_force_limits)
        
        return self.finger_forces.copy()
```

### Multi-limb Coordination

Coordinating multiple limbs in humanoid robots requires sophisticated control:

```python
class MultiLimbCoordinator:
    def __init__(self):
        # Define limb coordination priorities
        self.limb_priorities = {
            'right_arm': 1,
            'left_arm': 2, 
            'right_leg': 3,
            'left_leg': 4
        }
        
        # Task coordination system
        self.active_tasks = {}
        self.task_dependencies = {}
        
    def coordinate_task_execution(self, tasks):
        """
        Coordinate execution of multiple tasks across different limbs
        """
        # Resolve task conflicts and dependencies
        resolved_tasks = self.resolve_conflicts(tasks)
        
        # Schedule task execution
        execution_plan = self.schedule_tasks(resolved_tasks)
        
        return execution_plan
    
    def resolve_conflicts(self, tasks):
        """Resolve conflicts between simultaneous tasks"""
        resolved_tasks = {}
        
        for limb, task in tasks.items():
            conflict = self.check_task_conflict(limb, task)
            if conflict:
                # Resolve conflict based on priority
                if self.limb_priorities[limb] < self.limb_priorities[conflict['conflicting_limb']]:
                    # This limb has higher priority, adjust other task
                    resolved_tasks[conflict['conflicting_limb']] = self.adjust_task(
                        tasks[conflict['conflicting_limb']]
                    )
                else:
                    # Lower priority task needs adjustment
                    resolved_tasks[limb] = self.adjust_task(task)
            else:
                resolved_tasks[limb] = task
        
        return resolved_tasks
    
    def check_task_conflict(self, limb, task):
        """Check if a task conflicts with other active tasks"""
        # For now, a simple check for collision in workspace
        # In practice, this would check for workspace, timing, and priority conflicts
        return None
    
    def adjust_task(self, original_task):
        """Adjust task to avoid conflicts"""
        # Create a modified version of the task that avoids conflicts
        adjusted_task = original_task.copy()
        # Add delays, modify parameters, etc.
        return adjusted_task
    
    def schedule_tasks(self, resolved_tasks):
        """Schedule tasks for execution across different limbs"""
        # Create temporal schedule for task execution
        schedule = {}
        
        for limb, task in resolved_tasks.items():
            # Assign execution time based on task requirements
            schedule[limb] = {
                'task': task,
                'start_time': self.calculate_start_time(task),
                'duration': self.calculate_duration(task),
                'priority': self.limb_priorities[limb]
            }
        
        # Sort by start time and priority
        sorted_schedule = sorted(schedule.items(), 
                               key=lambda x: (x[1]['start_time'], x[1]['priority']))
        
        return sorted_schedule

class HumanoidMotionController:
    def __init__(self):
        self.walking_controller = WalkingController()
        self.grasp_planner = GraspPlanner()
        self.multi_limb_coordinator = MultiLimbCoordinator()
        self.safety_manager = HumanoidSafetyManager()
        
    def execute_complex_task(self, task_description):
        """
        Execute complex tasks that involve multiple limbs and modalities
        """
        # Parse high-level task into component actions
        subtasks = self.parse_task(task_description)
        
        # Coordinate execution across different limbs
        execution_plan = self.multi_limb_coordinator.coordinate_task_execution(subtasks)
        
        # Execute each component while maintaining safety
        for limb, task_info in execution_plan:
            if self.safety_manager.is_safe_to_execute(task_info['task']):
                # Execute the task component
                result = self.execute_task_component(limb, task_info['task'])
                
                # Monitor execution for safety
                self.safety_manager.monitor_execution(result)
            else:
                # Safety check failed, handle accordingly
                self.safety_manager.trigger_safety_protocol()
                break
        
        return execution_plan
    
    def parse_task(self, task_description):
        """Parse natural language task into component actions"""
        # This would involve NLP processing to extract task components
        # For example: "Walk to the table and pick up the red cup" 
        # would be parsed into walking and grasping subtasks
        subtasks = {}
        
        # Simplified parsing
        if 'walk' in task_description.lower():
            subtasks['right_leg'] = {'type': 'walking', 'destination': 'table'}
        if 'pick' in task_description.lower() or 'grasp' in task_description.lower():
            subtasks['right_arm'] = {'type': 'grasping', 'object': 'red cup'}
            
        return subtasks
    
    def execute_task_component(self, limb, task):
        """Execute a specific task component with the specified limb"""
        if task['type'] == 'walking':
            # Execute walking subtask
            destination = task['destination']
            # Implementation for walking to destination
            pass
        elif task['type'] == 'grasping':
            # Execute grasping subtask
            target_object = task['object']
            # Implementation for grasping object
            pass
        
        return {'status': 'complete', 'limb': limb}
```

## 6.4 Humanoid Control Architectures

### Hierarchical Control Systems

Humanoid robots require control systems at multiple levels:

- **High-level Planning**: Task planning and sequencing
- **Mid-level Control**: Trajectory generation and coordination
- **Low-level Control**: Joint servo control and feedback
- **Safety Systems**: Emergency stops and collision avoidance

### Central Pattern Generators (CPGs)

CPGs provide rhythmic patterns for locomotion:

```python
class CentralPatternGenerator:
    def __init__(self, dt=0.01):
        self.dt = dt
        self.oscillators = []
        self.connection_weights = []
        self.outputs = []
        
    def add_oscillator(self, frequency, phase, amplitude):
        """Add an oscillator to the CPG network"""
        oscillator = {
            'frequency': frequency,
            'phase': phase,
            'amplitude': amplitude,
            'state': 0.0  # Current state (typically an angle)
        }
        self.oscillators.append(oscillator)
        self.outputs.append(0.0)
        
    def connect_oscillators(self, i, j, weight):
        """Connect two oscillators with specified weight"""
        if len(self.connection_weights) <= i:
            self.connection_weights.extend([[] for _ in range(i - len(self.connection_weights) + 1)])
        while len(self.connection_weights[i]) <= j:
            self.connection_weights[i].append(0.0)
        self.connection_weights[i][j] = weight
        
    def step(self):
        """Update all oscillators for one time step"""
        for i, osc in enumerate(self.oscillators):
            # Update oscillator state based on dynamics
            # This is a simplified version; real CPGs use more complex dynamics
            input_sum = 0.0
            
            # Add inputs from connected oscillators
            if i < len(self.connection_weights):
                for j, weight in enumerate(self.connection_weights[i]):
                    if j < len(self.oscillators):
                        input_sum += weight * np.sin(self.oscillators[j]['state'] - osc['state'])
            
            # Update state with frequency and input
            dstate_dt = 2 * np.pi * osc['frequency'] + input_sum
            osc['state'] += dstate_dt * self.dt
            
            # Calculate output
            self.outputs[i] = osc['amplitude'] * np.sin(osc['state'] + osc['phase'])
    
    def get_outputs(self):
        """Get current outputs from all oscillators"""
        return self.outputs.copy()

class LocomotionCPG:
    def __init__(self):
        # Initialize CPG for bipedal locomotion
        self.cpg = CentralPatternGenerator(dt=0.01)
        
        # Add oscillators for different joints
        # Left leg oscillators
        self.cpg.add_oscillator(frequency=1.0, phase=0.0, amplitude=1.0)  # Left hip
        self.cpg.add_oscillator(frequency=1.0, phase=np.pi, amplitude=1.0)  # Right hip (anti-phase)
        self.cpg.add_oscillator(frequency=1.0, phase=0.0, amplitude=0.5)   # Left knee
        self.cpg.add_oscillator(frequency=1.0, phase=np.pi, amplitude=0.5)  # Right knee
        
        # Connect oscillators for coordinated movement
        self.cpg.connect_oscillators(0, 1, -1.0)  # Left-right hip coupling
        self.cpg.connect_oscillators(2, 3, -1.0)  # Left-right knee coupling
        self.cpg.connect_oscillators(0, 2, 0.5)   # Hip-knee coupling
        self.cpg.connect_oscillators(1, 3, 0.5)   # Hip-knee coupling (other side)
        
        # Parameters for gait adjustment
        self.step_frequency = 1.0
        self.step_amplitude = 1.0
    
    def update_gait_parameters(self, new_frequency, new_amplitude):
        """Update gait parameters"""
        self.step_frequency = new_frequency
        self.step_amplitude = new_amplitude
        
        # Update oscillator frequencies and amplitudes
        for i, osc in enumerate(self.cpg.oscillators):
            osc['frequency'] = new_frequency
            osc['amplitude'] = new_amplitude * osc['amplitude'] / 1.0  # Maintain relative amplitudes
    
    def get_leg_commands(self):
        """Get joint commands for both legs"""
        outputs = self.cpg.get_outputs()
        
        # Map CPG outputs to actual joint angles
        left_hip_cmd = outputs[0] if len(outputs) > 0 else 0.0
        right_hip_cmd = outputs[1] if len(outputs) > 1 else 0.0
        left_knee_cmd = outputs[2] if len(outputs) > 2 else 0.0
        right_knee_cmd = outputs[3] if len(outputs) > 3 else 0.0
        
        commands = {
            'left_leg': [left_hip_cmd, left_knee_cmd],
            'right_leg': [right_hip_cmd, right_knee_cmd]
        }
        
        self.cpg.step()  # Update CPG for next time step
        
        return commands
```

## 6.5 Safety and Emergency Systems

Safety is paramount in humanoid robot development due to their human-like form factor and interaction potential.

### Humanoid Safety Manager

```python
class HumanoidSafetyManager:
    def __init__(self):
        # Safety boundaries
        self.joint_limits = {
            'hip_pitch': (-120, 30),
            'knee': (0, 135),
            'ankle_pitch': (-45, 45),
            'shoulder_pitch': (-120, 120),
            'elbow': (0, 160)
        }
        
        self.workspace_limits = {
            'x': (-1.0, 1.0),
            'y': (-1.0, 1.0),
            'z': (0.1, 2.0)
        }
        
        self.force_limits = {
            'gripper_force': 50.0,  # N
            'collision_force': 100.0  # N
        }
        
        # Emergency systems
        self.emergency_stop = False
        self.fall_detected = False
        self.over_force_detected = False
        
        # Safety monitoring
        self.safety_monitoring_active = True
        
    def validate_command(self, joint_angles, cartesian_pos, forces):
        """Validate robot commands against safety limits"""
        safety_violations = []
        
        # Check joint limits
        for joint_name, angle in joint_angles.items():
            if joint_name in self.joint_limits:
                min_limit, max_limit = self.joint_limits[joint_name]
                if angle < min_limit or angle > max_limit:
                    safety_violations.append(f"Joint {joint_name} out of limits: {angle} not in [{min_limit}, {max_limit}]")
        
        # Check workspace limits
        if (cartesian_pos[0] < self.workspace_limits['x'][0] or 
            cartesian_pos[0] > self.workspace_limits['x'][1]):
            safety_violations.append(f"X position {cartesian_pos[0]} outside limits")
        
        if (cartesian_pos[1] < self.workspace_limits['y'][0] or 
            cartesian_pos[1] > self.workspace_limits['y'][1]):
            safety_violations.append(f"Y position {cartesian_pos[1]} outside limits")
        
        if (cartesian_pos[2] < self.workspace_limits['z'][0] or 
            cartesian_pos[2] > self.workspace_limits['z'][1]):
            safety_violations.append(f"Z position {cartesian_pos[2]} outside limits")
        
        # Check force limits
        for force_type, force_value in forces.items():
            if force_type in self.force_limits and force_value > self.force_limits[force_type]:
                safety_violations.append(f"Force limit exceeded for {force_type}: {force_value} > {self.force_limits[force_type]}")
        
        return len(safety_violations) == 0, safety_violations
    
    def monitor_execution(self, robot_state):
        """Monitor robot execution for safety violations"""
        if not self.safety_monitoring_active:
            return True, []
        
        # Check for various safety conditions
        is_safe, violations = self.validate_command(
            robot_state.get('joint_angles', {}),
            robot_state.get('cartesian_position', [0,0,0]),
            robot_state.get('applied_forces', {})
        )
        
        if not is_safe:
            self.trigger_safety_protocol()
            return False, violations
        
        # Check for fall detection
        imu_data = robot_state.get('imu_data', {})
        if self.detect_fall(imu_data):
            self.fall_detected = True
            self.trigger_safety_protocol()
            return False, ["Fall detected"]
        
        return True, []
    
    def detect_fall(self, imu_data):
        """Detect if robot is falling based on IMU data"""
        # Simplified fall detection
        # Check for excessive angular velocity or acceleration
        angular_vel = imu_data.get('angular_velocity', [0,0,0])
        linear_acc = imu_data.get('linear_acceleration', [0,0,0])
        
        # Fall threshold (these values would be tuned based on specific robot)
        ang_vel_threshold = 2.0  # rad/s
        acc_threshold = 15.0  # m/s^2
        
        if np.linalg.norm(angular_vel) > ang_vel_threshold:
            return True
        if np.linalg.norm(linear_acc) > acc_threshold:
            return True
        
        return False
    
    def trigger_safety_protocol(self):
        """Trigger safety protocol"""
        self.emergency_stop = True
        
        # Stop all joint movements
        # Activate joint brakes if available
        # Go to safe position
        # Log safety event
        
        print("SAFETY PROTOCOL TRIGGERED")
    
    def reset_safety(self):
        """Reset safety state after emergency stop"""
        self.emergency_stop = False
        self.fall_detected = False
        self.over_force_detected = False
        
        # Reset other safety flags
        print("Safety system reset")
```

## 6.6 Practical Example: Complete Humanoid System

Let's integrate the concepts to create a complete humanoid robot system:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration

class HumanoidRobotSystem(Node):
    def __init__(self):
        super().__init__('humanoid_robot_system')
        
        # Initialize control components
        self.safety_manager = HumanoidSafetyManager()
        self.walking_controller = WalkingController()
        self.cpg_locomotion = LocomotionCPG()
        self.grasp_planner = GraspPlanner()
        self.motion_controller = HumanoidMotionController()
        
        # ROS 2 interfaces
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.imu_subscriber = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.command_subscriber = self.create_subscription(Float64MultiArray, '/commands', self.command_callback, 10)
        self.pose_publisher = self.create_publisher(Pose, '/robot_pose', 10)
        
        # Robot state
        self.current_joint_angles = np.zeros(28)  # Example: 28 DOF humanoid
        self.imu_data = {'angular_velocity': [0,0,0], 'linear_acceleration': [0,0,0]}
        self.safety_violations = []
        
        # Control timer
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100 Hz
        
        self.get_logger().info('Humanoid Robot System initialized')
    
    def imu_callback(self, msg):
        """Handle IMU data"""
        self.imu_data = {
            'angular_velocity': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            'linear_acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        }
    
    def command_callback(self, msg):
        """Handle incoming commands"""
        # Process and validate commands
        command_data = np.array(msg.data)
        
        # Validate command with safety manager
        robot_state = {
            'joint_angles': dict(enumerate(self.current_joint_angles)),
            'cartesian_position': [0, 0, 0.85],  # Example position
            'applied_forces': {'gripper_force': 0.0},
            'imu_data': self.imu_data
        }
        
        is_safe, violations = self.safety_manager.validate_command(
            robot_state['joint_angles'],
            robot_state['cartesian_position'],
            robot_state['applied_forces']
        )
        
        if is_safe:
            # Execute command
            self.execute_command(command_data)
        else:
            self.safety_violations.extend(violations)
            self.get_logger().warn(f'Safety violations: {violations}')
    
    def execute_command(self, command_data):
        """Execute validated command"""
        # Map command data to joint positions
        if len(command_data) == len(self.current_joint_angles):
            self.current_joint_angles = command_data
        else:
            # Handle command mapping for different formats
            pass
    
    def control_loop(self):
        """Main control loop"""
        # Monitor robot safety
        robot_state = {
            'joint_angles': dict(enumerate(self.current_joint_angles)),
            'cartesian_position': [0, 0, 0.85],
            'applied_forces': {'gripper_force': 0.0},
            'imu_data': self.imu_data
        }
        
        is_safe, violations = self.safety_manager.monitor_execution(robot_state)
        
        if not is_safe:
            self.safety_violations.extend(violations)
            # Emergency stop logic
            self.emergency_stop_sequence()
            return
        
        # Generate joint commands using controllers
        joint_commands = self.generate_joint_commands()
        
        # Publish joint states
        self.publish_joint_states(joint_commands)
        
        # Update CPG for locomotion
        leg_commands = self.cpg_locomotion.get_leg_commands()
        
        # Publish pose estimate
        self.publish_pose_estimate()
    
    def generate_joint_commands(self):
        """Generate joint commands based on current tasks"""
        # This would integrate all control systems
        # For now, return current joint angles
        return self.current_joint_angles
    
    def publish_joint_states(self, joint_positions):
        """Publish joint state information"""
        msg = JointState()
        msg.name = [f'joint_{i}' for i in range(len(joint_positions))]
        msg.position = joint_positions.tolist()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        self.joint_state_publisher.publish(msg)
    
    def publish_pose_estimate(self):
        """Publish robot pose estimate"""
        msg = Pose()
        # Set position based on forward kinematics or estimation
        msg.position.x = 0.0  # Update with actual position
        msg.position.y = 0.0
        msg.position.z = 0.85  # Approximate height
        
        # Set orientation (simplified)
        msg.orientation.w = 1.0  # No rotation initially
        
        self.pose_publisher.publish(msg)
    
    def emergency_stop_sequence(self):
        """Execute emergency stop sequence"""
        self.get_logger().error('EMERGENCY STOP TRIGGERED')
        
        # Stop all motion
        zero_commands = np.zeros_like(self.current_joint_angles)
        self.current_joint_angles = zero_commands
        
        # Go to safe position if possible
        self.go_to_safe_position()
        
        # Log the event
        self.get_logger().info(f'Safety violations logged: {self.safety_violations}')
    
    def go_to_safe_position(self):
        """Move robot to predefined safe position"""
        # Define safe joint configurations
        safe_config = np.zeros_like(self.current_joint_angles)
        # Set safe positions (standing posture)
        safe_config[0::2] = 0.0  # Hip joints to neutral
        safe_config[1::2] = 0.0  # Knee joints to neutral
        # Other joints as appropriate
        
        self.current_joint_angles = safe_config

def main(args=None):
    rclpy.init(args=args)
    
    # Create and run the humanoid robot system
    humanoid_system = HumanoidRobotSystem()
    
    try:
        rclpy.spin(humanoid_system)
    except KeyboardInterrupt:
        pass
    finally:
        humanoid_system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 6.7 Summary

This chapter has explored the complex field of humanoid robot development, covering:

- The design principles and anthropomorphic considerations in humanoid robotics
- The challenges of bipedal locomotion and balance control using ZMP and CPG approaches
- The development of dexterous manipulation systems and grasp planning
- Multi-limb coordination strategies for complex tasks
- Safety systems and emergency protocols for humanoid robots
- Integration of all components into a complete humanoid robot system

Humanoid robots represent one of the most challenging domains in robotics due to their complexity, but they also offer unique advantages for human interaction and environment compatibility.

## 6.8 Exercises

### Exercise 6.1: Bipedal Walking Simulation
Create a simulation of bipedal walking using ZMP control. Implement a stable walking pattern for a simplified humanoid model.

### Exercise 6.2: Grasp Planning Algorithm
Develop a grasp planning algorithm that identifies optimal grasp points on 3D objects and generates appropriate hand configurations.

### Exercise 6.3: Multi-limb Coordination
Implement a system that coordinates movements between multiple limbs while avoiding conflicts and maintaining balance.

### Exercise 6.4: Safety System Integration
Design and implement a comprehensive safety system for a humanoid robot, including joint limits, fall detection, and emergency protocols.

### Exercise 6.5: Complete Humanoid Controller
Build a complete control system that integrates walking, manipulation, and safety functions for a humanoid robot.