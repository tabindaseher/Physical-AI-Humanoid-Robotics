---
sidebar_position: 6
title: Physical Robot Control Concepts
---

# Physical Robot Control Concepts: From Theory to Real-World Implementation

## Introduction

Physical robot control represents the critical bridge between computational planning and real-world execution. Unlike simulation, physical control must account for sensor noise, actuator limitations, environmental uncertainties, and real-time constraints. This chapter explores the fundamental concepts of physical robot control, focusing on the challenges and solutions specific to humanoid robotics.

## Control System Architecture

### Real-Time Control Hierarchy

Physical robot control typically follows a hierarchical architecture with different time scales:

```
High-Level Planning (seconds) → Trajectory Generation (100ms) → Feedback Control (1ms)
```

Each level has different requirements and constraints that must be carefully managed.

### Control Loop Timing

Real-time control systems require precise timing:

```python
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from scipy.interpolate import interp1d

class RealTimeController(Node):
    def __init__(self):
        super().__init__('real_time_controller')

        # Control loop parameters
        self.control_frequency = 1000.0  # Hz
        self.control_period = Duration(seconds=1.0/self.control_frequency)

        # Initialize control timer
        self.control_timer = self.create_timer(
            1.0/self.control_frequency,
            self.control_loop
        )

        # State and command subscribers/publishers
        self.state_sub = self.create_subscription(
            JointTrajectoryControllerState,
            'joint_states',
            self.state_callback,
            10
        )

        self.command_pub = self.create_publisher(
            JointTrajectory,
            'joint_trajectory',
            10
        )

        # Control state
        self.current_state = None
        self.desired_trajectory = None
        self.last_control_time = self.get_clock().now()

    def control_loop(self):
        """Main control loop running at specified frequency"""
        current_time = self.get_clock().now()

        if self.current_state is not None and self.desired_trajectory is not None:
            # Calculate desired state for current time
            desired_state = self.interpolate_trajectory(
                current_time,
                self.desired_trajectory
            )

            # Compute control command
            control_command = self.compute_control_command(
                self.current_state,
                desired_state
            )

            # Publish command
            self.publish_command(control_command)

            # Update timing
            self.last_control_time = current_time

    def state_callback(self, msg):
        """Update current robot state"""
        self.current_state = {
            'positions': np.array(msg.actual.positions),
            'velocities': np.array(msg.actual.velocities),
            'time': Time.from_msg(msg.header.stamp)
        }
```

### Safety and Emergency Systems

Physical robots require robust safety systems:

```python
class SafetyController(RealTimeController):
    def __init__(self):
        super().__init__()

        # Safety parameters
        self.joint_limits = self.load_joint_limits()
        self.velocity_limits = self.load_velocity_limits()
        self.torque_limits = self.load_torque_limits()

        # Emergency stop publisher
        self.emergency_stop_pub = self.create_publisher(
            Bool, 'emergency_stop', 10
        )

        # Safety timer for watchdog
        self.safety_timer = self.create_timer(
            0.1,  # Check every 100ms
            self.safety_check
        )

    def safety_check(self):
        """Perform safety checks"""
        if self.current_state is not None:
            # Check joint limits
            if self.exceeds_joint_limits():
                self.trigger_safety_stop("Joint limit exceeded")
                return

            # Check velocity limits
            if self.exceeds_velocity_limits():
                self.trigger_safety_stop("Velocity limit exceeded")
                return

            # Check torque limits
            if self.exceeds_torque_limits():
                self.trigger_safety_stop("Torque limit exceeded")
                return

    def exceeds_joint_limits(self):
        """Check if current position exceeds joint limits"""
        for i, pos in enumerate(self.current_state['positions']):
            if (pos < self.joint_limits[i]['min'] or
                pos > self.joint_limits[i]['max']):
                return True
        return False

    def trigger_safety_stop(self, reason):
        """Trigger emergency stop"""
        self.get_logger().error(f"Safety stop triggered: {reason}")

        # Publish emergency stop command
        stop_msg = Bool()
        stop_msg.data = True
        self.emergency_stop_pub.publish(stop_msg)

        # Stop all motion
        self.stop_all_joints()
```

## Joint Control Strategies

### Position Control

Position control is the most common approach for humanoid robots:

```python
class PositionController:
    def __init__(self, kp=100.0, ki=10.0, kd=10.0, dt=0.001):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.dt = dt  # Time step

        # PID state
        self.error_integral = 0.0
        self.previous_error = 0.0

    def compute_command(self, current_pos, desired_pos, current_vel=0.0):
        """Compute position control command"""
        error = desired_pos - current_pos

        # Proportional term
        p_term = self.kp * error

        # Integral term
        self.error_integral += error * self.dt
        i_term = self.ki * self.error_integral

        # Derivative term
        derivative = (error - self.previous_error) / self.dt
        d_term = self.kd * derivative

        # Update previous error
        self.previous_error = error

        # Compute command
        command = p_term + i_term + d_term

        return command
```

### Velocity Control

For certain applications, velocity control is more appropriate:

```python
class VelocityController:
    def __init__(self, kp=50.0, ki=5.0, kd=5.0, dt=0.001):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt

        self.velocity_integral = 0.0
        self.previous_velocity_error = 0.0

    def compute_command(self, current_vel, desired_vel):
        """Compute velocity control command"""
        error = desired_vel - current_vel

        # Proportional term
        p_term = self.kp * error

        # Integral term
        self.velocity_integral += error * self.dt
        i_term = self.ki * self.velocity_integral

        # Derivative term
        derivative = (error - self.previous_velocity_error) / self.dt
        d_term = self.kd * derivative

        self.previous_velocity_error = error

        return p_term + i_term + d_term
```

### Impedance Control

Impedance control is crucial for safe human-robot interaction:

```python
class ImpedanceController:
    def __init__(self, stiffness=1000.0, damping=200.0, dt=0.001):
        self.stiffness = stiffness  # Spring constant
        self.damping = damping      # Damping coefficient
        self.dt = dt

        # Desired equilibrium position
        self.equilibrium_pos = 0.0

    def compute_force(self, current_pos, current_vel, external_force=0.0):
        """Compute impedance control force"""
        # Spring force: F = k * (x_desired - x_current)
        spring_force = self.stiffness * (self.equilibrium_pos - current_pos)

        # Damping force: F = c * (v_desired - v_current)
        damping_force = self.damping * (0.0 - current_vel)  # Assuming desired vel = 0

        # Total impedance force
        total_force = spring_force + damping_force + external_force

        return total_force

    def update_equilibrium(self, new_equilibrium):
        """Update desired equilibrium position"""
        self.equilibrium_pos = new_equilibrium
```

## Whole-Body Control

### Center of Mass Control

For humanoid robots, CoM control is critical for balance:

```python
class CenterOfMassController:
    def __init__(self, robot_model):
        self.robot_model = robot_model
        self.desired_com = np.array([0.0, 0.0, 0.0])
        self.com_weights = np.array([1.0, 1.0, 0.5])  # Less weight on Z

    def compute_com(self, joint_positions):
        """Compute center of mass given joint positions"""
        total_mass = 0.0
        com = np.zeros(3)

        # Compute CoM using robot model
        for link in self.robot_model.links:
            if link.mass > 0:
                link_pose = self.robot_model.compute_link_pose(
                    link.name, joint_positions
                )
                link_com = link_pose.translation + link.com_offset
                com += link.mass * link_com
                total_mass += link.mass

        if total_mass > 0:
            com /= total_mass

        return com

    def compute_balance_control(self, current_com, desired_com):
        """Compute control commands to maintain balance"""
        com_error = desired_com - current_com

        # Weighted error
        weighted_error = com_error * self.com_weights

        # Compute required joint torques to move CoM
        jacobian = self.compute_com_jacobian()
        torques = np.dot(jacobian.T, weighted_error)

        return torques
```

### Zero Moment Point (ZMP) Control

ZMP is crucial for bipedal locomotion:

```python
class ZMPController:
    def __init__(self, robot_mass, gravity=9.81):
        self.robot_mass = robot_mass
        self.gravity = gravity
        self.zmp_threshold = 0.05  # 5cm tolerance

    def compute_zmp(self, com_pos, com_acc):
        """Compute Zero Moment Point"""
        zmp_x = com_pos[0] - (com_acc[0] * com_pos[2]) / (self.gravity + com_acc[2])
        zmp_y = com_pos[1] - (com_acc[1] * com_pos[2]) / (self.gravity + com_acc[2])

        return np.array([zmp_x, zmp_y])

    def check_balance(self, zmp_pos, support_polygon):
        """Check if ZMP is within support polygon"""
        # Simple rectangular support polygon check
        min_x, max_x = support_polygon['x_range']
        min_y, max_y = support_polygon['y_range']

        return (min_x <= zmp_pos[0] <= max_x and
                min_y <= zmp_pos[1] <= max_y)

    def compute_balance_correction(self, current_zmp, desired_zmp):
        """Compute balance correction commands"""
        zmp_error = desired_zmp - current_zmp

        # Generate corrective motion
        corrective_torques = self.balance_gains * zmp_error

        return corrective_torques
```

## Sensor Integration and Filtering

### Kalman Filtering for Sensor Fusion

Real robots require sensor fusion for accurate state estimation:

```python
import numpy as np

class KalmanFilter:
    def __init__(self, state_dim, measurement_dim):
        self.state_dim = state_dim
        self.measurement_dim = measurement_dim

        # State: [position, velocity]
        self.x = np.zeros(state_dim)  # State vector
        self.P = np.eye(state_dim)    # Covariance matrix
        self.Q = np.eye(state_dim) * 0.1  # Process noise
        self.R = np.eye(measurement_dim) * 0.5  # Measurement noise

        # State transition matrix (for constant velocity model)
        self.F = np.array([[1, 0.001], [0, 1]])  # dt = 0.001s
        self.H = np.array([[1, 0]])  # Measurement matrix

    def predict(self):
        """Prediction step"""
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def update(self, measurement):
        """Update step"""
        # Innovation
        y = measurement - np.dot(self.H, self.x)

        # Innovation covariance
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R

        # Kalman gain
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))

        # Update state
        self.x = self.x + np.dot(K, y)

        # Update covariance
        I_KH = np.eye(self.state_dim) - np.dot(K, self.H)
        self.P = np.dot(I_KH, self.P)

class SensorFusionController:
    def __init__(self):
        self.position_kf = KalmanFilter(state_dim=2, measurement_dim=1)
        self.imu_sub = None  # IMU subscriber
        self.encoder_sub = None  # Encoder subscriber

    def integrate_sensor_data(self, encoder_pos, imu_vel):
        """Integrate encoder and IMU data"""
        # Use encoder for position measurement
        self.position_kf.update(np.array([encoder_pos]))

        # Use IMU for velocity correction
        self.position_kf.x[1] = 0.7 * self.position_kf.x[1] + 0.3 * imu_vel

        # Return filtered position and velocity
        return self.position_kf.x[0], self.position_kf.x[1]
```

## Force Control and Compliance

### Force/Torque Control

For manipulation tasks, force control is essential:

```python
class ForceController:
    def __init__(self, desired_force, stiffness=1000.0):
        self.desired_force = desired_force
        self.stiffness = stiffness
        self.force_error_integral = 0.0
        self.previous_force_error = 0.0

    def compute_position_adjustment(self, current_force, current_position):
        """Compute position adjustment based on force error"""
        force_error = self.desired_force - current_force

        # Integrate error
        self.force_error_integral += force_error * 0.001  # dt = 1ms

        # PID control for force
        p_term = (1.0/self.stiffness) * force_error
        i_term = (0.1/self.stiffness) * self.force_error_integral
        d_term = (0.01/self.stiffness) * (
            (force_error - self.previous_force_error) / 0.001
        )

        self.previous_force_error = force_error

        # Compute position adjustment
        position_adjustment = p_term + i_term + d_term

        return current_position + position_adjustment
```

### Adaptive Control

Adaptive control adjusts parameters based on changing conditions:

```python
class AdaptiveController:
    def __init__(self, initial_kp=100.0, initial_ki=10.0, initial_kd=10.0):
        self.kp = initial_kp
        self.ki = initial_ki
        self.kd = initial_kd

        # Adaptation parameters
        self.adaptation_rate = 0.01
        self.error_history = []
        self.max_history = 100

    def adapt_parameters(self, current_error):
        """Adapt control parameters based on error history"""
        # Add current error to history
        self.error_history.append(current_error)
        if len(self.error_history) > self.max_history:
            self.error_history.pop(0)

        if len(self.error_history) >= 10:
            # Calculate error statistics
            avg_error = np.mean(self.error_history)
            error_variance = np.var(self.error_history)

            # Adapt gains based on error characteristics
            if error_variance > 0.1:  # High variance indicates oscillation
                self.kp *= 0.95  # Reduce proportional gain
                self.kd *= 1.05  # Increase derivative gain
            elif abs(avg_error) > 0.05:  # Steady-state error
                self.ki *= 1.02  # Increase integral gain

    def compute_command(self, current_pos, desired_pos, current_vel=0.0):
        """Compute command with adaptive parameters"""
        error = desired_pos - current_pos

        # Adapt parameters
        self.adapt_parameters(error)

        # Standard PID computation with adapted parameters
        p_term = self.kp * error
        i_term = self.ki * np.sum(self.error_history) * 0.001
        d_term = self.kd * (error - self.previous_error) / 0.001

        self.previous_error = error

        return p_term + i_term + d_term
```

## Hardware-Specific Considerations

### Motor Control Interfaces

Different motor types require different control approaches:

```python
class MotorInterface:
    def __init__(self, motor_type, hardware_params):
        self.motor_type = motor_type  # 'servo', 'stepper', 'brushless'
        self.params = hardware_params
        self.temperature = 0.0
        self.current_draw = 0.0

    def set_position(self, position):
        """Set motor position"""
        if self.motor_type == 'servo':
            # Send position command to servo
            self.send_servo_command(position)
        elif self.motor_type == 'brushless':
            # Use PID control with encoder feedback
            self.send_brushless_command(position)

    def check_motor_health(self):
        """Check motor health parameters"""
        health_status = {
            'temperature': self.temperature,
            'current': self.current_draw,
            'voltage': self.get_voltage(),
            'overheating': self.temperature > self.params['max_temp'],
            'overcurrent': self.current_draw > self.params['max_current']
        }
        return health_status

class HardwareAbstractionLayer:
    def __init__(self):
        self.motors = {}  # Dictionary of motor interfaces
        self.sensors = {}  # Dictionary of sensor interfaces

    def initialize_motors(self, motor_configs):
        """Initialize all motors"""
        for motor_id, config in motor_configs.items():
            self.motors[motor_id] = MotorInterface(
                config['type'],
                config['parameters']
            )

    def update_hardware_state(self):
        """Update state from all hardware components"""
        for motor_id, motor in self.motors.items():
            # Check motor health
            health = motor.check_motor_health()
            if health['overheating'] or health['overcurrent']:
                self.trigger_motor_protection(motor_id)

    def trigger_motor_protection(self, motor_id):
        """Trigger protection for overheating motor"""
        self.get_logger().warn(f"Motor {motor_id} protection triggered")
        self.motors[motor_id].set_position(self.get_safe_position(motor_id))
```

## Control System Validation

### Hardware-in-the-Loop Testing

Testing control systems with real hardware:

```python
class HILTestController:
    def __init__(self):
        self.simulation_running = False
        self.hardware_connected = False
        self.test_results = []

    def run_hil_test(self, test_trajectory, test_duration):
        """Run hardware-in-the-loop test"""
        start_time = time.time()

        while time.time() - start_time < test_duration:
            # Get current state from hardware
            hardware_state = self.get_hardware_state()

            # Get desired state from simulation
            sim_state = self.get_simulation_state()

            # Compute control command
            command = self.compute_control_command(
                hardware_state,
                sim_state
            )

            # Apply command to hardware
            self.apply_command_to_hardware(command)

            # Record results
            self.record_test_result(hardware_state, sim_state, command)

            # Small delay to maintain timing
            time.sleep(0.001)

    def validate_control_performance(self):
        """Validate control performance metrics"""
        # Calculate tracking error
        tracking_errors = [
            result['error'] for result in self.test_results
        ]

        # Calculate performance metrics
        rmse = np.sqrt(np.mean(np.square(tracking_errors)))
        max_error = np.max(np.abs(tracking_errors))
        avg_error = np.mean(np.abs(tracking_errors))

        return {
            'rmse': rmse,
            'max_error': max_error,
            'avg_error': avg_error,
            'success_rate': self.calculate_success_rate()
        }
```

## Safety and Emergency Procedures

### Emergency Stop Systems

Critical for physical robot safety:

```python
class EmergencySystem:
    def __init__(self):
        self.emergency_active = False
        self.emergency_reason = ""
        self.emergency_timer = None

    def activate_emergency_stop(self, reason="Unknown"):
        """Activate emergency stop"""
        self.emergency_active = True
        self.emergency_reason = reason

        # Stop all motors
        self.stop_all_motors()

        # Log emergency
        self.log_emergency_event(reason)

        # Start recovery timer
        self.emergency_timer = time.time()

    def deactivate_emergency_stop(self):
        """Deactivate emergency stop"""
        self.emergency_active = False
        self.emergency_reason = ""
        self.emergency_timer = None

    def check_emergency_conditions(self):
        """Check for emergency conditions"""
        # Check for collision
        if self.detect_collision():
            self.activate_emergency_stop("Collision detected")
            return True

        # Check for joint limit violation
        if self.exceeds_joint_limits():
            self.activate_emergency_stop("Joint limit exceeded")
            return True

        # Check for excessive temperature
        if self.detect_overheating():
            self.activate_emergency_stop("Overheating detected")
            return True

        return False

    def stop_all_motors(self):
        """Send stop command to all motors"""
        for motor_id in self.motors:
            self.motors[motor_id].set_position(self.get_current_position(motor_id))
```

## Best Practices

### Design Principles

1. **Modularity**: Keep control components modular and testable
2. **Safety First**: Implement safety systems as the highest priority
3. **Real-time Constraints**: Meet timing requirements consistently
4. **Robustness**: Handle sensor failures and unexpected conditions
5. **Maintainability**: Write clear, well-documented code

### Performance Optimization

1. **Efficient Algorithms**: Use computationally efficient control algorithms
2. **Memory Management**: Minimize memory allocation in control loops
3. **Communication Efficiency**: Optimize message passing and data transfer
4. **Hardware Utilization**: Make efficient use of available hardware resources

## Learning Objectives

After studying this chapter, students should be able to:

1. Implement real-time control systems with proper timing
2. Apply different control strategies (position, velocity, impedance)
3. Design safety and emergency stop systems
4. Integrate sensor data for state estimation
5. Implement force control and compliance
6. Validate control systems with real hardware

## Prerequisites

- Understanding of control theory fundamentals
- Knowledge of ROS 2 control interfaces
- Basic understanding of robot kinematics
- Familiarity with real-time systems concepts

## References

1. Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2020). *Robot Modeling and Control* (2nd ed.). Wiley.
2. Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control* (3rd ed.). Pearson.
3. Siciliano, B., & Khatib, O. (2016). *Springer Handbook of Robotics*. Springer. Chapter on Robot Control.
4. Murray, R. M., Li, Z., & Sastry, S. S. (1994). *A Mathematical Introduction to Robotic Manipulation*. CRC Press.

## Exercises

1. Implement a PID controller for a single joint
2. Create a safety system for a robotic arm
3. Design a sensor fusion system using Kalman filtering
4. Implement impedance control for safe human interaction
5. Create a ZMP-based balance controller for a bipedal robot
6. Validate a control system using hardware-in-the-loop testing