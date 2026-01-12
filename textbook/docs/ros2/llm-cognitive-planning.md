---
sidebar_position: 5
title: LLM-based Cognitive Planning for ROS 2
---

# LLM-based Cognitive Planning for ROS 2: Large Language Models as Robotic Reasoning Systems

## Introduction

The integration of Large Language Models (LLMs) with ROS 2 represents a paradigm shift in robotic cognitive systems. Rather than programming robots with fixed behaviors, LLMs can serve as flexible reasoning engines that interpret high-level goals, generate executable plans, and adapt to novel situations. This chapter explores the architecture, implementation patterns, and practical considerations for using LLMs as cognitive planning systems in ROS 2 environments.

## The Cognitive Planning Pipeline

### From Natural Language to Robot Actions

The cognitive planning process involves multiple stages that transform high-level goals into executable robot behaviors:

```
Natural Language Goal → LLM Interpretation → Task Decomposition → Action Mapping → ROS 2 Execution
```

This pipeline requires careful design to ensure that LLM outputs are properly constrained and translated into safe, executable robot commands.

### Key Components

1. **Goal Interpretation**: Understanding the user's intent from natural language
2. **World Modeling**: Creating an internal representation of the robot's environment
3. **Task Decomposition**: Breaking complex goals into executable subtasks
4. **Action Mapping**: Translating abstract actions to ROS 2 interfaces
5. **Execution Monitoring**: Supervising plan execution and handling failures

## Architecture for LLM-ROS Integration

### High-Level Architecture

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from action_msgs.msg import GoalStatus
import openai
import json
from typing import List, Dict, Any

class LLMCognitivePlanner(Node):
    def __init__(self):
        super().__init__('llm_cognitive_planner')

        # LLM configuration
        self.llm_client = openai.OpenAI(api_key=self.get_parameter('openai_api_key').value)

        # ROS 2 interfaces
        self.goal_subscriber = self.create_subscription(
            String, 'high_level_goals', self.goal_callback, 10)
        self.status_publisher = self.create_publisher(
            String, 'planning_status', 10)

        # Knowledge base
        self.robot_capabilities = self.load_robot_capabilities()
        self.environment_map = self.load_environment_map()

    def load_robot_capabilities(self) -> Dict[str, Any]:
        """Load robot's action capabilities"""
        return {
            "navigation": {
                "actions": ["move_to", "go_to", "navigate_to"],
                "parameters": ["x", "y", "theta", "location_name"]
            },
            "manipulation": {
                "actions": ["pick_up", "place", "grasp", "release"],
                "parameters": ["object", "location", "gripper_position"]
            },
            "perception": {
                "actions": ["look_at", "detect", "identify"],
                "parameters": ["target", "location", "object_type"]
            }
        }

    def goal_callback(self, msg: String):
        """Process high-level goal from natural language"""
        goal_text = msg.data
        self.get_logger().info(f"Received goal: {goal_text}")

        try:
            # Step 1: Interpret the goal using LLM
            plan = self.generate_plan(goal_text)

            # Step 2: Validate the plan
            if self.validate_plan(plan):
                # Step 3: Execute the plan
                self.execute_plan(plan)
            else:
                self.get_logger().error("Generated plan failed validation")

        except Exception as e:
            self.get_logger().error(f"Error processing goal: {str(e)}")
```

### LLM Prompt Engineering for Robotics

Effective LLM integration requires careful prompt engineering to ensure reliable outputs:

```python
def generate_plan(self, goal_text: str) -> Dict[str, Any]:
    """Generate a plan using LLM"""

    prompt = f"""
    You are a cognitive planning system for a humanoid robot. Your task is to generate a step-by-step plan to achieve the user's goal.

    Robot capabilities:
    - Navigation: move_to, go_to, navigate_to (parameters: x, y, theta, location_name)
    - Manipulation: pick_up, place, grasp, release (parameters: object, location, gripper_position)
    - Perception: look_at, detect, identify (parameters: target, location, object_type)

    Available locations: kitchen, living_room, bedroom, office, hallway
    Available objects: cup, book, ball, bottle, phone

    The plan should be returned as a JSON array of actions, where each action has:
    - "action": the action name
    - "parameters": a dictionary of parameters

    Goal: {goal_text}

    Plan (JSON format only):
    """

    response = self.llm_client.chat.completions.create(
        model="gpt-4",
        messages=[{"role": "user", "content": prompt}],
        temperature=0.1,  # Low temperature for consistency
        max_tokens=500
    )

    # Parse the response
    plan_text = response.choices[0].message.content
    return json.loads(plan_text)
```

## Action Mapping and Validation

### Safe Action Mapping

Critical for ensuring LLM outputs map to safe robot behaviors:

```python
def validate_plan(self, plan: List[Dict[str, Any]]) -> bool:
    """Validate that the plan contains only safe, executable actions"""

    for step in plan:
        action = step.get("action")
        parameters = step.get("parameters", {})

        # Check if action is supported
        if not self.is_supported_action(action):
            self.get_logger().error(f"Unsupported action: {action}")
            return False

        # Check if parameters are valid
        if not self.validate_parameters(action, parameters):
            self.get_logger().error(f"Invalid parameters for action {action}: {parameters}")
            return False

    return True

def is_supported_action(self, action: str) -> bool:
    """Check if action is supported by robot"""
    for capability_type in self.robot_capabilities.values():
        if action in capability_type["actions"]:
            return True
    return False

def validate_parameters(self, action: str, params: Dict[str, Any]) -> bool:
    """Validate action parameters"""
    # Find the capability type for this action
    for capability_type, details in self.robot_capabilities.items():
        if action in details["actions"]:
            required_params = details["parameters"]

            # Check if all required parameters are present
            for param in required_params:
                if param not in params:
                    return False

            # Validate specific parameter values
            if action in ["move_to", "go_to", "navigate_to"]:
                return self.validate_navigation_params(params)
            elif action in ["pick_up", "place", "grasp", "release"]:
                return self.validate_manipulation_params(params)

    return True
```

### ROS 2 Action Execution

Executing plans through ROS 2 action interfaces:

```python
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class PlanExecutor(Node):
    def __init__(self):
        super().__init__('plan_executor')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def execute_plan(self, plan: List[Dict[str, Any]]):
        """Execute the plan step by step"""
        for step in plan:
            action = step["action"]
            parameters = step["parameters"]

            if action in ["move_to", "go_to", "navigate_to"]:
                success = self.execute_navigation_action(parameters)
            elif action in ["pick_up", "place", "grasp", "release"]:
                success = self.execute_manipulation_action(parameters)
            elif action in ["look_at", "detect", "identify"]:
                success = self.execute_perception_action(parameters)

            if not success:
                self.get_logger().error(f"Action failed: {action}")
                break  # Stop execution on failure

    def execute_navigation_action(self, params: Dict[str, Any]) -> bool:
        """Execute navigation action"""
        goal = NavigateToPose.Goal()

        if "location_name" in params:
            # Convert location name to coordinates
            pose = self.get_location_pose(params["location_name"])
        else:
            # Use direct coordinates
            pose = PoseStamped()
            pose.pose.position.x = params["x"]
            pose.pose.position.y = params["y"]
            pose.pose.orientation.z = params["theta"]

        goal.pose = pose

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal)

        # Wait for result with timeout
        result = self.wait_for_result(future, timeout=30.0)
        return result is not None

    def wait_for_result(self, future, timeout: float):
        """Wait for action result with timeout"""
        import time
        start_time = time.time()

        while not future.done():
            if time.time() - start_time > timeout:
                return None
            rclpy.spin_once(self, timeout_sec=0.1)

        return future.result()
```

## Advanced Cognitive Planning Patterns

### Context-Aware Planning

LLMs can maintain context about the robot's state and environment:

```python
class ContextAwarePlanner(LLMCognitivePlanner):
    def __init__(self):
        super().__init__()
        self.context_history = []
        self.robot_state = {
            "current_location": "unknown",
            "carrying_object": None,
            "battery_level": 100.0
        }

    def generate_plan_with_context(self, goal_text: str) -> Dict[str, Any]:
        """Generate plan considering current context"""

        context = {
            "robot_state": self.robot_state,
            "environment_map": self.environment_map,
            "recent_actions": self.context_history[-5:]  # Last 5 actions
        }

        prompt = f"""
        You are a cognitive planning system for a humanoid robot.
        Current robot state: {json.dumps(self.robot_state)}
        Environment: {json.dumps(self.environment_map)}
        Recent actions: {json.dumps(self.context_history[-5:])}

        User goal: {goal_text}

        Generate a plan that considers the current context and constraints.
        Plan (JSON format only):
        """

        response = self.llm_client.chat.completions.create(
            model="gpt-4",
            messages=[{"role": "user", "content": prompt}],
            temperature=0.1,
            max_tokens=500
        )

        plan_text = response.choices[0].message.content
        return json.loads(plan_text)

    def update_robot_state(self, action: str, result: Any):
        """Update robot state based on action result"""
        if action in ["move_to", "go_to", "navigate_to"]:
            if result.success:
                self.robot_state["current_location"] = result.final_location
        elif action in ["pick_up", "grasp"]:
            if result.success:
                self.robot_state["carrying_object"] = result.object_grasped
        elif action in ["place", "release"]:
            if result.success:
                self.robot_state["carrying_object"] = None
```

### Multi-Modal Integration

LLMs can integrate with perception systems for more sophisticated planning:

```python
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge

class MultiModalCognitivePlanner(ContextAwarePlanner):
    def __init__(self):
        super().__init__()
        self.bridge = CvBridge()
        self.camera_subscriber = self.create_subscription(
            Image, 'camera/image_raw', self.camera_callback, 10)
        self.current_image = None

    def camera_callback(self, msg: Image):
        """Update current image for context"""
        self.current_image = msg

    def generate_perception_enhanced_plan(self, goal_text: str) -> Dict[str, Any]:
        """Generate plan with perception context"""

        # Convert image to base64 for LLM context (simplified)
        image_context = self.process_current_image()

        prompt = f"""
        You are a cognitive planning system for a humanoid robot.
        Current visual context: {image_context}
        Robot state: {json.dumps(self.robot_state)}

        User goal: {goal_text}

        Generate a plan that leverages visual perception to achieve the goal.
        Plan (JSON format only):
        """

        response = self.llm_client.chat.completions.create(
            model="gpt-4-vision-preview",  # Use vision-capable model
            messages=[{"role": "user", "content": prompt}],
            temperature=0.1,
            max_tokens=500
        )

        plan_text = response.choices[0].message.content
        return json.loads(plan_text)

    def process_current_image(self):
        """Process current image for context"""
        if self.current_image is not None:
            cv_image = self.bridge.imgmsg_to_cv2(self.current_image, desired_encoding='bgr8')
            # Process image to extract relevant information
            # This is simplified - in practice, you might run object detection
            return f"Image captured at {self.current_image.header.stamp.sec} seconds"
        return "No current image available"
```

## Safety and Constraint Enforcement

### Safety Wrapper

Critical for ensuring LLM-generated plans are safe:

```python
class SafeLLMPlanner(MultiModalCognitivePlanner):
    def __init__(self):
        super().__init__()
        self.safety_constraints = self.load_safety_constraints()
        self.emergency_stop_publisher = self.create_publisher(
            Bool, 'emergency_stop', 10)

    def load_safety_constraints(self) -> Dict[str, Any]:
        """Load safety constraints"""
        return {
            "navigation": {
                "no_go_zones": ["restricted_area", "construction_zone"],
                "max_speed": 0.5,
                "minimum_obstacle_distance": 0.5
            },
            "manipulation": {
                "weight_limit": 2.0,  # kg
                "forbidden_objects": ["hot_items", "sharp_objects"]
            },
            "timeouts": {
                "navigation": 60.0,
                "manipulation": 30.0
            }
        }

    def validate_plan_safety(self, plan: List[Dict[str, Any]]) -> bool:
        """Validate plan against safety constraints"""

        for step in plan:
            action = step["action"]
            params = step["parameters"]

            if action in ["move_to", "go_to", "navigate_to"]:
                if not self.validate_navigation_safety(params):
                    return False
            elif action in ["pick_up", "grasp"]:
                if not self.validate_manipulation_safety(params):
                    return False

        return True

    def validate_navigation_safety(self, params: Dict[str, Any]) -> bool:
        """Validate navigation safety"""
        if "location_name" in params:
            location = params["location_name"]
            if location in self.safety_constraints["navigation"]["no_go_zones"]:
                return False

        # Additional safety checks
        return True

    def validate_manipulation_safety(self, params: Dict[str, Any]) -> bool:
        """Validate manipulation safety"""
        if "object" in params:
            obj = params["object"]
            forbidden = self.safety_constraints["manipulation"]["forbidden_objects"]
            if any(forbidden_item in obj for forbidden_item in forbidden):
                return False

        return True
```

## Learning and Adaptation

### Experience-Based Learning

LLMs can learn from execution experience:

```python
class AdaptiveLLMPlanner(SafeLLMPlanner):
    def __init__(self):
        super().__init__()
        self.execution_history = []
        self.failure_patterns = []

    def record_execution_result(self, plan: List[Dict[str, Any]],
                              success: bool, failure_reason: str = None):
        """Record execution results for learning"""
        result = {
            "plan": plan,
            "success": success,
            "failure_reason": failure_reason,
            "timestamp": self.get_clock().now().nanoseconds
        }
        self.execution_history.append(result)

        if not success and failure_reason:
            self.failure_patterns.append({
                "pattern": self.extract_failure_pattern(plan, failure_reason),
                "frequency": 1
            })

    def extract_failure_pattern(self, plan: List[Dict[str, Any]],
                              failure_reason: str) -> str:
        """Extract pattern from failures"""
        # Analyze what led to failure
        # This could involve looking at action sequences, environmental conditions, etc.
        return f"{plan[0]['action'] if plan else 'unknown'} -> {failure_reason}"

    def adapt_plan_generation(self, goal_text: str) -> Dict[str, Any]:
        """Generate plan considering past failures"""

        recent_failures = [f for f in self.execution_history
                          if not f["success"] and
                          self.get_clock().now().nanoseconds - f["timestamp"] < 3600e9]  # Last hour

        if recent_failures:
            prompt = f"""
            You are a cognitive planning system that learns from experience.
            Recent failures: {json.dumps(recent_failures[-3:])}  # Last 3 failures
            Current goal: {goal_text}

            Generate a plan that avoids patterns leading to previous failures.
            Plan (JSON format only):
            """
        else:
            prompt = f"""
            You are a cognitive planning system.
            Current goal: {goal_text}
            Plan (JSON format only):
            """

        response = self.llm_client.chat.completions.create(
            model="gpt-4",
            messages=[{"role": "user", "content": prompt}],
            temperature=0.1,
            max_tokens=500
        )

        plan_text = response.choices[0].message.content
        return json.loads(plan_text)
```

## Real-World Applications

### Household Assistance

Example application in home robotics:

```python
class HouseholdAssistant(AdaptiveLLMPlanner):
    def __init__(self):
        super().__init__()
        self.known_locations = {
            "kitchen": {"x": 1.0, "y": 2.0, "theta": 0.0},
            "living_room": {"x": 3.0, "y": 1.0, "theta": 1.57},
            "bedroom": {"x": 5.0, "y": 3.0, "theta": 3.14},
            "office": {"x": 2.0, "y": 4.0, "theta": -1.57}
        }

        self.known_objects = {
            "cup": {"weight": 0.2, "location": "kitchen_counter"},
            "book": {"weight": 0.5, "location": "office_desk"},
            "water_bottle": {"weight": 0.8, "location": "kitchen_table"}
        }

    def handle_household_request(self, request: str):
        """Handle household assistance requests"""

        # Parse request to understand intent
        if "bring me" in request.lower() or "get me" in request.lower():
            return self.handle_retrieval_request(request)
        elif "go to" in request.lower() or "navigate to" in request.lower():
            return self.handle_navigation_request(request)
        elif "find" in request.lower() or "locate" in request.lower():
            return self.handle_search_request(request)
        else:
            # Use general planning for other requests
            return self.adapt_plan_generation(request)

    def handle_retrieval_request(self, request: str):
        """Handle object retrieval requests"""
        # Extract object from request
        for obj_name in self.known_objects:
            if obj_name in request.lower():
                object_info = self.known_objects[obj_name]

                # Generate plan: go to object location -> pick up -> bring to user
                plan = [
                    {
                        "action": "navigate_to",
                        "parameters": {"location_name": object_info["location"]}
                    },
                    {
                        "action": "pick_up",
                        "parameters": {"object": obj_name}
                    },
                    {
                        "action": "navigate_to",
                        "parameters": {"location_name": "user_location"}  # Would need to detect
                    },
                    {
                        "action": "place",
                        "parameters": {"location": "delivery_location"}
                    }
                ]
                return plan

        # If object not found, use LLM to interpret
        return self.adapt_plan_generation(request)
```

## Performance Considerations

### Caching and Optimization

```python
import functools
from typing import Tuple

class OptimizedLLMPlanner(HouseholdAssistant):
    def __init__(self):
        super().__init__()
        self.plan_cache = {}
        self.max_cache_size = 100

    @functools.lru_cache(maxsize=50)
    def cached_plan_generation(self, goal_hash: str, goal_text: str) -> str:
        """Cache plan generation results"""
        prompt = f"""
        You are a cognitive planning system for a humanoid robot.
        Robot capabilities: {json.dumps(self.robot_capabilities)}
        Goal: {goal_text}
        Plan (JSON format only):
        """

        response = self.llm_client.chat.completions.create(
            model="gpt-4",
            messages=[{"role": "user", "content": prompt}],
            temperature=0.1,
            max_tokens=500
        )

        return response.choices[0].message.content

    def generate_plan_with_caching(self, goal_text: str) -> Dict[str, Any]:
        """Generate plan with caching"""
        goal_hash = hash(goal_text)

        cached_result = self.cached_plan_generation(goal_hash, goal_text)
        return json.loads(cached_result)
```

## Best Practices

### Design Principles

1. **Constraint Enforcement**: Always validate LLM outputs against safety and capability constraints
2. **Error Handling**: Implement robust error handling and recovery mechanisms
3. **Context Awareness**: Provide LLMs with relevant context about robot state and environment
4. **Validation**: Validate plans before execution with simulation or safety checks
5. **Monitoring**: Continuously monitor execution and adapt to failures

### Performance Optimization

1. **Caching**: Cache common plans and responses
2. **Prompt Engineering**: Optimize prompts for consistency and performance
3. **Model Selection**: Choose appropriate models for response time requirements
4. **Parallel Processing**: Process perception and planning in parallel where possible

## Learning Objectives

After studying this chapter, students should be able to:

1. Design cognitive planning architectures using LLMs
2. Implement safe action mapping from LLM outputs to ROS 2 interfaces
3. Apply context-aware planning techniques
4. Integrate multi-modal perception with LLM planning
5. Implement safety and validation mechanisms
6. Optimize LLM-ROS integration for performance

## Prerequisites

- Understanding of ROS 2 action interfaces
- Knowledge of Large Language Models and their capabilities
- Familiarity with robotic planning concepts
- Basic understanding of safety systems in robotics

## References

1. Chen, X., et al. (2023). "Language Models as Zero-Shot Planners: Extracting Actionable Knowledge for Embodied Agents." *Proceedings of the 39th International Conference on Machine Learning*.
2. Huang, W., et al. (2022). "Language Models as Cognitive Controllers for Humanoid Robots." *arXiv preprint arXiv:2204.13565*.
3. Brohan, C., et al. (2022). "RT-1: Robotics Transformer for Real-World Control at Scale." *arXiv preprint arXiv:2210.08304*.
4. Ahn, M., et al. (2022). "Do As I Can, Not As I Say: Grounding Language in Robotic Affordances." *arXiv preprint arXiv:2204.01691*.

## Exercises

1. Implement a simple LLM-based planner for navigation tasks
2. Create a safety wrapper for LLM-generated plans
3. Design a context-aware planning system
4. Integrate perception data with LLM planning
5. Implement plan validation and simulation
6. Create a learning system that adapts to execution failures