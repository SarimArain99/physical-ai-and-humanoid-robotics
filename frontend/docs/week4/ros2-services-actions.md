---
sidebar_position: 4
---

# ROS 2 Services and Actions

This chapter explores advanced communication patterns in ROS 2: services for synchronous request-response communication and actions for goal-oriented, long-running tasks.

## Learning Objectives

By the end of this week, you will be able to:
- Implement ROS 2 services for synchronous communication
- Create and use actions for goal-oriented tasks
- Understand when to use services vs actions vs topics
- Handle feedback and result messages in actions
- Build complex robotic behaviors using action servers

## Introduction to Services

Services provide a request-response communication pattern where a service client sends a request and waits for a response from a service server. This is synchronous communication that blocks until a response is received.

### Service Definition

Services are defined using `.srv` files that specify the request and response types:

```
# Request (before ---)
string name
int32 age
---
# Response (after ---)
bool success
string message
```

### Service Server Implementation

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response

def main():
    rclpy.init()
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client Implementation

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    rclpy.init()
    minimal_client = MinimalClient()
    response = minimal_client.send_request(1, 2)
    minimal_client.get_logger().info(f'Result: {response.sum}')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Service Patterns

### Custom Service Definition

Let's create a custom service for robot navigation:

```python
# Custom service definition: NavigateTo.srv
# geometry_msgs/Point target_point
# ---
# bool success
# string message
# float32 distance_traveled
```

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from my_robot_interfaces.srv import NavigateTo  # Custom service

class NavigationService(Node):
    def __init__(self):
        super().__init__('navigation_service')
        self.srv = self.create_service(
            NavigateTo,
            'navigate_to',
            self.navigate_to_callback
        )

        # Publishers for navigation status
        self.status_pub = self.create_publisher(Bool, 'navigation_active', 10)
        self.target_pub = self.create_publisher(Point, 'navigation_target', 10)

        # Simulate robot position
        self.current_position = Point(x=0.0, y=0.0, z=0.0)

    def navigate_to_callback(self, request, response):
        """Handle navigation request"""
        target = request.target_point

        # Publish navigation status
        status_msg = Bool()
        status_msg.data = True
        self.status_pub.publish(status_msg)

        # Publish target
        self.target_pub.publish(target)

        # Simulate navigation process
        distance = self.calculate_distance(self.current_position, target)

        # In a real implementation, this would involve path planning and execution
        self.get_logger().info(f'Navigating to ({target.x}, {target.y})')

        # Update current position (simulation)
        self.current_position = target

        # Set response
        response.success = True
        response.message = f'Reached target ({target.x}, {target.y})'
        response.distance_traveled = distance

        # Publish completion status
        status_msg.data = False
        self.status_pub.publish(status_msg)

        return response

    def calculate_distance(self, point1, point2):
        """Calculate Euclidean distance between two points"""
        dx = point2.x - point1.x
        dy = point2.y - point1.y
        dz = point2.z - point1.z
        return (dx*dx + dy*dy + dz*dz)**0.5

def main():
    rclpy.init()
    nav_service = NavigationService()

    try:
        rclpy.spin(nav_service)
    except KeyboardInterrupt:
        pass
    finally:
        nav_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Introduction to Actions

Actions are used for long-running tasks that may take significant time to complete. They provide goal, feedback, and result mechanisms, making them ideal for navigation, manipulation, and other complex tasks.

### Action Definition

Actions are defined using `.action` files with three parts:

```
# Goal
int32 order
---
# Result
int32[] sequence
string result
---
# Feedback
int32[] sequence
```

### Action Server Implementation

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result

def main():
    rclpy.init()
    fibonacci_action_server = FibonacciActionServer()
    rclpy.spin(fibonacci_action_server)
    fibonacci_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Action Client Implementation

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.sequence}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()

def main():
    rclpy.init()
    action_client = FibonacciActionClient()
    action_client.send_goal(10)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
```

## Advanced Action Patterns for Robotics

### Navigation Action Server

```python
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math

class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_action_server')

        # TF2 for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Action server
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_navigate_to_pose,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Robot state
        self.current_pose = None
        self.navigation_active = False

    def goal_callback(self, goal_request):
        """Accept or reject navigation goal"""
        self.get_logger().info(f'Received navigation goal: {goal_request.pose.pose}')

        # Check if navigation is already active
        if self.navigation_active:
            self.get_logger().info('Navigation already active, rejecting new goal')
            return GoalResponse.REJECT

        # Validate goal (simple check for now)
        if self.is_valid_goal(goal_request.pose):
            return GoalResponse.ACCEPT
        else:
            return GoalResponse.REJECT

    def cancel_callback(self, goal_handle):
        """Accept or reject navigation cancellation"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def is_valid_goal(self, pose):
        """Validate if the goal pose is reachable"""
        # In a real implementation, check if goal is in valid map area
        # and not in obstacle space
        return True

    def execute_navigate_to_pose(self, goal_handle):
        """Execute navigation action"""
        self.get_logger().info('Starting navigation task')

        # Set navigation active
        self.navigation_active = True

        # Get goal pose
        goal_pose = goal_handle.request.pose
        target_x = goal_pose.pose.position.x
        target_y = goal_pose.pose.position.y

        # Initialize feedback
        feedback_msg = NavigateToPose.Feedback()
        result = NavigateToPose.Result()

        try:
            while rclpy.ok():
                # Check if goal was cancelled
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.navigation_active = False
                    result.result = result.CANCELED
                    return result

                # Get current robot pose
                current_pose = self.get_current_pose()
                if current_pose is None:
                    self.get_logger().warn('Cannot get current pose')
                    continue

                # Calculate distance to goal
                distance = math.sqrt(
                    (target_x - current_pose.position.x)**2 +
                    (target_y - current_pose.position.y)**2
                )

                # Update feedback
                feedback_msg.current_pose = current_pose
                feedback_msg.distance_remaining = distance
                goal_handle.publish_feedback(feedback_msg)

                # Check if goal reached
                if distance < 0.5:  # 50cm tolerance
                    self.get_logger().info('Goal reached successfully')
                    goal_handle.succeed()
                    self.navigation_active = False
                    result.result = result.SUCCEEDED
                    return result

                # Simulate navigation progress (in real system, this would control the robot)
                self.navigate_towards_goal(current_pose, goal_pose)

                # Sleep to control loop rate
                self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))

        except Exception as e:
            self.get_logger().error(f'Navigation failed: {str(e)}')
            goal_handle.abort()
            self.navigation_active = False
            result.result = result.FAILED
            return result

    def get_current_pose(self):
        """Get current robot pose from TF tree"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation

            return pose.pose
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform: {ex}')
            return None

    def navigate_towards_goal(self, current_pose, goal_pose):
        """Simulate navigation towards goal"""
        # In a real implementation, this would send velocity commands
        # to move the robot towards the goal
        pass

def main():
    rclpy.init()
    nav_server = NavigationActionServer()

    try:
        executor = MultiThreadedExecutor()
        executor.add_node(nav_server)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        nav_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## When to Use Each Communication Pattern

| Pattern | Use Case | Characteristics |
|---------|----------|-----------------|
| Topics | Continuous data streams | Asynchronous, broadcast, no acknowledgment |
| Services | Simple request-response | Synchronous, blocking, request-reply |
| Actions | Long-running goal-oriented tasks | Asynchronous with feedback, cancelable, progress reporting |

### Decision Matrix

Use **topics** when:
- Streaming sensor data continuously
- Broadcasting status information
- Publishing control commands at high frequency
- Multiple subscribers need the same data

Use **services** when:
- Need immediate response to a query
- Operation is relatively quick (< 1 second)
- Simple request-response pattern suffices
- Synchronous behavior is acceptable

Use **actions** when:
- Task takes significant time to complete
- Need to provide feedback during execution
- Want to allow cancellation
- Need to report results upon completion

## Parameter Integration with Services and Actions

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.srv import SetParameters

class ParameterizedActionServer(Node):
    def __init__(self):
        super().__init__('parameterized_action_server')

        # Declare parameters with descriptors
        self.declare_parameter(
            'navigation_speed',
            0.5,
            ParameterDescriptor(description='Maximum navigation speed (m/s)')
        )

        self.declare_parameter(
            'safety_distance',
            0.5,
            ParameterDescriptor(description='Minimum safety distance to obstacles (m)')
        )

        # Dynamic parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        """Handle parameter changes"""
        for param in params:
            if param.name == 'navigation_speed':
                if param.value > 2.0:
                    return SetParametersResult(successful=False, reason='Speed too high')
            elif param.name == 'safety_distance':
                if param.value < 0.1:
                    return SetParametersResult(successful=False, reason='Safety distance too small')

        return SetParametersResult(successful=True)
```

## Weekly Hands-On Exercise

Create a complete robotic system using services and actions:

1. Implement a service that accepts waypoints and plans a path
2. Create an action server for executing navigation tasks
3. Build a client that sends navigation goals and monitors progress
4. Add parameter integration to configure navigation behavior
5. Test the system with different goal configurations

## Review Questions

1. What is the main difference between services and actions in ROS 2?
2. When would you use an action instead of a service?
3. How do actions handle long-running tasks with feedback?
4. What are the advantages of using parameters with services and actions?

## Further Reading

- [ROS 2 Actions Documentation](https://docs.ros.org/en/humble/Tutorials/Actions/Creating-an-Action.html)
- "Programming Robots with ROS" by Morgan Quigley
- "Effective Robotics Programming with ROS" by Anil Mahtani