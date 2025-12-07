---
sidebar_position: 5
---

# Advanced ROS 2 Concepts: Launch Files, Parameters, and Lifecycle Nodes

This chapter covers advanced ROS 2 concepts including launch files for system management, parameter systems for configuration, and lifecycle nodes for complex state management in robotic systems.

## Learning Objectives

By the end of this week, you will be able to:
- Create and use complex launch files for system management
- Implement parameter systems for runtime configuration
- Design and implement lifecycle nodes for complex state management
- Optimize ROS 2 systems for performance and reliability
- Debug complex ROS 2 systems with advanced tools

## Advanced Launch Files

Launch files provide a powerful way to manage complex robot systems by starting multiple nodes with specific configurations simultaneously.

### Basic Launch File Structure

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, LifecycleNode, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='my_robot',
        description='Name of the robot'
    )

    # Create nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': use_sim_time},
            PathJoinSubstitution([FindPackageShare('my_robot_description'), 'config', 'robot_params.yaml'])
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_sim_time)  # Only start if using sim time
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_robot_name_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
    ])
```

### Conditional Launch with Events

```python
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart, OnProcessIO
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_testing.actions

def launch_setup(context, *args, **kwargs):
    """Function to create launch description based on arguments"""
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    debug_mode = LaunchConfiguration('debug').perform(context)

    # Conditional node creation
    nodes = []

    # Always start the main robot controller
    robot_controller = Node(
        package='my_robot_control',
        executable='robot_controller',
        name='robot_controller',
        parameters=[
            {'use_sim_time': use_sim_time == 'true'},
            {'debug_mode': debug_mode == 'true'}
        ]
    )
    nodes.append(robot_controller)

    # Add debugging tools only in debug mode
    if debug_mode == 'true':
        debug_node = Node(
            package='my_robot_debug',
            executable='debug_monitor',
            name='debug_monitor',
            parameters=[{'use_sim_time': use_sim_time == 'true'}]
        )
        nodes.append(debug_node)

    return nodes

def generate_launch_description():
    # Declare launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    declare_debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug mode'
    )

    # Create launch description with conditional nodes
    opaque_function = OpaqueFunction(function=launch_setup)

    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_debug_arg,
        opaque_function,
    ])
```

### Composable Nodes (Components)

Composable nodes allow multiple nodes to run in the same process, reducing communication overhead:

```python
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Create a container for composable nodes
    container = ComposableNodeContainer(
        name='image_processing_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_node',
                parameters=[{'use_sim_time': True}],
                remappings=[
                    ('image', '/camera/image_raw'),
                    ('camera_info', '/camera/camera_info'),
                    ('image_rect', '/camera/image_rect')
                ]
            ),
            ComposableNode(
                package='cv_bridge',
                plugin='cv_bridge::CvtColorNode',
                name='cvt_color_node',
                parameters=[{'use_sim_time': True}],
                remappings=[
                    ('image_raw', '/camera/image_rect'),
                    ('image', '/camera/image_bgr')
                ]
            ),
            ComposableNode(
                package='my_vision_package',
                plugin='my_vision_package::ObjectDetectionNode',
                name='object_detection_node',
                parameters=[{'use_sim_time': True}],
                remappings=[
                    ('image', '/camera/image_bgr'),
                    ('detections', '/camera/detections')
                ]
            )
        ],
        output='screen'
    )

    return LaunchDescription([container])
```

## Advanced Parameter Systems

### Parameter Declaration and Management

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange
from rcl_interfaces.srv import SetParameters

class AdvancedParameterNode(Node):
    def __init__(self):
        super().__init__('advanced_parameter_node')

        # Declare parameters with descriptors and ranges
        self.declare_parameter(
            'robot.radius',
            0.3,
            ParameterDescriptor(
                description='Robot radius in meters',
                floating_point_range=[FloatingPointRange(from_value=0.1, to_value=2.0, step=0.01)]
            )
        )

        self.declare_parameter(
            'navigation.max_speed',
            1.0,
            ParameterDescriptor(
                description='Maximum navigation speed in m/s',
                floating_point_range=[FloatingPointRange(from_value=0.1, to_value=5.0, step=0.1)]
            )
        )

        self.declare_parameter(
            'control.frequency',
            50,
            ParameterDescriptor(
                description='Control loop frequency in Hz',
                integer_range=[IntegerRange(from_value=10, to_value=200, step=1)]
            )
        )

        # Declare parameter with different types
        self.declare_parameter('robot.name', 'my_robot')
        self.declare_parameter('robot.enabled', True)
        self.declare_parameter('robot.joint_names', ['joint1', 'joint2', 'joint3'])

        # Add callback for parameter changes
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Timer to periodically check parameter changes
        self.param_check_timer = self.create_timer(1.0, self.check_parameters)

    def parameters_callback(self, parameters):
        """Callback for parameter changes"""
        for param in parameters:
            if param.name == 'navigation.max_speed':
                if param.value > 3.0:
                    return SetParametersResult(
                        successful=False,
                        reason='Speed too high for safe operation'
                    )
            elif param.name == 'robot.radius':
                if param.value < 0.05:
                    return SetParametersResult(
                        successful=False,
                        reason='Robot radius too small'
                    )

        return SetParametersResult(successful=True)

    def check_parameters(self):
        """Periodically check parameter values"""
        robot_radius = self.get_parameter('robot.radius').value
        max_speed = self.get_parameter('navigation.max_speed').value

        # Log changes or trigger actions based on parameter values
        if max_speed > 2.0:
            self.get_logger().warn('High speed mode enabled - ensure safe operation')

        self.get_logger().info(f'Robot radius: {robot_radius}, Max speed: {max_speed}')

def main():
    rclpy.init()
    node = AdvancedParameterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Parameter Files and Loading

```yaml
# config/my_robot_params.yaml
/**:
  ros__parameters:
    robot:
      radius: 0.3
      height: 0.5
      mass: 20.0
      joint_limits:
        min_position: -3.14
        max_position: 3.14
        max_velocity: 2.0
        max_effort: 100.0

    navigation:
      max_speed: 1.0
      min_distance: 0.5
      inflation_radius: 0.8
      planner_frequency: 5.0
      controller_frequency: 20.0

    control:
      kp: 1.0
      ki: 0.1
      kd: 0.05
      frequency: 100

    sensors:
      lidar:
        enabled: true
        range_min: 0.1
        range_max: 30.0
        scan_frequency: 10.0
      camera:
        enabled: true
        resolution: [640, 480]
        fps: 30
```

```python
# Loading parameters from file
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Declare arguments
    params_file = LaunchConfiguration('params_file')

    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('my_robot_bringup'),
            'config',
            'my_robot_params.yaml'
        ]),
        description='Full path to params file'
    )

    # Node with parameter file
    robot_controller = Node(
        package='my_robot_control',
        executable='robot_controller',
        name='robot_controller',
        parameters=[params_file],
        output='screen'
    )

    return LaunchDescription([
        declare_params_file_arg,
        robot_controller,
    ])
```

## Lifecycle Nodes

Lifecycle nodes provide a standardized way to manage node state through a well-defined lifecycle, which is crucial for complex robotic systems.

### Lifecycle Node Implementation

```python
from lifecycle_py import LifecycleNode
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node
from rclpy.lifecycle import LifecycleState, TransitionCallbackReturn
import time

class AdvancedLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('advanced_lifecycle_node')

        # Initialize variables
        self.data_buffer = []
        self.is_processing = False

        self.get_logger().info('Lifecycle node initialized in unconfigured state')

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the node"""
        self.get_logger().info('Configuring node...')

        # Initialize resources
        self.data_buffer = []
        self.is_processing = False

        # Create publishers, subscribers, timers
        self.pub = self.create_publisher(String, 'lifecycle_data', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('Node configured successfully')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Clean up resources"""
        self.get_logger().info('Cleaning up node...')

        # Destroy publishers, subscribers, timers
        self.destroy_publisher(self.pub)
        self.destroy_timer(self.timer)

        # Clear buffers
        self.data_buffer.clear()

        self.get_logger().info('Node cleaned up successfully')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate the node"""
        self.get_logger().info('Activating node...')

        # Resume normal operation
        self.is_processing = True

        # Activate publishers and subscribers
        self.pub.on_activate()

        self.get_logger().info('Node activated successfully')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate the node"""
        self.get_logger().info('Deactivating node...')

        # Pause normal operation
        self.is_processing = False

        # Deactivate publishers and subscribers
        self.pub.on_deactivate()

        self.get_logger().info('Node deactivated successfully')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Shutdown the node"""
        self.get_logger().info('Shutting down node...')

        # Perform final cleanup
        self.data_buffer.clear()

        self.get_logger().info('Node shutdown successfully')
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handle errors"""
        self.get_logger().error('Error state reached')

        # Attempt recovery or prepare for shutdown
        self.is_processing = False

        return TransitionCallbackReturn.SUCCESS

    def timer_callback(self):
        """Timer callback for periodic processing"""
        if self.is_processing:
            # Simulate data processing
            data = f"Lifecycle data at {time.time()}"

            msg = String()
            msg.data = data
            self.pub.publish(msg)

            self.get_logger().info(f'Published: {data}')

def main():
    rclpy.init()
    node = AdvancedLifecycleNode()

    try:
        # Spin the node - it will remain in its current state
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Managing Lifecycle Nodes

```python
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class LifecycleManager(Node):
    def __init__(self):
        super().__init__('lifecycle_manager')

        # Callback groups for managing service calls
        self.manager_callback_group = MutuallyExclusiveCallbackGroup()

        # Service clients for managing lifecycle nodes
        self.change_state_clients = {}
        self.get_state_clients = {}

        # List of managed nodes
        self.managed_nodes = [
            'robot_controller',
            'navigation_server',
            'perception_module'
        ]

        # Initialize clients for each managed node
        for node_name in self.managed_nodes:
            change_state_client = self.create_client(
                ChangeState,
                f'/{node_name}/change_state',
                callback_group=self.manager_callback_group
            )
            get_state_client = self.create_client(
                GetState,
                f'/{node_name}/get_state',
                callback_group=self.manager_callback_group
            )

            self.change_state_clients[node_name] = change_state_client
            self.get_state_clients[node_name] = get_state_client

            # Wait for services to be available
            while not change_state_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {node_name}/change_state service...')

            while not get_state_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {node_name}/get_state service...')

        # Timer to periodically check node states
        self.state_check_timer = self.create_timer(5.0, self.check_all_states)

    def change_node_state(self, node_name: str, transition_id: int) -> bool:
        """Change the state of a lifecycle node"""
        client = self.change_state_clients[node_name]

        request = ChangeState.Request()
        request.transition.id = transition_id

        future = client.call_async(request)

        # Wait for response
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            success = response.success
            if success:
                self.get_logger().info(f'Successfully changed {node_name} state')
            else:
                self.get_logger().error(f'Failed to change {node_name} state: {response.error_message}')
            return success
        else:
            self.get_logger().error(f'Timeout waiting for {node_name} state change response')
            return False

    def get_node_state(self, node_name: str) -> str:
        """Get the current state of a lifecycle node"""
        client = self.get_state_clients[node_name]

        request = GetState.Request()
        future = client.call_async(request)

        # Wait for response
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            state_id = response.current_state.id
            state_label = self.get_state_label(state_id)
            return state_label
        else:
            self.get_logger().error(f'Timeout waiting for {node_name} state response')
            return 'UNKNOWN'

    def get_state_label(self, state_id: int) -> str:
        """Convert state ID to human-readable label"""
        state_labels = {
            0: 'UNCONFIGURED',
            1: 'INACTIVE',
            2: 'ACTIVE',
            3: 'FINALIZED'
        }
        return state_labels.get(state_id, f'UNKNOWN({state_id})')

    def check_all_states(self):
        """Check the state of all managed nodes"""
        for node_name in self.managed_nodes:
            state = self.get_node_state(node_name)
            self.get_logger().info(f'{node_name} is in state: {state}')

    def startup_sequence(self):
        """Execute startup sequence for all nodes"""
        self.get_logger().info('Starting up all managed nodes...')

        # Configure all nodes
        for node_name in self.managed_nodes:
            self.get_logger().info(f'Configuring {node_name}...')
            if self.change_node_state(node_name, Transition.TRANSITION_CONFIGURE):
                self.get_logger().info(f'Successfully configured {node_name}')
            else:
                self.get_logger().error(f'Failed to configure {node_name}')

        # Activate all nodes
        for node_name in self.managed_nodes:
            self.get_logger().info(f'Activating {node_name}...')
            if self.change_node_state(node_name, Transition.TRANSITION_ACTIVATE):
                self.get_logger().info(f'Successfully activated {node_name}')
            else:
                self.get_logger().error(f'Failed to activate {node_name}')

def main():
    rclpy.init()
    manager = LifecycleManager()

    # Execute startup sequence
    manager.startup_sequence()

    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        pass
    finally:
        manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance Optimization

### Efficient Message Handling

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from collections import deque
import time

class OptimizedNode(Node):
    def __init__(self):
        super().__init__('optimized_node')

        # Use appropriate QoS for performance
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,  # Only keep the latest message to reduce memory usage
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Use best effort for sensor data
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # Create subscription with optimized QoS
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile
        )

        # Use deque for efficient data structure operations
        self.scan_buffer = deque(maxlen=10)  # Only keep last 10 scans
        self.processing_times = deque(maxlen=100)  # Track performance

        # Timer for periodic processing
        self.processing_timer = self.create_timer(0.1, self.process_scan_data)

        self.last_process_time = time.time()

    def scan_callback(self, msg):
        """Efficient callback that minimizes processing time"""
        # Add to buffer immediately, process later
        self.scan_buffer.append(msg)

        # Minimal processing in callback
        current_time = time.time()
        processing_time = current_time - self.last_process_time
        self.processing_times.append(processing_time)
        self.last_process_time = current_time

    def process_scan_data(self):
        """Batch process data outside of callback"""
        if len(self.scan_buffer) == 0:
            return

        # Process all buffered data at once
        while len(self.scan_buffer) > 0:
            scan_msg = self.scan_buffer.popleft()

            # Perform actual processing
            obstacles = self.detect_obstacles(scan_msg)
            if obstacles:
                # Publish results if needed
                pass

    def detect_obstacles(self, scan_msg):
        """Optimized obstacle detection"""
        # Use numpy for efficient array operations
        import numpy as np

        # Convert to numpy array for efficient processing
        ranges = np.array(scan_msg.ranges)

        # Filter invalid ranges
        valid_mask = (ranges > scan_msg.range_min) & (ranges < scan_msg.range_max)
        valid_ranges = ranges[valid_mask]

        # Detect obstacles (simplified example)
        obstacle_threshold = 0.5  # meters
        obstacles = valid_ranges[valid_ranges < obstacle_threshold]

        return obstacles

    def get_average_processing_time(self):
        """Monitor performance"""
        if len(self.processing_times) == 0:
            return 0.0
        return sum(self.processing_times) / len(self.processing_times)
```

### Memory Management

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import ctypes
import struct

class MemoryEfficientNode(Node):
    def __init__(self):
        super().__init__('memory_efficient_node')

        # For memory-intensive operations, use generators and iterators
        self.subscription = self.create_subscription(
            PointCloud2,
            'point_cloud',
            self.pointcloud_callback,
            1  # Minimal queue size
        )

        # Pre-allocate buffers to avoid memory allocation during runtime
        self.point_buffer = bytearray(1024 * 1024)  # 1MB pre-allocated buffer
        self.processed_points = []

    def pointcloud_callback(self, msg):
        """Memory-efficient point cloud processing"""
        # Instead of copying data, work with the original message when possible
        try:
            # Process point cloud efficiently
            points = self.extract_points_memory_efficient(msg)

            # Process points without creating intermediate large data structures
            processed = self.filter_points_in_place(points)

            # Limit output to avoid memory buildup
            if len(processed) > 1000:  # Limit output size
                processed = processed[:1000]

        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')

    def extract_points_memory_efficient(self, point_cloud_msg):
        """Extract points with minimal memory allocation"""
        # Calculate point size
        point_step = point_cloud_msg.point_step
        row_step = point_cloud_msg.row_step

        # Use memoryview for zero-copy access to data
        data_view = memoryview(point_cloud_msg.data)

        points = []
        for row_start_idx in range(0, len(data_view), row_step):
            for col_idx in range(0, point_cloud_msg.width * point_step, point_step):
                point_offset = row_start_idx + col_idx

                if point_offset + point_step <= len(data_view):
                    # Extract x, y, z coordinates directly from bytes
                    x = struct.unpack('f', data_view[point_offset:point_offset+4])[0]
                    y = struct.unpack('f', data_view[point_offset+4:point_offset+8])[0]
                    z = struct.unpack('f', data_view[point_offset+8:point_offset+12])[0]

                    points.append((x, y, z))

        return points

    def filter_points_in_place(self, points):
        """Filter points without creating new data structures unnecessarily"""
        # Use list comprehension which is more efficient than loops
        filtered = [(x, y, z) for x, y, z in points if self.is_valid_point(x, y, z)]
        return filtered

    def is_valid_point(self, x, y, z):
        """Check if point is valid (not NaN or infinity)"""
        return (not (math.isnan(x) or math.isnan(y) or math.isnan(z)) and
                not (math.isinf(x) or math.isinf(y) or math.isinf(z)))
```

## Debugging Advanced ROS 2 Systems

### Advanced Logging

```python
import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
import traceback
import sys

class DebuggableNode(Node):
    def __init__(self):
        super().__init__('debuggable_node')

        # Set up different logging levels for different components
        self.get_logger().set_level(LoggingSeverity.DEBUG)

        # Use structured logging with context
        self.robot_id = 'robot_001'
        self.component = 'navigation'

        # Performance monitoring
        self.performance_log = []

        # Start diagnostic timer
        self.diagnostic_timer = self.create_timer(10.0, self.log_diagnostics)

    def log_with_context(self, level, message, **kwargs):
        """Log with additional context information"""
        context_str = f"[{self.robot_id}:{self.component}]"

        if kwargs:
            context_str += f" {kwargs}"

        if level == 'DEBUG':
            self.get_logger().debug(f"{context_str} {message}")
        elif level == 'INFO':
            self.get_logger().info(f"{context_str} {message}")
        elif level == 'WARN':
            self.get_logger().warn(f"{context_str} {message}")
        elif level == 'ERROR':
            self.get_logger().error(f"{context_str} {message}")
        elif level == 'FATAL':
            self.get_logger().fatal(f"{context_str} {message}")

    def safe_execute(self, func, *args, **kwargs):
        """Safely execute a function with error handling"""
        try:
            start_time = self.get_clock().now().nanoseconds
            result = func(*args, **kwargs)
            end_time = self.get_clock().now().nanoseconds

            execution_time = (end_time - start_time) / 1e9  # Convert to seconds

            # Log performance
            self.performance_log.append({
                'function': func.__name__,
                'execution_time': execution_time,
                'timestamp': self.get_clock().now().nanoseconds
            })

            return result
        except Exception as e:
            # Log the full traceback
            error_msg = f"Error in {func.__name__}: {str(e)}"
            error_traceback = traceback.format_exc()

            self.get_logger().error(error_msg)
            self.get_logger().error(f"Traceback: {error_traceback}")

            # Optionally, send error to diagnostic topic
            self.report_error(func.__name__, str(e), error_traceback)

            return None

    def report_error(self, function_name, error_message, traceback_str):
        """Report error to diagnostic system"""
        # In a real system, this might publish to a diagnostic topic
        self.get_logger().error(f"Diagnostics - Function: {function_name}")
        self.get_logger().error(f"Error: {error_message}")

    def log_diagnostics(self):
        """Log periodic diagnostics"""
        if self.performance_log:
            avg_time = sum(entry['execution_time'] for entry in self.performance_log) / len(self.performance_log)

            self.get_logger().info(f"Average execution time: {avg_time:.4f}s over {len(self.performance_log)} calls")

            # Clear old performance data to avoid memory buildup
            if len(self.performance_log) > 1000:
                self.performance_log = self.performance_log[-500:]  # Keep last 500 entries

def main():
    rclpy.init()
    node = DebuggableNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')
    except Exception as e:
        node.get_logger().fatal(f'Fatal error: {e}')
        exc_type, exc_value, exc_traceback = sys.exc_info()
        node.get_logger().fatal(f'Exception traceback: {traceback.format_exception(exc_type, exc_value, exc_traceback)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Weekly Hands-On Exercise

Create an advanced ROS 2 system with the following components:

1. Implement a launch file that starts multiple nodes with different configurations
2. Create a parameter system that allows runtime configuration of robot behavior
3. Design a lifecycle node for managing a complex subsystem
4. Optimize one of your nodes for performance and memory efficiency
5. Add comprehensive logging and debugging capabilities to your system

## Review Questions

1. What are the advantages of using lifecycle nodes over regular nodes?
2. How do launch files improve system management in ROS 2?
3. What are the best practices for parameter management in ROS 2?
4. How can you optimize ROS 2 systems for performance?

## Further Reading

- [ROS 2 Launch Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
- [ROS 2 Parameters Guide](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
- [Lifecycle Nodes Tutorial](https://docs.ros.org/en/humble/Tutorials/Advanced/Lifecycle-Nodes.html)
- "Effective Robotics Programming with ROS" by Anil Mahtani