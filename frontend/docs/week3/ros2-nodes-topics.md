---
sidebar_position: 3
---

# ROS 2 Architecture and Core Concepts: Nodes and Topics

This chapter introduces the Robot Operating System 2 (ROS 2) architecture, focusing on the fundamental concepts of nodes and topics that form the backbone of robotic communication.

## Learning Objectives

By the end of this week, you will be able to:
- Understand ROS 2 architecture and its core concepts
- Create and manage ROS 2 nodes
- Implement publishers and subscribers using topics
- Build basic ROS 2 packages with Python
- Design topic-based communication patterns for robots

## Introduction to ROS 2 Architecture

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. Unlike traditional operating systems, ROS 2 is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.

### Key Architectural Components

ROS 2 architecture consists of several key components:

1. **Nodes**: Processes performing computation
2. **Topics**: Named buses over which nodes exchange messages
3. **Messages**: Data structures exchanged between nodes
4. **Services**: Synchronous request/response communication
5. **Actions**: Goal-oriented communication with feedback
6. **Parameters**: Configuration values shared across nodes

### DDS-Based Communication

ROS 2 uses Data Distribution Service (DDS) as its underlying communication middleware, providing:
- Real-time performance
- Deterministic behavior for multi-robot systems
- Distributed system support
- Security features out of the box

## Creating ROS 2 Nodes

A node is an executable that uses ROS 2 to communicate with other nodes. Nodes are the fundamental building blocks of a ROS 2 system.

### Basic Node Structure

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Hello from ROS 2!')

def main(args=None):
    rclpy.init(args=args)
    minimal_node = MinimalNode()

    try:
        rclpy.spin(minimal_node)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node with Parameters

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('robot_name', 'my_robot')
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('debug_mode', False)

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.debug_mode = self.get_parameter('debug_mode').value

        self.get_logger().info(f'Initialized {self.robot_name} with max velocity {self.max_velocity}')

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()

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

## Topics and Publishers

Topics enable asynchronous, broadcast communication between nodes. A publisher node sends messages to a topic, and subscriber nodes receive those messages.

### Creating a Publisher

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating a Subscriber

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()

    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Quality of Service (QoS) Settings

ROS 2 provides Quality of Service (QoS) settings to configure communication behavior:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import String

class QoSPublisher(Node):
    def __init__(self):
        super().__init__('qos_publisher')

        # Create a QoS profile with specific settings
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.publisher_ = self.create_publisher(String, 'qos_topic', qos_profile)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'QoS message {self.i}'
        self.publisher_.publish(msg)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    qos_publisher = QoSPublisher()

    try:
        rclpy.spin(qos_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        qos_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Topic Patterns

### Publisher with Custom Message Types

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')

        # Publisher for LiDAR data
        self.lidar_publisher = self.create_publisher(LaserScan, '/scan', 10)

        # Publisher for velocity commands
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer for sensor simulation
        self.timer = self.create_timer(0.1, self.publish_sensor_data)
        self.angle = 0.0

    def publish_sensor_data(self):
        # Simulate LiDAR scan
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'laser_frame'

        scan_msg.angle_min = -np.pi / 2
        scan_msg.angle_max = np.pi / 2
        scan_msg.angle_increment = np.pi / 180  # 1 degree
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.1
        scan_msg.range_max = 30.0

        # Generate simulated ranges with some obstacles
        num_ranges = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment) + 1
        ranges = [2.0 + 0.5 * np.sin(self.angle + i * 0.1) for i in range(num_ranges)]
        scan_msg.ranges = ranges
        scan_msg.intensities = [100.0] * num_ranges

        self.lidar_publisher.publish(scan_msg)

        # Publish velocity command (simple navigation)
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.5  # Move forward
        cmd_msg.angular.z = 0.1 * np.sin(self.angle)  # Gentle turn
        self.cmd_publisher.publish(cmd_msg)

        self.angle += 0.01

def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = SensorPublisher()

    try:
        rclpy.spin(sensor_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Building ROS 2 Packages

### Package Structure

A typical ROS 2 package has the following structure:

```
my_robot_package/
├── CMakeLists.txt          # Build configuration for C++
├── package.xml             # Package metadata
├── setup.py                # Python package setup
├── setup.cfg               # Installation configuration
├── my_robot_package/       # Python module
│   ├── __init__.py
│   └── my_nodes.py
└── test/                   # Test files
```

### package.xml Template

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_package</name>
  <version>0.0.0</version>
  <description>Example ROS 2 package for robot control</description>
  <maintainer email="user@example.com">User Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### setup.py Template

```python
from setuptools import find_packages, setup

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Example ROS 2 package for robot control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_publisher = my_robot_package.my_publisher:main',
            'my_subscriber = my_robot_package.my_subscriber:main',
        ],
    },
)
```

## Launch Files for Node Management

Launch files allow you to start multiple nodes with a single command:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),

        # Launch publisher node
        Node(
            package='my_robot_package',
            executable='my_publisher',
            name='publisher_node',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen'
        ),

        # Launch subscriber node
        Node(
            package='my_robot_package',
            executable='my_subscriber',
            name='subscriber_node',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen'
        )
    ])
```

## Weekly Hands-On Exercise

Create a complete ROS 2 system that demonstrates nodes and topics:

1. Create a ROS 2 package with publisher and subscriber nodes
2. Implement a sensor publisher that simulates LiDAR data
3. Create a subscriber that processes the sensor data to detect obstacles
4. Add a second publisher that sends velocity commands based on obstacle detection
5. Use launch files to start all nodes simultaneously

## Review Questions

1. What is the difference between ROS 1 and ROS 2 architecture?
2. How do Quality of Service settings affect communication in ROS 2?
3. What are the advantages of using topics for robot communication?
4. How do launch files simplify node management in complex systems?

## Further Reading

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- "Programming Robots with ROS" by Morgan Quigley
- "A Gentle Introduction to ROS" by Jason M. O'Kane