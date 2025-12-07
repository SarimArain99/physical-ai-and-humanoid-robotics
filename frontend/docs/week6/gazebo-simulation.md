---
sidebar_position: 6
---

# Robot Simulation with Gazebo

This chapter covers Gazebo simulation environment setup, robot modeling, physics simulation, and integration with ROS 2 for testing robotic systems in realistic virtual environments.

## Learning Objectives

By the end of this week, you will be able to:
- Set up and configure Gazebo simulation environment
- Create robot models and environments for simulation
- Implement sensor simulation and physics properties
- Integrate Gazebo with ROS 2 for realistic testing
- Design custom worlds and simulation scenarios

## Introduction to Gazebo

Gazebo is a 3D dynamic simulator that provides accurate physics simulation and rendering. It's widely used in robotics for testing algorithms, robot designs, and control systems before deploying on real hardware.

### Key Features of Gazebo
- **Accurate physics simulation** using ODE, Bullet, Simbody, or DART
- **High-quality rendering** with OGRE
- **Multiple sensors** (camera, lidar, IMU, etc.)
- **Plugins system** for custom functionality
- **Integration with ROS/ROS 2**

### Installation and Setup

```bash
# Install Gazebo Garden (recommended for ROS 2 Humble)
sudo apt install gazebo libgazebo-dev

# Install Gazebo ROS packages
sudo apt install ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins

# Verify installation
gz sim --version
```

## Creating Robot Models

### URDF for Gazebo Integration

```xml
<?xml version="1.0"?>
<robot name="simple_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Gazebo-specific properties -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>
</robot>
```

### Adding Differential Drive Controller

```xml
<!-- Differential Drive Plugin -->
<gazebo>
  <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.3</wheel_separation>
    <wheel_diameter>0.1</wheel_diameter>
    <command_topic>cmd_vel</command_topic>
    <odometry_topic>odom</odometry_topic>
    <odometry_frame>odom</odometry_frame>
    <robot_base_frame>base_link</robot_base_frame>
  </plugin>
</gazebo>
```

### Complete Robot Model with Sensors

```xml
<?xml version="1.0"?>
<robot name="mobile_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.2" length="0.1"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.2" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.4" ixy="0" ixz="0" iyy="0.4" iyz="0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Wheels -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.2 -0.05" rpy="1.5707 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.2 -0.05" rpy="1.5707 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Camera sensor -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo plugins and materials -->
  <gazebo reference="base_link">
    <material>Gazebo/Gray</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
  </gazebo>

  <gazebo reference="left_wheel">
    <material>Gazebo/Black</material>
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>

  <gazebo reference="right_wheel">
    <material>Gazebo/Black</material>
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>

  <gazebo reference="camera_link">
    <material>Gazebo/Red</material>
  </gazebo>

  <!-- Differential drive plugin -->
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.4</wheel_separation>
      <wheel_diameter>0.2</wheel_diameter>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>
    </plugin>
  </gazebo>

  <!-- Camera sensor plugin -->
  <gazebo reference="camera_link">
    <sensor name="camera" type="camera">
      <update_rate>30</update_rate>
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <frame_name>camera_link</frame_name>
        <topic_name>camera/image_raw</topic_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- LiDAR sensor plugin -->
  <gazebo reference="base_link">
    <sensor name="lidar" type="ray">
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="lidar_controller" filename="libgazebo_ros_ray.so">
        <frame_name>base_link</frame_name>
        <topic_name>scan</topic_name>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

## Launching Gazebo with ROS 2

### Launch File Example

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Choose one of the world files from `/gazebo_ros/worlds`'
    )

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': 'false',
        }.items()
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'mobile_robot',
            '-x', '0',
            '-y', '0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': open('/path/to/robot.urdf').read()
        }]
    )

    return LaunchDescription([
        world_arg,
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])
```

## Sensor Simulation in Gazebo

### Camera Sensor Configuration

```xml
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <update_rate>30</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>  <!-- 80 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>camera_link</frame_name>
      <topic_name>camera/image_raw</topic_name>
      <camera_info_topic_name>camera/camera_info</camera_info_topic_name>
    </plugin>
  </sensor>
</gazebo>
```

### LiDAR Sensor Configuration

```xml
<gazebo reference="lidar_link">
  <sensor name="lidar" type="ray">
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
        <vertical>
          <samples>1</samples>
          <resolution>1</resolution>
          <min_angle>0</min_angle>
          <max_angle>0</max_angle>
        </vertical>
      </scan>
      <range>
        <min>0.1</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray.so">
      <frame_name>lidar_link</frame_name>
      <topic_name>scan</topic_name>
      <gaussian_noise>0.01</gaussian_noise>
    </plugin>
  </sensor>
</gazebo>
```

### IMU Sensor Configuration

```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>true</visualize>
    <topic>__default_topic__</topic>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
      <frame_name>imu_link</frame_name>
      <topic_name>imu/data</topic_name>
      <serviceName>imu/service</serviceName>
    </plugin>
  </sensor>
</gazebo>
```

## Physics Properties and World Creation

### Creating a Custom World

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="simple_world">
    <!-- Include models -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Define custom objects -->
    <model name="simple_obstacle">
      <pose>2 2 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Table with objects -->
    <model name="table">
      <pose>0 0 0 0 0 0</pose>
      <link name="table_surface">
        <collision name="collision">
          <geometry>
            <box>
              <size>1.5 1 0.05</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.5 1 0.05</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.6 0.4 1</ambient>
            <diffuse>0.8 0.6 0.4 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1.0</iyy>
            <iyz>0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Physics engine configuration -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>
  </world>
</sdf>
```

## Advanced Gazebo Concepts

### Custom Gazebo Plugins

```cpp
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace gazebo {

class CustomController : public ModelPlugin {
public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
        this->model = _model;
        this->world = _model->GetWorld();

        // Get joints
        this->leftJoint = _model->GetJoint("left_wheel_joint");
        this->rightJoint = _model->GetJoint("right_wheel_joint");

        // Initialize ROS
        if (!rclcpp::ok()) {
            rclcpp::init(argc, argv);
        }

        this->node = std::make_shared<rclcpp::Node>("gazebo_custom_controller");

        // Create ROS subscriber
        this->cmdVelSub = this->node->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&CustomController::cmdVelCallback, this, std::placeholders::_1));

        // Connect to Gazebo update event
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&CustomController::OnUpdate, this));
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        this->targetLinearVel = msg->linear.x;
        this->targetAngularVel = msg->angular.z;
    }

    void OnUpdate() {
        // Calculate wheel velocities based on target linear and angular velocities
        double wheelSep = 0.4; // Wheel separation
        double wheelRadius = 0.1; // Wheel radius

        double leftVel = (this->targetLinearVel - this->targetAngularVel * wheelSep / 2.0) / wheelRadius;
        double rightVel = (this->targetLinearVel + this->targetAngularVel * wheelSep / 2.0) / wheelRadius;

        // Apply velocities to joints
        this->leftJoint->SetParam("vel", 0, leftVel);
        this->rightJoint->SetParam("vel", 0, rightVel);
    }

    physics::ModelPtr model;
    physics::WorldPtr world;
    physics::JointPtr leftJoint;
    physics::JointPtr rightJoint;
    event::ConnectionPtr updateConnection;

    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;

    double targetLinearVel = 0.0;
    double targetAngularVel = 0.0;
};

// Register plugin
GZ_REGISTER_MODEL_PLUGIN(CustomController)
}
```

### Gazebo ROS Bridge

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
import tf2_ros
from tf2_geometry_msgs import PointStamped

class GazeboROSBridge(Node):
    def __init__(self):
        super().__init__('gazebo_ros_bridge')

        # Publishers for sensor data
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Subscribers for control commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer for publishing data
        self.timer = self.create_timer(0.05, self.publish_sensor_data)  # 20 Hz

        # Robot state
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def cmd_vel_callback(self, msg):
        """Handle velocity commands from ROS"""
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z

        # In a real system, this would send the command to Gazebo
        self.get_logger().info(f'Received cmd_vel: linear={self.linear_vel}, angular={self.angular_vel}')

    def publish_sensor_data(self):
        """Publish simulated sensor data"""
        # Simulate odometry
        self.simulate_odometry()

        # Simulate LiDAR scan
        self.simulate_lidar_scan()

        # Simulate camera image
        self.simulate_camera_image()

    def simulate_odometry(self):
        """Simulate odometry data"""
        # Update robot position based on velocities
        dt = 0.05  # 20 Hz
        self.x += self.linear_vel * dt * cos(self.theta)
        self.y += self.linear_vel * dt * sin(self.theta)
        self.theta += self.angular_vel * dt

        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Orientation (convert theta to quaternion)
        from tf_transformations import quaternion_from_euler
        quat = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        # Velocity
        odom_msg.twist.twist.linear.x = self.linear_vel
        odom_msg.twist.twist.angular.z = self.angular_vel

        self.odom_pub.publish(odom_msg)

        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)

    def simulate_lidar_scan(self):
        """Simulate LiDAR scan data"""
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'laser_frame'

        scan_msg.angle_min = -3.14159
        scan_msg.angle_max = 3.14159
        scan_msg.angle_increment = 0.0174533  # 1 degree
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.1
        scan_msg.range_max = 30.0

        # Simulate ranges with some obstacles
        num_ranges = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment) + 1
        ranges = [30.0] * num_ranges  # Default to max range

        # Add some obstacles at specific angles
        obstacle_angles = [0, 45, 90, 135, 180, 225, 270, 315]  # In degrees
        for angle_deg in obstacle_angles:
            angle_rad = math.radians(angle_deg)
            idx = int((angle_rad - scan_msg.angle_min) / scan_msg.angle_increment)
            if 0 <= idx < len(ranges):
                ranges[idx] = 2.0  # Obstacle at 2m

        scan_msg.ranges = ranges
        self.scan_pub.publish(scan_msg)

    def simulate_camera_image(self):
        """Simulate camera image data"""
        # This would normally come from Gazebo, but we'll create a dummy image
        image_msg = Image()
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = 'camera_frame'
        image_msg.height = 480
        image_msg.width = 640
        image_msg.encoding = 'rgb8'
        image_msg.is_bigendian = False
        image_msg.step = 640 * 3  # Width * bytes per pixel

        # Create dummy image data (all white)
        image_data = [255] * (640 * 480 * 3)  # White image
        image_msg.data = image_data

        self.image_pub.publish(image_msg)

def main():
    rclpy.init()
    bridge = GazeboROSBridge()

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Simulation Best Practices

### Performance Optimization

```python
# Efficient sensor simulation
class OptimizedSensorSimulator:
    def __init__(self):
        # Use efficient data structures
        self.scan_buffer = collections.deque(maxlen=5)  # Only keep last 5 scans

        # Pre-allocate arrays for performance
        self.range_array = np.empty(360, dtype=np.float32)
        self.intensity_array = np.empty(360, dtype=np.float32)

    def simulate_scan(self, robot_pose, obstacles):
        """Efficiently simulate LiDAR scan"""
        # Vectorized operations instead of loops
        angles = np.linspace(-np.pi, np.pi, 360, dtype=np.float32)

        # Calculate distances to obstacles in vectorized manner
        robot_x, robot_y, robot_theta = robot_pose

        # Transform obstacle coordinates to robot frame
        rel_x = obstacles[:, 0] - robot_x
        rel_y = obstacles[:, 1] - robot_y

        # Calculate distances and angles
        dists = np.sqrt(rel_x**2 + rel_y**2)
        angles_to_obstacles = np.arctan2(rel_y, rel_x) - robot_theta

        # Bin obstacles into scan sectors
        sector_indices = ((angles_to_obstacles + np.pi) / (2*np.pi) * 360).astype(int)
        sector_indices = np.clip(sector_indices, 0, 359)

        # Find minimum distance in each sector
        self.range_array.fill(np.inf)
        for i, sector in enumerate(sector_indices):
            if dists[i] < self.range_array[sector]:
                self.range_array[sector] = dists[i]

        # Convert inf to max range
        self.range_array[self.range_array == np.inf] = 30.0

        return self.range_array
```

### Debugging Simulation Issues

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

class SimulationDiagnostics(Node):
    def __init__(self):
        super().__init__('simulation_diagnostics')

        # Publishers for diagnostics
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.sim_time_pub = self.create_publisher(Float32, '/sim_time', 10)

        # Timer for diagnostic checks
        self.diag_timer = self.create_timer(1.0, self.check_diagnostics)

        # Simulation state tracking
        self.last_sim_time = 0.0
        self.simulation_running = True

    def check_diagnostics(self):
        """Check simulation health and publish diagnostics"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        # Check simulation time progression
        current_time = self.get_clock().now().nanoseconds / 1e9
        time_diff = current_time - self.last_sim_time

        if time_diff < 0.01:  # Less than 10ms since last check
            status = DiagnosticStatus()
            status.name = 'Simulation Clock'
            status.level = DiagnosticStatus.ERROR
            status.message = 'Simulation time not progressing'
            diag_array.status.append(status)
            self.simulation_running = False
        else:
            status = DiagnosticStatus()
            status.name = 'Simulation Clock'
            status.level = DiagnosticStatus.OK
            status.message = f'Simulation running at {1.0/time_diff:.2f} Hz'
            diag_array.status.append(status)
            self.simulation_running = True

        # Check physics engine
        physics_status = DiagnosticStatus()
        physics_status.name = 'Physics Engine'
        physics_status.level = DiagnosticStatus.OK
        physics_status.message = 'Running normally'
        diag_array.status.append(physics_status)

        # Check sensor data
        sensor_status = DiagnosticStatus()
        sensor_status.name = 'Sensor Data'
        sensor_status.level = DiagnosticStatus.WARN
        sensor_status.message = 'Check sensor topics for data'
        diag_array.status.append(sensor_status)

        self.diag_pub.publish(diag_array)
        self.last_sim_time = current_time

def main():
    rclpy.init()
    diagnostics = SimulationDiagnostics()

    try:
        rclpy.spin(diagnostics)
    except KeyboardInterrupt:
        pass
    finally:
        diagnostics.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Weekly Hands-On Exercise

Create a complete simulation environment:

1. Design a robot model with differential drive and sensors
2. Create a custom world with obstacles and interesting features
3. Set up ROS 2 integration with the simulation
4. Implement a simple navigation task in the simulated environment
5. Test your robot's performance in different simulated scenarios

## Review Questions

1. What are the main advantages of using Gazebo for robotics simulation?
2. How do you integrate ROS 2 with Gazebo simulation?
3. What are the key components needed for sensor simulation in Gazebo?
4. How can you optimize simulation performance for large environments?

## Further Reading

- [Gazebo Documentation](http://gazebosim.org/)
- [ROS 2 with Gazebo Tutorial](https://classic.gazebosim.org/tutorials?tut=ros2_overview)
- "Robotics, Vision and Control" by Peter Corke
- "Programming Robots with ROS" by Morgan Quigley