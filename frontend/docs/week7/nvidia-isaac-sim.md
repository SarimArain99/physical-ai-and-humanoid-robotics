---
sidebar_position: 7
---

# NVIDIA Isaac Sim Fundamentals

This chapter introduces NVIDIA Isaac Sim, a comprehensive simulation environment for robotics and AI applications, focusing on GPU-accelerated physics and rendering.

## Learning Objectives

By the end of this week, you will be able to:
- Install and configure NVIDIA Isaac Sim
- Create and configure robot assets for simulation
- Implement sensor simulation with realistic physics
- Integrate Isaac Sim with ROS 2 and other frameworks

## Prerequisites

- Understanding of digital twin concepts (Weeks 5-6)
- Basic knowledge of NVIDIA GPU computing
- Understanding of ROS 2 concepts (Weeks 1-4)

## Introduction to NVIDIA Isaac Sim

NVIDIA Isaac Sim is an end-to-end simulation application and synthetic data generation tool for robotics. It's built on NVIDIA Omniverse and provides:

- GPU-accelerated physics simulation using PhysX
- High-fidelity rendering with RTX ray tracing
- Synthetic data generation for AI training
- Native ROS 2 and ROS 1 support
- Extensive robot asset library
- Integration with Isaac Lab for reinforcement learning

## Installing NVIDIA Isaac Sim

### System Requirements
- NVIDIA GPU with RTX or GTX 1080/1080Ti or better
- CUDA 11.8 or newer
- Ubuntu 20.04 or 22.04 (or Windows 10/11)
- 8GB+ RAM (16GB+ recommended)

### Installation Methods

#### Method 1: Docker (Recommended)
```bash
# Pull the Isaac Sim Docker image
docker pull nvcr.io/nvidia/isaac-sim:4.0.0

# Run Isaac Sim container
docker run --gpus all -it --rm \
  --network=host \
  --env "NVIDIA_VISIBLE_DEVICES=all" \
  --env "NVIDIA_DRIVER_CAPABILITIES=all" \
  --volume $HOME/isaac-sim-cache:/isaac-sim-cache \
  nvcr.io/nvidia/isaac-sim:4.0.0
```

#### Method 2: Isaac Sim on AWS
```bash
# Launch Isaac Sim instance on AWS
# Follow the AWS Marketplace Isaac Sim guide
```

## Creating Robot Assets

### URDF to Isaac Sim Conversion

Isaac Sim can import URDF files and convert them to its internal representation:

```python
import omni
from pxr import UsdGeom, Sdf
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage

# Load a robot from the asset library
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    print("Could not find Isaac Sim assets. Please check your installation.")
else:
    # Add a robot to the stage
    franka_asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_instanceable.usd"
    add_reference_to_stage(usd_path=franka_asset_path, prim_path="/World/Franka")
```

### Custom Robot Definition

For custom robots, create a USD file with proper joint and link definitions:

```usda
#usda 1.0
(
    doc = "Custom Robot Definition for Isaac Sim"
    metersPerUnit = 0.01
    upAxis = "Y"
)

def Xform "World"
{
    def Xform "Robot"
    {
        # Base link
        def Xform "base_link"
        {
            def Sphere "visual"
            {
                radius = 0.1
            }
            def Sphere "collision"
            {
                radius = 0.1
            }
        }

        # First joint (revolute)
        def PhysicsRevoluteJoint "joint1"
        {
            PhysicsJointStateAPI.timeSamples = {0: 0}
        }

        # First link
        def Xform "link1"
        {
            def Capsule "visual"
            {
                radius = 0.05
                height = 0.3
            }
            def Capsule "collision"
            {
                radius = 0.05
                height = 0.3
            }
        }
    }
}
```

## Physics Simulation

### Configuring Physics Properties

```python
import omni
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import RigidPrim, Articulation

# Create the world instance
world = World(stage_units_in_meters=1.0)

# Add physics scene
scene = world.scene
scene.add_default_ground_plane()

# Configure physics settings
physics_dt = 1.0 / 60.0  # 60 Hz physics update
rendering_dt = 1.0 / 60.0  # 60 Hz rendering update

world.set_physics_dt(physics_dt)
world.set_rendering_dt(rendering_dt)

# Add a robot to the simulation
robot = world.scene.add(
    Robot(
        prim_path="/World/Robot",
        name="my_robot",
        usd_path="/path/to/robot.usd",
        position=[0, 0, 0.5],
        orientation=[0, 0, 0, 1]
    )
)
```

### Material Properties and Friction

```python
from omni.isaac.core.materials import PhysicsMaterial
from omni.isaac.core.prims import RigidPrim

# Create a custom physics material
material = PhysicsMaterial(
    prim_path="/World/Looks/CustomMaterial",
    static_friction=0.5,
    dynamic_friction=0.4,
    restitution=0.1  # Bounciness
)

# Apply material to a rigid body
rigid_prim = RigidPrim(prim_path="/World/Robot/base_link")
rigid_prim.apply_physics_material(material)
```

## Sensor Simulation

### RGB Camera Sensor

```python
from omni.isaac.sensor import Camera
import numpy as np

# Add a camera sensor to the robot
camera = Camera(
    prim_path="/World/Robot/camera",
    name="camera",
    position=np.array([0.1, 0.1, 0.1]),
    orientation=np.array([0, 0, 0, 1])
)

# Configure camera properties
camera.set_focal_length(24.0)  # mm
camera.set_horizontal_aperture(20.955)  # mm
camera.set_vertical_aperture(15.29)  # mm
camera.set_resolution((640, 480))

# Initialize the world to enable sensors
world.reset()

# Capture images
for i in range(100):
    world.step(render=True)
    if i % 10 == 0:
        rgb_image = camera.get_rgb()
        print(f"Captured RGB image at step {i}")
```

### LiDAR Sensor

```python
from omni.isaac.range_sensor import LidarRtx
import numpy as np

# Add a LiDAR sensor to the robot
lidar = LidarRtx(
    prim_path="/World/Robot/lidar",
    name="my_lidar",
    translation=np.array([0.0, 0.0, 0.2]),
    orientation=np.array([0.0, 0.0, 0.0, 1.0])
)

# Configure LiDAR properties
lidar.set_sensor_param(
    sensor_horizontal_fov=360.0,  # degrees
    sensor_vertical_fov=30.0,     # degrees
    sensor_horizontal_resolution=0.25,  # degrees
    sensor_vertical_resolution=2.0,     # degrees
    max_range=50.0  # meters
)

# Initialize and capture data
world.reset()

for i in range(100):
    world.step(render=True)
    if i % 10 == 0:
        # Get LiDAR data
        lidar_data = lidar.get_linear_depth_data()
        print(f"LiDAR data shape: {lidar_data.shape}")
```

### IMU Sensor

```python
from omni.isaac.core.sensors import ImuSensor
import numpy as np

# Add an IMU sensor to the robot
imu = ImuSensor(
    prim_path="/World/Robot/imu",
    name="imu_sensor",
    position=np.array([0.0, 0.0, 0.1]),
    orientation=np.array([0.0, 0.0, 0.0, 1.0])
)

# Get IMU data (linear acceleration and angular velocity)
world.reset()

for i in range(100):
    world.step(render=True)
    if i % 10 == 0:
        linear_acceleration = imu.get_linear_acceleration()
        angular_velocity = imu.get_angular_velocity()
        print(f"IMU - Acc: {linear_acceleration}, Vel: {angular_velocity}")
```

## ROS 2 Integration

### Setting up ROS Bridge

```python
# Enable ROS bridge in Isaac Sim
from omni.isaac.ros_bridge import ROSBridge

# Initialize ROS bridge
ros_bridge = ROSBridge()

# Set up ROS topics
import rclpy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist

# Initialize ROS node
rclpy.init()
ros_node = rclpy.create_node('isaac_sim_ros_bridge')

# Create publishers and subscribers
cmd_vel_sub = ros_node.create_subscription(Twist, '/cmd_vel', cmd_vel_callback, 10)
image_pub = ros_node.create_publisher(Image, '/camera/image_raw', 10)
scan_pub = ros_node.create_publisher(LaserScan, '/scan', 10)

def cmd_vel_callback(msg):
    # Process velocity commands from ROS
    linear_x = msg.linear.x
    angular_z = msg.angular.z

    # Apply to robot in Isaac Sim
    apply_robot_velocity(linear_x, angular_z)

def publish_sensor_data():
    # Publish camera image
    rgb_image = camera.get_rgb()
    ros_image = convert_to_ros_image(rgb_image)
    image_pub.publish(ros_image)

    # Publish LiDAR scan
    lidar_data = lidar.get_linear_depth_data()
    ros_scan = convert_to_ros_scan(lidar_data)
    scan_pub.publish(ros_scan)
```

### Isaac Sim ROS Extension

```python
# Using Isaac Sim's built-in ROS extension
from omni.isaac.ros_bridge.scripts import ros_bridge_extension

# Enable the ROS bridge extension
ros_bridge_extension.enable_ros_bridge()

# Configure ROS topics and message types
# Isaac Sim automatically handles message conversion
```

## Isaac Lab Integration

Isaac Sim integrates with Isaac Lab for reinforcement learning and robot learning:

```python
# Example of using Isaac Lab with Isaac Sim
import omni
from omni.isaac.lab_tasks.utils import parse_env_cfg
from omni.isaac.lab_tasks.manager_based.locomotion.velocity.config.unitree_a1 import agents

# Parse configuration
env_cfg = parse_env_cfg("Unitree-A1-Flat")
env_cfg.scene.num_envs = 16
env_cfg.terminations.time_out = True

# Create environment
from omni.isaac.lab.envs import ManagerBasedRLEnv
env = ManagerBasedRLEnv(cfg=env_cfg)

# Run simulation
for i in range(1000):
    # Generate random actions
    actions = torch.randn_like(env.action_buffer)

    # Apply actions and step simulation
    obs_dict, rew_dict, terminated_dict, truncated_dict, info_dict = env.step(actions)

    # Check if episode is done
    if terminated_dict.any() or truncated_dict.any():
        env.reset()
```

## Synthetic Data Generation

### Generating Training Data

```python
from omni.isaac.synthetic_utils import SyntheticDataWriter
import os

# Create synthetic data writer
data_writer = SyntheticDataWriter(
    output_dir=os.path.join(os.getcwd(), "synthetic_data"),
    num_envs=1,
    frames_per_instance=100
)

# Configure data to capture
data_writer.add_data_type("rgb", camera)
data_writer.add_data_type("depth", depth_camera)
data_writer.add_data_type("seg", segmentation_camera)

# Generate synthetic data
for episode in range(100):
    # Randomize environment
    randomize_environment()

    for frame in range(100):
        world.step(render=True)

        # Capture synthetic data
        data_writer.write_frame(episode, frame)
```

## Hands-on Exercise

Create a complete Isaac Sim environment:

1. Install Isaac Sim using Docker
2. Create a custom robot asset with proper URDF/USD conversion
3. Add RGB camera, LiDAR, and IMU sensors to the robot
4. Configure physics properties and materials
5. Set up ROS 2 integration
6. Generate synthetic data for AI training
7. Test the simulation with basic movement commands

## Review Questions

1. What are the main advantages of NVIDIA Isaac Sim over other simulation platforms?
2. How does GPU-accelerated physics in Isaac Sim benefit robotics simulation?
3. What is the role of Isaac Lab in the Isaac ecosystem?

## Further Reading

- [NVIDIA Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html)
- [Isaac Lab Documentation](https://isaac-sim.github.io/IsaacLab/)
- "GPU Ray Tracing in Unity" by Sean Larkin