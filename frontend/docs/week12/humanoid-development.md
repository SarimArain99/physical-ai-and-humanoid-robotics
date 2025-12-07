---
sidebar_position: 12
---

# Humanoid Robot Development: Kinematics, Dynamics, and Control

This chapter covers humanoid robot development focusing on kinematics, dynamics, bipedal locomotion, balance control, and natural human-robot interaction design as specified in the course requirements.

## Learning Objectives

By the end of this week, you will be able to:
- Implement humanoid robot kinematics and dynamics
- Design bipedal locomotion and balance control systems
- Create manipulation and grasping systems for humanoid hands
- Develop natural human-robot interaction protocols
- Apply control algorithms for humanoid movement

## Prerequisites

- Understanding of AI-robot brains (Week 9)
- Knowledge of sensor fusion for Physical AI (Week 11)
- Familiarity with Isaac ROS (Week 10)
- Basic understanding of robot control theory

## Introduction to Humanoid Robots

Humanoid robots are designed to mimic human form and function, offering several advantages:

- **Human-compatible environments**: Can operate in spaces designed for humans
- **Natural interaction**: Humans find them more approachable and intuitive to interact with
- **Versatile manipulation**: Human-like hands enable dexterous manipulation
- **Social acceptance**: More readily accepted in human-centered environments

### Key Challenges in Humanoid Development

1. **Bipedal Locomotion**: Maintaining balance on two legs
2. **Complex Kinematics**: Many degrees of freedom (DOF) requiring sophisticated control
3. **Dynamic Stability**: Managing center of mass and zero moment point (ZMP)
4. **Real-time Control**: Meeting strict timing requirements for stability
5. **Safety**: Ensuring safe interaction with humans and environment

## Humanoid Kinematics

### Forward and Inverse Kinematics for Humanoid Robots

```python
import numpy as np
import math
from typing import List, Tuple, Dict
from dataclasses import dataclass

@dataclass
class JointLimits:
    """Joint limits for humanoid robot"""
    min_angle: float
    max_angle: float
    max_velocity: float
    max_torque: float

class HumanoidKinematics:
    """Kinematics for humanoid robot with 2 arms, 2 legs, and head"""
    def __init__(self):
        # Define humanoid structure (simplified for clarity)
        self.links = {
            'torso': {'length': 0.6, 'mass': 10.0},
            'left_upper_arm': {'length': 0.3, 'mass': 2.0},
            'left_lower_arm': {'length': 0.3, 'mass': 1.5},
            'right_upper_arm': {'length': 0.3, 'mass': 2.0},
            'right_lower_arm': {'length': 0.3, 'mass': 1.5},
            'left_upper_leg': {'length': 0.4, 'mass': 3.0},
            'left_lower_leg': {'length': 0.4, 'mass': 2.5},
            'right_upper_leg': {'length': 0.4, 'mass': 3.0},
            'right_lower_leg': {'length': 0.4, 'mass': 2.5}
        }

        # Define joint limits
        self.joint_limits = self.define_joint_limits()

        # Define DH parameters for each limb
        self.dh_params = self.define_dh_parameters()

    def define_joint_limits(self) -> Dict[str, JointLimits]:
        """Define joint limits for humanoid robot"""
        return {
            # Neck joints
            'neck_yaw': JointLimits(-0.5, 0.5, 1.0, 10.0),
            'neck_pitch': JointLimits(-0.5, 0.5, 1.0, 10.0),

            # Left arm joints
            'left_shoulder_pitch': JointLimits(-2.0, 2.0, 2.0, 50.0),
            'left_shoulder_roll': JointLimits(-0.5, 1.5, 2.0, 50.0),
            'left_shoulder_yaw': JointLimits(-2.0, 1.0, 2.0, 50.0),
            'left_elbow': JointLimits(-2.0, 0.0, 2.0, 40.0),
            'left_wrist_pitch': JointLimits(-1.0, 1.0, 1.0, 20.0),
            'left_wrist_yaw': JointLimits(-1.0, 1.0, 1.0, 20.0),

            # Right arm joints
            'right_shoulder_pitch': JointLimits(-2.0, 2.0, 2.0, 50.0),
            'right_shoulder_roll': JointLimits(-1.5, 0.5, 2.0, 50.0),
            'right_shoulder_yaw': JointLimits(-1.0, 2.0, 2.0, 50.0),
            'right_elbow': JointLimits(0.0, 2.0, 2.0, 40.0),
            'right_wrist_pitch': JointLimits(-1.0, 1.0, 1.0, 20.0),
            'right_wrist_yaw': JointLimits(-1.0, 1.0, 1.0, 20.0),

            # Left leg joints
            'left_hip_yaw': JointLimits(-0.4, 0.4, 1.0, 100.0),
            'left_hip_roll': JointLimits(-0.5, 0.5, 1.0, 100.0),
            'left_hip_pitch': JointLimits(-1.5, 0.5, 1.0, 100.0),
            'left_knee': JointLimits(0.0, 2.5, 1.0, 100.0),
            'left_ankle_pitch': JointLimits(-0.5, 0.5, 0.5, 50.0),
            'left_ankle_roll': JointLimits(-0.3, 0.3, 0.5, 50.0),

            # Right leg joints
            'right_hip_yaw': JointLimits(-0.4, 0.4, 1.0, 100.0),
            'right_hip_roll': JointLimits(-0.5, 0.5, 1.0, 100.0),
            'right_hip_pitch': JointLimits(-1.5, 0.5, 1.0, 100.0),
            'right_knee': JointLimits(0.0, 2.5, 1.0, 100.0),
            'right_ankle_pitch': JointLimits(-0.5, 0.5, 0.5, 50.0),
            'right_ankle_roll': JointLimits(-0.3, 0.3, 0.5, 50.0)
        }

    def define_dh_parameters(self) -> Dict[str, List[Dict]]:
        """Define DH parameters for each limb"""
        return {
            'left_arm': [
                {'a': 0, 'alpha': -np.pi/2, 'd': 0.1, 'theta': 0},  # Shoulder joint 1
                {'a': 0, 'alpha': np.pi/2, 'd': 0, 'theta': 0},     # Shoulder joint 2
                {'a': 0.3, 'alpha': 0, 'd': 0, 'theta': 0},         # Shoulder joint 3
                {'a': 0, 'alpha': -np.pi/2, 'd': 0, 'theta': 0},    # Elbow joint
                {'a': 0, 'alpha': np.pi/2, 'd': 0.3, 'theta': 0},   # Wrist joint 1
                {'a': 0, 'alpha': 0, 'd': 0, 'theta': 0}            # Wrist joint 2
            ],
            'right_arm': [
                {'a': 0, 'alpha': -np.pi/2, 'd': 0.1, 'theta': 0},  # Shoulder joint 1
                {'a': 0, 'alpha': np.pi/2, 'd': 0, 'theta': 0},     # Shoulder joint 2
                {'a': 0.3, 'alpha': 0, 'd': 0, 'theta': 0},         # Shoulder joint 3
                {'a': 0, 'alpha': -np.pi/2, 'd': 0, 'theta': 0},    # Elbow joint
                {'a': 0, 'alpha': np.pi/2, 'd': 0.3, 'theta': 0},   # Wrist joint 1
                {'a': 0, 'alpha': 0, 'd': 0, 'theta': 0}            # Wrist joint 2
            ],
            'left_leg': [
                {'a': 0, 'alpha': 0, 'd': 0, 'theta': 0},           # Hip joint 1
                {'a': 0, 'alpha': -np.pi/2, 'd': 0, 'theta': 0},    # Hip joint 2
                {'a': 0, 'alpha': np.pi/2, 'd': -0.4, 'theta': 0},  # Hip joint 3
                {'a': 0, 'alpha': 0, 'd': -0.4, 'theta': 0},        # Knee joint
                {'a': 0, 'alpha': 0, 'd': 0, 'theta': 0}            # Ankle joint
            ],
            'right_leg': [
                {'a': 0, 'alpha': 0, 'd': 0, 'theta': 0},           # Hip joint 1
                {'a': 0, 'alpha': -np.pi/2, 'd': 0, 'theta': 0},    # Hip joint 2
                {'a': 0, 'alpha': np.pi/2, 'd': -0.4, 'theta': 0},  # Hip joint 3
                {'a': 0, 'alpha': 0, 'd': -0.4, 'theta': 0},        # Knee joint
                {'a': 0, 'alpha': 0, 'd': 0, 'theta': 0}            # Ankle joint
            ]
        }

    def dh_transform(self, a: float, alpha: float, d: float, theta: float) -> np.ndarray:
        """Calculate Denavit-Hartenberg transformation matrix"""
        ct, st = math.cos(theta), math.sin(theta)
        ca, sa = math.cos(alpha), math.sin(alpha)

        T = np.array([
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])

        return T

    def forward_kinematics_arm(self, joint_angles: List[float], arm_type: str) -> Dict[str, np.ndarray]:
        """Calculate forward kinematics for arm"""
        if arm_type not in ['left_arm', 'right_arm']:
            raise ValueError("Arm type must be 'left_arm' or 'right_arm'")

        dh_params = self.dh_params[arm_type]
        transforms = []

        current_transform = np.eye(4)  # Start with identity matrix

        for i, (dh, angle) in enumerate(zip(dh_params, joint_angles)):
            # Update theta with joint angle
            modified_dh = dh.copy()
            modified_dh['theta'] = angle

            # Calculate transform for this joint
            joint_transform = self.dh_transform(
                modified_dh['a'],
                modified_dh['alpha'],
                modified_dh['d'],
                modified_dh['theta']
            )

            # Accumulate transforms
            current_transform = current_transform @ joint_transform
            transforms.append(current_transform.copy())

        return {
            'shoulder': transforms[2] if len(transforms) > 2 else transforms[-1],  # After 3 DOF shoulder
            'elbow': transforms[3] if len(transforms) > 3 else transforms[-1],    # After elbow
            'wrist': transforms[4] if len(transforms) > 4 else transforms[-1],    # After wrist joints
            'hand': transforms[-1]  # Final hand position
        }

    def forward_kinematics_leg(self, joint_angles: List[float], leg_type: str) -> Dict[str, np.ndarray]:
        """Calculate forward kinematics for leg"""
        if leg_type not in ['left_leg', 'right_leg']:
            raise ValueError("Leg type must be 'left_leg' or 'right_leg'")

        dh_params = self.dh_params[leg_type]
        transforms = []

        current_transform = np.eye(4)  # Start with identity matrix

        for i, (dh, angle) in enumerate(zip(dh_params, joint_angles)):
            # Update theta with joint angle
            modified_dh = dh.copy()
            modified_dh['theta'] = angle

            # Calculate transform for this joint
            joint_transform = self.dh_transform(
                modified_dh['a'],
                modified_dh['alpha'],
                modified_dh['d'],
                modified_dh['theta']
            )

            # Accumulate transforms
            current_transform = current_transform @ joint_transform
            transforms.append(current_transform.copy())

        return {
            'hip': transforms[2] if len(transforms) > 2 else transforms[-1],    # After hip joints
            'knee': transforms[3] if len(transforms) > 3 else transforms[-1],  # After knee
            'ankle': transforms[4] if len(transforms) > 4 else transforms[-1], # After ankle
            'foot': transforms[-1]  # Final foot position
        }

    def inverse_kinematics_arm(self, target_pose: np.ndarray, arm_type: str,
                              current_angles: List[float] = None) -> List[float]:
        """Calculate inverse kinematics for arm using Jacobian transpose method"""
        if current_angles is None:
            current_angles = [0.0] * 6  # Default joint angles

        # Use iterative Jacobian transpose method
        max_iterations = 100
        tolerance = 0.001

        current_angles = np.array(current_angles)

        for iteration in range(max_iterations):
            # Calculate current end-effector position
            fk_result = self.forward_kinematics_arm(current_angles.tolist(), arm_type)
            current_pose = fk_result['hand']

            # Calculate error
            pos_error = target_pose[:3, 3] - current_pose[:3, 3]
            rot_error = self.rotation_matrix_to_axis_angle(
                target_pose[:3, :3] @ current_pose[:3, :3].T
            )

            total_error = np.concatenate([pos_error, rot_error[:3]])

            if np.linalg.norm(total_error) < tolerance:
                break  # Solution found

            # Calculate Jacobian
            jacobian = self.calculate_jacobian_arm(current_angles, arm_type)

            # Calculate joint delta using Jacobian transpose
            joint_delta = jacobian.T @ total_error * 0.1  # Learning rate

            # Update joint angles
            current_angles = current_angles + joint_delta

            # Apply joint limits
            for i, angle in enumerate(current_angles):
                joint_name = self.get_joint_name(arm_type, i)
                limits = self.joint_limits[joint_name]
                current_angles[i] = np.clip(angle, limits.min_angle, limits.max_angle)

        return current_angles.tolist()

    def calculate_jacobian_arm(self, joint_angles: np.ndarray, arm_type: str) -> np.ndarray:
        """Calculate Jacobian matrix for arm using analytical method"""
        # This is a simplified Jacobian calculation
        # In practice, you would derive the analytical Jacobian based on the DH parameters
        jacobian = np.zeros((6, 6))  # 6 DOF arm, 6x6 Jacobian

        # Calculate Jacobian columns for each joint
        for i in range(6):
            # Simplified calculation - in practice, this would be derived analytically
            # from the forward kinematics equations
            jacobian[:, i] = np.random.rand(6) * 0.1  # Placeholder values

        return jacobian

    def rotation_matrix_to_axis_angle(self, R: np.ndarray) -> np.ndarray:
        """Convert rotation matrix to axis-angle representation"""
        angle = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))

        if abs(angle) < 1e-6:  # Very small rotation
            return np.zeros(3)

        axis = np.array([
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1]
        ]) / (2 * np.sin(angle))

        return axis * angle

    def get_joint_name(self, arm_type: str, joint_index: int) -> str:
        """Get joint name for a specific arm and joint index"""
        joint_names = {
            'left_arm': [
                'left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw',
                'left_elbow', 'left_wrist_pitch', 'left_wrist_yaw'
            ],
            'right_arm': [
                'right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw',
                'right_elbow', 'right_wrist_pitch', 'right_wrist_yaw'
            ]
        }
        return joint_names[arm_type][joint_index]
```

## Balance Control and Bipedal Locomotion

### Center of Mass and Zero Moment Point Control

```python
class BalanceController:
    """Balance controller for humanoid robots using ZMP and CoM control"""
    def __init__(self):
        # Robot physical parameters
        self.com_height = 0.85  # Center of mass height (meters)
        self.foot_separation = 0.2  # Lateral foot separation (meters)
        self.gravity = 9.81

        # Balance control parameters
        self.com_tracking_gain = 10.0
        self.zmp_tracking_gain = 5.0
        self.ankle_strategy_gain = 20.0
        self.hip_strategy_gain = 15.0
        self.com_admittance_gain = 5.0

        # Support polygon parameters
        self.support_polygon = self.calculate_support_polygon()

        # Balance state
        self.com_position = np.array([0.0, 0.0, self.com_height])
        self.com_velocity = np.array([0.0, 0.0, 0.0])
        self.com_acceleration = np.array([0.0, 0.0, 0.0])
        self.zmp_position = np.array([0.0, 0.0])
        self.current_support_state = 'double'  # 'left', 'right', or 'double'

    def calculate_support_polygon(self) -> Dict:
        """Calculate current support polygon based on foot positions"""
        # Define support polygon as convex hull of feet contact points
        # For bipedal: rectangle around both feet
        return {
            'left_front': np.array([0.15, self.foot_separation/2, 0]),
            'left_back': np.array([-0.15, self.foot_separation/2, 0]),
            'right_front': np.array([0.15, -self.foot_separation/2, 0]),
            'right_back': np.array([-0.15, -self.foot_separation/2, 0]),
            'center': np.array([0, 0, 0]),
            'area': self.foot_separation * 0.3  # Approximate support area
        }

    def compute_balance_control(self, robot_state: Dict, dt: float) -> Dict:
        """Compute balance control commands based on current state"""
        # Get current CoM position and velocity
        current_com = robot_state.get('com_position', self.com_position)
        current_com_vel = robot_state.get('com_velocity', self.com_velocity)
        current_com_acc = robot_state.get('com_acceleration', self.com_acceleration)

        # Get foot positions and contact states
        left_foot_pos = robot_state.get('left_foot_position', np.array([0.0, 0.1, 0.0]))
        right_foot_pos = robot_state.get('right_foot_position', np.array([0.0, -0.1, 0.0]))
        left_contact = robot_state.get('left_foot_contact', True)
        right_contact = robot_state.get('right_foot_contact', True)

        # Update support state
        self.current_support_state = self.determine_support_state(left_contact, right_contact)

        # Calculate Zero Moment Point (ZMP)
        zmp = self.calculate_zmp(current_com, current_com_vel, current_com_acc)

        # Calculate desired ZMP based on gait phase and stability requirements
        desired_zmp = self.calculate_desired_zmp(robot_state, dt)

        # Calculate ZMP error
        zmp_error = desired_zmp - zmp[:2]  # Only x, y components

        # Calculate CoM tracking error
        com_error = desired_zmp - current_com[:2]

        # Balance control commands using multiple strategies
        ankle_control = self.ankle_strategy(zmp_error, current_com)
        hip_control = self.hip_strategy(com_error, current_com_vel)
        com_control = self.com_admittance_control(com_error, current_com_vel)

        # Combine balance strategies
        balance_commands = {
            'ankle_control': ankle_control,
            'hip_control': hip_control,
            'com_control': com_control,
            'combined_control': self.combine_balance_strategies(
                ankle_control, hip_control, com_control
            )
        }

        # Calculate stability metrics
        stability_metrics = self.calculate_stability_metrics(
            zmp, current_com, left_foot_pos, right_foot_pos
        )

        return {
            'balance_commands': balance_commands,
            'stability_metrics': stability_metrics,
            'desired_zmp': desired_zmp,
            'current_zmp': zmp[:2],
            'zmp_error': zmp_error
        }

    def calculate_zmp(self, com_pos: np.ndarray, com_vel: np.ndarray, com_acc: np.ndarray) -> np.ndarray:
        """Calculate Zero Moment Point from CoM state"""
        # ZMP = CoM_xy - (CoM_z / g) * CoM_vel_xy - (CoM_z / g^2) * CoM_acc_xy
        g = self.gravity
        zmp_x = com_pos[0] - (com_pos[2] / g) * com_vel[0] - (com_pos[2] / g**2) * com_acc[0]
        zmp_y = com_pos[1] - (com_pos[2] / g) * com_vel[1] - (com_pos[2] / g**2) * com_acc[1]

        return np.array([zmp_x, zmp_y, 0.0])

    def calculate_desired_zmp(self, robot_state: Dict, dt: float) -> np.ndarray:
        """Calculate desired ZMP based on gait phase and stability requirements"""
        # For standing: target center of support polygon
        if self.current_support_state == 'double':
            # Calculate midpoint between feet
            left_foot = robot_state.get('left_foot_position', np.array([0.0, 0.1, 0.0]))
            right_foot = robot_state.get('right_foot_position', np.array([0.0, -0.1, 0.0]))

            center_x = (left_foot[0] + right_foot[0]) / 2.0
            center_y = (left_foot[1] + right_foot[1]) / 2.0

            # Add small offset for dynamic balance
            desired_zmp = np.array([center_x, center_y])

        elif self.current_support_state == 'left':
            # Single support on left foot
            left_foot = robot_state.get('left_foot_position', np.array([0.0, 0.1, 0.0]))
            desired_zmp = np.array([left_foot[0], left_foot[1]])

        elif self.current_support_state == 'right':
            # Single support on right foot
            right_foot = robot_state.get('right_foot_position', np.array([0.0, -0.1, 0.0]))
            desired_zmp = np.array([right_foot[0], right_foot[1]])
        else:
            # Default: center of robot
            desired_zmp = np.array([0.0, 0.0])

        return desired_zmp

    def ankle_strategy(self, zmp_error: np.ndarray, com_pos: np.ndarray) -> np.ndarray:
        """Ankle strategy for balance control"""
        # Small ankle adjustments for fine balance control
        ankle_command = zmp_error * self.ankle_strategy_gain

        # Limit ankle command to reasonable values
        ankle_command = np.clip(ankle_command, -0.1, 0.1)  # Limited to 10 degrees

        return ankle_command

    def hip_strategy(self, com_error: np.ndarray, com_vel: np.ndarray) -> np.ndarray:
        """Hip strategy for balance control"""
        # Hip adjustments for larger balance corrections
        hip_command = (com_error * self.hip_strategy_gain +
                      com_vel[:2] * 0.5)  # Include velocity damping

        # Limit hip command to reasonable values
        hip_command = np.clip(hip_command, -0.3, 0.3)  # Limited to 30 degrees

        return hip_command

    def com_admittance_control(self, com_error: np.ndarray, com_vel: np.ndarray) -> np.ndarray:
        """Center of mass admittance control for balance"""
        # Move CoM to track desired position
        com_command = com_error * self.com_admittance_gain

        # Include velocity feedback for damping
        com_command -= com_vel[:2] * 0.1

        # Limit CoM command
        com_command = np.clip(com_command, -0.05, 0.05)  # Limited CoM movement

        return com_command

    def combine_balance_strategies(self, ankle_cmd: np.ndarray,
                                 hip_cmd: np.ndarray,
                                 com_cmd: np.ndarray) -> np.ndarray:
        """Combine different balance strategies"""
        # Weight different strategies based on error magnitude
        total_error = np.linalg.norm(ankle_cmd) + np.linalg.norm(hip_cmd)

        if total_error < 0.05:  # Small errors
            # Use ankle strategy primarily
            weights = [0.8, 0.1, 0.1]  # ankle, hip, com
        elif total_error < 0.15:  # Medium errors
            # Use ankle and hip strategies
            weights = [0.5, 0.4, 0.1]  # ankle, hip, com
        else:  # Large errors
            # Use all strategies with emphasis on hip
            weights = [0.2, 0.6, 0.2]  # ankle, hip, com

        combined = (weights[0] * ankle_cmd +
                   weights[1] * hip_cmd +
                   weights[2] * com_cmd)

        return combined

    def determine_support_state(self, left_contact: bool, right_contact: bool) -> str:
        """Determine current support state"""
        if left_contact and right_contact:
            return 'double'
        elif left_contact and not right_contact:
            return 'left'
        elif not left_contact and right_contact:
            return 'right'
        else:
            return 'none'  # No contact (flying phase during walking)

    def calculate_stability_metrics(self, zmp: np.ndarray, com_pos: np.ndarray,
                                  left_foot_pos: np.ndarray, right_foot_pos: np.ndarray) -> Dict:
        """Calculate various stability metrics"""
        # Calculate distance from ZMP to support polygon edge
        zmp_x, zmp_y = zmp[0], zmp[1]

        # Define support polygon boundaries
        min_x = min(left_foot_pos[0], right_foot_pos[0]) - 0.1
        max_x = max(left_foot_pos[0], right_foot_pos[0]) + 0.1
        min_y = min(left_foot_pos[1], right_foot_pos[1]) - 0.1
        max_y = max(left_foot_pos[1], right_foot_pos[1]) + 0.1

        # Calculate margins to edges
        margin_x = min(zmp_x - min_x, max_x - zmp_x)
        margin_y = min(zmp_y - min_y, max_y - zmp_y)
        margin_min = min(margin_x, margin_y)

        # Calculate distance from CoM to ZMP
        com_zmp_distance = np.linalg.norm(com_pos[:2] - zmp[:2])

        # Calculate capture point (point where robot needs to step to stop)
        com_velocity = np.array([0.1, 0.0, 0.0])  # Placeholder - would come from robot state
        capture_point = self.calculate_capture_point(com_pos[:2], com_velocity[:2])

        return {
            'zmp_margin': margin_min,
            'com_zmp_distance': com_zmp_distance,
            'capture_point': capture_point,
            'is_stable': margin_min > 0.05,  # 5cm stability margin
            'stability_index': max(0.0, margin_min / 0.2),  # Normalize to [0,1]
            'support_polygon_area': (max_x - min_x) * (max_y - min_y)
        }

    def calculate_capture_point(self, com_pos: np.ndarray, com_vel: np.ndarray) -> np.ndarray:
        """Calculate capture point for balance control"""
        # Capture point = CoM + CoM_velocity * sqrt(CoM_height / gravity)
        omega = math.sqrt(self.com_height / self.gravity)
        capture_point = com_pos + com_vel / omega
        return capture_point
```

## Manipulation and Grasping with Humanoid Hands

### Humanoid Hand Control and Grasping

```python
class HumanoidHandController:
    """Controller for humanoid hand manipulation and grasping"""
    def __init__(self):
        # Hand structure parameters
        self.thumb_joints = 4  # 4 DOF thumb
        self.finger_joints = 3  # 3 DOF per finger (excluding thumb)
        self.num_fingers = 4  # 4 fingers (excluding thumb)
        self.total_hand_dof = self.thumb_joints + self.finger_joints * self.num_fingers  # 16 DOF

        # Grasp types and configurations
        self.grasp_configs = {
            'power': {
                'description': 'Strong grip using all fingers',
                'joint_angles': [1.0, 1.0, 1.0, 1.0,  # thumb
                                1.0, 1.0, 1.0,  # index
                                1.0, 1.0, 1.0,  # middle
                                1.0, 1.0, 1.0,  # ring
                                1.0, 1.0, 1.0]  # little
            },
            'precision': {
                'description': 'Fine manipulation with thumb and index finger',
                'joint_angles': [0.5, 0.5, 0.0, 0.0,  # thumb (positioned for precision)
                                0.5, 0.5, 0.5,  # index (partially closed)
                                0.0, 0.0, 0.0,  # middle (open)
                                0.0, 0.0, 0.0,  # ring (open)
                                0.0, 0.0, 0.0]  # little (open)
            },
            'tripod': {
                'description': 'Three-finger precision grasp',
                'joint_angles': [0.3, 0.3, 0.0, 0.0,  # thumb
                                0.6, 0.6, 0.6,  # index
                                0.6, 0.6, 0.6,  # middle
                                0.2, 0.2, 0.2,  # ring (slightly closed)
                                0.2, 0.2, 0.2]  # little (slightly closed)
            }
        }

        # Object properties for grasp planning
        self.object_properties = {
            'cylindrical': {
                'grasp_types': ['power', 'cylindrical'],
                'preferred_orientation': [0, 0, 1]  # Upward orientation
            },
            'rectangular': {
                'grasp_types': ['tripod', 'power'],
                'preferred_orientation': [0, 1, 0]  # Horizontal orientation
            },
            'spherical': {
                'grasp_types': ['spherical', 'power'],
                'preferred_orientation': [0, 0, 1]  # Any orientation acceptable
            }
        }

    def plan_grasp(self, object_info: Dict) -> Dict:
        """Plan appropriate grasp for object"""
        object_type = object_info.get('type', 'unknown')
        object_size = object_info.get('size', [0.1, 0.1, 0.1])  # [width, height, depth]
        object_weight = object_info.get('weight', 0.5)  # kg

        # Determine appropriate grasp type based on object properties
        grasp_type = self.select_grasp_type(object_info)

        # Generate grasp configuration
        grasp_config = self.generate_grasp_configuration(grasp_type, object_info)

        # Calculate grasp approach trajectory
        approach_trajectory = self.calculate_approach_trajectory(
            object_info['position'], grasp_config
        )

        return {
            'grasp_type': grasp_type,
            'grasp_configuration': grasp_config,
            'approach_trajectory': approach_trajectory,
            'success_probability': self.estimate_grasp_success(grasp_type, object_info)
        }

    def select_grasp_type(self, object_info: Dict) -> str:
        """Select appropriate grasp type for object"""
        object_type = object_info.get('type', 'unknown')
        object_size = object_info.get('size', [0.1, 0.1, 0.1])
        object_weight = object_info.get('weight', 0.5)

        # Rule-based grasp selection
        if object_type in ['cylindrical', 'bottle', 'cup']:
            if object_weight < 0.5 and max(object_size) < 0.1:  # Light and small
                return 'precision'
            else:
                return 'power'
        elif object_type in ['rectangular', 'box', 'book']:
            if min(object_size) < 0.05:  # Thin object
                return 'tripod'
            else:
                return 'power'
        elif object_type in ['spherical', 'ball']:
            return 'spherical'
        else:
            # Unknown object type - use power grasp for safety
            return 'power'

    def generate_grasp_configuration(self, grasp_type: str, object_info: Dict) -> List[float]:
        """Generate joint configuration for specific grasp type"""
        if grasp_type in self.grasp_configs:
            return self.grasp_configs[grasp_type]['joint_angles']
        else:
            # Return default power grasp
            return self.grasp_configs['power']['joint_angles']

    def calculate_approach_trajectory(self, object_position: np.ndarray,
                                    grasp_config: List[float]) -> List[np.ndarray]:
        """Calculate approach trajectory for grasping"""
        # Calculate approach position (above object)
        approach_offset = np.array([0.0, 0.0, 0.1])  # 10cm above object
        approach_position = object_position + approach_offset

        # Calculate pre-grasp position (away from object)
        pregrasp_offset = np.array([0.0, 0.0, -0.15])  # 15cm away
        pregrasp_position = object_position + pregrasp_offset

        # Generate trajectory points
        trajectory = [
            pregrasp_position,  # Pre-grasp position
            approach_position,  # Approach position
            object_position     # Final grasp position
        ]

        return trajectory

    def execute_grasp(self, grasp_plan: Dict) -> bool:
        """Execute the planned grasp"""
        try:
            # Move to pre-grasp position
            self.move_hand_to_position(grasp_plan['approach_trajectory'][0])

            # Move to approach position
            self.move_hand_to_position(grasp_plan['approach_trajectory'][1])

            # Move to final grasp position
            self.move_hand_to_position(grasp_plan['approach_trajectory'][2])

            # Execute grasp configuration
            self.set_hand_configuration(grasp_plan['grasp_configuration'])

            # Close fingers
            self.close_fingers()

            # Verify grasp success
            grasp_successful = self.verify_grasp_success()

            return grasp_successful

        except Exception as e:
            print(f"Error executing grasp: {e}")
            return False

    def move_hand_to_position(self, position: np.ndarray):
        """Move hand to specified position using inverse kinematics"""
        # This would use arm inverse kinematics to position the hand
        # For now, we'll simulate the movement
        print(f"Moving hand to position: {position}")

    def set_hand_configuration(self, joint_angles: List[float]):
        """Set hand joint configuration"""
        print(f"Setting hand configuration: {joint_angles}")

    def close_fingers(self):
        """Close fingers for grasping"""
        print("Closing fingers for grasp")

    def verify_grasp_success(self) -> bool:
        """Verify if grasp was successful using tactile sensors"""
        # In a real system, this would check tactile sensor readings
        # For simulation, return random success
        import random
        return random.random() > 0.2  # 80% success rate

    def estimate_grasp_success(self, grasp_type: str, object_info: Dict) -> float:
        """Estimate probability of successful grasp"""
        object_type = object_info.get('type', 'unknown')
        object_weight = object_info.get('weight', 0.5)
        object_surface = object_info.get('surface', 'normal')  # smooth, rough, slippery

        # Base success probability
        base_prob = 0.8

        # Adjust for object weight
        if object_weight > 1.0:  # Heavy object
            base_prob -= 0.2
        elif object_weight < 0.1:  # Very light object
            base_prob -= 0.1

        # Adjust for surface properties
        if object_surface == 'slippery':
            base_prob -= 0.3
        elif object_surface == 'rough':
            base_prob += 0.1

        # Adjust for grasp type appropriateness
        if grasp_type == 'precision' and object_weight > 0.5:
            base_prob -= 0.2  # Precision grasp not suitable for heavy objects

        return max(0.1, min(1.0, base_prob))  # Clamp between 0.1 and 1.0

class ManipulationController:
    """High-level manipulation controller for humanoid robots"""
    def __init__(self):
        self.left_hand = HumanoidHandController()
        self.right_hand = HumanoidHandController()
        self.arm_controller = ArmController()
        self.grasp_planner = GraspPlanner()
        self.trajectory_generator = TrajectoryGenerator()

    def pick_object(self, object_info: Dict, hand: str = 'right') -> bool:
        """Pick up an object using specified hand"""
        if hand not in ['left', 'right']:
            raise ValueError("Hand must be 'left' or 'right'")

        hand_controller = self.left_hand if hand == 'left' else self.right_hand

        # Plan grasp
        grasp_plan = hand_controller.plan_grasp(object_info)

        # Check if grasp is feasible
        if grasp_plan['success_probability'] < 0.5:
            print(f"Grasp not feasible: {grasp_plan['success_probability']:.2f}")
            return False

        # Execute grasp
        success = hand_controller.execute_grasp(grasp_plan)

        return success

    def place_object(self, object_position: np.ndarray, placement_surface: str,
                    hand: str = 'right') -> bool:
        """Place object at specified position"""
        if hand not in ['left', 'right']:
            raise ValueError("Hand must be 'left' or 'right'")

        hand_controller = self.left_hand if hand == 'left' else self.right_hand

        # Calculate placement trajectory
        approach_pos = object_position + np.array([0.0, 0.0, 0.05])  # 5cm above placement
        retract_pos = object_position + np.array([0.0, 0.0, 0.15])  # 15cm above placement

        # Execute placement
        try:
            # Move to approach position
            hand_controller.move_hand_to_position(approach_pos)

            # Move to placement position
            hand_controller.move_hand_to_position(object_position)

            # Open fingers to release object
            self.open_fingers(hand)

            # Retract hand
            hand_controller.move_hand_to_position(retract_pos)

            return True

        except Exception as e:
            print(f"Error placing object: {e}")
            return False

    def open_fingers(self, hand: str):
        """Open fingers to release object"""
        if hand == 'left':
            # Open left hand fingers
            pass
        else:
            # Open right hand fingers
            pass

    def manipulate_object(self, manipulation_task: Dict) -> bool:
        """Perform complex manipulation task"""
        task_type = manipulation_task.get('type', 'unknown')
        object_info = manipulation_task.get('object', {})
        target_position = manipulation_task.get('target_position', np.zeros(3))

        if task_type == 'pick_and_place':
            # Pick object with one hand
            hand = manipulation_task.get('hand', 'right')
            pick_success = self.pick_object(object_info, hand)

            if not pick_success:
                return False

            # Place object at target position
            place_success = self.place_object(target_position, 'surface', hand)
            return place_success

        elif task_type == 'pour':
            # Pick container
            hand = manipulation_task.get('hand', 'right')
            container_info = manipulation_task.get('container', {})
            pick_success = self.pick_object(container_info, hand)

            if not pick_success:
                return False

            # Pour action - tilt container
            pour_trajectory = self.calculate_pour_trajectory(
                container_info['position'], target_position
            )

            for position in pour_trajectory:
                self.move_hand_to_position(hand, position)

            # Return to upright position
            return True

        elif task_type == 'assembly':
            # Complex multi-step manipulation
            return self.perform_assembly(manipulation_task)

        else:
            print(f"Unknown manipulation task: {task_type}")
            return False

    def calculate_pour_trajectory(self, container_pos: np.ndarray,
                                target_pos: np.ndarray) -> List[np.ndarray]:
        """Calculate trajectory for pouring action"""
        # Calculate intermediate positions for pouring
        # This would involve tilting the container gradually

        trajectory = []
        pour_steps = 10

        for i in range(pour_steps):
            # Gradually tilt container
            tilt_angle = i * (np.pi/4) / pour_steps  # Tilt to 45 degrees
            position = container_pos.copy()
            position[2] += 0.05 * i / pour_steps  # Raise slightly during pour

            trajectory.append(position)

        # Return to original position
        trajectory.append(container_pos)

        return trajectory

    def perform_assembly(self, task_info: Dict) -> bool:
        """Perform assembly task with multiple objects"""
        # Complex task involving multiple objects and precise manipulation
        # This would use a task and motion planner

        objects = task_info.get('objects', [])
        assembly_steps = task_info.get('steps', [])

        for step in assembly_steps:
            # Execute each assembly step
            step_type = step['type']
            object_ids = step['objects']

            if step_type == 'align':
                # Align objects for assembly
                self.align_objects([objects[i] for i in object_ids])
            elif step_type == 'connect':
                # Connect aligned objects
                self.connect_objects([objects[i] for i in object_ids])
            elif step_type == 'verify':
                # Verify assembly success
                if not self.verify_assembly_step(step):
                    return False

        return True

    def align_objects(self, objects: List[Dict]):
        """Align objects for assembly"""
        print(f"Aligning {len(objects)} objects for assembly")

    def connect_objects(self, objects: List[Dict]):
        """Connect aligned objects"""
        print(f"Connecting {len(objects)} objects")

    def verify_assembly_step(self, step: Dict) -> bool:
        """Verify that assembly step was successful"""
        # In practice, this would use vision or force feedback
        return True  # For simulation
```

## Natural Human-Robot Interaction Design

### Social Interaction Framework

```python
class SocialInteractionFramework:
    """Framework for natural human-robot interaction"""
    def __init__(self):
        self.social_behavior_engine = SocialBehaviorEngine()
        self.gesture_generator = GestureGenerator()
        self.expression_controller = ExpressionController()
        self.voice_interaction = VoiceInteractionSystem()

    def engage_with_human(self, human_info: Dict, interaction_type: str) -> bool:
        """Engage with human using appropriate social behaviors"""
        # Determine appropriate interaction based on context
        context = self.analyze_interaction_context(human_info, interaction_type)

        # Generate appropriate social response
        social_response = self.social_behavior_engine.generate_response(
            human_info, context
        )

        # Execute multimodal response
        self.execute_multimodal_response(social_response)

        return True

    def analyze_interaction_context(self, human_info: Dict, interaction_type: str) -> Dict:
        """Analyze context for appropriate interaction"""
        return {
            'proximity': self.calculate_proximity(human_info),
            'attention': self.estimate_attention(human_info),
            'emotion': self.estimate_emotion(human_info),
            'cultural_context': self.estimate_cultural_context(human_info),
            'task_context': interaction_type
        }

    def calculate_proximity(self, human_info: Dict) -> str:
        """Calculate proximity level for interaction"""
        distance = human_info.get('distance', 2.0)

        if distance < 0.5:
            return 'intimate'
        elif distance < 1.2:
            return 'personal'
        elif distance < 3.6:
            return 'social'
        else:
            return 'public'

    def estimate_attention(self, human_info: Dict) -> float:
        """Estimate how much attention human is paying to robot"""
        # This would use gaze tracking, pose estimation, etc.
        # For now, return a placeholder
        return 0.8  # 80% attention

    def estimate_emotion(self, human_info: Dict) -> str:
        """Estimate human emotion from facial expressions, voice, etc."""
        # This would use emotion recognition models
        # For now, return a placeholder
        return 'neutral'

    def estimate_cultural_context(self, human_info: Dict) -> str:
        """Estimate cultural context for appropriate interaction style"""
        # This would analyze appearance, behavior, etc.
        # For now, return a placeholder
        return 'universal'  # Use universal social norms

    def execute_multimodal_response(self, response: Dict):
        """Execute multimodal social response"""
        # Execute facial expression
        if 'expression' in response:
            self.expression_controller.set_expression(response['expression'])

        # Execute gesture
        if 'gesture' in response:
            self.gesture_generator.execute_gesture(response['gesture'])

        # Execute speech
        if 'speech' in response:
            self.voice_interaction.speak(response['speech'])

        # Execute movement
        if 'movement' in response:
            self.execute_movement(response['movement'])

    def execute_movement(self, movement: Dict):
        """Execute appropriate movement for social interaction"""
        # This would move the robot appropriately
        print(f"Executing social movement: {movement}")

class SocialBehaviorEngine:
    """Engine for generating appropriate social behaviors"""
    def __init__(self):
        self.social_rules = self.load_social_rules()

    def load_social_rules(self) -> Dict:
        """Load social interaction rules"""
        return {
            'greetings': {
                'first_encounter': {
                    'greeting_type': 'formal',
                    'approach_distance': 1.0,
                    'duration': 3.0,
                    'gestures': ['wave', 'nod'],
                    'expressions': ['smile']
                },
                'returning_encounter': {
                    'greeting_type': 'informal',
                    'approach_distance': 0.8,
                    'duration': 2.0,
                    'gestures': ['wave'],
                    'expressions': ['happy']
                }
            },
            'conversations': {
                'attentive_listening': {
                    'head_tilt': 5.0,
                    'eye_contact_duration': 3.0,
                    'gesture_frequency': 0.2,
                    'response_time': 1.0
                },
                'active_participation': {
                    'head_nodding': True,
                    'back_channeling': True,
                    'gesture_sync': True
                }
            },
            'cultural_adaptations': {
                'western': {
                    'eye_contact': True,
                    'handshake': True,
                    'personal_space': 1.0
                },
                'eastern': {
                    'bow_angle': 15.0,
                    'eye_contact': False,
                    'personal_space': 1.2
                }
            }
        }

    def generate_response(self, human_info: Dict, context: Dict) -> Dict:
        """Generate appropriate social response based on context"""
        proximity = context['proximity']
        attention = context['attention']
        emotion = context['emotion']
        cultural_context = context['cultural_context']
        task_context = context['task_context']

        response = {}

        if proximity == 'intimate' and attention > 0.7:
            # Direct engagement with high attention
            if emotion == 'happy':
                response.update({
                    'expression': 'joyful',
                    'gesture': 'enthusiastic_wave',
                    'speech': 'Hello! You seem to be in a great mood!',
                    'movement': {'type': 'lean_forward', 'amount': 0.1}
                })
            elif emotion == 'sad':
                response.update({
                    'expression': 'concerned',
                    'gesture': 'comforting_gesture',
                    'speech': 'Are you okay? I\'m here if you need assistance.',
                    'movement': {'type': 'move_closer', 'distance': 0.3}
                })
            else:
                response.update({
                    'expression': 'attentive',
                    'gesture': 'open_posture',
                    'speech': 'Hello! How can I assist you today?',
                    'movement': {'type': 'maintain_distance', 'distance': 0.8}
                })

        elif proximity == 'personal':
            # Appropriate for personal space
            response.update({
                'expression': 'friendly',
                'gesture': 'wave',
                'speech': 'Hi there! How can I help you?',
                'movement': {'type': 'stand_still'}
            })

        elif proximity == 'social':
            # Social distance interaction
            response.update({
                'expression': 'polite',
                'gesture': 'nod',
                'speech': 'Good day! I\'m ready to assist.',
                'movement': {'type': 'approach', 'distance': 1.0}
            })

        else:  # Public distance
            response.update({
                'expression': 'neutral',
                'gesture': 'acknowledge',
                'speech': 'Hello from over here!',
                'movement': {'type': 'no_approach'}
            })

        # Cultural adaptation
        cultural_rules = self.social_rules['cultural_adaptations'].get(cultural_context, {})
        if cultural_rules:
            response.update(self.adapt_to_culture(response, cultural_rules))

        return response

    def adapt_to_culture(self, response: Dict, cultural_rules: Dict) -> Dict:
        """Adapt response based on cultural context"""
        # Modify response based on cultural preferences
        if 'bow_angle' in cultural_rules:
            response['gesture'] = f'bow_{cultural_rules["bow_angle"]}deg'

        if cultural_rules.get('eye_contact', True) == False:
            response['expression'] = response['expression'].replace('attentive', 'respectful')

        return response

class GestureGenerator:
    """Generate appropriate gestures for social interaction"""
    def __init__(self):
        self.gesture_library = {
            'greeting': {
                'wave': self.generate_wave_gesture,
                'nod': self.generate_nod_gesture,
                'bow': self.generate_bow_gesture
            },
            'acknowledgment': {
                'thumbs_up': self.generate_thumbs_up_gesture,
                'pointing': self.generate_pointing_gesture
            },
            'attention': {
                'attention_pose': self.generate_attention_pose,
                'head_turn': self.generate_head_turn_gesture
            }
        }

    def execute_gesture(self, gesture_name: str):
        """Execute specified gesture"""
        if '_' in gesture_name:
            base_gesture = gesture_name.split('_')[0]
        else:
            base_gesture = gesture_name

        if base_gesture in self.gesture_library['greeting']:
            self.gesture_library['greeting'][base_gesture]()
        elif base_gesture in self.gesture_library['acknowledgment']:
            self.gesture_library['acknowledgment'][base_gesture]()
        elif base_gesture in self.gesture_library['attention']:
            self.gesture_library['attention'][base_gesture]()
        else:
            print(f"Gesture not found: {gesture_name}")

    def generate_wave_gesture(self):
        """Generate waving gesture"""
        # This would animate the robot's arm/wrist joints
        print("Executing wave gesture")

    def generate_nod_gesture(self):
        """Generate nodding gesture"""
        # This would animate the robot's head joints
        print("Executing nod gesture")

    def generate_bow_gesture(self):
        """Generate bowing gesture"""
        # This would bend the robot's torso
        print("Executing bow gesture")

    def generate_attention_pose(self):
        """Generate attention-getting pose"""
        print("Executing attention pose")

class ExpressionController:
    """Control robot facial expressions"""
    def __init__(self):
        self.expression_map = {
            'neutral': [0.0, 0.0, 0.0, 0.0, 0.0],  # [eyes, brows, mouth, cheeks, jaw]
            'happy': [0.5, 0.3, 0.8, 0.2, 0.0],
            'sad': [0.2, -0.3, -0.5, 0.0, 0.0],
            'surprised': [0.8, 0.6, 0.2, 0.0, 0.0],
            'angry': [0.1, -0.5, -0.3, 0.0, 0.5],
            'attentive': [0.7, 0.2, 0.1, 0.3, 0.0],
            'joyful': [0.8, 0.4, 0.9, 0.6, 0.1],
            'concerned': [0.4, -0.2, -0.2, 0.1, 0.0]
        }

    def set_expression(self, expression_name: str):
        """Set robot facial expression"""
        if expression_name in self.expression_map:
            expression_values = self.expression_map[expression_name]
            print(f"Setting expression: {expression_name} with values: {expression_values}")
            # In practice, this would control facial servos or animated mesh
        else:
            print(f"Expression not found: {expression_name}")

class VoiceInteractionSystem:
    """System for voice-based interaction"""
    def __init__(self):
        self.voice_settings = {
            'pitch': 1.0,
            'speed': 1.0,
            'tone': 'friendly',
            'volume': 0.7
        }

    def speak(self, text: str):
        """Speak text with appropriate voice settings"""
        print(f"Robot says: {text}")
        # In practice, this would use text-to-speech system

    def adjust_voice_for_emotion(self, emotion: str):
        """Adjust voice parameters based on emotion context"""
        emotion_settings = {
            'happy': {'pitch': 1.1, 'speed': 1.2, 'tone': 'enthusiastic'},
            'sad': {'pitch': 0.9, 'speed': 0.8, 'tone': 'empathetic'},
            'angry': {'pitch': 0.8, 'speed': 1.3, 'tone': 'firm'},
            'neutral': {'pitch': 1.0, 'speed': 1.0, 'tone': 'professional'}
        }

        if emotion in emotion_settings:
            settings = emotion_settings[emotion]
            self.voice_settings.update(settings)
            print(f"Adjusted voice for {emotion} emotion: {settings}")
```

## Hands-on Exercise

Create a complete humanoid interaction system:

1. Implement the kinematics system for a humanoid robot
2. Create a balance controller using ZMP principles
3. Design manipulation and grasping systems for humanoid hands
4. Build a social interaction framework
5. Integrate with perception systems for natural interaction
6. Test with human participants and evaluate interaction quality

## Review Questions

1. What are the key challenges in bipedal locomotion for humanoid robots?
2. How does the Zero Moment Point (ZMP) approach enable stable walking?
3. What factors influence grasp selection for robotic manipulation?
4. How do social interaction protocols improve human-robot acceptance?

## Further Reading

- "Humanoid Robotics: A Reference" by Veloso
- "Robotics: Modelling, Planning and Control" by Siciliano
- "Socially Embedded Interaction Robotics" by Breazeal
- "Bipedal Walking: Modeling, Simulation, and Control" by Kajita