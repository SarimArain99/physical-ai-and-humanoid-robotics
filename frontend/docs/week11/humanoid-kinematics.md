---
sidebar_position: 11
---

# Humanoid Robot Kinematics

This chapter covers the mathematical foundations of humanoid robot kinematics, including forward and inverse kinematics, balance control, and bipedal locomotion principles essential for physical AI applications.

## Learning Objectives

By the end of this week, you will be able to:
- Understand humanoid robot kinematic structures and joint configurations
- Implement forward and inverse kinematics for humanoid robots
- Design balance control algorithms for bipedal locomotion
- Apply kinematic constraints for humanoid movement
- Integrate kinematic models with perception and control systems

## Prerequisites

- Understanding of ROS 2 concepts (Weeks 3-5)
- Basic knowledge of linear algebra and transformation matrices
- Understanding of Isaac Sim (Weeks 8-10)
- Familiarity with sensor fusion (Week 11)

## Introduction to Humanoid Kinematics

Humanoid robots have anthropomorphic structures designed to operate in human environments. Their kinematic chains are complex, involving:

- Multiple limbs (legs, arms, head)
- Bipedal locomotion requirements
- Balance and stability considerations
- Manipulation capabilities with dexterous hands

### Key Kinematic Challenges

1. **Degrees of Freedom**: Managing complex joint configurations
2. **Balance Control**: Maintaining stability during locomotion
3. **Collision Avoidance**: Ensuring self-collision prevention
4. **Workspace Optimization**: Maximizing operational workspace
5. **Smooth Transitions**: Ensuring stable gait transitions

## Forward Kinematics for Humanoid Robots

Forward kinematics computes the end-effector position given joint angles:

```python
import numpy as np
import math
from typing import List, Dict, Tuple

class HumanoidKinematics:
    """Humanoid robot kinematics calculator"""
    def __init__(self):
        # Define humanoid joint structure
        self.joint_names = [
            # Torso
            'torso_pitch', 'torso_roll', 'torso_yaw',
            # Left leg
            'left_hip_yaw', 'left_hip_roll', 'left_hip_pitch',
            'left_knee', 'left_ankle_pitch', 'left_ankle_roll',
            # Right leg
            'right_hip_yaw', 'right_hip_roll', 'right_hip_pitch',
            'right_knee', 'right_ankle_pitch', 'right_ankle_roll',
            # Left arm
            'left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw',
            'left_elbow', 'left_wrist_pitch', 'left_wrist_yaw',
            # Right arm
            'right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw',
            'right_elbow', 'right_wrist_pitch', 'right_wrist_yaw',
            # Head
            'neck_yaw', 'neck_pitch'
        ]

        # Joint limits (in radians)
        self.joint_limits = self.define_joint_limits()

        # Link lengths and offsets
        self.link_parameters = self.define_link_parameters()

    def define_joint_limits(self) -> Dict[str, Tuple[float, float]]:
        """Define joint limits for humanoid robot"""
        return {
            'torso_pitch': (-0.5, 0.5),
            'torso_roll': (-0.3, 0.3),
            'torso_yaw': (-0.5, 0.5),
            'left_hip_yaw': (-0.4, 0.4),
            'left_hip_roll': (-0.5, 0.5),
            'left_hip_pitch': (-1.5, 0.5),
            'left_knee': (0.0, 2.5),
            'left_ankle_pitch': (-0.5, 0.5),
            'left_ankle_roll': (-0.3, 0.3),
            'right_hip_yaw': (-0.4, 0.4),
            'right_hip_roll': (-0.5, 0.5),
            'right_hip_pitch': (-1.5, 0.5),
            'right_knee': (0.0, 2.5),
            'right_ankle_pitch': (-0.5, 0.5),
            'right_ankle_roll': (-0.3, 0.3),
            'left_shoulder_pitch': (-2.0, 2.0),
            'left_shoulder_roll': (-0.5, 1.5),
            'left_shoulder_yaw': (-2.0, 1.0),
            'left_elbow': (-2.0, 0.0),
            'left_wrist_pitch': (-1.0, 1.0),
            'left_wrist_yaw': (-1.0, 1.0),
            'right_shoulder_pitch': (-2.0, 2.0),
            'right_shoulder_roll': (-1.5, 0.5),
            'right_shoulder_yaw': (-1.0, 2.0),
            'right_elbow': (0.0, 2.0),
            'right_wrist_pitch': (-1.0, 1.0),
            'right_wrist_yaw': (-1.0, 1.0),
            'neck_yaw': (-0.5, 0.5),
            'neck_pitch': (-0.5, 0.5)
        }

    def define_link_parameters(self) -> Dict[str, Dict]:
        """Define DH parameters and link properties for humanoid"""
        return {
            # Torso links
            'torso': {
                'length': 0.5,  # torso height
                'offset': [0, 0, 0.5]  # from pelvis
            },
            # Leg links
            'upper_leg': {
                'length': 0.4,  # thigh length
                'offset': [0, 0, -0.4]  # from hip
            },
            'lower_leg': {
                'length': 0.4,  # shin length
                'offset': [0, 0, -0.4]  # from knee
            },
            'foot': {
                'length': 0.25,  # foot length
                'offset': [0.1, 0, -0.1]  # from ankle
            },
            # Arm links
            'upper_arm': {
                'length': 0.3,  # upper arm length
                'offset': [0.15, 0, 0.1]  # from shoulder
            },
            'lower_arm': {
                'length': 0.3,  # forearm length
                'offset': [0.3, 0, 0]  # from elbow
            },
            'hand': {
                'length': 0.15,  # hand length
                'offset': [0.15, 0, 0]  # from wrist
            }
        }

    def dh_transform(self, theta: float, d: float, a: float, alpha: float) -> np.ndarray:
        """Compute Denavit-Hartenberg transformation matrix"""
        ct, st = math.cos(theta), math.sin(theta)
        ca, sa = math.cos(alpha), math.sin(alpha)

        T = np.array([
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])

        return T

    def forward_kinematics_leg(self, joint_angles: List[float], leg_side: str) -> Dict[str, np.ndarray]:
        """Compute forward kinematics for a single leg"""
        if leg_side not in ['left', 'right']:
            raise ValueError("Leg side must be 'left' or 'right'")

        # Extract joint angles for this leg
        if leg_side == 'left':
            hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll = joint_angles
        else:  # right
            hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll = joint_angles

        # Define DH parameters for leg (simplified model)
        # Hip (3 DOF) -> Knee -> Ankle (2 DOF)
        transforms = []

        # Hip yaw joint
        T_hip_yaw = self.dh_transform(hip_yaw, 0, 0, 0)
        transforms.append(T_hip_yaw)

        # Hip roll joint
        T_hip_roll = self.dh_transform(hip_roll, 0, 0, math.pi/2)
        transforms.append(T_hip_yaw @ T_hip_roll)

        # Hip pitch joint
        T_hip_pitch = self.dh_transform(hip_pitch, 0, 0, -math.pi/2)
        transforms.append(transforms[1] @ T_hip_pitch)

        # Upper leg (thigh)
        T_upper_leg = self.dh_transform(0, -0.4, 0, 0)  # 40cm thigh
        transforms.append(transforms[2] @ T_upper_leg)

        # Knee joint
        T_knee = self.dh_transform(knee, 0, 0, 0)
        transforms.append(transforms[3] @ T_knee)

        # Lower leg (shin)
        T_lower_leg = self.dh_transform(0, -0.4, 0, 0)  # 40cm shin
        transforms.append(transforms[4] @ T_lower_leg)

        # Ankle pitch joint
        T_ankle_pitch = self.dh_transform(ankle_pitch, 0, 0, 0)
        transforms.append(transforms[5] @ T_ankle_pitch)

        # Ankle roll joint
        T_ankle_roll = self.dh_transform(0, 0, 0, math.pi/2)
        transforms.append(transforms[6] @ T_ankle_roll)

        # Foot
        T_foot = self.dh_transform(0, -0.05, 0, 0)  # 5cm foot offset
        transforms.append(transforms[7] @ T_foot)

        return {
            'hip': transforms[2],  # After 3 DOF hip joints
            'knee': transforms[5],  # After knee joint
            'ankle': transforms[7],  # After ankle joints
            'foot': transforms[8]   # Final foot position
        }

    def forward_kinematics_arm(self, joint_angles: List[float], arm_side: str) -> Dict[str, np.ndarray]:
        """Compute forward kinematics for a single arm"""
        if arm_side not in ['left', 'right']:
            raise ValueError("Arm side must be 'left' or 'right'")

        # Extract joint angles for this arm
        if arm_side == 'left':
            shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_pitch, wrist_yaw = joint_angles
        else:  # right
            shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_pitch, wrist_yaw = joint_angles

        # Define DH parameters for arm
        transforms = []

        # Shoulder pitch
        T_shoulder_pitch = self.dh_transform(shoulder_pitch, 0, 0, 0)
        transforms.append(T_shoulder_pitch)

        # Shoulder roll
        T_shoulder_roll = self.dh_transform(shoulder_roll, 0, 0, math.pi/2)
        transforms.append(transforms[0] @ T_shoulder_roll)

        # Shoulder yaw
        T_shoulder_yaw = self.dh_transform(shoulder_yaw, 0, 0, -math.pi/2)
        transforms.append(transforms[1] @ T_shoulder_yaw)

        # Upper arm
        T_upper_arm = self.dh_transform(0, -0.3, 0, 0)  # 30cm upper arm
        transforms.append(transforms[2] @ T_upper_arm)

        # Elbow joint
        T_elbow = self.dh_transform(elbow, 0, 0, 0)
        transforms.append(transforms[3] @ T_elbow)

        # Lower arm
        T_lower_arm = self.dh_transform(0, -0.3, 0, 0)  # 30cm forearm
        transforms.append(transforms[4] @ T_lower_arm)

        # Wrist pitch
        T_wrist_pitch = self.dh_transform(wrist_pitch, 0, 0, 0)
        transforms.append(transforms[5] @ T_wrist_pitch)

        # Wrist yaw
        T_wrist_yaw = self.dh_transform(wrist_yaw, 0, 0, 0)
        transforms.append(transforms[6] @ T_wrist_yaw)

        # Hand
        T_hand = self.dh_transform(0, -0.15, 0, 0)  # 15cm hand
        transforms.append(transforms[7] @ T_hand)

        return {
            'shoulder': transforms[2],  # After 3 DOF shoulder joints
            'elbow': transforms[4],     # After elbow joint
            'wrist': transforms[6],     # After wrist joints
            'hand': transforms[7]       # Final hand position
        }

    def compute_full_humanoid_fk(self, joint_angles: Dict[str, List[float]]) -> Dict[str, Dict[str, np.ndarray]]:
        """Compute full forward kinematics for humanoid robot"""
        result = {}

        # Compute leg FK
        if 'left_leg' in joint_angles:
            result['left_leg'] = self.forward_kinematics_leg(joint_angles['left_leg'], 'left')
        if 'right_leg' in joint_angles:
            result['right_leg'] = self.forward_kinematics_leg(joint_angles['right_leg'], 'right')

        # Compute arm FK
        if 'left_arm' in joint_angles:
            result['left_arm'] = self.forward_kinematics_arm(joint_angles['left_arm'], 'left')
        if 'right_arm' in joint_angles:
            result['right_arm'] = self.forward_kinematics_arm(joint_angles['right_arm'], 'right')

        # Compute head FK
        if 'head' in joint_angles:
            neck_yaw, neck_pitch = joint_angles['head']
            T_neck_yaw = self.dh_transform(neck_yaw, 0, 0, 0)
            T_neck_pitch = self.dh_transform(neck_pitch, 0, 0, math.pi/2)
            T_head = T_neck_yaw @ T_neck_pitch
            result['head'] = {'position': T_head}

        return result

    def get_end_effector_position(self, transform_matrix: np.ndarray) -> np.ndarray:
        """Extract end-effector position from transformation matrix"""
        return transform_matrix[:3, 3]
```

## Inverse Kinematics for Humanoid Control

```python
class HumanoidInverseKinematics:
    """Inverse kinematics solver for humanoid robots"""
    def __init__(self, kinematics_model: HumanoidKinematics):
        self.kinematics = kinematics_model

    def compute_inverse_kinematics_leg(self, target_position: np.ndarray,
                                     target_orientation: np.ndarray,
                                     leg_side: str,
                                     current_angles: List[float] = None) -> List[float]:
        """Compute inverse kinematics for leg to reach target position"""
        # Use Jacobian-based iterative method for IK
        if current_angles is None:
            current_angles = [0.0] * 6  # Default joint angles

        # Target pose as 4x4 transformation matrix
        target_pose = np.eye(4)
        target_pose[:3, 3] = target_position
        target_pose[:3, :3] = self.quaternion_to_rotation_matrix(target_orientation)

        # Iterative IK solution
        max_iterations = 100
        tolerance = 0.001

        for i in range(max_iterations):
            # Compute current end-effector pose
            joint_angles_dict = {
                'left_leg' if leg_side == 'left' else 'right_leg': current_angles
            }
            fk_result = self.kinematics.compute_full_humanoid_fk(joint_angles_dict)

            current_pose = fk_result[f'{leg_side}_leg']['foot']

            # Calculate error
            position_error = target_position - self.kinematics.get_end_effector_position(current_pose)
            orientation_error = self.calculate_orientation_error(
                current_pose[:3, :3], target_pose[:3, :3]
            )

            total_error = np.concatenate([position_error, orientation_error])

            if np.linalg.norm(total_error) < tolerance:
                break  # Solution found

            # Compute Jacobian
            jacobian = self.compute_jacobian_leg(current_angles, leg_side)

            # Compute joint angle update
            if jacobian.shape[0] == jacobian.shape[1]:
                # Square Jacobian - use direct inverse
                joint_delta = np.linalg.inv(jacobian) @ total_error
            else:
                # Non-square Jacobian - use pseudoinverse
                joint_delta = np.linalg.pinv(jacobian) @ total_error

            # Update joint angles
            current_angles = [a + da for a, da in zip(current_angles, joint_delta)]

            # Apply joint limits
            current_angles = self.apply_joint_limits(current_angles, leg_side)

        return current_angles

    def compute_inverse_kinematics_arm(self, target_position: np.ndarray,
                                     target_orientation: np.ndarray,
                                     arm_side: str,
                                     current_angles: List[float] = None) -> List[float]:
        """Compute inverse kinematics for arm to reach target position"""
        if current_angles is None:
            current_angles = [0.0] * 6  # Default joint angles

        # Target pose as 4x4 transformation matrix
        target_pose = np.eye(4)
        target_pose[:3, 3] = target_position
        target_pose[:3, :3] = self.quaternion_to_rotation_matrix(target_orientation)

        # Iterative IK solution
        max_iterations = 100
        tolerance = 0.001

        for i in range(max_iterations):
            # Compute current end-effector pose
            joint_angles_dict = {
                'left_arm' if arm_side == 'left' else 'right_arm': current_angles
            }
            fk_result = self.kinematics.compute_full_humanoid_fk(joint_angles_dict)

            current_pose = fk_result[f'{arm_side}_arm']['hand']

            # Calculate error
            position_error = target_position - self.kinematics.get_end_effector_position(current_pose)
            orientation_error = self.calculate_orientation_error(
                current_pose[:3, :3], target_pose[:3, :3]
            )

            total_error = np.concatenate([position_error, orientation_error])

            if np.linalg.norm(total_error) < tolerance:
                break  # Solution found

            # Compute Jacobian
            jacobian = self.compute_jacobian_arm(current_angles, arm_side)

            # Compute joint angle update
            if jacobian.shape[0] == jacobian.shape[1]:
                # Square Jacobian - use direct inverse
                joint_delta = np.linalg.inv(jacobian) @ total_error
            else:
                # Non-square Jacobian - use pseudoinverse
                joint_delta = np.linalg.pinv(jacobian) @ total_error

            # Update joint angles
            current_angles = [a + da for a, da in zip(current_angles, joint_delta)]

            # Apply joint limits
            current_angles = self.apply_joint_limits_arm(current_angles, arm_side)

        return current_angles

    def compute_jacobian_leg(self, joint_angles: List[float], leg_side: str) -> np.ndarray:
        """Compute Jacobian matrix for leg using analytical method"""
        # This is a simplified Jacobian calculation
        # In practice, you would compute the full analytical Jacobian
        # or use numerical differentiation

        # Jacobian relates joint velocities to end-effector velocities
        # J = [∂f/∂θ₁, ∂f/∂θ₂, ..., ∂f/∂θₙ]

        jacobian = np.zeros((6, 6))  # 6 DOF leg, 6x6 Jacobian

        # Simplified Jacobian computation
        # In practice, this would use the full kinematic chain derivatives
        for i in range(6):
            jacobian[:, i] = np.random.rand(6) * 0.1  # Placeholder values

        return jacobian

    def compute_jacobian_arm(self, joint_angles: List[float], arm_side: str) -> np.ndarray:
        """Compute Jacobian matrix for arm"""
        jacobian = np.zeros((6, 6))  # 6 DOF arm, 6x6 Jacobian

        # Simplified Jacobian computation
        # In practice, this would use the full kinematic chain derivatives
        for i in range(6):
            jacobian[:, i] = np.random.rand(6) * 0.1  # Placeholder values

        return jacobian

    def quaternion_to_rotation_matrix(self, quaternion: np.ndarray) -> np.ndarray:
        """Convert quaternion to rotation matrix"""
        qw, qx, qy, qz = quaternion

        # Convert to rotation matrix
        R = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
            [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
            [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
        ])

        return R

    def calculate_orientation_error(self, current_R: np.ndarray, target_R: np.ndarray) -> np.ndarray:
        """Calculate orientation error between two rotation matrices"""
        # Compute relative rotation
        R_rel = target_R @ current_R.T

        # Convert to axis-angle representation
        angle, axis = self.rotation_matrix_to_axis_angle(R_rel)

        # Return orientation error as axis-angle scaled by angle
        return axis * angle

    def rotation_matrix_to_axis_angle(self, R: np.ndarray) -> Tuple[float, np.ndarray]:
        """Convert rotation matrix to axis-angle representation"""
        angle = math.acos(max(-1, min(1, (np.trace(R) - 1) / 2)))

        if abs(angle) < 1e-6:  # Very small rotation
            return 0.0, np.array([0, 0, 1])  # Return arbitrary axis

        axis = np.array([
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1]
        ]) / (2 * math.sin(angle))

        axis = axis / np.linalg.norm(axis)  # Normalize axis

        return angle, axis

    def apply_joint_limits(self, angles: List[float], leg_side: str) -> List[float]:
        """Apply joint limits to prevent damage"""
        limited_angles = []

        # Get appropriate joint limits
        if leg_side == 'left':
            joint_names = ['left_hip_yaw', 'left_hip_roll', 'left_hip_pitch',
                          'left_knee', 'left_ankle_pitch', 'left_ankle_roll']
        else:
            joint_names = ['right_hip_yaw', 'right_hip_roll', 'right_hip_pitch',
                          'right_knee', 'right_ankle_pitch', 'right_ankle_roll']

        for i, angle in enumerate(angles):
            joint_name = joint_names[i]
            min_limit, max_limit = self.kinematics.joint_limits[joint_name]
            limited_angle = max(min_limit, min(max_limit, angle))
            limited_angles.append(limited_angle)

        return limited_angles

    def apply_joint_limits_arm(self, angles: List[float], arm_side: str) -> List[float]:
        """Apply joint limits to arm joints"""
        limited_angles = []

        # Get appropriate joint limits
        if arm_side == 'left':
            joint_names = ['left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw',
                          'left_elbow', 'left_wrist_pitch', 'left_wrist_yaw']
        else:
            joint_names = ['right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw',
                          'right_elbow', 'right_wrist_pitch', 'right_wrist_yaw']

        for i, angle in enumerate(angles):
            joint_name = joint_names[i]
            min_limit, max_limit = self.kinematics.joint_limits[joint_name]
            limited_angle = max(min_limit, min(max_limit, angle))
            limited_angles.append(limited_angle)

        return limited_angles
```

## Balance Control and Bipedal Locomotion

### Center of Mass and Zero Moment Point Control

```python
class BalanceController:
    """Balance controller for humanoid robots"""
    def __init__(self):
        # Robot physical parameters
        self.com_height = 0.85  # Center of mass height (meters)
        self.foot_separation = 0.2  # Lateral foot separation (meters)
        self.gravity = 9.81

        # Balance control gains
        self.com_x_gain = 10.0
        self.com_y_gain = 10.0
        self.com_z_gain = 5.0
        self.omega_gain = 50.0  # Angular velocity feedback

        # Support polygon parameters
        self.support_polygon = self.calculate_support_polygon()

        # Balance state
        self.com_position = np.array([0.0, 0.0, self.com_height])
        self.com_velocity = np.array([0.0, 0.0, 0.0])
        self.com_acceleration = np.array([0.0, 0.0, 0.0])

    def calculate_support_polygon(self) -> Dict:
        """Calculate current support polygon based on foot positions"""
        # Define support polygon as convex hull of feet contact points
        # For bipedal: rectangle around both feet
        return {
            'left_front': np.array([0.15, self.foot_separation/2, 0]),
            'left_back': np.array([-0.15, self.foot_separation/2, 0]),
            'right_front': np.array([0.15, -self.foot_separation/2, 0]),
            'right_back': np.array([-0.15, -self.foot_separation/2, 0]),
            'center': np.array([0, 0, 0])
        }

    def compute_balance_control(self, robot_state: Dict) -> Dict:
        """Compute balance control commands based on current state"""
        # Get current CoM position and velocity
        current_com = robot_state.get('com_position', self.com_position)
        current_com_vel = robot_state.get('com_velocity', self.com_velocity)

        # Calculate Zero Moment Point (ZMP)
        zmp = self.calculate_zmp(current_com, current_com_vel)

        # Calculate desired ZMP based on support polygon
        desired_zmp = self.calculate_desired_zmp(robot_state)

        # Calculate ZMP error
        zmp_error = desired_zmp - zmp[:2]  # Only x, y components

        # Calculate CoM tracking error
        com_error = desired_zmp - current_com[:2]

        # Balance control commands
        balance_commands = {
            'com_x_correction': self.com_x_gain * com_error[0],
            'com_y_correction': self.com_y_gain * com_error[1],
            'zmp_x_correction': self.com_x_gain * zmp_error[0],
            'zmp_y_correction': self.com_y_gain * zmp_error[1],
            'angular_velocity_feedback': self.omega_gain * current_com_vel[2]  # z component
        }

        # Calculate stability metrics
        stability_metrics = self.calculate_stability_metrics(zmp, current_com)

        return {
            'balance_commands': balance_commands,
            'stability_metrics': stability_metrics,
            'desired_zmp': desired_zmp,
            'current_zmp': zmp[:2]
        }

    def calculate_zmp(self, com_pos: np.ndarray, com_vel: np.ndarray) -> np.ndarray:
        """Calculate Zero Moment Point from CoM state"""
        # ZMP = CoM_xy - (CoM_z / g) * CoM_acc_xy
        # In steady state: ZMP ≈ CoM_xy (assuming no acceleration)
        # For dynamic state: incorporate velocity and acceleration terms

        # Simplified ZMP calculation
        zmp_x = com_pos[0] - (com_pos[2] / self.gravity) * com_vel[0]
        zmp_y = com_pos[1] - (com_pos[2] / self.gravity) * com_vel[1]

        return np.array([zmp_x, zmp_y, 0.0])

    def calculate_desired_zmp(self, robot_state: Dict) -> np.ndarray:
        """Calculate desired ZMP based on gait phase and foot positions"""
        # For walking, desired ZMP follows a trajectory between feet
        # For standing, desired ZMP is at support polygon center

        left_foot_pos = robot_state.get('left_foot_position', np.array([0.0, 0.1, 0.0]))
        right_foot_pos = robot_state.get('right_foot_position', np.array([0.0, -0.1, 0.0]))

        # Calculate midpoint between feet
        center_of_support = (left_foot_pos + right_foot_pos) / 2.0

        # For simple standing, target center of support polygon
        # In gait, this would follow the gait pattern
        return center_of_support[:2]  # x, y only

    def calculate_stability_metrics(self, zmp: np.ndarray, com_pos: np.ndarray) -> Dict:
        """Calculate various stability metrics"""
        # Calculate distance from ZMP to support polygon edge
        zmp_x, zmp_y = zmp[0], zmp[1]

        # Get support polygon boundaries
        poly = self.support_polygon
        min_x = min([p[0] for p in [poly['left_front'], poly['left_back'],
                                  poly['right_front'], poly['right_back']]])
        max_x = max([p[0] for p in [poly['left_front'], poly['left_back'],
                                  poly['right_front'], poly['right_back']]])
        min_y = min([p[1] for p in [poly['left_front'], poly['left_back'],
                                  poly['right_front'], poly['right_back']]])
        max_y = max([p[1] for p in [poly['left_front'], poly['left_back'],
                                  poly['right_front'], poly['right_back']]])

        # Calculate margins to edges
        margin_x = min(zmp_x - min_x, max_x - zmp_x)
        margin_y = min(zmp_y - min_y, max_y - zmp_y)
        margin_min = min(margin_x, margin_y)

        # Calculate distance from CoM to ZMP
        com_zmp_distance = np.linalg.norm(com_pos[:2] - zmp[:2])

        return {
            'zmp_margin': margin_min,
            'com_zmp_distance': com_zmp_distance,
            'is_stable': margin_min > 0.05,  # 5cm stability margin
            'stability_index': max(0.0, margin_min / 0.2)  # Normalize to [0,1]
        }

    def compute_foot_placement_strategy(self, velocity_command: np.ndarray) -> Dict:
        """Compute optimal foot placement for balance during locomotion"""
        # Calculate desired foot placement based on velocity command
        # This implements Capture Point theory for bipedal walking

        vx, vy = velocity_command[0], velocity_command[1]

        # Calculate time constant for inverted pendulum
        omega = math.sqrt(self.gravity / self.com_height)

        # Calculate time to step
        step_time = 0.8  # seconds (typical step duration)

        # Calculate capture point (point where robot needs to step to stop)
        capture_point_x = vx / omega
        capture_point_y = vy / omega

        # Calculate desired foot placement
        # This is a simplified model - in practice, more complex planning is needed
        desired_left_foot = np.array([
            capture_point_x + 0.1,  # 10cm offset
            capture_point_y + self.foot_separation / 2,
            0.0
        ])

        desired_right_foot = np.array([
            capture_point_x + 0.1,  # 10cm offset
            capture_point_y - self.foot_separation / 2,
            0.0
        ])

        return {
            'left_foot_target': desired_left_foot,
            'right_foot_target': desired_right_foot,
            'capture_point': np.array([capture_point_x, capture_point_y, 0.0])
        }
```

## Humanoid Gait Planning and Control

### Walking Pattern Generator

```python
class GaitPatternGenerator:
    """Generate walking patterns for humanoid robots"""
    def __init__(self):
        # Gait parameters
        self.step_length = 0.3  # meters
        self.step_height = 0.1  # meters (for foot clearance)
        self.step_duration = 0.8  # seconds per step
        self.double_support_ratio = 0.1  # 10% double support phase
        self.stride_width = 0.2  # distance between footprints

        # Timing parameters
        self.stance_duration = self.step_duration * (1 - self.double_support_ratio)
        self.swing_duration = self.step_duration * self.double_support_ratio

    def generate_walk_pattern(self, velocity_command: np.ndarray,
                            angular_command: float = 0.0) -> Dict:
        """Generate walking pattern based on velocity and angular commands"""
        vx, vy, vz = velocity_command

        # Calculate step frequency based on desired velocity
        step_frequency = self.calculate_step_frequency(np.linalg.norm([vx, vy]))
        step_timing = 1.0 / step_frequency

        # Generate footstep sequence
        footsteps = self.calculate_footsteps(vx, vy, angular_command, step_timing)

        # Generate joint trajectories for walking
        left_leg_trajectory, right_leg_trajectory = self.generate_leg_trajectories(
            footsteps, step_timing
        )

        # Generate arm swing patterns for balance
        left_arm_pattern, right_arm_pattern = self.generate_arm_swing_patterns(
            left_leg_trajectory, right_leg_trajectory
        )

        return {
            'footsteps': footsteps,
            'left_leg_trajectory': left_leg_trajectory,
            'right_leg_trajectory': right_leg_trajectory,
            'left_arm_pattern': left_arm_pattern,
            'right_arm_pattern': right_arm_pattern,
            'gait_parameters': {
                'step_frequency': step_frequency,
                'step_length': self.step_length,
                'step_height': self.step_height,
                'stride_width': self.stride_width
            }
        }

    def calculate_step_frequency(self, linear_velocity: float) -> float:
        """Calculate appropriate step frequency for given velocity"""
        # Simple model: step frequency scales with velocity
        base_freq = 1.0  # Hz at 0 velocity
        max_freq = 2.5  # Hz at maximum walking speed
        max_velocity = 1.0  # m/s maximum walking speed

        # Scale frequency with velocity
        scaled_freq = base_freq + (linear_velocity / max_velocity) * (max_freq - base_freq)
        return min(scaled_freq, max_freq)

    def calculate_footsteps(self, vx: float, vy: float, omega: float,
                          step_timing: float) -> List[Dict]:
        """Calculate sequence of footsteps for walking"""
        footsteps = []

        # Calculate number of steps to plan ahead
        num_steps = 10

        # Current robot position and orientation
        current_x, current_y, current_theta = 0.0, 0.0, 0.0

        # Plan alternating footsteps
        for i in range(num_steps):
            # Calculate movement for this step
            dx = vx * step_timing
            dy = vy * step_timing
            dtheta = omega * step_timing

            # Determine which foot to step with (alternating)
            is_left_step = (i % 2) == 0

            # Calculate target foot position
            if is_left_step:
                # Left foot moves forward and maintains lateral position
                foot_x = current_x + dx * math.cos(current_theta) - dy * math.sin(current_theta)
                foot_y = current_y + dx * math.sin(current_theta) + dy * math.cos(current_theta) + self.stride_width / 2
                foot_z = 0.0
            else:
                # Right foot moves forward and maintains lateral position
                foot_x = current_x + dx * math.cos(current_theta) - dy * math.sin(current_theta)
                foot_y = current_y + dx * math.sin(current_theta) + dy * math.cos(current_theta) - self.stride_width / 2
                foot_z = 0.0

            # Add step to sequence
            footsteps.append({
                'step_number': i,
                'foot': 'left' if is_left_step else 'right',
                'position': np.array([foot_x, foot_y, foot_z]),
                'timing': i * step_timing,
                'orientation': current_theta + dtheta,
                'step_type': 'normal'  # Could be 'turn', 'sidestep', etc.
            })

            # Update robot position for next step calculation
            current_x += dx * math.cos(current_theta) - dy * math.sin(current_theta)
            current_y += dx * math.sin(current_theta) + dy * math.cos(current_theta)
            current_theta += dtheta

        return footsteps

    def generate_leg_trajectories(self, footsteps: List[Dict],
                                step_timing: float) -> Tuple[np.ndarray, np.ndarray]:
        """Generate joint trajectories for leg movement"""
        # Calculate trajectory duration
        total_time = len(footsteps) * step_timing
        dt = 0.01  # 100 Hz
        time_points = np.arange(0, total_time, dt)

        # Initialize trajectory arrays
        num_joints = 6  # hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll
        left_trajectory = np.zeros((len(time_points), num_joints))
        right_trajectory = np.zeros((len(time_points), num_joints))

        # Generate trajectories using interpolating splines
        for t_idx, t in enumerate(time_points):
            # Determine current gait phase
            step_idx = int(t / step_timing)
            if step_idx >= len(footsteps):
                step_idx = len(footsteps) - 1

            # Get current step information
            current_step = footsteps[step_idx]

            # Generate leg movement based on gait phase
            phase_in_step = (t % step_timing) / step_timing

            if current_step['foot'] == 'left':
                # Left leg is swing leg, right leg is stance leg
                left_pos = self.calculate_swing_trajectory(
                    current_step['position'], phase_in_step, 'left'
                )
                right_pos = self.calculate_stance_trajectory(
                    current_step['position'], phase_in_step, 'right'
                )
            else:
                # Right leg is swing leg, left leg is stance leg
                left_pos = self.calculate_stance_trajectory(
                    current_step['position'], phase_in_step, 'left'
                )
                right_pos = self.calculate_swing_trajectory(
                    current_step['position'], phase_in_step, 'right'
                )

            # Convert foot positions to joint angles using inverse kinematics
            left_joints = self.foot_position_to_joints(left_pos, 'left')
            right_joints = self.foot_position_to_joints(right_pos, 'right')

            left_trajectory[t_idx] = left_joints
            right_trajectory[t_idx] = right_joints

        return left_trajectory, right_trajectory

    def calculate_swing_trajectory(self, target_pos: np.ndarray,
                                 phase: float, leg_side: str) -> np.ndarray:
        """Calculate swing leg trajectory with foot clearance"""
        # Use cubic spline for smooth trajectory
        # Swing phase: lift foot, move forward, lower foot

        if phase < 0.3:  # Lift phase (30% of swing)
            lift_ratio = phase / 0.3
            height = self.step_height * (3 * lift_ratio**2 - 2 * lift_ratio**3)
            x_pos = target_pos[0] - self.step_length * (1 - lift_ratio)
            y_pos = target_pos[1]
        elif phase < 0.7:  # Forward phase (40% of swing)
            forward_ratio = (phase - 0.3) / 0.4
            height = self.step_height
            x_pos = target_pos[0] - self.step_length * (1 - 0.3) + self.step_length * forward_ratio
            y_pos = target_pos[1]
        else:  # Lower phase (30% of swing)
            lower_ratio = (phase - 0.7) / 0.3
            height = self.step_height * (1 - (3 * lower_ratio**2 - 2 * lower_ratio**3))
            x_pos = target_pos[0]
            y_pos = target_pos[1]

        return np.array([x_pos, y_pos, height])

    def calculate_stance_trajectory(self, target_pos: np.ndarray,
                                  phase: float, leg_side: str) -> np.ndarray:
        """Calculate stance leg trajectory (remains in contact)"""
        # Stance leg remains in contact with ground during stance phase
        # May adjust slightly for balance
        stance_pos = target_pos.copy()

        # Small adjustments for balance during stance
        balance_adjustment = 0.01 * math.sin(2 * math.pi * phase)  # Small oscillation
        stance_pos[2] += balance_adjustment  # Height adjustment

        return stance_pos

    def foot_position_to_joints(self, foot_pos: np.ndarray, leg_side: str) -> np.ndarray:
        """Convert foot position to joint angles using inverse kinematics"""
        # This is a simplified 3D position to joint angle conversion
        # In practice, this would use the full IK solver

        # For demonstration, we'll use a simple analytical solution
        # for a 3-link leg (hip, knee, ankle) in the sagittal plane
        x, y, z = foot_pos

        # Calculate leg length (simplified)
        leg_length = 0.8  # Sum of thigh and shin lengths

        # Calculate hip pitch angle
        hip_pitch = math.atan2(z, x) if x != 0 else 0

        # Calculate knee angle based on leg extension
        distance = math.sqrt(x**2 + z**2)
        if distance < leg_length:
            # Leg can reach target
            knee_angle = math.pi - 2 * math.asin(distance / leg_length)
        else:
            # Target too far, extend leg fully
            knee_angle = 0

        # Calculate ankle angle for proper foot orientation
        ankle_pitch = -hip_pitch - knee_angle

        # For full 6 DOF leg, we would need a complete IK solution
        # This is a simplified 3 DOF solution
        if leg_side == 'left':
            return np.array([0, 0, hip_pitch, knee_angle, ankle_pitch, 0])  # [hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll]
        else:
            return np.array([0, 0, hip_pitch, knee_angle, ankle_pitch, 0])

    def generate_arm_swing_patterns(self, left_leg_traj: np.ndarray,
                                   right_leg_traj: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Generate arm swing patterns to maintain balance during walking"""
        # Arm swings oppose leg movements for balance
        # Left arm swings opposite to right leg, right arm swings opposite to left leg

        num_time_steps = len(left_leg_traj)
        num_arm_joints = 6  # shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_pitch, wrist_yaw

        left_arm_traj = np.zeros((num_time_steps, num_arm_joints))
        right_arm_traj = np.zeros((num_time_steps, num_arm_joints))

        for i in range(num_time_steps):
            # Calculate phase of gait cycle
            gait_phase = (i * 0.01) % (2 * self.step_duration) / (2 * self.step_duration)

            # Left arm opposes right leg
            left_arm_swing = math.sin(2 * math.pi * gait_phase + math.pi) * 0.2  # 0.2 rad swing amplitude
            left_arm_traj[i, 0] = 0.1 + left_arm_swing  # Add neutral position offset

            # Right arm opposes left leg
            right_arm_swing = math.sin(2 * math.pi * gait_phase) * 0.2
            right_arm_traj[i, 0] = 0.1 + right_arm_swing  # Add neutral position offset

            # Add slight elbow movement for natural arm swing
            left_arm_traj[i, 3] = 0.1 + math.sin(2 * math.pi * gait_phase + math.pi) * 0.05
            right_arm_traj[i, 3] = 0.1 + math.sin(2 * math.pi * gait_phase) * 0.05

        return left_arm_traj, right_arm_traj
```

## Hands-on Exercise

Implement a complete humanoid interaction system:

1. Create a kinematics model for a humanoid robot
2. Implement inverse kinematics solvers for arms and legs
3. Design a balance controller using ZMP principles
4. Generate natural walking patterns with gait planning
5. Integrate with perception systems for obstacle avoidance
6. Test with Isaac Sim or Gazebo simulation

## Review Questions

1. What are the main challenges in humanoid robot kinematics compared to simpler robots?
2. How does Zero Moment Point (ZMP) control enable stable bipedal locomotion?
3. What role does inverse kinematics play in humanoid manipulation?
4. How do arm swing patterns contribute to balance during walking?

## Further Reading

- "Humanoid Robotics: A Reference" by Veloso
- "Bipedal Locomotion: Modeling, Simulation, Control" by Goswami & Vadakkepat
- "Robotics: Control, Sensing, Vision, and Intelligence" by Fu, Gonzalez, and Lee
- "Dynamic Walking in Humanoid Robots" by Kajita et al.