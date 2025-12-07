---
sidebar_position: 10
---

# Sim-to-Real Transfer: Bridging Simulation and Reality

This chapter focuses on sim-to-real transfer techniques, covering how to successfully deploy models trained in simulation to real robotic systems, addressing the reality gap and domain adaptation challenges.

## Learning Objectives

By the end of this week, you will be able to:
- Understand the reality gap and domain shift challenges in robotics
- Apply domain randomization techniques for robust simulation training
- Implement system identification for simulation calibration
- Design domain adaptation networks for sim-to-real transfer
- Evaluate and validate sim-to-real transfer performance
- Optimize transfer techniques for humanoid robot applications

## Prerequisites

- Understanding of simulation environments (Weeks 6-9)
- Knowledge of reinforcement learning (Week 8)
- Basic understanding of computer vision and perception
- Familiarity with neural network concepts

## Introduction to Sim-to-Real Transfer

Sim-to-real transfer, also known as domain transfer, is the process of taking policies, models, or behaviors trained in simulation and successfully deploying them on real robots. This approach is crucial because:

- **Safety**: Testing in simulation is safer than real-world trials
- **Cost**: Simulation is significantly cheaper than real hardware
- **Speed**: Simulation can run faster than real-time
- **Repeatability**: Controlled conditions for consistent experiments
- **Scalability**: Large-scale training is feasible in simulation

However, the "reality gap" poses significant challenges due to differences between simulated and real environments.

### The Reality Gap Problem

The reality gap consists of several factors:

1. **Visual Domain Shift**: Differences in lighting, textures, colors, and sensor noise
2. **Dynamics Mismatch**: Inaccuracies in physics simulation
3. **Actuator Differences**: Real motors behave differently than simulated ones
4. **Sensor Noise**: Real sensors have noise and delays not captured in simulation
5. **Environmental Factors**: Unmodeled forces, vibrations, and disturbances

## Domain Randomization Techniques

Domain randomization is a technique that increases the diversity of simulation environments to make models robust to domain shifts.

### Visual Domain Randomization

```python
import numpy as np
import cv2
import random
from typing import Dict, Tuple, Optional

class VisualDomainRandomizer:
    def __init__(self):
        self.randomization_params = {
            'lighting': {
                'intensity_range': (0.3, 2.0),
                'color_temperature_range': (3000, 8000),
                'direction_variance': 0.5
            },
            'textures': {
                'roughness_range': (0.0, 1.0),
                'metallic_range': (0.0, 1.0),
                'normal_map_strength_range': (0.0, 0.5)
            },
            'colors': {
                'hue_shift_range': (-10, 10),
                'saturation_range': (0.5, 1.5),
                'brightness_range': (0.7, 1.3)
            },
            'post_processing': {
                'blur_range': (0.0, 2.0),
                'noise_std_range': (0.0, 0.05),
                'compression_quality_range': (50, 100)
            }
        }

    def randomize_image(self, image: np.ndarray) -> np.ndarray:
        """Apply visual domain randomization to an image"""
        # Apply color adjustments
        randomized_image = self.randomize_colors(image)

        # Apply lighting effects
        randomized_image = self.randomize_lighting(randomized_image)

        # Apply noise and blur
        randomized_image = self.add_noise_blur(randomized_image)

        # Apply compression artifacts
        randomized_image = self.add_compression_artifacts(randomized_image)

        return randomized_image

    def randomize_colors(self, image: np.ndarray) -> np.ndarray:
        """Randomize colors using HSV adjustments"""
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV).astype(np.float32)

        # Random hue shift
        hue_shift = random.uniform(
            self.randomization_params['colors']['hue_shift_range'][0],
            self.randomization_params['colors']['hue_shift_range'][1]
        )
        hsv[:, :, 0] = (hsv[:, :, 0] + hue_shift) % 180

        # Random saturation adjustment
        sat_factor = random.uniform(
            self.randomization_params['colors']['saturation_range'][0],
            self.randomization_params['colors']['saturation_range'][1]
        )
        hsv[:, :, 1] = np.clip(hsv[:, :, 1] * sat_factor, 0, 255)

        # Random brightness adjustment
        bright_factor = random.uniform(
            self.randomization_params['colors']['brightness_range'][0],
            self.randomization_params['colors']['brightness_range'][1]
        )
        hsv[:, :, 2] = np.clip(hsv[:, :, 2] * bright_factor, 0, 255)

        # Convert back to RGB
        result = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2RGB)
        return result

    def randomize_lighting(self, image: np.ndarray) -> np.ndarray:
        """Randomize lighting conditions"""
        # Random intensity adjustment
        intensity = random.uniform(
            self.randomization_params['lighting']['intensity_range'][0],
            self.randomization_params['lighting']['intensity_range'][1]
        )

        # Apply intensity
        adjusted = image.astype(np.float32) * intensity
        result = np.clip(adjusted, 0, 255).astype(np.uint8)

        return result

    def add_noise_blur(self, image: np.ndarray) -> np.ndarray:
        """Add noise and blur to simulate sensor imperfections"""
        # Add Gaussian noise
        noise_std = random.uniform(
            self.randomization_params['post_processing']['noise_std_range'][0],
            self.randomization_params['post_processing']['noise_std_range'][1]
        )
        noise = np.random.normal(0, noise_std, image.shape).astype(np.float32)
        noisy_image = np.clip(image.astype(np.float32) + noise * 255, 0, 255).astype(np.uint8)

        # Add blur
        blur_kernel = random.uniform(
            self.randomization_params['post_processing']['blur_range'][0],
            self.randomization_params['post_processing']['blur_range'][1]
        )

        if blur_kernel > 0:
            kernel_size = int(blur_kernel * 2) + 1  # Ensure odd kernel size
            kernel_size = max(1, kernel_size)  # Minimum kernel size of 1
            blurred = cv2.GaussianBlur(noisy_image, (kernel_size, kernel_size), 0)
            return blurred

        return noisy_image

    def add_compression_artifacts(self, image: np.ndarray) -> np.ndarray:
        """Simulate JPEG compression artifacts"""
        quality = random.randint(
            self.randomization_params['post_processing']['compression_quality_range'][0],
            self.randomization_params['post_processing']['compression_quality_range'][1]
        )

        # Encode and decode to simulate compression
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        result, encoded_img = cv2.imencode('.jpg', image, encode_param)
        if result:
            decoded_img = cv2.imdecode(encoded_img, 1)
            return decoded_img

        return image

class DynamicsDomainRandomizer:
    """Randomize physical parameters for robust simulation"""
    def __init__(self):
        self.params = {
            'mass': {'factor_range': (0.8, 1.2)},
            'friction': {'factor_range': (0.5, 2.0)},
            'restitution': {'factor_range': (0.0, 0.5)},
            'damping': {'factor_range': (0.8, 1.2)},
            'actuator_delay': {'range': (0.0, 0.05)},  # seconds
            'sensor_noise': {'std_range': (0.0, 0.1)}
        }

    def randomize_mass(self, nominal_mass: float) -> float:
        """Randomize mass parameter"""
        factor = random.uniform(
            self.params['mass']['factor_range'][0],
            self.params['mass']['factor_range'][1]
        )
        return nominal_mass * factor

    def randomize_friction(self, nominal_friction: float) -> float:
        """Randomize friction coefficient"""
        factor = random.uniform(
            self.params['friction']['factor_range'][0],
            self.params['friction']['factor_range'][1]
        )
        return min(nominal_friction * factor, 10.0)  # Cap at reasonable value

    def randomize_restitution(self) -> float:
        """Randomize restitution (bounciness)"""
        return random.uniform(
            self.params['restitution']['factor_range'][0],
            self.params['restitution']['factor_range'][1]
        )

    def get_actuator_delay(self) -> float:
        """Get random actuator delay"""
        return random.uniform(
            self.params['actuator_delay']['range'][0],
            self.params['actuator_delay']['range'][1]
        )

    def get_sensor_noise_std(self) -> float:
        """Get random sensor noise standard deviation"""
        return random.uniform(
            self.params['sensor_noise']['std_range'][0],
            self.params['sensor_noise']['std_range'][1]
        )
```

### Systematic Domain Randomization

```python
import torch
import torch.nn as nn
import numpy as np
from typing import List, Dict, Any

class SystematicDomainRandomization:
    """Systematically randomize simulation parameters"""
    def __init__(self, num_envs: int = 64):
        self.num_envs = num_envs
        self.visual_randomizer = VisualDomainRandomizer()
        self.dynamics_randomizer = DynamicsDomainRandomizer()

        # Store randomization values for each environment
        self.env_params = [{} for _ in range(num_envs)]
        self.current_env = 0

    def setup_environments(self):
        """Setup multiple environments with different randomizations"""
        for i in range(self.num_envs):
            # Visual parameters
            self.env_params[i]['lighting_intensity'] = random.uniform(0.3, 2.0)
            self.env_params[i]['texture_roughness'] = random.uniform(0.0, 1.0)

            # Dynamics parameters
            self.env_params[i]['mass_multiplier'] = random.uniform(0.8, 1.2)
            self.env_params[i]['friction_multiplier'] = random.uniform(0.5, 2.0)
            self.env_params[i]['actuator_delay'] = random.uniform(0.0, 0.05)

    def get_current_env_params(self) -> Dict[str, Any]:
        """Get parameters for current environment"""
        return self.env_params[self.current_env]

    def switch_environment(self):
        """Switch to next environment"""
        self.current_env = (self.current_env + 1) % self.num_envs

class ProgressiveDomainRandomization:
    """Progressive domain randomization that increases difficulty over time"""
    def __init__(self):
        self.training_step = 0
        self.progression_schedule = {
            0: {'range_factor': 0.0},      # No randomization initially
            1000: {'range_factor': 0.2},   # 20% of full range after 1000 steps
            5000: {'range_factor': 0.5},   # 50% after 5000 steps
            10000: {'range_factor': 0.8},  # 80% after 10000 steps
            20000: {'range_factor': 1.0}   # Full range after 20000 steps
        }

    def get_randomization_factor(self) -> float:
        """Get current randomization factor based on training progress"""
        # Find the highest milestone that's been reached
        factor = 0.0
        for milestone, params in sorted(self.progression_schedule.items()):
            if self.training_step >= milestone:
                factor = params['range_factor']

        return factor

    def update_training_step(self, step: int):
        """Update current training step"""
        self.training_step = step

    def apply_progressive_randomization(self, base_params: Dict[str, float]) -> Dict[str, float]:
        """Apply progressive domain randomization"""
        factor = self.get_randomization_factor()

        randomized_params = {}
        for param_name, base_value in base_params.items():
            # Calculate randomization range based on factor
            if 'friction' in param_name:
                rand_range = (0.5, 2.0)  # Friction multiplier range
            elif 'mass' in param_name:
                rand_range = (0.8, 1.2)  # Mass multiplier range
            else:
                rand_range = (0.9, 1.1)  # General multiplier range

            # Calculate effective range based on progression
            effective_range = 1.0 + factor * (rand_range[1] - 1.0)
            effective_range_min = 1.0 - factor * (1.0 - rand_range[0])

            multiplier = random.uniform(effective_range_min, effective_range)
            randomized_params[param_name] = base_value * multiplier

        return randomized_params
```

## System Identification for Simulation Calibration

System identification involves characterizing the real robot's dynamics to calibrate the simulation.

```python
import numpy as np
from scipy.optimize import minimize
from sklearn.linear_model import LinearRegression
from typing import Tuple, Dict, List
import matplotlib.pyplot as plt

class SystemIdentifier:
    """System identification for robot dynamics calibration"""
    def __init__(self):
        self.identification_data = {
            'inputs': [],      # Applied forces/torques
            'outputs': [],     # Observed positions/velocities
            'timestamps': []
        }

        self.identified_params = {}
        self.simulation_params = {}

    def collect_identification_data(self, inputs: np.ndarray, outputs: np.ndarray,
                                  timestamps: np.ndarray):
        """Collect data for system identification"""
        self.identification_data['inputs'].extend(inputs.tolist())
        self.identification_data['outputs'].extend(outputs.tolist())
        self.identification_data['timestamps'].extend(timestamps.tolist())

    def identify_mass_matrix(self) -> np.ndarray:
        """Identify mass matrix using least squares"""
        # Collect input-output data pairs
        U = np.array(self.identification_data['inputs'])
        Y = np.array(self.identification_data['outputs'])

        # For mass identification, we can use force-acceleration relationship
        # F = M * a => M = F / a (for single DOF)
        # For multi-DOF: Y = U * M^(-1) => M^(-1) = (U^T * U)^(-1) * U^T * Y

        if U.shape[0] < U.shape[1]:
            raise ValueError("Need more data points than parameters")

        # Solve for inverse mass matrix
        try:
            U_pinv = np.linalg.pinv(U)
            Minv = U_pinv @ Y
            M = np.linalg.inv(Minv)

            self.identified_params['mass_matrix'] = M
            return M
        except np.linalg.LinAlgError:
            print("Matrix inversion failed, using pseudo-inverse")
            Minv = np.linalg.pinv(U) @ Y
            M = np.linalg.pinv(Minv)
            self.identified_params['mass_matrix'] = M
            return M

    def identify_friction_parameters(self) -> Dict[str, float]:
        """Identify friction parameters (static and dynamic)"""
        # Collect data during slow movements to isolate friction
        velocities = []
        required_torques = []

        for i in range(len(self.identification_data['inputs']) - 1):
            dt = (self.identification_data['timestamps'][i+1] -
                  self.identification_data['timestamps'][i])

            if dt > 0:
                vel = (self.identification_data['outputs'][i+1][0] -
                      self.identification_data['outputs'][i][0]) / dt
                velocities.append(abs(vel))
                required_torques.append(abs(self.identification_data['inputs'][i][0]))

        velocities = np.array(velocities)
        torques = np.array(required_torques)

        # Friction model: tau = tau_static_sign + B * v (Coulomb + viscous)
        # For low velocities, tau ≈ tau_static (approximately)
        low_vel_indices = velocities < 0.01
        if np.sum(low_vel_indices) > 0:
            static_friction = np.mean(torques[low_vel_indices])
        else:
            static_friction = 0.0

        # For higher velocities, fit linear model to get viscous friction
        high_vel_indices = velocities > 0.05
        if np.sum(high_vel_indices) > 1:
            vel_high = velocities[high_vel_indices]
            tau_high = torques[high_vel_indices]

            # Fit: tau = tau_static + B * v
            # We already estimated tau_static, so: B = (tau - tau_static) / v
            B = np.mean((tau_high - static_friction) / vel_high)
        else:
            B = 0.0

        friction_params = {
            'static_friction': static_friction,
            'viscous_friction': B,
            'friction_model': 'coulomb_viscous'
        }

        self.identified_params['friction'] = friction_params
        return friction_params

    def identify_actuator_dynamics(self) -> Dict[str, float]:
        """Identify actuator dynamics (delay, bandwidth, saturation)"""
        # Analyze step response data
        input_changes = []
        output_delays = []

        for i in range(len(self.identification_data['inputs']) - 1):
            input_change = (np.array(self.identification_data['inputs'][i+1]) -
                           np.array(self.identification_data['inputs'][i]))
            output_change = (np.array(self.identification_data['outputs'][i+1]) -
                            np.array(self.identification_data['outputs'][i]))

            if np.linalg.norm(input_change) > 0.1:  # Significant input change
                dt = (self.identification_data['timestamps'][i+1] -
                      self.identification_data['timestamps'][i])

                input_changes.append(input_change)
                output_delays.append(dt)

        if input_changes:
            # Estimate delay as average delay between input and output
            avg_delay = np.mean(output_delays) if output_delays else 0.0

            # Estimate actuator bandwidth from frequency response
            # (would need sinusoidal input data for proper identification)
            bandwidth = 10.0  # Hz (placeholder - would be identified properly)

            actuator_params = {
                'delay': avg_delay,
                'bandwidth': bandwidth,
                'saturation_limit': 100.0  # Nm (placeholder)
            }

            self.identified_params['actuator_dynamics'] = actuator_params
            return actuator_params

        return {}

    def calibrate_simulation(self, sim_robot) -> bool:
        """Calibrate simulation parameters based on identified real parameters"""
        if not self.identified_params:
            print("No identified parameters available for calibration")
            return False

        # Update simulation with identified parameters
        if 'mass_matrix' in self.identified_params:
            sim_robot.set_mass_matrix(self.identified_params['mass_matrix'])

        if 'friction' in self.identified_params:
            friction_params = self.identified_params['friction']
            sim_robot.set_friction_coefficients(
                static=friction_params['static_friction'],
                viscous=friction_params['viscous_friction']
            )

        if 'actuator_dynamics' in self.identified_params:
            actuator_params = self.identified_params['actuator_dynamics']
            sim_robot.set_actuator_dynamics(
                delay=actuator_params['delay'],
                bandwidth=actuator_params['bandwidth'],
                saturation_limit=actuator_params['saturation_limit']
            )

        print("Simulation calibrated with identified parameters")
        return True

class SimulationCalibrator:
    """Complete simulation calibration pipeline"""
    def __init__(self):
        self.system_identifier = SystemIdentifier()
        self.parameter_optimizer = ParameterOptimizer()

    def calibrate_with_multiple_trials(self, real_robot, sim_robot,
                                     excitation_signals: List[np.ndarray]):
        """Calibrate using multiple excitation trials"""
        all_identified_params = []

        for trial, signal in enumerate(excitation_signals):
            print(f"Running excitation trial {trial + 1}/{len(excitation_signals)}")

            # Apply excitation signal to real robot
            real_inputs, real_outputs, real_timestamps = self.apply_excitation(
                real_robot, signal
            )

            # Collect data
            self.system_identifier.collect_identification_data(
                real_inputs, real_outputs, real_timestamps
            )

        # Identify parameters from all trials
        mass_matrix = self.system_identifier.identify_mass_matrix()
        friction_params = self.system_identifier.identify_friction_parameters()
        actuator_params = self.system_identifier.identify_actuator_dynamics()

        # Calibrate simulation
        success = self.system_identifier.calibrate_simulation(sim_robot)

        return success

    def apply_excitation(self, robot, signal: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Apply excitation signal and collect response data"""
        inputs = []
        outputs = []
        timestamps = []

        for i, cmd in enumerate(signal):
            # Apply command
            robot.apply_command(cmd)

            # Measure response
            output = robot.get_state()
            timestamp = robot.get_time()

            inputs.append(cmd)
            outputs.append(output)
            timestamps.append(timestamp)

            # Small delay to allow system to respond
            robot.sleep(0.001)

        return np.array(inputs), np.array(outputs), np.array(timestamps)
```

## Domain Adaptation Networks

Domain adaptation networks learn to map between simulation and real-world representations.

```python
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader, TensorDataset
from typing import Tuple, Optional

class DomainAdaptationNetwork(nn.Module):
    """Neural network for domain adaptation between sim and real"""
    def __init__(self, input_dim: int, hidden_dim: int = 256, output_dim: int = None):
        super(DomainAdaptationNetwork, self).__init__()

        if output_dim is None:
            output_dim = input_dim

        # Encoder to extract domain-invariant features
        self.encoder = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim // 2),
            nn.ReLU(),
            nn.Linear(hidden_dim // 2, hidden_dim // 4),
        )

        # Separate decoders for sim-to-real and real-to-sim
        self.sim_to_real_decoder = nn.Sequential(
            nn.Linear(hidden_dim // 4, hidden_dim // 2),
            nn.ReLU(),
            nn.Linear(hidden_dim // 2, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, output_dim),
        )

        self.real_to_sim_decoder = nn.Sequential(
            nn.Linear(hidden_dim // 4, hidden_dim // 2),
            nn.ReLU(),
            nn.Linear(hidden_dim // 2, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, output_dim),
        )

        # Domain classifier to encourage domain-invariant features
        self.domain_classifier = nn.Sequential(
            nn.Linear(hidden_dim // 4, 64),
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Linear(32, 2),  # Binary classification: sim vs real
        )

    def forward(self, x: torch.Tensor, domain: str = 'sim_to_real') -> torch.Tensor:
        """Forward pass through domain adaptation network"""
        # Encode to domain-invariant features
        encoded = self.encoder(x)

        if domain == 'sim_to_real':
            return self.sim_to_real_decoder(encoded)
        elif domain == 'real_to_sim':
            return self.real_to_sim_decoder(encoded)
        else:
            raise ValueError(f"Unknown domain: {domain}")

    def classify_domain(self, x: torch.Tensor) -> torch.Tensor:
        """Classify whether input is from sim or real domain"""
        encoded = self.encoder(x)
        return self.domain_classifier(encoded)

class AdversarialDomainAdaptation:
    """Adversarial domain adaptation using domain confusion"""
    def __init__(self, input_dim: int, hidden_dim: int = 256):
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        # Domain adaptation network
        self.da_network = DomainAdaptationNetwork(input_dim, hidden_dim).to(self.device)

        # Optimizers
        self.feature_extractor_opt = torch.optim.Adam(
            list(self.da_network.encoder.parameters()) +
            list(self.da_network.sim_to_real_decoder.parameters()) +
            list(self.da_network.real_to_sim_decoder.parameters()),
            lr=1e-4
        )

        self.domain_classifier_opt = torch.optim.Adam(
            self.da_network.domain_classifier.parameters(),
            lr=1e-4
        )

        # Loss functions
        self.mse_loss = nn.MSELoss()
        self.cross_entropy = nn.CrossEntropyLoss()

        self.adv_weight = 0.5  # Weight for adversarial loss

    def train_step(self, sim_data: torch.Tensor, real_data: torch.Tensor) -> Dict[str, float]:
        """Single training step for adversarial domain adaptation"""
        sim_data = sim_data.to(self.device)
        real_data = real_data.to(self.device)

        # Create domain labels (0 for sim, 1 for real)
        sim_labels = torch.zeros(sim_data.size(0), dtype=torch.long, device=self.device)
        real_labels = torch.ones(real_data.size(0), dtype=torch.long, device=self.device)

        losses = {}

        # Step 1: Train domain classifier to distinguish domains
        self.domain_classifier_opt.zero_grad()

        # Classify sim data
        sim_encoded = self.da_network.encoder(sim_data)
        sim_domain_logits = self.da_network.domain_classifier(sim_encoded)

        # Classify real data
        real_encoded = self.da_network.encoder(real_data.detach())  # Detach to prevent gradient flow
        real_domain_logits = self.da_network.domain_classifier(real_encoded)

        # Domain classification loss
        domain_loss = (self.cross_entropy(sim_domain_logits, sim_labels) +
                      self.cross_entropy(real_domain_logits, real_labels))

        domain_loss.backward()
        self.domain_classifier_opt.step()

        # Step 2: Train feature extractor to confuse domain classifier
        self.feature_extractor_opt.zero_grad()

        # Get encoded features
        sim_encoded_adv = self.da_network.encoder(sim_data)
        real_encoded_adv = self.da_network.encoder(real_data)

        # Domain confusion loss (try to fool classifier)
        sim_domain_logits_adv = self.da_network.domain_classifier(sim_encoded_adv)
        real_domain_logits_adv = self.da_network.domain_classifier(real_encoded_adv)

        # Target domain: try to make both look like "mixed" domain
        # Use soft labels to encourage domain confusion
        soft_sim_target = torch.ones(sim_data.size(0), 2, device=self.device) * 0.5
        soft_real_target = torch.ones(real_data.size(0), 2, device=self.device) * 0.5

        # Use MSE loss for soft targets
        domain_confusion_loss = (F.mse_loss(F.softmax(sim_domain_logits_adv, dim=1), soft_sim_target) +
                                F.mse_loss(F.softmax(real_domain_logits_adv, dim=1), soft_real_target))

        # Reconstruction loss (cycle consistency)
        sim_to_real = self.da_network(sim_data, 'sim_to_real')
        real_to_sim = self.da_network(real_data, 'real_to_sim')

        cycle_sim = self.da_network(sim_to_real, 'real_to_sim')
        cycle_real = self.da_network(real_to_sim, 'sim_to_real')

        reconstruction_loss = (self.mse_loss(cycle_sim, sim_data) +
                              self.mse_loss(cycle_real, real_data))

        # Total loss
        total_loss = (reconstruction_loss +
                     self.adv_weight * domain_confusion_loss)

        total_loss.backward()
        self.feature_extractor_opt.step()

        losses = {
            'domain_loss': domain_loss.item(),
            'domain_confusion_loss': domain_confusion_loss.item(),
            'reconstruction_loss': reconstruction_loss.item(),
            'total_loss': total_loss.item()
        }

        return losses

    def adapt_data(self, data: torch.Tensor, source_domain: str, target_domain: str) -> torch.Tensor:
        """Adapt data from source domain to target domain"""
        self.da_network.eval()
        with torch.no_grad():
            if source_domain == 'sim' and target_domain == 'real':
                return self.da_network(data, 'sim_to_real')
            elif source_domain == 'real' and target_domain == 'sim':
                return self.da_network(data, 'real_to_sim')
            else:
                return data  # No adaptation needed
```

## Reinforcement Learning for Robust Policies

Making policies robust to domain shifts through RL techniques:

```python
import torch
import torch.nn as nn
import numpy as np
from typing import Tuple, Dict, Any
import random

class DomainRobustPolicy(nn.Module):
    """Policy network designed to be robust to domain shifts"""
    def __init__(self, state_dim: int, action_dim: int, hidden_dim: int = 256):
        super(DomainRobustPolicy, self).__init__()

        # Shared feature extractor
        self.feature_extractor = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim // 2),
            nn.ReLU(),
        )

        # Domain-specific normalization layers to handle different input distributions
        self.sim_norm = nn.BatchNorm1d(hidden_dim // 2)
        self.real_norm = nn.BatchNorm1d(hidden_dim // 2)

        # Policy head
        self.policy_head = nn.Sequential(
            nn.Linear(hidden_dim // 2, hidden_dim // 2),
            nn.ReLU(),
            nn.Linear(hidden_dim // 2, hidden_dim // 4),
            nn.ReLU(),
            nn.Linear(hidden_dim // 4, action_dim),
            nn.Tanh()  # Actions are bounded
        )

        # Domain discriminator to encourage domain-invariant representations
        self.domain_discriminator = nn.Sequential(
            nn.Linear(hidden_dim // 2, 64),
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Linear(32, 1),
            nn.Sigmoid()
        )

    def forward(self, state: torch.Tensor, domain: str = 'sim') -> torch.Tensor:
        """Forward pass with domain-specific normalization"""
        features = self.feature_extractor(state)

        # Apply domain-specific normalization
        if domain == 'sim':
            normalized_features = self.sim_norm(features)
        else:  # real
            normalized_features = self.real_norm(features)

        action = self.policy_head(normalized_features)
        return action

    def get_features(self, state: torch.Tensor) -> torch.Tensor:
        """Extract features for domain discrimination"""
        features = self.feature_extractor(state)
        return features

class RobustRLTrainer:
    """Trainer for domain-robust reinforcement learning"""
    def __init__(self, policy: DomainRobustPolicy, learning_rate: float = 3e-4):
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.policy = policy.to(self.device)

        # Optimizers
        self.policy_opt = torch.optim.Adam(policy.parameters(), lr=learning_rate)

        # Loss functions
        self.mse_loss = nn.MSELoss()

        # Training parameters
        self.domain_confusion_weight = 0.1
        self.action_regularization_weight = 0.01

    def compute_domain_confusion_loss(self, sim_features: torch.Tensor,
                                    real_features: torch.Tensor) -> torch.Tensor:
        """Compute loss to encourage domain-invariant features"""
        # Discriminator should output 0.5 for both domains (can't distinguish)
        sim_domain_pred = self.policy.domain_discriminator(sim_features)
        real_domain_pred = self.policy.domain_discriminator(real_features)

        # Target: both should be 0.5 (domain confusion)
        target = torch.ones_like(sim_domain_pred) * 0.5

        domain_confusion_loss = (self.mse_loss(sim_domain_pred, target) +
                                self.mse_loss(real_domain_pred, target))

        return domain_confusion_loss

    def train_step(self, sim_batch: Dict[str, torch.Tensor],
                  real_batch: Dict[str, torch.Tensor]) -> Dict[str, float]:
        """Single training step with both sim and real data"""
        # Move data to device
        sim_states = sim_batch['states'].to(self.device)
        sim_actions = sim_batch['actions'].to(self.device)
        sim_rewards = sim_batch['rewards'].to(self.device)

        real_states = real_batch['states'].to(self.device)
        real_actions = real_batch['actions'].to(self.device)
        real_rewards = real_batch['rewards'].to(self.device)

        losses = {}

        # Compute features for domain confusion
        sim_features = self.policy.get_features(sim_states)
        real_features = self.policy.get_features(real_states)

        # Compute domain confusion loss
        domain_confusion_loss = self.compute_domain_confusion_loss(sim_features, real_features)

        # Compute policy losses on both domains
        sim_policy_loss = self.mse_loss(
            self.policy(sim_states, 'sim'),
            sim_actions
        )

        real_policy_loss = self.mse_loss(
            self.policy(real_states, 'real'),
            real_actions
        )

        # Action regularization (encourage smooth actions)
        sim_smooth_actions = self.policy(sim_states, 'sim')
        real_smooth_actions = self.policy(real_states, 'real')

        action_reg_loss = (torch.mean(torch.abs(sim_smooth_actions)) +
                          torch.mean(torch.abs(real_smooth_actions)))

        # Total loss
        total_loss = (sim_policy_loss + real_policy_loss +
                     self.domain_confusion_weight * domain_confusion_loss +
                     self.action_regularization_weight * action_reg_loss)

        # Backpropagate
        self.policy_opt.zero_grad()
        total_loss.backward()
        self.policy_opt.step()

        losses = {
            'sim_policy_loss': sim_policy_loss.item(),
            'real_policy_loss': real_policy_loss.item(),
            'domain_confusion_loss': domain_confusion_loss.item(),
            'action_reg_loss': action_regusion_loss.item(),
            'total_loss': total_loss.item()
        }

        return losses

def create_adaptive_training_pipeline():
    """Create a complete adaptive training pipeline"""

    # 1. Initialize domain randomization
    visual_randomizer = VisualDomainRandomizer()
    dynamics_randomizer = DynamicsDomainRandomizer()
    prog_randomizer = ProgressiveDomainRandomization()

    # 2. Initialize domain adaptation network
    da_network = AdversarialDomainAdaptation(input_dim=256)

    # 3. Initialize robust policy
    policy = DomainRobustPolicy(state_dim=64, action_dim=8)  # Example dimensions
    trainer = RobustRLTrainer(policy)

    return {
        'visual_randomizer': visual_randomizer,
        'dynamics_randomizer': dynamics_randomizer,
        'prog_randomizer': prog_randomizer,
        'domain_adapter': da_network,
        'robust_policy': policy,
        'trainer': trainer
    }
```

## Humanoid-Specific Sim-to-Real Considerations

Special considerations for humanoid robot applications:

```python
class HumanoidSimToRealAdapter:
    """Specialized adapter for humanoid robot sim-to-real transfer"""
    def __init__(self):
        # Humanoid-specific parameters
        self.com_height = 0.8  # Center of mass height (meters)
        self.step_length = 0.3  # Typical step length (meters)
        self.leg_length = 0.8   # Leg length (meters)

        # Balance-related parameters
        self.zmp_margin = 0.05  # Zero Moment Point safety margin
        self.com_height_tolerance = 0.1  # Acceptable CoM height variation

        # Domain adaptation for humanoid-specific features
        self.balance_adaptation_net = self._build_balance_adaptation_network()
        self.gait_adaptation_net = self._build_gait_adaptation_network()

    def _build_balance_adaptation_network(self):
        """Build network for adapting balance control between sim and real"""
        return nn.Sequential(
            nn.Linear(12, 64),  # 6 DOF CoM pos/vel + 6 IMU readings
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Linear(32, 6),   # Adapted CoM pos/vel
        )

    def _build_gait_adaptation_network(self):
        """Build network for adapting gait patterns"""
        return nn.Sequential(
            nn.Linear(8, 32),   # 4 joint angles * 2 legs
            nn.ReLU(),
            nn.Linear(32, 16),
            nn.ReLU(),
            nn.Linear(16, 8),   # Adapted joint angles
        )

    def adapt_balance_control(self, sim_balance_state: np.ndarray,
                            real_sensor_data: Dict) -> np.ndarray:
        """Adapt balance control from simulation to real robot"""
        # Combine sim balance state with real sensor data
        combined_input = self._prepare_balance_input(sim_balance_state, real_sensor_data)

        # Apply domain adaptation
        adapted_state = self.balance_adaptation_net(
            torch.FloatTensor(combined_input).unsqueeze(0)
        ).detach().numpy()

        return adapted_state[0]

    def adapt_gait_pattern(self, sim_gait_params: Dict[str, float],
                          real_robot_state: Dict) -> Dict[str, float]:
        """Adapt gait parameters from simulation to real robot"""
        # Extract relevant features
        input_features = self._extract_gait_features(sim_gait_params, real_robot_state)

        # Apply adaptation
        adapted_features = self.gait_adaptation_net(
            torch.FloatTensor(input_features).unsqueeze(0)
        ).detach().numpy()

        # Convert back to gait parameters
        adapted_params = self._features_to_gait_params(adapted_features[0], real_robot_state)

        return adapted_params

    def _prepare_balance_input(self, sim_state: np.ndarray,
                             real_data: Dict) -> np.ndarray:
        """Prepare input combining sim and real data for balance adaptation"""
        # Simulated CoM state: [x, y, z, vx, vy, vz]
        sim_com = sim_state[:6]

        # Real sensor data: IMU readings, joint angles, etc.
        real_imu = np.array([
            real_data.get('roll', 0), real_data.get('pitch', 0), real_data.get('yaw', 0),
            real_data.get('angular_vel_x', 0), real_data.get('angular_vel_y', 0), real_data.get('angular_vel_z', 0)
        ])

        # Combine features
        combined = np.concatenate([sim_com, real_imu])
        return combined

    def _extract_gait_features(self, sim_params: Dict, real_state: Dict) -> np.ndarray:
        """Extract features for gait adaptation"""
        features = []

        # Simulated gait parameters
        features.extend([
            sim_params.get('swing_height', 0.1),
            sim_params.get('step_length', 0.3),
            sim_params.get('stance_duration', 0.5),
            sim_params.get('swing_duration', 0.3)
        ])

        # Real robot state for adaptation
        features.extend([
            real_state.get('com_height', 0.8),
            real_state.get('pelvis_orientation', 0),
            real_state.get('current_speed', 0),
            real_state.get('terrain_inclination', 0)
        ])

        return np.array(features)

    def _features_to_gait_params(self, adapted_features: np.ndarray,
                               real_state: Dict) -> Dict[str, float]:
        """Convert adapted features back to gait parameters"""
        return {
            'swing_height': adapted_features[0],
            'step_length': adapted_features[1],
            'stance_duration': adapted_features[2],
            'swing_duration': adapted_features[3],
            'hip_pitch_offset': adapted_features[4],
            'knee_angle_offset': adapted_features[5],
            'ankle_correction': adapted_features[6],
            'pelvis_leveling': adapted_features[7]
        }

    def validate_humanoid_transfer(self, sim_policy, real_robot) -> Dict[str, float]:
        """Validate humanoid-specific sim-to-real transfer"""
        metrics = {}

        # Test balance stability
        balance_success = self._test_balance_transfer(sim_policy, real_robot)
        metrics['balance_success_rate'] = balance_success

        # Test gait stability
        gait_success = self._test_gait_transfer(sim_policy, real_robot)
        metrics['gait_success_rate'] = gait_success

        # Test obstacle negotiation
        obstacle_success = self._test_obstacle_transfer(sim_policy, real_robot)
        metrics['obstacle_success_rate'] = obstacle_success

        # Overall success metric
        metrics['overall_success'] = np.mean([
            balance_success, gait_success, obstacle_success
        ])

        return metrics

    def _test_balance_transfer(self, sim_policy, real_robot) -> float:
        """Test if adapted balance control works on real robot"""
        # Implementation would test balance control
        # This is a simplified placeholder
        return 0.85  # 85% success rate

    def _test_gait_transfer(self, sim_policy, real_robot) -> float:
        """Test if adapted gait works on real robot"""
        # Implementation would test walking patterns
        return 0.78  # 78% success rate

    def _test_obstacle_transfer(self, sim_policy, real_robot) -> float:
        """Test if obstacle navigation transfers successfully"""
        # Implementation would test obstacle avoidance
        return 0.82  # 82% success rate
```

## Evaluation and Validation

### Quantitative Evaluation Metrics

```python
import numpy as np
from typing import Dict, List, Tuple
import matplotlib.pyplot as plt

class SimToRealEvaluator:
    """Comprehensive evaluation framework for sim-to-real transfer"""
    def __init__(self):
        self.metrics_history = {
            'transfer_gap': [],
            'domain_similarity': [],
            'task_performance': [],
            'stability_metrics': []
        }

    def compute_transfer_gap(self, sim_performance: float,
                           real_performance: float) -> float:
        """Compute the transfer gap between sim and real performance"""
        if sim_performance == 0:
            return float('inf') if real_performance != 0 else 0

        # Normalized transfer gap
        gap = abs(sim_performance - real_performance) / abs(sim_performance)
        return min(gap, 10.0)  # Cap extremely large gaps

    def compute_domain_similarity(self, sim_features: np.ndarray,
                                real_features: np.ndarray) -> float:
        """Compute similarity between sim and real feature distributions"""
        from scipy.spatial.distance import wasserstein_distance

        # Compute Wasserstein distance between distributions
        if sim_features.ndim > 1:
            # Compute distance for each feature dimension
            distances = []
            for i in range(sim_features.shape[1]):
                dist = wasserstein_distance(
                    sim_features[:, i], real_features[:, i]
                )
                distances.append(dist)
            avg_distance = np.mean(distances)
        else:
            avg_distance = wasserstein_distance(sim_features, real_features)

        # Convert distance to similarity (higher is better)
        similarity = 1.0 / (1.0 + avg_distance)
        return similarity

    def compute_task_performance(self, task_results: Dict[str, float]) -> float:
        """Compute overall task performance metric"""
        # Weight different aspects of performance
        weights = {
            'success_rate': 0.4,
            'efficiency': 0.3,
            'safety': 0.2,
            'smoothness': 0.1
        }

        total_score = 0.0
        for metric, weight in weights.items():
            if metric in task_results:
                total_score += weight * task_results[metric]

        return total_score

    def compute_stability_metrics(self, time_series_data: np.ndarray) -> Dict[str, float]:
        """Compute stability metrics from time series data"""
        metrics = {}

        # Compute variance (lower is more stable)
        variance = np.var(time_series_data, axis=0)
        metrics['variance'] = np.mean(variance)

        # Compute power spectral density (for oscillation detection)
        if len(time_series_data) > 1:
            fft_data = np.fft.fft(time_series_data, axis=0)
            power_spectrum = np.abs(fft_data) ** 2
            dominant_freq_idx = np.argmax(power_spectrum[1:], axis=0) + 1  # Skip DC component
            metrics['dominant_frequency'] = np.mean(dominant_freq_idx)

        # Compute Lyapunov exponent approximation (for chaos detection)
        if len(time_series_data) > 2:
            diff = np.diff(time_series_data, axis=0)
            divergence = np.mean(np.abs(diff))
            metrics['divergence_rate'] = divergence

        return metrics

    def evaluate_transfer(self, sim_data: Dict, real_data: Dict) -> Dict[str, float]:
        """Comprehensive evaluation of sim-to-real transfer"""
        results = {}

        # 1. Performance comparison
        sim_perf = sim_data.get('performance', 0.0)
        real_perf = real_data.get('performance', 0.0)
        results['transfer_gap'] = self.compute_transfer_gap(sim_perf, real_perf)

        # 2. Domain similarity
        sim_features = sim_data.get('features', np.array([]))
        real_features = real_data.get('features', np.array([]))
        if len(sim_features) > 0 and len(real_features) > 0:
            results['domain_similarity'] = self.compute_domain_similarity(
                sim_features, real_features
            )

        # 3. Task-specific metrics
        sim_task_results = sim_data.get('task_results', {})
        real_task_results = real_data.get('task_results', {})
        results['sim_task_performance'] = self.compute_task_performance(sim_task_results)
        results['real_task_performance'] = self.compute_task_performance(real_task_results)

        # 4. Stability analysis
        sim_timeseries = sim_data.get('timeseries', np.array([]))
        real_timeseries = real_data.get('timeseries', np.array([]))
        if len(real_timeseries) > 0:
            real_stability = self.compute_stability_metrics(real_timeseries)
            results.update({
                f'real_{k}': v for k, v in real_stability.items()
            })

        # 5. Overall transfer score
        # Combine metrics with appropriate weights
        gap_penalty = results['transfer_gap'] * 0.4
        similarity_bonus = results.get('domain_similarity', 0) * 0.3
        performance_ratio = (results['real_task_performance'] /
                           max(results['sim_task_performance'], 0.001)) * 0.3

        results['overall_transfer_score'] = max(0, 1.0 - gap_penalty + similarity_bonus + performance_ratio)

        # Store in history
        self.metrics_history['transfer_gap'].append(results['transfer_gap'])
        self.metrics_history['domain_similarity'].append(results.get('domain_similarity', 0))
        self.metrics_history['task_performance'].append(results['real_task_performance'])

        return results

    def plot_evaluation_results(self, results: Dict[str, float]):
        """Plot evaluation results"""
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))

        # Transfer gap over time
        axes[0, 0].plot(self.metrics_history['transfer_gap'])
        axes[0, 0].set_title('Transfer Gap Over Time')
        axes[0, 0].set_xlabel('Evaluation #')
        axes[0, 0].set_ylabel('Gap')

        # Domain similarity over time
        axes[0, 1].plot(self.metrics_history['domain_similarity'])
        axes[0, 1].set_title('Domain Similarity Over Time')
        axes[0, 1].set_xlabel('Evaluation #')
        axes[0, 1].set_ylabel('Similarity')

        # Task performance over time
        axes[1, 0].plot(self.metrics_history['task_performance'])
        axes[1, 0].set_title('Task Performance Over Time')
        axes[1, 0].set_xlabel('Evaluation #')
        axes[1, 0].set_ylabel('Performance')

        # Overall scores
        overall_scores = [
            max(0, 1.0 - gap * 0.4 + sim * 0.3 + perf * 0.3)
            for gap, sim, perf in zip(
                self.metrics_history['transfer_gap'],
                self.metrics_history['domain_similarity'],
                self.metrics_history['task_performance']
            )
        ]
        axes[1, 1].plot(overall_scores)
        axes[1, 1].set_title('Overall Transfer Score Over Time')
        axes[1, 1].set_xlabel('Evaluation #')
        axes[1, 1].set_ylabel('Score')

        plt.tight_layout()
        plt.show()

def run_comprehensive_evaluation():
    """Run a comprehensive sim-to-real evaluation"""
    evaluator = SimToRealEvaluator()

    # Example evaluation (in practice, this would use real data)
    sim_data = {
        'performance': 0.95,
        'features': np.random.randn(100, 10),  # Simulated features
        'task_results': {
            'success_rate': 0.95,
            'efficiency': 0.88,
            'safety': 0.92,
            'smoothness': 0.90
        },
        'timeseries': np.random.randn(1000, 6)  # Simulated time series
    }

    real_data = {
        'performance': 0.78,
        'features': np.random.randn(100, 10) * 1.2,  # Slightly different distribution
        'task_results': {
            'success_rate': 0.78,
            'efficiency': 0.72,
            'safety': 0.85,
            'smoothness': 0.75
        },
        'timeseries': np.random.randn(1000, 6) * 1.1  # Slightly different
    }

    results = evaluator.evaluate_transfer(sim_data, real_data)

    print("=== Sim-to-Real Transfer Evaluation Results ===")
    for metric, value in results.items():
        print(f"{metric}: {value:.4f}")

    return results

if __name__ == "__main__":
    results = run_comprehensive_evaluation()
```

## Best Practices and Guidelines

### Practical Guidelines for Successful Transfer

```python
class SimToRealBestPractices:
    """Best practices and guidelines for successful sim-to-real transfer"""

    @staticmethod
    def get_visual_domain_randomization_guide() -> Dict[str, Any]:
        """Guidelines for visual domain randomization"""
        return {
            "minimum_requirements": {
                "texture_variability": "Randomize materials, colors, and lighting conditions",
                "geometric_variability": "Include diverse shapes and sizes of objects",
                "sensor_noise": "Add realistic noise, blur, and compression artifacts",
                "dynamic_elements": "Include moving objects and changing backgrounds"
            },
            "progressive_approach": {
                "start_simple": "Begin with minimal randomization and gradually increase",
                "monitor_performance": "Track performance degradation during randomization",
                "adjust_ranges": "Increase randomization ranges based on model robustness"
            },
            "common_pitfalls": [
                "Over-randomization that makes sim unrealistic",
                "Under-randomization that doesn't bridge the gap",
                "Ignoring sensor-specific artifacts (e.g., rolling shutter for cameras)"
            ]
        }

    @staticmethod
    def get_dynamics_calibration_guide() -> Dict[str, Any]:
        """Guidelines for dynamics calibration"""
        return {
            "identification_process": {
                "excitation_design": "Use persistently exciting signals that excite all modes",
                "data_quality": "Ensure high-frequency sampling and minimal noise",
                "validation": "Validate on held-out data not used for identification"
            },
            "calibration_targets": {
                "mass_properties": "Identify mass, center of mass, and inertia tensor",
                "friction_models": "Characterize static, Coulomb, and viscous friction",
                "actuator_dynamics": "Model delays, bandwidth, and saturation effects"
            },
            "iterative_refinement": [
                "Start with basic parameters and add complexity gradually",
                "Cross-validate between simulation and reality frequently",
                "Focus on parameters that most affect task performance"
            ]
        }

    @staticmethod
    def get_training_strategies() -> Dict[str, Any]:
        """Effective training strategies for robust policies"""
        return {
            "domain_randomization": {
                "range_setting": "Set ranges based on real-world variability estimates",
                "correlation_handling": "Consider correlations between different parameters",
                "temporal_consistency": "Maintain parameter consistency within episodes"
            },
            "curriculum_learning": {
                "difficulty_progression": "Gradually increase environment complexity",
                "skill_building": "Master basic skills before complex behaviors",
                "transfer_validation": "Validate transfer after each curriculum stage"
            },
            "meta_learning": {
                "rapid_adaptation": "Train for quick adaptation to new environments",
                "parameter_efficient": "Use techniques like LoRA for efficient fine-tuning",
                "online_learning": "Enable continuous learning during deployment"
            }
        }

    @staticmethod
    def get_evaluation_protocols() -> Dict[str, Any]:
        """Standardized evaluation protocols"""
        return {
            "pre_transfer_tests": [
                "Validate simulation fidelity with system ID",
                "Test basic control capabilities in sim",
                "Establish baseline performance metrics"
            ],
            "transfer_validation": [
                "Test on multiple real-world scenarios",
                "Measure performance degradation quantitatively",
                "Assess safety and stability margins"
            ],
            "post_transfer_refinement": [
                "Collect real-world data for fine-tuning",
                "Update domain randomization based on findings",
                "Iterate the transfer process"
            ]
        }

def print_best_practices_summary():
    """Print a summary of best practices"""
    practices = SimToRealBestPractices()

    print("=== SIM-TO-REAL TRANSFER BEST PRACTICES ===\n")

    print("1. VISUAL DOMAIN RANDOMIZATION:")
    vdr_guide = practices.get_visual_domain_randomization_guide()
    for req, desc in vdr_guide["minimum_requirements"].items():
        print(f"   • {req.replace('_', ' ').title()}: {desc}")

    print("\n2. DYNAMICS CALIBRATION:")
    dyn_guide = practices.get_dynamics_calibration_guide()
    for aspect, desc in dyn_guide["calibration_targets"].items():
        print(f"   • {aspect.replace('_', ' ').title()}: {desc}")

    print("\n3. TRAINING STRATEGIES:")
    train_guide = practices.get_training_strategies()
    for strategy, details in train_guide.items():
        print(f"   • {strategy.replace('_', ' ').title()}")

    print("\n4. EVALUATION PROTOCOLS:")
    eval_protocols = practices.get_evaluation_protocols()
    for phase, tasks in eval_protocols.items():
        print(f"   • {phase.replace('_', ' ').title()}:")
        for task in tasks:
            print(f"     - {task}")

if __name__ == "__main__":
    print_best_practices_summary()
```

## Hands-on Exercise

Create a complete sim-to-real transfer pipeline:

1. Implement domain randomization for your simulation environment
2. Collect system identification data from a real or simulated robot
3. Build a domain adaptation network
4. Train a robust policy using adversarial techniques
5. Evaluate the transfer performance quantitatively
6. Analyze the results and identify improvement areas
7. Iterate the process to improve transfer success

## Review Questions

1. What is the "reality gap" and why is it challenging in robotics?
2. How does domain randomization help bridge the sim-to-real gap?
3. What are the key components of a successful sim-to-real transfer system?
4. How do you evaluate the success of sim-to-real transfer?

## Further Reading

- "Sim-to-Real Transfer in Deep Reinforcement Learning for Robotics" by James et al.
- "Domain Randomization for Transferring Deep Neural Networks" by Tobin et al.
- "Generalizing Skills with Semi-Supervised Reinforcement Learning" by Finn et al.
- "Learning Agile Robotic Locomotion Skills by Imitating Animals" by Peng et al.