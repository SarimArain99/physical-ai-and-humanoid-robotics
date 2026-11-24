/**
 * Quiz Data Definitions
 * Feature: 002-ui-improvements
 * Task: T073
 *
 * Quiz questions by course section/module
 */

/**
 * ROS 2 Fundamentals Quizzes
 */
export const ROS2_QUIZZES = {
  'ros2-intro': {
    id: 'ros2-intro',
    title: 'ROS 2 Introduction',
    questions: [
      {
        id: 'ros2-intro-q1',
        question: 'What is the primary communication pattern used in ROS 2?',
        options: [
          'Shared memory',
          'Publish/Subscribe',
          'Direct function calls',
          'Message queues'
        ],
        correctAnswer: 1,
        difficulty: 'Easy',
        explanation: 'ROS 2 primarily uses a publish/subscribe pattern for asynchronous communication between nodes via topics.'
      },
      {
        id: 'ros2-intro-q2',
        question: 'Which middleware does ROS 2 use for underlying communication?',
        options: [
          'ZeroMQ',
          'DDS (Data Distribution Service)',
          'gRPC',
          'MessagePack'
        ],
        correctAnswer: 1,
        difficulty: 'Easy',
        explanation: 'ROS 2 uses DDS (Data Distribution Service) as its underlying middleware for reliable, real-time communication.'
      },
      {
        id: 'ros2-intro-q3',
        question: 'What is a "node" in ROS 2?',
        options: [
          'A hardware component',
          'A process that performs computation',
          'A data type',
          'A configuration file'
        ],
        correctAnswer: 1,
        difficulty: 'Easy',
        explanation: 'In ROS 2, a node is a process that performs computation and communicates with other nodes using topics, services, or actions.'
      }
    ]
  },

  'ros2-topics': {
    id: 'ros2-topics',
    title: 'ROS 2 Topics and Publishers',
    questions: [
      {
        id: 'ros2-topics-q1',
        question: 'What is a "topic" in ROS 2?',
        options: [
          'A category of data',
          'A named channel for asynchronous communication',
          'A storage location',
          'A type definition'
        ],
        correctAnswer: 1,
        difficulty: 'Easy',
        explanation: 'A topic is a named channel used for asynchronous publish/subscribe communication between nodes.'
      },
      {
        id: 'ros2-topics-q2',
        question: 'Can multiple publishers publish to the same topic?',
        options: [
          'No, only one publisher per topic',
          'Yes, multiple publishers can publish to the same topic',
          'Only if they use different message types',
          'Only with special configuration'
        ],
        correctAnswer: 1,
        difficulty: 'Medium',
        explanation: 'ROS 2 allows multiple publishers to publish to the same topic. All subscribers will receive messages from all publishers.'
      },
      {
        id: 'ros2-topics-q3',
        question: 'What QoS policy is best for sensor data that requires low latency?',
        options: [
          'RELIABLE',
          'BEST_EFFORT',
          'SENSOR_DATA profile',
          'SERVICES_DEFAULT'
        ],
        correctAnswer: 2,
        difficulty: 'Hard',
        explanation: 'The SENSOR_DATA QoS profile is optimized for sensor data, using BEST_EFFORT reliability for low latency delivery.'
      }
    ]
  },

  'ros2-services': {
    id: 'ros2-services',
    title: 'ROS 2 Services and Actions',
    questions: [
      {
        id: 'ros2-services-q1',
        question: 'What is the key difference between a topic and a service?',
        options: [
          'Topics are faster',
          'Services are synchronous (request/response), topics are asynchronous',
          'Services use less bandwidth',
          'Topics require more nodes'
        ],
        correctAnswer: 1,
        difficulty: 'Medium',
        explanation: 'Services provide synchronous request/response communication, while topics provide asynchronous publish/subscribe messaging.'
      },
      {
        id: 'ros2-services-q2',
        question: 'When would you use an Action instead of a Service?',
        options: [
          'For quick operations',
          'For long-running tasks with cancellation support',
          'For one-way communication',
          'For broadcasting data'
        ],
        correctAnswer: 1,
        difficulty: 'Medium',
        explanation: 'Actions are designed for long-running tasks that provide progress feedback and can be cancelled, unlike services.'
      },
      {
        id: 'ros2-services-q3',
        question: 'What are the three message types used in ROS 2 Actions?',
        options: [
          'Request, Response, Result',
          'Goal, Result, Feedback',
          'Input, Output, Status',
          'Start, Update, Complete'
        ],
        correctAnswer: 1,
        difficulty: 'Hard',
        explanation: 'ROS 2 Actions use three message types: Goal (to initiate), Result (final outcome), and Feedback (progress updates).'
      }
    ]
  }
};

/**
 * Digital Twin Quizzes
 */
export const DIGITAL_TWIN_QUIZZES = {
  'digital-twin-intro': {
    id: 'digital-twin-intro',
    title: 'Digital Twin Fundamentals',
    questions: [
      {
        id: 'dt-intro-q1',
        question: 'What is a digital twin?',
        options: [
          'A 3D model of a robot',
          'A virtual replica of a physical system that syncs in real-time',
          'A simulation for testing',
          'A backup of robot data'
        ],
        correctAnswer: 1,
        difficulty: 'Easy',
        explanation: 'A digital twin is a virtual replica of a physical system that maintains bidirectional synchronization with the real asset.'
      },
      {
        id: 'dt-intro-q2',
        question: 'What is the primary benefit of using a digital twin in robotics?',
        options: [
          'Lower computational requirements',
          'Real-time monitoring and prediction without physical risk',
          'Easier hardware maintenance',
          'Reduced network traffic'
        ],
        correctAnswer: 1,
        difficulty: 'Medium',
        explanation: 'Digital twins enable real-time monitoring, testing, and prediction without risking physical equipment or causing downtime.'
      }
    ]
  }
};

/**
 * NVIDIA Isaac Quizzes
 */
export const ISAAC_QUIZZES = {
  'isaac-intro': {
    id: 'isaac-intro',
    title: 'NVIDIA Isaac Sim Introduction',
    questions: [
      {
        id: 'isaac-intro-q1',
        question: 'What physics engine does NVIDIA Isaac Sim use?',
        options: [
          'Bullet Physics',
          'PhysX',
          'ODE (Open Dynamics Engine)',
          'MuJoCo'
        ],
        correctAnswer: 1,
        difficulty: 'Easy',
        explanation: 'NVIDIA Isaac Sim uses PhysX, NVIDIA"s own physics engine, for accurate simulation of rigid bodies, joints, and collisions.'
      },
      {
        id: 'isaac-intro-q2',
        question: 'What is "Isaac Sim" primarily used for?',
        options: [
          'Robot hardware design',
          'Robot simulation and testing in virtual environments',
          'Robot operating system',
          'Robot manufacturing'
        ],
        correctAnswer: 1,
        difficulty: 'Easy',
        explanation: 'Isaac Sim is a robotics simulation platform that allows testing robots in realistic virtual environments before deployment.'
      }
    ]
  }
};

/**
 * VLA (Vision-Language-Action) Quizzes
 */
export const VLA_QUIZZES = {
  'vla-intro': {
    id: 'vla-intro',
    title: 'VLA Models Fundamentals',
    questions: [
      {
        id: 'vla-intro-q1',
        question: 'What does VLA stand for in robotics?',
        options: [
          'Vision-Language-Architecture',
          'Vision-Language-Action',
          'Visual-Learning-Algorithm',
          'Virtual-Logic-Array'
        ],
        correctAnswer: 1,
        difficulty: 'Easy',
        explanation: 'VLA stands for Vision-Language-Action, referring to models that process visual and language inputs to generate robotic actions.'
      },
      {
        id: 'vla-intro-q2',
        question: 'What is the key advantage of VLA models in robotics?',
        options: [
          'Lower power consumption',
          'Direct mapping from visual perception to action without separate modules',
          'Faster computation',
          'Less training data required'
        ],
        correctAnswer: 1,
        difficulty: 'Medium',
        explanation: 'VLA models enable direct end-to-end mapping from visual and language inputs to actions, simplifying the robot control pipeline.'
      }
    ]
  }
};

/**
 * Course Overview + Physical AI Quizzes (Intro, Weeks 1-2)
 */
export const INTRO_QUIZZES = {
  'course-overview': {
    id: 'course-overview',
    title: 'Course Overview',
    questions: [
      {
        id: 'course-overview-q1',
        question: 'How is this 13-week textbook organized?',
        options: [
          'Into six parts, from Physical AI basics to the Week 13 capstone',
          'Into three unrelated sections',
          'Into one chapter per robot',
          'Into 13 standalone weeks with no order'
        ],
        correctAnswer: 0,
        difficulty: 'Easy',
        explanation: 'The course overview organises the book into six parts: Introduction to Physical AI (Weeks 1–2), ROS 2 Fundamentals (Weeks 3–5), Robot Simulation (Weeks 6–7), NVIDIA Isaac Platform (Weeks 8–10), Humanoid Robot Development (Weeks 11–12), and Conversational Robotics and Capstone (Week 13).'
      },
      {
        id: 'course-overview-q2',
        question: 'Which weeks cover ROS 2 Fundamentals?',
        options: [
          'Weeks 1–2',
          'Weeks 3–5',
          'Weeks 6–7',
          'Weeks 8–10'
        ],
        correctAnswer: 1,
        difficulty: 'Easy',
        explanation: 'The course overview puts ROS 2 Fundamentals in Weeks 3–5.'
      },
      {
        id: 'course-overview-q3',
        question: 'What are the prerequisites for this course?',
        options: [
          'Basic programming, linear algebra and calculus, and the Linux command line',
          'A PhD in robotics',
          'A humanoid robot at home',
          'No prerequisites at all'
        ],
        correctAnswer: 0,
        difficulty: 'Easy',
        explanation: 'The prerequisites are basic programming knowledge (Python preferred), understanding of linear algebra and calculus, and familiarity with the Linux command line.'
      }
    ]
  }
};

export const PHYSICAL_AI_QUIZZES = {
  'physical-ai-intro': {
    id: 'physical-ai-intro',
    title: 'Introduction to Physical AI',
    questions: [
      {
        id: 'physical-ai-intro-q1',
        question: 'What is the key difference between digital AI and Physical AI?',
        options: [
          'Physical AI runs on faster hardware',
          'Physical AI operates in the real world and its actions have real physical consequences',
          'Physical AI needs no sensors',
          'Physical AI only works in simulation'
        ],
        correctAnswer: 1,
        difficulty: 'Easy',
        explanation: 'Unlike digital AI, Physical AI operates in real physical environments, processes sensor data, outputs physical actions, and carries critical safety requirements — a wrong action can damage the robot, the room, or a person.'
      },
      {
        id: 'physical-ai-intro-q2',
        question: 'What are the three steps of the sense–think–act loop?',
        options: [
          'Sense, decide, act — then sense again',
          'Think, sense, act',
          'Act, sense, think',
          'Sense, act, think'
        ],
        correctAnswer: 0,
        difficulty: 'Easy',
        explanation: 'Every physical AI system runs the same loop: sense the world, decide what to do, act — then sense again.'
      },
      {
        id: 'physical-ai-intro-q3',
        question: 'Which four problems make Physical AI hard, according to this chapter?',
        options: [
          'Uncertainty, irreversible actions, strict timing, and the reality gap',
          'Slow internet, small datasets, old hardware, and bad documentation',
          'Too many sensors, too few motors, heavy batteries, and loud fans',
          'Expensive robots, rare parts, few experts, and no simulators'
        ],
        correctAnswer: 0,
        difficulty: 'Medium',
        explanation: 'The chapter names four problems: the world is uncertain, actions cannot be undone, time is strict, and simulation is not reality (the reality gap).'
      }
    ]
  },

  'embodied-intro': {
    id: 'embodied-intro',
    title: 'Embodied Intelligence',
    questions: [
      {
        id: 'embodied-intro-q1',
        question: 'What does embodied cognition claim about cognition?',
        options: [
          'It is symbol processing that runs independently of the body',
          'It emerges from the dynamic interaction between the body, the environment, and the task',
          'It lives only in the brain’s neural networks',
          'It is a fixed set of pre-programmed rules'
        ],
        correctAnswer: 1,
        difficulty: 'Medium',
        explanation: 'Embodied cognition posits that cognitive processes emerge from the dynamic interaction between the agent’s body, the environment, and the task — cognition is grounded in sensorimotor experience.'
      },
      {
        id: 'embodied-intro-q2',
        question: 'What are sensorimotor contingencies?',
        options: [
          'Random noise in sensor readings',
          'Coupled loops in which actions change sensory inputs and sensory inputs guide the next action',
          'A type of electric motor',
          'A calibration routine run once at startup'
        ],
        correctAnswer: 1,
        difficulty: 'Medium',
        explanation: 'Sensory inputs and motor outputs are not independent but form coupled loops: actions affect sensory inputs, and sensory inputs guide actions.'
      },
      {
        id: 'embodied-intro-q3',
        question: 'How does morphological computation reduce the work the controller must do?',
        options: [
          'By adding more sensors',
          'By letting the physical form do part of the work — compliant arms adapt to shapes and walking exploits natural pendulum dynamics',
          'By running the controller at a higher frequency',
          'By removing the body from the control loop'
        ],
        correctAnswer: 2,
        difficulty: 'Medium',
        explanation: 'Morphological computation means the physical form contributes to intelligent behavior: compliant arms adapt to object shapes without complex control, and dynamic walking exploits the body’s natural pendulum-like dynamics.'
      }
    ]
  }
};

/**
 * Advanced ROS 2 Quizzes (Week 5)
 */
export const ROS2_ADVANCED_QUIZZES = {
  'ros2-advanced': {
    id: 'ros2-advanced',
    title: 'Advanced ROS 2 Concepts',
    questions: [
      {
        id: 'ros2-advanced-q1',
        question: 'What do launch files let you do in ROS 2?',
        options: [
          'Start multiple nodes with specific configurations simultaneously',
          'Compile C++ packages into binaries',
          'Replace the need for nodes entirely',
          'Flash firmware onto the robot'
        ],
        correctAnswer: 0,
        difficulty: 'Easy',
        explanation: 'Launch files manage complex robot systems by starting multiple nodes with specific configurations simultaneously.'
      },
      {
        id: 'ros2-advanced-q2',
        question: 'What is the main advantage of lifecycle nodes over regular nodes?',
        options: [
          'They run faster',
          'They provide a standardized way to manage node state through a well-defined lifecycle',
          'They need no configuration',
          'They work without ROS 2'
        ],
        correctAnswer: 1,
        difficulty: 'Medium',
        explanation: 'Lifecycle nodes provide a standardized way to manage node state through a well-defined lifecycle — for example, on_configure creates publishers, subscribers and timers, and on_cleanup destroys them.'
      },
      {
        id: 'ros2-advanced-q3',
        question: 'What are ROS 2 parameters used for?',
        options: [
          'Runtime configuration of a node, declared with descriptors and valid value ranges',
          'Storing the robot’s Wi-Fi password',
          'Replacing launch files',
          'Drawing the robot’s map'
        ],
        correctAnswer: 0,
        difficulty: 'Medium',
        explanation: 'Parameters configure a node at runtime; they are declared with descriptors and ranges — for example, robot.radius declared as 0.3 with a valid range of 0.1 to 2.0 metres.'
      }
    ]
  }
};

/**
 * NVIDIA Isaac Platform Quizzes (Weeks 8-10)
 */
export const ISAAC_PLATFORM_QUIZZES = {
  'isaac-platform': {
    id: 'isaac-platform',
    title: 'NVIDIA Isaac Platform',
    questions: [
      {
        id: 'isaac-platform-q1',
        question: 'Which Isaac component is the reinforcement learning and imitation learning framework?',
        options: [
          'Isaac Sim',
          'Isaac ROS',
          'Isaac Lab',
          'Isaac Apps'
        ],
        correctAnswer: 2,
        difficulty: 'Easy',
        explanation: 'The platform has four parts: Isaac Sim for simulation, Isaac ROS for perception and navigation, Isaac Lab as the reinforcement learning and imitation learning framework, and Isaac Apps as pre-built applications.'
      },
      {
        id: 'isaac-platform-q2',
        question: 'Which physics engine powers Isaac Sim’s GPU-accelerated simulation?',
        options: [
          'Bullet Physics',
          'ODE (Open Dynamics Engine)',
          'PhysX',
          'MuJoCo'
        ],
        correctAnswer: 2,
        difficulty: 'Easy',
        explanation: 'Isaac Sim’s key features list GPU-accelerated physics simulation using PhysX, plus RTX ray tracing for photorealistic rendering.'
      },
      {
        id: 'isaac-platform-q3',
        question: 'What is Isaac Sim’s synthetic data generation used for?',
        options: [
          'Training AI models',
          'Designing robot hardware',
          'Replacing ROS 2',
          'Calibrating the robot’s battery'
        ],
        correctAnswer: 0,
        difficulty: 'Easy',
        explanation: 'Synthetic data generation for AI training is one of Isaac Sim’s key features, with a dedicated section on setting up synthetic data capture.'
      }
    ]
  },

  'isaac-ros-intro': {
    id: 'isaac-ros-intro',
    title: 'Isaac ROS',
    questions: [
      {
        id: 'isaac-ros-intro-q1',
        question: 'What is Isaac ROS?',
        options: [
          'A collection of hardware-accelerated perception and navigation packages that use NVIDIA GPUs',
          'A robot operating system that replaces ROS 2',
          'A 3D simulator for humanoid robots',
          'A CAD tool for designing robot parts'
        ],
        correctAnswer: 0,
        difficulty: 'Easy',
        explanation: 'Isaac ROS is a collection of hardware-accelerated perception and navigation packages that leverage NVIDIA GPUs for real-time robotics applications.'
      },
      {
        id: 'isaac-ros-intro-q2',
        question: 'Which Isaac ROS package performs visual SLAM?',
        options: [
          'Isaac ROS Apriltag',
          'Isaac ROS DNN Inference',
          'Isaac ROS VSLAM',
          'Isaac ROS Manipulators'
        ],
        correctAnswer: 2,
        difficulty: 'Medium',
        explanation: 'The key packages list includes Isaac ROS VSLAM for Visual Simultaneous Localization and Mapping.'
      },
      {
        id: 'isaac-ros-intro-q3',
        question: 'What GPU hardware does Isaac ROS require?',
        options: [
          'An NVIDIA GPU with Tensor Cores — RTX series recommended',
          'Any integrated laptop GPU',
          'CPU only, no GPU',
          'An AMD GPU'
        ],
        correctAnswer: 0,
        difficulty: 'Easy',
        explanation: 'The system requirements list an NVIDIA GPU with Tensor Cores (RTX series recommended), CUDA 11.8 or newer, Ubuntu 20.04 or 22.04, and ROS 2 Humble.'
      }
    ]
  },

  'sim-to-real': {
    id: 'sim-to-real',
    title: 'Sim-to-Real Transfer',
    questions: [
      {
        id: 'sim-to-real-q1',
        question: 'What is the “reality gap”?',
        options: [
          'The differences between simulated and real environments — visual domain shift, dynamics mismatch, actuator differences, sensor noise, and unmodelled forces',
          'The time it takes a robot to boot',
          'The gap between two robots walking in formation',
          'The difference between two simulation tools'
        ],
        correctAnswer: 0,
        difficulty: 'Medium',
        explanation: 'The reality gap consists of visual domain shift, dynamics mismatch, actuator differences, sensor noise, and environmental factors such as unmodelled forces and vibrations.'
      },
      {
        id: 'sim-to-real-q2',
        question: 'How does domain randomization help bridge the sim-to-real gap?',
        options: [
          'It increases the diversity of simulation environments so models become robust to domain shifts',
          'It makes the simulation photorealistic',
          'It removes all noise from the simulation',
          'It trains the model only on real-world data'
        ],
        correctAnswer: 0,
        difficulty: 'Medium',
        explanation: 'Domain randomization increases the diversity of simulation environments to make models robust to domain shifts — for example, randomising lighting, textures, and colours.'
      },
      {
        id: 'sim-to-real-q3',
        question: 'Why is it useful to train in simulation before deploying on a real robot?',
        options: [
          'Simulation is safer, cheaper, faster than real-time, repeatable, and scalable',
          'Real robots cannot learn anything',
          'Simulation replaces the real robot entirely',
          'Policies trained in simulation never need the real robot'
        ],
        correctAnswer: 0,
        difficulty: 'Easy',
        explanation: 'The chapter lists five reasons: safety, cost, speed (simulation can run faster than real-time), repeatability, and scalability.'
      }
    ]
  }
};

/**
 * Humanoid Development Quizzes (Weeks 11-12)
 */
export const HUMANOID_QUIZZES = {
  'humanoid-kinematics': {
    id: 'humanoid-kinematics',
    title: 'Humanoid Kinematics',
    questions: [
      {
        id: 'humanoid-kinematics-q1',
        question: 'What does forward kinematics compute?',
        options: [
          'The end-effector position given the joint angles',
          'The joint angles needed to reach a target position',
          'The robot’s battery level',
          'The fastest walking speed'
        ],
        correctAnswer: 0,
        difficulty: 'Easy',
        explanation: 'Forward kinematics computes the end-effector position given joint angles.'
      },
      {
        id: 'humanoid-kinematics-q2',
        question: 'What does inverse kinematics compute?',
        options: [
          'The end-effector position given the joint angles',
          'The joint angles needed to move a limb to a target position',
          'The weight of each robot link',
          'The colour of the robot’s cameras'
        ],
        correctAnswer: 1,
        difficulty: 'Medium',
        explanation: 'The inverse kinematics solver computes the joint angles for a leg to reach a target position, using a Jacobian-based iterative method.'
      },
      {
        id: 'humanoid-kinematics-q3',
        question: 'How does Zero Moment Point (ZMP) control keep a humanoid stable?',
        options: [
          'By keeping the ZMP inside the support polygon formed by the feet contact points',
          'By locking every joint during walking',
          'By walking as fast as possible',
          'By removing the arms to lower the centre of mass'
        ],
        correctAnswer: 0,
        difficulty: 'Hard',
        explanation: 'The balance controller calculates the ZMP from the centre-of-mass state and steers it toward a desired ZMP inside the support polygon — the convex hull of the feet contact points — and measures stability by the ZMP’s margin to the polygon edge.'
      }
    ]
  },

  'humanoid-development': {
    id: 'humanoid-development',
    title: 'Humanoid Development',
    questions: [
      {
        id: 'humanoid-development-q1',
        question: 'How many degrees of freedom does the chapter’s humanoid hand have?',
        options: [
          '10',
          '12',
          '16 — 4 in the thumb and 3 in each of the 4 fingers',
          '20'
        ],
        correctAnswer: 2,
        difficulty: 'Medium',
        explanation: 'The hand has a 4-DOF thumb and 3 DOF per finger across 4 fingers, for 16 degrees of freedom in total.'
      },
      {
        id: 'humanoid-development-q2',
        question: 'Which grasp type is used for fine manipulation with the thumb and index finger?',
        options: [
          'Power grasp',
          'Precision grasp',
          'Tripod grasp',
          'Spherical grasp'
        ],
        correctAnswer: 1,
        difficulty: 'Easy',
        explanation: 'The precision grasp is fine manipulation with thumb and index finger; the power grasp uses all fingers for a strong grip, and the tripod grasp is a three-finger precision grasp.'
      },
      {
        id: 'humanoid-development-q3',
        question: 'What decides which grasp the robot selects for an object?',
        options: [
          'The object’s type, size, and weight',
          'The object’s colour',
          'The time of day',
          'The robot’s battery level'
        ],
        correctAnswer: 0,
        difficulty: 'Medium',
        explanation: 'Grasp selection is rule-based on the object’s type, size, and weight — a light, small cylinder gets a precision grasp, a thin box gets a tripod grasp, and a ball gets a spherical grasp.'
      }
    ]
  }
};

/**
 * Export all quizzes
 */
export const ALL_QUIZZES = {
  ...INTRO_QUIZZES,
  ...PHYSICAL_AI_QUIZZES,
  ...ROS2_QUIZZES,
  ...ROS2_ADVANCED_QUIZZES,
  ...DIGITAL_TWIN_QUIZZES,
  ...ISAAC_QUIZZES,
  ...ISAAC_PLATFORM_QUIZZES,
  ...VLA_QUIZZES,
  ...HUMANOID_QUIZZES
};

/**
 * Get quiz by ID
 */
export function getQuiz(id) {
  return ALL_QUIZZES[id] || null;
}

/**
 * Get all quiz IDs
 */
export function getQuizIds() {
  return Object.keys(ALL_QUIZZES);
}

/**
 * Get quizzes by module
 */
export function getQuizzesByModule(module) {
  switch (module) {
    case 'intro':
      return INTRO_QUIZZES;
    case 'physicalAi':
      return PHYSICAL_AI_QUIZZES;
    case 'ros2':
      return ROS2_QUIZZES;
    case 'ros2Advanced':
      return ROS2_ADVANCED_QUIZZES;
    case 'digitalTwin':
      return DIGITAL_TWIN_QUIZZES;
    case 'isaac':
      return ISAAC_QUIZZES;
    case 'isaacPlatform':
      return ISAAC_PLATFORM_QUIZZES;
    case 'vla':
      return VLA_QUIZZES;
    case 'humanoid':
      return HUMANOID_QUIZZES;
    default:
      return {};
  }
}
