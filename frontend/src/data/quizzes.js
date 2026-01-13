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
          'RELIBLE',
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
 * Export all quizzes
 */
export const ALL_QUIZZES = {
  ...ROS2_QUIZZES,
  ...DIGITAL_TWIN_QUIZZES,
  ...ISAAC_QUIZZES,
  ...VLA_QUIZZES
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
    case 'ros2':
      return ROS2_QUIZZES;
    case 'digitalTwin':
      return DIGITAL_TWIN_QUIZZES;
    case 'isaac':
      return ISAAC_QUIZZES;
    case 'vla':
      return VLA_QUIZZES;
    default:
      return {};
  }
}
