/**
 * Interactive Diagram Data
 * Feature: 002-ui-improvements
 * Task: T064
 *
 * Defines interactive diagram node structures for ROS 2 and other course concepts
 */

/**
 * ROS 2 Architecture Diagram
 * Shows the core components of ROS 2 and their relationships
 */
export const ROS2_ARCHITECTURE = {
  id: 'ros2-architecture',
  title: 'ROS 2 Architecture',
  description: 'Core components and communication patterns in ROS 2',
  nodes: [
    {
      id: 'node-publisher',
      label: 'Publisher Node',
      type: 'component',
      x: 100,
      y: 100,
      description: 'A ROS 2 node that publishes messages to a topic',
      details: ['Uses rclcpp::Publisher<T>', 'Sends data via publish()', 'Can publish to multiple topics']
    },
    {
      id: 'node-subscriber',
      label: 'Subscriber Node',
      type: 'component',
      x: 500,
      y: 100,
      description: 'A ROS 2 node that subscribes to and receives messages',
      details: ['Uses rclcpp::Subscription<T>', 'Reives data via callback', 'Can subscribe to multiple topics']
    },
    {
      id: 'topic',
      label: 'Topic',
      type: 'channel',
      x: 300,
      y: 100,
      description: 'Named channel for asynchronous message passing',
      details: ['Pub/Sub communication', 'Type-safe messaging', 'Decouples components']
    },
    {
      id: 'service-server',
      label: 'Service Server',
      type: 'component',
      x: 100,
      y: 250,
      description: 'Provides a synchronous request-response service',
      details: ['Receives requests', 'Sends responses', 'Blocks until response ready']
    },
    {
      id: 'service-client',
      label: 'Service Client',
      type: 'component',
      x: 500,
      y: 250,
      description: 'Makes requests to a service server',
      details: ['Sends requests', 'Waits for response', 'Can be async or sync']
    },
    {
      id: 'dds',
      label: 'DDS (Data Distribution Service)',
      type: 'middleware',
      x: 300,
      y: 350,
      description: 'Underlying middleware for ROS 2 communication',
      details: ['Transport layer', 'Discovery mechanisms', 'QoS policies']
    }
  ],
  connections: [
    { from: 'node-publisher', to: 'topic', label: 'publish()' },
    { from: 'topic', to: 'node-subscriber', label: 'callback()' },
    { from: 'service-client', to: 'service-server', label: 'request', style: 'bidirectional' },
    { from: 'topic', to: 'dds', label: 'transport', style: 'dashed' }
  ]
};

/**
 * ROS 2 Nodes and Topics Diagram
 * Shows how nodes interact through topics in a typical system
 */
export const ROS2_NODES_TOPICS = {
  id: 'ros2-nodes-topics',
  title: 'ROS 2 Nodes & Topics',
  description: 'How nodes communicate via topics in a robotics system',
  nodes: [
    {
      id: 'camera-node',
      label: 'Camera Node',
      type: 'sensor',
      x: 80,
      y: 120,
      description: 'Publishes camera image data',
      details: ['sensor_msgs/Image', '30 FPS typical', 'RGB or grayscale']
    },
    {
      id: 'image-topic',
      label: '/camera/image_raw',
      type: 'topic',
      x: 280,
      y: 120,
      description: 'Topic carrying raw image data',
      details: ['Type: sensor_msgs/Image', 'QoS: Sensor data', 'High bandwidth']
    },
    {
      id: 'processor-node',
      label: 'Image Processor Node',
      type: 'processing',
      x: 450,
      y: 60,
      description: 'Processes images for object detection',
      details: ['OpenCV integration', 'Detects objects', 'Publishes results']
    },
    {
      id: 'display-node',
      label: 'Display Node',
      type: 'output',
      x: 450,
      y: 180,
      description: 'Displays images to user',
      details: ['Visualization', 'Can record video', 'GUI or web display']
    },
    {
      id: 'detection-topic',
      label: '/detector/objects',
      type: 'topic',
      x: 600,
      y: 60,
      description: 'Detected object information',
      details: ['Type: custom msg', 'Bounding boxes', 'Confidence scores']
    }
  ],
  connections: [
    { from: 'camera-node', to: 'image-topic', label: 'Image' },
    { from: 'image-topic', to: 'processor-node', label: 'Subscribe' },
    { from: 'image-topic', to: 'display-node', label: 'Subscribe' },
    { from: 'processor-node', to: 'detection-topic', label: 'Objects' }
  ]
};

/**
 * ROS 2 Services and Actions Diagram
 * Shows synchronous (service) and asynchronous (action) communication
 */
export const ROS2_SERVICES_ACTIONS = {
  id: 'ros2-services-actions',
  title: 'ROS 2 Services & Actions',
  description: 'Request-response patterns in ROS 2',
  nodes: [
    {
      id: 'client',
      label: 'Client Node',
      type: 'component',
      x: 80,
      y: 80,
      description: 'Initiates service requests',
      details: ['Synchronous call', 'Blocks for response', 'Simple use cases']
    },
    {
      id: 'service-server',
      label: 'Service Server',
      type: 'server',
      x: 300,
      y: 80,
      description: 'Processes service requests',
      details: ['Receives request', 'Processes data', 'Returns response']
    },
    {
      id: 'action-client',
      label: 'Action Client',
      type: 'component',
      x: 80,
      y: 220,
      description: 'Initiates long-running tasks',
      details: ['Asynchronous', 'Can cancel', 'Progress feedback']
    },
    {
      id: 'action-server',
      label: 'Action Server',
      type: 'server',
      x: 300,
      y: 220,
      description: 'Handles long-running tasks',
      details: ['Goal acceptance', 'Progress updates', 'Result reporting']
    },
    {
      id: 'feedback-topic',
      label: '/action/feedback',
      type: 'topic',
      x: 450,
      y: 180,
      description: 'Progress feedback channel',
      details: ['Periodic updates', 'Not required', 'Status information']
    },
    {
      id: 'result-topic',
      label: '/action/result',
      type: 'topic',
      x: 450,
      y: 260,
      description: 'Final result channel',
      details: ['Sent once', 'Final outcome', 'Success/failure']
    }
  ],
  connections: [
    { from: 'client', to: 'service-server', label: 'Request/Response', style: 'bidirectional' },
    { from: 'action-client', to: 'action-server', label: 'Goal', style: 'bidirectional' },
    { from: 'action-server', to: 'feedback-topic', label: 'Publish' },
    { from: 'feedback-topic', to: 'action-client', label: 'Subscribe' },
    { from: 'action-server', to: 'result-topic', label: 'Publish' },
    { from: 'result-topic', to: 'action-client', label: 'Subscribe' }
  ]
};

/**
 * Digital Twin Architecture Diagram
 * Shows the relationship between physical robot and digital twin
 */
export const DIGITAL_TWIN_ARCHITECTURE = {
  id: 'digital-twin-architecture',
  title: 'Digital Twin Architecture',
  description: 'Physical robot synchronized with virtual simulation',
  nodes: [
    {
      id: 'physical-robot',
      label: 'Physical Robot',
      type: 'physical',
      x: 100,
      y: 150,
      description: 'Real-world robot hardware',
      details: ['Sensors', 'Actuators', 'Compute unit']
    },
    {
      id: 'ros2-bridge',
      label: 'ROS 2 Bridge',
      type: 'middleware',
      x: 300,
      y: 150,
      description: 'Communication layer',
      details: ['JSON over WebSocket', 'ROS topics', 'Bidirectional sync']
    },
    {
      id: 'digital-twin',
      label: 'Digital Twin',
      type: 'virtual',
      x: 500,
      y: 150,
      description: 'Virtual robot simulation',
      details: ['Gazebo/Isaac Sim', 'Physics engine', 'Real-time sync']
    },
    {
      id: 'state-sync',
      label: 'State Synchronization',
      type: 'dataflow',
      x: 300,
      y: 280,
      description: 'Bi-directional state updates',
      details: ['Joint positions', 'Sensor data', 'Commands']
    }
  ],
  connections: [
    { from: 'physical-robot', to: 'ros2-bridge', label: 'Telemetry' },
    { from: 'ros2-bridge', to: 'digital-twin', label: 'State updates' },
    { from: 'ros2-bridge', to: 'state-sync', label: 'Sync', style: 'bidirectional' }
  ]
};

/**
 * Navigation Stack Diagram
 * Shows ROS 2 Navigation2 architecture
 */
export const NAV2_STACK = {
  id: 'nav2-stack',
  title: 'Navigation2 Stack',
  description: 'ROS 2 Navigation2 architecture for autonomous mobile robots',
  nodes: [
    {
      id: 'map-server',
      label: 'Map Server',
      type: 'server',
      x: 100,
      y: 100,
      description: 'Provides static map data',
      details: ['Occupancy grid', 'Map server', 'Static map source']
    },
    {
      id: 'amcl',
      label: 'AMCL',
      type: 'localization',
      x: 100,
      y: 220,
      description: 'Adaptive Monte Carlo Localization',
      details: ['Particle filter', 'Pose estimation', 'Localization']
    },
    {
      id: 'planner',
      label: 'Planner Server',
      type: 'planner',
      x: 300,
      y: 160,
      description: 'Global and local path planning',
      details: ['A* / NavFn', 'DWB / TEB', 'Path computation']
    },
    {
      id: 'controller',
      label: 'Controller Server',
      type: 'controller',
      x: 500,
      y: 160,
      description: 'Executes motion commands',
      details: ['Pure pursuit', 'PID control', 'Velocity commands']
    },
    {
      id: 'recovery',
      label: 'Recovery Behaviors',
      type: 'behavior',
      x: 400,
      y: 280,
      description: 'Handles stuck situations',
      details: ['Back up', 'Spin', 'Wait']
    }
  ],
  connections: [
    { from: 'map-server', to: 'planner', label: 'Map' },
    { from: 'amcl', to: 'planner', label: 'Pose' },
    { from: 'planner', to: 'controller', label: 'Path' },
    { from: 'controller', to: 'recovery', label: 'Trigger', style: 'dashed' }
  ]
};

/**
 * Export all diagrams
 */
export const DIAGRAMS = {
  'ros2-architecture': ROS2_ARCHITECTURE,
  'ros2-nodes-topics': ROS2_NODES_TOPICS,
  'ros2-services-actions': ROS2_SERVICES_ACTIONS,
  'digital-twin-architecture': DIGITAL_TWIN_ARCHITECTURE,
  'nav2-stack': NAV2_STACK
};

/**
 * Get diagram by ID
 */
export function getDiagram(id) {
  return DIAGRAMS[id] || null;
}

/**
 * Get all diagram IDs
 */
export function getDiagramIds() {
  return Object.keys(DIAGRAMS);
}
