import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Weeks 1-2: Introduction to Physical AI',
      items: [
        'week1/intro-physical-ai',
        'week2/embodied-intelligence'
      ],
    },
    {
      type: 'category',
      label: 'Weeks 3-5: ROS 2 Fundamentals',
      items: [
        'week3/ros2-nodes-topics',
        'week4/ros2-services-actions',
        'week5/ros2-fundamentals-advanced'
      ],
    },
    {
      type: 'category',
      label: 'Weeks 6-7: Robot Simulation with Gazebo',
      items: [
        'week6/gazebo-simulation',
        'week7/nvidia-isaac-sim'
      ],
    },
    {
      type: 'category',
      label: 'Weeks 8-10: NVIDIA Isaac Platform',
      items: [
        'week8/nvidia-isaac-sim',
        'week9/isaac-ros',
        'week10/sim-to-real-transfer'
      ],
    },
    {
      type: 'category',
      label: 'Weeks 11-12: Humanoid Robot Development',
      items: [
        'week11/humanoid-kinematics',
        'week12/humanoid-development'
      ],
    },
    {
      type: 'category',
      label: 'Week 13: Conversational Robotics',
      items: [
        'week13/conversational-robotics'
      ],
    },
  ],
};

export default sidebars;