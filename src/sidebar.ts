// src/sidebar.ts
export default {
  mySidebar: [
    {
      type: 'doc',
      id: 'index', // This will be the home page
      label: 'Home'
    },
    {
      type: 'category',
      label: 'Chapter 1: Introduction to Physical AI',
      items: [
        'ch1-intro-physical-ai',
        'ch1-learning-outcomes',
        'ch1-key-concepts',
        'ch1-exercises'
      ],
      collapsed: false
    },
    {
      type: 'category',
      label: 'Chapter 2: Foundations of Humanoid Robotics',
      items: [
        'ch2-foundations-humanoid',
        'ch2-learning-outcomes',
        'ch2-key-concepts',
        'ch2-exercises'
      ],
      collapsed: false
    },
    {
      type: 'category',
      label: 'Chapter 3: ROS 2 - The Robotic Nervous System',
      items: [
        'ch3-ros2-nervous-system',
        'ch3-learning-outcomes',
        'ch3-key-concepts',
        'ch3-exercises'
      ],
      collapsed: false
    },
    {
      type: 'category',
      label: 'Chapter 4: Digital Twin - Gazebo & Unity',
      items: [
        'ch4-digital-twin-gazebo-unity',
        'ch4-learning-outcomes',
        'ch4-key-concepts',
        'ch4-exercises'
      ],
      collapsed: false
    },
    {
      type: 'category',
      label: 'Chapter 5: NVIDIA Isaac - AI-Robot Brain',
      items: [
        'ch5-nvidia-isaac-ai-brain',
        'ch5-learning-outcomes',
        'ch5-key-concepts',
        'ch5-exercises'
      ],
      collapsed: false
    },
    {
      type: 'category',
      label: 'Chapter 6: Vision-Language-Action (VLA)',
      items: [
        'ch6-vla-integration',
        'ch6-learning-outcomes',
        'ch6-key-concepts',
        'ch6-exercises'
      ],
      collapsed: false
    },
    {
      type: 'category',
      label: 'Chapter 7: Capstone - Autonomous Humanoid',
      items: [
        'ch7-capstone-autonomous-humanoid',
        'ch7-learning-outcomes',
        'ch7-key-concepts',
        'ch7-exercises'
      ],
      collapsed: false
    },
    {
      type: 'category',
      label: 'Appendices',
      items: [
        'appendix-a-ros2-cheatsheet',
        'appendix-b-gazebo-setup',
        'appendix-c-isaac-ros-tutorials',
        'appendix-d-vla-implementation',
        'glossary'
      ],
      collapsed: true
    }
  ],
};