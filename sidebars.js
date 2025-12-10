// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'index',
    {
      type: 'category',
      label: 'Chapter 1: Introduction to Physical AI',
      collapsed: false,
      items: [
        'chapter-01/index',
        'chapter-01/01-intro-physical-ai',
        'chapter-01/02-learning-outcomes',
        'chapter-01/03-key-concepts',
        'chapter-01/04-exercises'
      ]
    },
    {
      type: 'category',
      label: 'Chapter 2: Architecture - Foundations of Humanoid Robotics',
      collapsed: false,
      items: [
        'chapter-02/01-foundations-humanoid',
        'chapter-02/02-learning-outcomes',
        'chapter-02/03-key-concepts',
        'chapter-02/04-exercises'
      ]
    },
    {
      type: 'category',
      label: 'Chapter 3: Architecture - ROS2 Nervous System',
      collapsed: false,
      items: [
        'chapter-03/01-ros2-nervous-system',
        'chapter-03/02-learning-outcomes',
        'chapter-03/03-key-concepts',
        'chapter-03/04-exercises'
      ]
    },
    {
      type: 'category',
      label: 'Chapter 4: Backend - Digital Twin with Gazebo & Unity',
      collapsed: false,
      items: [
        'chapter-04/01-digital-twin-gazebo-unity',
        'chapter-04/02-learning-outcomes',
        'chapter-04/03-key-concepts',
        'chapter-04/04-exercises'
      ]
    },
    {
      type: 'category',
      label: 'Chapter 5: Backend - NVIDIA Isaac AI Brain',
      collapsed: false,
      items: [
        'chapter-05/01-nvidia-isaac-ai-brain',
        'chapter-05/02-learning-outcomes',
        'chapter-05/03-key-concepts',
        'chapter-05/04-exercises'
      ]
    },
    {
      type: 'category',
      label: 'Chapter 6: Frontend - User Interfaces',
      collapsed: false,
      items: [
        'chapter-06/01-intro-frontend',
        'chapter-06/02-learning-outcomes',
        'chapter-06/03-key-concepts',
        'chapter-06/04-exercises'
      ]
    },
    {
      type: 'category',
      label: 'Chapter 7: API Integration - VLA Integration',
      collapsed: false,
      items: [
        'chapter-07/01-vla-integration',
        'chapter-07/02-learning-outcomes',
        'chapter-07/03-key-concepts',
        'chapter-07/04-exercises'
      ]
    },
    {
      type: 'category',
      label: 'Chapter 8: Setup & Deployment',
      collapsed: false,
      items: [
        'chapter-08/01-capstone-autonomous-humanoid',
        'chapter-08/02-learning-outcomes',
        'chapter-08/03-key-concepts',
        'chapter-08/04-exercises'
      ]
    },
    {
      type: 'category',
      label: 'Appendices',
      collapsed: true,
      items: [
        'appendices/appendix-a-ros2-cheatsheet',
        'appendices/appendix-b-gazebo-setup',
        'appendices/appendix-c-isaac-ros-tutorials',
        'appendices/appendix-d-vla-implementation',
        'appendices/appendix-e-development-resources',
        'appendices/glossary',
        'appendices/book-structure'
      ]
    }
  ]
};

module.exports = sidebars;