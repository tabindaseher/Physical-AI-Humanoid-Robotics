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
        'chapter-01/intro-physical-ai',
        'chapter-01/learning-outcomes',
        'chapter-01/key-concepts',
        'chapter-01/exercises'
      ]
    },
    {
      type: 'category',
      label: 'Chapter 2: Architecture - Foundations of Humanoid Robotics',
      collapsed: false,
      items: [
        'chapter-02/foundations-humanoid',
        'chapter-02/learning-outcomes',
        'chapter-02/key-concepts',
        'chapter-02/exercises'
      ]
    },
    {
      type: 'category',
      label: 'Chapter 3: Backend - Digital Twin with Gazebo & Unity',
      collapsed: false,
      items: [
        'chapter-03/digital-twin-gazebo-unity',
        'chapter-03/learning-outcomes',
        'chapter-03/key-concepts',
        'chapter-03/exercises'
      ]
    },
    {
      type: 'category',
      label: 'Chapter 4: Backend - The AI-Robot Brain (NVIDIA Isaac)',
      collapsed: false,
      items: [
        'chapter-04/nvidia-isaac-ai-brain',
        'chapter-04/learning-outcomes',
        'chapter-04/key-concepts',
        'chapter-04/exercises'
      ]
    },
    {
      type: 'category',
      label: 'Chapter 5: API Integration - Vision-Language-Action (VLA)',
      collapsed: false,
      items: [
        'chapter-05/vla-integration',
        'chapter-05/learning-outcomes',
        'chapter-05/key-concepts',
        'chapter-05/exercises'
      ]
    },
    {
      type: 'category',
      label: 'Chapter 6: Frontend - Humanoid Robot Development',
      collapsed: false,
      items: [
        'chapter-06/intro-frontend',
        'chapter-06/learning-outcomes',
        'chapter-06/key-concepts',
        'chapter-06/exercises'
      ]
    },
    {
      type: 'category',
      label: 'Chapter 7: API Integration - Conversational Robotics',
      collapsed: false,
      items: [
        'chapter-07/conversational-robotics',
        'chapter-07/learning-outcomes',
        'chapter-07/key-concepts',
        'chapter-07/exercises'
      ]
    },
    {
      type: 'category',
      label: 'Chapter 8: Setup & Deployment - Capstone Project - The Autonomous Humanoid',
      collapsed: false,
      items: [
        'chapter-08/capstone-autonomous-humanoid',
        'chapter-08/learning-outcomes',
        'chapter-08/key-concepts',
        'chapter-08/exercises'
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
