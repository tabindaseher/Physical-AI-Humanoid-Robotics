// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      collapsed: false,
      items: [
        'intro/index',
        'intro/ch1-intro-physical-ai',
        'intro/ch1-learning-outcomes',
        'intro/ch1-key-concepts',
        'intro/ch1-exercises'
      ]
    },
    {
      type: 'category',
      label: 'Frontend',
      collapsed: true,
      items: [
        'frontend/intro-frontend',
      ]
    },
    {
      type: 'category',
      label: 'Architecture',
      collapsed: false,
      items: [
        'architecture/ch2-foundations-humanoid',
        'architecture/ch2-learning-outcomes',
        'architecture/ch2-key-concepts',
        'architecture/ch2-exercises',
        'architecture/ch3-ros2-nervous-system',
        'architecture/ch3-learning-outcomes',
        'architecture/ch3-key-concepts',
        'architecture/ch3-exercises'
      ]
    },
    {
      type: 'category',
      label: 'Backend',
      collapsed: false,
      items: [
        'backend/ch4-digital-twin-gazebo-unity',
        'backend/ch4-learning-outcomes',
        'backend/ch4-key-concepts',
        'backend/ch4-exercises',
        'backend/ch5-nvidia-isaac-ai-brain',
        'backend/ch5-learning-outcomes',
        'backend/ch5-key-concepts',
        'backend/ch5-exercises'
      ]
    },
    {
      type: 'category',
      label: 'API Integration',
      collapsed: false,
      items: [
        'api/ch6-vla-integration',
        'api/ch6-learning-outcomes',
        'api/ch6-key-concepts',
        'api/ch6-exercises'
      ]
    },
    {
      type: 'category',
      label: 'Setup & Deployment',
      collapsed: false,
      items: [
        'setup/ch7-capstone-autonomous-humanoid',
        'setup/ch7-learning-outcomes',
        'setup/ch7-key-concepts',
        'setup/ch7-exercises',
        'setup/appendix-a-ros2-cheatsheet',
        'setup/appendix-b-gazebo-setup',
        'setup/appendix-c-isaac-ros-tutorials',
        'setup/appendix-d-vla-implementation',
        'setup/appendix-e-development-resources',
        'setup/glossary',
        'setup/book-structure'
      ]
    }
  ]
};

module.exports = sidebars;