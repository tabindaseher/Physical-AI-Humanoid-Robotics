// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Physical AI & Humanoid Robotics',
      items: [
        {
          type: 'category',
          label: 'Chapter 01: Introduction to Physical AI',
          items: [
            'chapter-01/index',
            'chapter-01/intro-physical-ai',
            'chapter-01/learning-outcomes',
            'chapter-01/key-concepts',
            'chapter-01/exercises'
          ],
        },
        {
          type: 'category',
          label: 'Chapter 02: Foundations of Humanoid Robotics',
          items: [
            'chapter-02/index',
            'chapter-02/foundations-humanoid',
            'chapter-02/learning-outcomes',
            'chapter-02/key-concepts',
            'chapter-02/exercises'
          ],
        },
        {
          type: 'category',
          label: 'Chapter 03: Digital Twins & Simulation',
          items: [
            'chapter-03/index',
            'chapter-03/digital-twin-gazebo-unity',
            'chapter-03/learning-outcomes',
            'chapter-03/key-concepts',
            'chapter-03/exercises'
          ],
        },
        {
          type: 'category',
          label: 'Chapter 04: NVIDIA Isaac AI Brain',
          items: [
            'chapter-04/index',
            'chapter-04/nvidia-isaac-ai-brain',
            'chapter-04/learning-outcomes',
            'chapter-04/key-concepts',
            'chapter-04/exercises'
          ],
        },
        {
          type: 'category',
          label: 'Chapter 05: VLA Integration',
          items: [
            'chapter-05/index',
            'chapter-05/vla-integration',
            'chapter-05/learning-outcomes',
            'chapter-05/key-concepts',
            'chapter-05/exercises'
          ],
        },
        {
          type: 'category',
          label: 'Chapter 06: Frontend Implementation',
          items: [
            'chapter-06/index',
            'chapter-06/intro-frontend',
            'chapter-06/learning-outcomes',
            'chapter-06/key-concepts',
            'chapter-06/exercises'
          ],
        },
        {
          type: 'category',
          label: 'Chapter 07: Conversational Robotics',
          items: [
            'chapter-07/index',
            'chapter-07/conversational-robotics',
            'chapter-07/learning-outcomes',
            'chapter-07/key-concepts',
            'chapter-07/exercises'
          ],
        },
        {
          type: 'category',
          label: 'Chapter 08: Capstone - Autonomous Humanoid',
          items: [
            'chapter-08/index',
            'chapter-08/capstone-autonomous-humanoid',
            'chapter-08/learning-outcomes',
            'chapter-08/key-concepts',
            'chapter-08/exercises'
          ],
        },
        {
          type: 'category',
          label: 'Appendices',
          items: [
            'appendices/index',
            'appendices/book-structure',
            'appendices/glossary',
            'appendices/appendix-a-ros2-cheatsheet',
            'appendices/appendix-b-gazebo-setup',
            'appendices/appendix-c-isaac-ros-tutorials',
            'appendices/appendix-d-vla-implementation',
            'appendices/appendix-e-development-resources'
          ],
        }
      ],
    },
    'chat',
  ],
};

module.exports = sidebars;