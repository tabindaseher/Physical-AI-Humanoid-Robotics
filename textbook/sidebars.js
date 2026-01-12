// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Foundations of Physical AI',
      items: [
        'foundations/intro',
        'foundations/embodied-intelligence',
        'foundations/sensors',
        'foundations/digital-vs-physical-ai',
        'foundations/learning-objectives',
        'foundations/assessments',
        'foundations/readability-analysis',
      ],
    },
    {
      type: 'category',
      label: 'ROS 2 - Robotic Nervous System',
      items: [
        'ros2/architecture',
        'ros2/python-agents',
        'ros2/urdf',
        'ros2/ai-agent-integration',
        'ros2/llm-cognitive-planning',
        'ros2/physical-control-concepts',
        'ros2/practical-examples',
        'ros2/advanced-rag-queries',
        'ros2/chatbot-testing',
        'ros2/academic-references',
        'ros2/assessment-materials',
        'ros2/academic-rigor',
      ],
    },
    {
      type: 'category',
      label: 'Simulation & Digital Twins',
      items: [
        'simulation/gazebo',
      ],
    },
    {
      type: 'category',
      label: 'Vision-Language-Action Systems',
      items: [
        'vla/multimodal-ai-guide',
      ],
    },
  ],
};

export default sidebars;