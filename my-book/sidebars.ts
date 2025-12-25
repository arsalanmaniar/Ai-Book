import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 * - create an ordered group of docs
 * - render a sidebar for each doc of that group
 * - provide next/previous navigation
 *
 * The sidebars can be generated from the filesystem, or explicitly defined here.
 *
 * Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Main sidebar for the Physical AI & Humanoid Robotics Textbook
  simulationSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: Introduction to Physical AI & Robotics',
      collapsed: true,
      items: [
        'chapter-1-introduction/introduction',
        'chapter-1-introduction/fundamentals',
        'chapter-1-introduction/overview',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      collapsed: true,
      items: [
        {
          type: 'category',
          label: 'Chapter 4: Physics Simulation in Gazebo',
          collapsed: true,
          items: [
            'chapter-4-gazebo-physics/introduction',
            'chapter-4-gazebo-physics/physics-fundamentals',
            'chapter-4-gazebo-physics/gravity-collision-modeling',
            'chapter-4-gazebo-physics/humanoid-simulation',
            'chapter-4-gazebo-physics/exercises',
            'chapter-4-gazebo-physics/debugging-troubleshooting',
            'chapter-4-gazebo-physics/assessment',
            'chapter-4-gazebo-physics/summary',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 5: Unity Rendering and Human-Robot Interaction',
          collapsed: true,
          items: [
            'chapter-5-unity-rendering/high-fidelity-rendering',
            'chapter-5-unity-rendering/human-robot-interaction',
            'chapter-5-unity-rendering/exercises',
            'chapter-5-unity-rendering/assessment',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 6: Sensor Simulation',
          collapsed: true,
          items: [
            'chapter-6-sensor-simulation/final-assessment',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Advanced Simulation Techniques',
      collapsed: true,
      items: [
        'chapter-3-advanced-simulation/introduction',
        'chapter-3-advanced-simulation/techniques',
        'chapter-3-advanced-simulation/applications',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA) Integration',
      collapsed: true,
      items: [
        {
          type: 'category',
          label: 'Chapter 7: LLM-Robotics Integration',
          collapsed: true,
          items: [
            'chapter-7-vla-integration/introduction',
            'chapter-7-vla-integration/llm-robotics-integration',
            'chapter-7-vla-integration/voice-to-action-pipeline',
            'chapter-7-vla-integration/exercises',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 8: Advanced VLA Techniques',
          collapsed: true,
          items: [
            'chapter-8-advanced-vla/whisper-integration',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 9: Capstone - Autonomous Humanoid',
          collapsed: true,
          items: [
            'chapter-9-capstone-humanoid/autonomous-humanoid-design',
          ],
        },
      ],
    },
  ],
};

export default sidebars;