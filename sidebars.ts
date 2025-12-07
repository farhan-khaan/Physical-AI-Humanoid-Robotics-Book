import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Main sidebar with custom organization
  tutorialSidebar: [
    'intro',
    'learning outcomes',
    'Why Physical AI Matters',
    {
      type: 'category',
      label: '📚 Physical AI & Humanoid Robotics',
      collapsed: false,
      items: [
        'physical-ai/index',
        {
          type: 'category',
          label: 'Chapter 1: Embodied Intelligence',
          items: [
            'physical-ai/embodied-intelligence/intro',
            'physical-ai/embodied-intelligence/what-is-embodied-intelligence',
            'physical-ai/embodied-intelligence/sense-think-act-loop',
            'physical-ai/embodied-intelligence/applications',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Sensors & Actuators',
          items: [
            'physical-ai/sensors-actuators/intro',
            'physical-ai/sensors-actuators/sensor-types',
            'physical-ai/sensors-actuators/actuator-types',
            'physical-ai/sensors-actuators/sensor-integration',
            'physical-ai/sensors-actuators/exercises',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Simulation',
          items: [
            'physical-ai/simulation/intro',
            'physical-ai/simulation/digital-twins',
            'physical-ai/simulation/simulation-platforms',
            'physical-ai/simulation/setting-up-simulation',
            'physical-ai/simulation/sim-to-real',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 4: Control Strategies',
          items: [
            'physical-ai/control-strategies/intro',
            'physical-ai/control-strategies/reactive-control',
            'physical-ai/control-strategies/deliberative-control',
            'physical-ai/control-strategies/hybrid-architectures',
            'physical-ai/control-strategies/learned-control',
            'physical-ai/control-strategies/real-world-challenges',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 5: Capstone Project',
          items: [
            'physical-ai/capstone/intro',
            'physical-ai/capstone/project-ideas',
            'physical-ai/capstone/requirements',
            'physical-ai/capstone/evaluation-rubric',
            'physical-ai/capstone/examples',
          ],
        },
      ],
    },
  ],
};

export default sidebars;
