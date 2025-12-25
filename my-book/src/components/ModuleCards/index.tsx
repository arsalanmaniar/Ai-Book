import React from 'react';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './styles.module.css';

interface ModuleCardProps {
  title: string;
  description: string;
  icon: string;
  to: string;
  color: string;
}

const ModuleCard: React.FC<ModuleCardProps> = ({ title, description, icon, to, color }) => {
  return (
    <Link to={to} className={styles.moduleCard} style={{ borderColor: color }}>
      <div className={styles.cardIcon} style={{ backgroundColor: color }}>
        {icon}
      </div>
      <div className={styles.cardContent}>
        <h3 className={styles.cardTitle} style={{ color: color }}>
          {title}
        </h3>
        <p className={styles.cardDescription}>
          {description}
        </p>
      </div>
    </Link>
  );
};

const ModuleCards: React.FC = () => {
  return (
    <div className={styles.moduleCardsContainer}>
      <ModuleCard
        title="Module 1: Introduction to Physical AI & Robotics"
        description="Learn the fundamentals of Physical AI and how it bridges artificial intelligence with real-world applications."
        icon="🤖"
        to="/docs/chapter-1-introduction/introduction"
        color="#00aaff"
      />
      <ModuleCard
        title="Module 2: The Digital Twin (Gazebo & Unity)"
        description="Explore physics simulation in Gazebo and Unity rendering for creating digital twins of robotic systems."
        icon="🎮"
        to="/docs/chapter-4-gazebo-physics/introduction"
        color="#00e6e6"
      />
      <ModuleCard
        title="Module 3: Advanced Simulation Techniques"
        description="Master advanced simulation techniques for improved sim-to-real transfer and domain randomization."
        icon="🔬"
        to="/docs/chapter-3-advanced-simulation/introduction"
        color="#00aaff"
      />
      <ModuleCard
        title="Module 4: Vision-Language-Action (VLA) Integration"
        description="Implement LLM-robotics integration with voice-to-action pipelines and cognitive planning."
        icon="🧠"
        to="/docs/chapter-7-vla-integration/introduction"
        color="#00e6e6"
      />
    </div>
  );
};

export default ModuleCards;