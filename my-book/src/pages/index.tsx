import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import ModuleCards from '@site/src/components/ModuleCards';
import BlogHighlights from '@site/src/components/BlogHighlights';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
            <Heading as="h1" className="hero__title">
              {siteConfig.title}
            </Heading>
            <p className="hero__subtitle">{siteConfig.tagline}</p>
            <div className={styles.buttons}>
              <Link
                className="button button--primary button--lg"
                to="/docs/intro">
                Start Learning →
              </Link>
              <Link
                className="button button--secondary button--lg"
                to="/docs/chapter-7-vla-integration/introduction">
                Explore VLA Systems
              </Link>
            </div>
          </div>
          <div className={styles.heroVisual}>
            <div className={styles.roboticIcon}></div>
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Advanced Simulation Techniques for AI and Robotics">
      <HomepageHeader />
      <main>
        <section className={styles.moduleSection}>
          <div className="container">
            <div className={styles.sectionHeader}>
              <Heading as="h2" className={styles.sectionTitle}>
                Explore the Modules
              </Heading>
              <p className={styles.sectionSubtitle}>
                Navigate through the comprehensive curriculum covering Physical AI and Humanoid Robotics
              </p>
            </div>
            <ModuleCards />
          </div>
        </section>
        <BlogHighlights />
      </main>
    </Layout>
  );
}