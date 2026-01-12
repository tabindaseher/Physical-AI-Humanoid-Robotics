import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">
          <span className={styles.bookIcon}>📚</span> {siteConfig.title}
        </h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Reading
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Home`}
      description="Physical AI & Humanoid Robotics - An AI-native textbook on embodied intelligence">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <div className="col col--4">
                <h3>Foundations of Physical AI</h3>
                <p>Explore the theoretical foundations of embodied intelligence and physical AI systems.</p>
              </div>
              <div className="col col--4">
                <h3>ROS 2 Integration</h3>
                <p>Learn about Robot Operating System integration and practical robotics applications.</p>
              </div>
              <div className="col col--4">
                <h3>Simulation & Control</h3>
                <p>Master simulation environments and control systems for humanoid robots.</p>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}