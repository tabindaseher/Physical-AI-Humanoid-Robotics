import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx(styles.heroSection, styles.animatedBackground)}>
      {/* Animated background elements */}
      <div className={clsx(styles.backgroundElement)}></div>
      <div className={clsx(styles.backgroundElement)}></div>
      <div className={clsx(styles.backgroundElement)}></div>

      <div className={styles.heroContent}>
        <div className={styles.bookCoverContainer}>
          <img
            src="/img/book-cover.svg"
            alt="Physical AI & Humanoid Robotics Book Cover"
            className={styles.bookCoverImage}
          />
        </div>
        <div className={styles.textContent}>
          <h1 className={clsx(styles.heroTitle)}>{siteConfig.title}</h1>
          <p className={clsx(styles.heroSubtitle)}>{siteConfig.tagline}</p>
        </div>
        <div className={styles.buttons}>
          <Link
            className={clsx(styles.gradientButton)}
            to="/docs/chapter-01/">  {/* Changed to existing chapter */}
            Start Reading Now
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
      title={`Welcome to ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics - A comprehensive guide to embodied artificial intelligence">
      <HomepageHeader />
      <main>
        <div className="container padding-top--xl padding-bottom--xl">
          <div className="row">
            <div className="col col--8 col--offset-2">
              <h2 className="text--center padding-bottom--md">About This Book</h2>
              <p className="text--center">
                This comprehensive guide explores the fascinating intersection of artificial intelligence and robotics, 
                focusing on embodied intelligence and humanoid systems. You'll learn about physical AI principles, 
                ROS 2 architecture, digital twins, AI-robot brains, and conversational robotics.
              </p>
            </div>
          </div>
        </div>
        <HomepageFeatures />
        <div className="container padding-top--xl padding-bottom--xl">
          <div className="row">
            <div className="col col--10 col--offset-1">
              <div className="card">
                <div className="card__header text--center">
                  <h2>Ready to Dive In?</h2>
                </div>
                <div className="card__body text--center">
                  <p>Begin your journey into the future of robotics and artificial intelligence</p>
                </div>
                <div className="card__footer text--center">
                  <Link
                    className="button button--primary button--lg"
                    to="/docs/chapter-01/">
                    Get Started
                  </Link>
                </div>
              </div>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}