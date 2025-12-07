import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Get Started with Physical AI
          </Link>
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
      description="AI-Native Textbook on Physical AI & Humanoid Robotics">
      <HomepageHeader />
      <main>
        <section className={styles.modulesSection}>
          <div className={styles.modulesContainer}>
            <Heading as="h2" className={clsx('margin-bottom--lg')}>
              Course Modules
            </Heading>

            <div className={styles.weeksGrid}>
              <div className={styles.weekCard}>
                <div className={styles.weekNumber}>Weeks 1-2</div>
                <div className={styles.weekTitle}>Introduction to Physical AI</div>
                <div className={styles.weekDescription}>Foundations of embodied intelligence and physical AI principles</div>
              </div>

              <div className={styles.weekCard}>
                <div className={styles.weekNumber}>Weeks 3-5</div>
                <div className={styles.weekTitle}>ROS 2 Fundamentals</div>
                <div className={styles.weekDescription}>Robot Operating System 2 for robot control and communication</div>
              </div>

              <div className={styles.weekCard}>
                <div className={styles.weekNumber}>Weeks 6-7</div>
                <div className={styles.weekTitle}>Robot Simulation</div>
                <div className={styles.weekDescription}>Gazebo and Unity for digital twin development</div>
              </div>

              <div className={styles.weekCard}>
                <div className={styles.weekNumber}>Weeks 8-10</div>
                <div className={styles.weekTitle}>NVIDIA Isaac Platform</div>
                <div className={styles.weekDescription}>AI-powered perception and navigation with Isaac Sim/ROS</div>
              </div>

              <div className={styles.weekCard}>
                <div className={styles.weekNumber}>Weeks 11-12</div>
                <div className={styles.weekTitle}>Humanoid Robot Development</div>
                <div className={styles.weekDescription}>Kinematics, dynamics, and control for bipedal robots</div>
              </div>

              <div className={styles.weekCard}>
                <div className={styles.weekNumber}>Week 13</div>
                <div className={styles.weekTitle}>Conversational Robotics</div>
                <div className={styles.weekDescription}>Vision-Language-Action systems for natural interaction</div>
              </div>
            </div>
          </div>
        </section>

        <HomepageFeatures />
      </main>
    </Layout>
  );
}
