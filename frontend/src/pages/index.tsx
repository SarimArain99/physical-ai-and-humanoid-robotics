import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';
import HeroAnimation from '@site/src/components/HeroAnimation';
import WeekCards from '@site/src/components/WeekCards';
// T092: Lazy load ThreeDRobot for better performance
import { lazy, Suspense } from 'react';

const ThreeDRobot = lazy(() => import('@site/src/components/ThreeDRobot'));

import styles from './index.module.css';

/**
 * Homepage Component
 * Feature: 002-ui-improvements
 * Task: T060 - Integrate ThreeDRobot into homepage hero section
 */
function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <HeroAnimation staggerDelay={150}>
        <div className="container">
          <div className={styles.heroContent}>
            <div className={styles.heroText}>
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
            <div className={styles.heroVisual}>
              <Suspense fallback={<div className={styles.threeRobotLoading}>Loading 3D Robot...</div>}>
                <ThreeDRobot
                  autoRotate={true}
                  rotateSpeed={0.005}
                  enableDrag={true}
                  width={280}
                  height={350}
                />
              </Suspense>
            </div>
          </div>
        </div>
      </HeroAnimation>
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

            <WeekCards />
          </div>
        </section>

        <HomepageFeatures />
      </main>
    </Layout>
  );
}
