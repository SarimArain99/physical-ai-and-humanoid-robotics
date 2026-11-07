import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';
import HeroAnimation from '@site/src/components/HeroAnimation';
import HeroRobot from '@site/src/components/HeroRobot';
import WeekCards from '@site/src/components/WeekCards';

import styles from './index.module.css';

/**
 * Homepage Component
 * The hero uses a static robot illustration (HeroRobot) — the old
 * three.js loader never visibly rendered and showed a fake loading state.
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
                  Start Reading
                </Link>
              </div>
            </div>
            <div className={styles.heroVisual}>
              <HeroRobot width={280} height={350} />
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
