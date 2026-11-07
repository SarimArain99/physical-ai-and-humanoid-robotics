import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';
import HeroRobot from '@site/src/components/HeroRobot';
import WeekCards from '@site/src/components/WeekCards';

import styles from './index.module.css';

/**
 * Homepage Component
 * The hero uses a static robot illustration (HeroRobot) — the old
 * three.js loader never visibly rendered and showed a fake loading state.
 * NOTE: no HeroAnimation wrapper here on purpose. HeroAnimation set
 * inline `opacity: 0` on its children and relied on a stylesheet class
 * to bring it back to 1 — inline styles beat stylesheets, so the hero
 * heading, tagline and button stayed invisible in the rendered page
 * even though they were in the DOM. The hero renders with no animation.
 */
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
                className="button button--secondary button--lg"
                to="/docs/intro">
                Get Started with Physical AI
              </Link>
            </div>
          </div>
          <div className={styles.heroVisual}>
            <HeroRobot width={280} height={350} />
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
