import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';
import WeekCards from '@site/src/components/WeekCards';

import styles from './index.module.css';

/**
 * Homepage Component
 * Professional typographic hero — no robot imagery anywhere on the
 * front page, by the owner's decision. No HeroAnimation wrapper here
 * on purpose: HeroAnimation set inline `opacity: 0` on its children
 * and relied on a stylesheet class to bring it back to 1 — inline
 * styles beat stylesheets, so the hero heading, tagline and button
 * stayed invisible in the rendered page even though they were in the
 * DOM. The hero renders with no animation.
 */
function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className={styles.heroInner}>
        <p className={styles.heroEyebrow}>A free 13-week textbook</p>
        <Heading as="h1" className={styles.heroTitle}>
          {siteConfig.title}
        </Heading>
        <p className={styles.heroSubtitle}>{siteConfig.tagline}</p>
        <div className={styles.heroButtons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Get Started with Physical AI
          </Link>
          <Link
            className={clsx('button button--lg', styles.heroSecondaryButton)}
            to="#course-modules">
            Browse the Chapters
          </Link>
        </div>
        <dl className={styles.heroMeta}>
          <div className={styles.heroMetaItem}>
            <dt>Duration</dt>
            <dd>13 weeks</dd>
          </div>
          <div className={styles.heroMetaItem}>
            <dt>Chapters</dt>
            <dd>13 chapters</dd>
          </div>
          <div className={styles.heroMetaItem}>
            <dt>Practice</dt>
            <dd>Quizzes included</dd>
          </div>
          <div className={styles.heroMetaItem}>
            <dt>Price</dt>
            <dd>Free</dd>
          </div>
        </dl>
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
            <Heading as="h2" id="course-modules" className={clsx('margin-bottom--lg')}>
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
