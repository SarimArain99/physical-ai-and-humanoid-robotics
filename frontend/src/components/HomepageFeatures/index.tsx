import type { ReactNode } from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  index: string;
  title: string;
  description: ReactNode;
};

/**
 * Homepage features — typographic, no imagery. The front page carries
 * no robot illustrations (owner's decision); each feature is numbered
 * and set in type, like a published textbook's contents page.
 */
const FeatureList: FeatureItem[] = [
  {
    index: '01',
    title: 'Embodied Intelligence',
    description: (
      <>
        Learn how <strong>Vision-Language-Action (VLA)</strong> models help robots
        perceive, reason, and act in the real world.
      </>
    ),
  },
  {
    index: '02',
    title: 'Sim-to-Real Transfer',
    description: (
      <>
        Use <strong>digital twins</strong> to train and test robot policies in NVIDIA Isaac Sim
        and Gazebo before deploying them to real robots.
      </>
    ),
  },
  {
    index: '03',
    title: 'Humanoid Control Stack',
    description: (
      <>
        Work through the full software stack: <strong>ROS 2</strong> middleware and sensor fusion,
        whole-body control, and bipedal locomotion.
      </>
    ),
  },
];

function Feature({ index, title, description }: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className={styles.featureCard}>
        <span className={styles.featureIndex} aria-hidden="true">{index}</span>
        <Heading as="h3" className={styles.featureTitle}>{title}</Heading>
        <p className={styles.featureDescription}>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
