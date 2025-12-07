import type { ReactNode } from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Embodied Intelligence',
    Svg: require('@site/static/img/robot-svgrepo-com.svg').default,
    description: (
      <>
        Move beyond chatbots. Learn how to deploy <strong>Vision-Language-Action (VLA)</strong> models
        into physical hardware to perceive, reason, and act in the real world.
      </>
    ),
  },
  {
    title: 'Sim-to-Real Transfer',
    Svg: require('@site/static/img/transfer-svgrepo-com.svg').default,
    description: (
      <>
        Master the art of <strong>Digital Twins</strong>. Train robust policies in NVIDIA Isaac Sim
        and Gazebo using Reinforcement Learning before deploying to real robots.
      </>
    ),
  },
  {
    title: 'Humanoid Control Stack',
    Svg: require('@site/static/img/humanoid-svgrepo-com.svg').default,
    description: (
      <>
        Build the full software stack: from <strong>ROS 2</strong> middleware and sensor fusion
        to whole-body control and stable bipedal locomotion algorithms.
      </>
    ),
  },
];

function Feature({ title, Svg, description }: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
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
