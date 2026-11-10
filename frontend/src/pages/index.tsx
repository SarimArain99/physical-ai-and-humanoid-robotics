import type {ReactNode} from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import SearchBar from '@theme/SearchBar';
import SenseThinkActLoop from '@site/src/components/SenseThinkActLoop';

import styles from './index.module.css';

/**
 * Homepage — dark-premium reading experience for the book.
 * Eight sections: hero (headline, one sentence, two actions, search,
 * the Sense–Think–Act loop, honest facts), what you will build,
 * course journey by phase, a real sample lesson, tools with honest
 * states, FAQ, closing call to action with the author strip.
 * Every number is real: 13 weeks, 13 chapters, 6 quiz sets with
 * 15 questions, free. No invented testimonials, counts or outcomes.
 */

const OUTCOMES = [
  {
    title: 'A ROS 2 system that talks',
    text: 'Publisher and subscriber nodes passing sensor data over topics.',
    week: 'Week 3',
    link: '/docs/week3/ros2-nodes-topics',
  },
  {
    title: 'A robot in simulation',
    text: 'A Gazebo world to test in before touching hardware.',
    week: 'Week 6',
    link: '/docs/week6/gazebo-simulation',
  },
  {
    title: 'A policy trained in Isaac Sim',
    text: 'Perception and planning on the NVIDIA Isaac platform.',
    week: 'Weeks 7 to 8',
    link: '/docs/week7/nvidia-isaac-sim',
  },
  {
    title: 'A transfer that survives reality',
    text: 'Domain randomization and calibration that close the gap.',
    week: 'Week 10',
    link: '/docs/week10/sim-to-real-transfer',
  },
  {
    title: 'A conversational humanoid',
    text: 'Speech, planning, and control in one capstone build.',
    week: 'Week 13',
    link: '/docs/week13/conversational-robotics',
  },
];

const PHASES = [
  {
    name: 'Introduction',
    range: 'Weeks 1 to 2',
    chapters: [
      { week: 1, title: 'Introduction to Physical AI', link: '/docs/week1/intro-physical-ai' },
      { week: 2, title: 'Embodied Intelligence', link: '/docs/week2/embodied-intelligence' },
    ],
  },
  {
    name: 'ROS 2',
    range: 'Weeks 3 to 5',
    chapters: [
      { week: 3, title: 'ROS 2 Architecture and Core Concepts: Nodes and Topics', link: '/docs/week3/ros2-nodes-topics' },
      { week: 4, title: 'ROS 2 Services and Actions', link: '/docs/week4/ros2-services-actions' },
      { week: 5, title: 'Advanced ROS 2 Concepts: Launch Files, Parameters, and Lifecycle Nodes', link: '/docs/week5/ros2-fundamentals-advanced' },
    ],
  },
  {
    name: 'Simulation',
    range: 'Weeks 6 to 7',
    chapters: [
      { week: 6, title: 'Robot Simulation with Gazebo', link: '/docs/week6/gazebo-simulation' },
      { week: 7, title: 'NVIDIA Isaac Sim Fundamentals', link: '/docs/week7/nvidia-isaac-sim' },
    ],
  },
  {
    name: 'Isaac',
    range: 'Weeks 8 to 10',
    chapters: [
      { week: 8, title: 'NVIDIA Isaac Platform: AI-Powered Robotics', link: '/docs/week8/nvidia-isaac-sim' },
      { week: 9, title: 'Isaac ROS: Hardware-Accelerated Perception and Navigation', link: '/docs/week9/isaac-ros' },
      { week: 10, title: 'Sim-to-Real Transfer: Bridging Simulation and Reality', link: '/docs/week10/sim-to-real-transfer' },
    ],
  },
  {
    name: 'Humanoid',
    range: 'Weeks 11 to 12',
    chapters: [
      { week: 11, title: 'Humanoid Robot Kinematics', link: '/docs/week11/humanoid-kinematics' },
      { week: 12, title: 'Humanoid Robot Development: Kinematics, Dynamics, and Control', link: '/docs/week12/humanoid-development' },
    ],
  },
  {
    name: 'Conversational',
    range: 'Week 13',
    chapters: [
      { week: 13, title: 'Conversational Robotics and Capstone Integration', link: '/docs/week13/conversational-robotics' },
    ],
  },
];

const TOOLS = [
  {
    name: 'ROS 2',
    text: 'Middleware for nodes, topics, services, and actions.',
    weeks: 'Weeks 3 to 5',
  },
  {
    name: 'Gazebo',
    text: 'Robot simulation for safe practice.',
    weeks: 'Week 6',
  },
  {
    name: 'NVIDIA Isaac Sim',
    text: 'Simulation on the NVIDIA Isaac platform.',
    weeks: 'Weeks 7 to 8',
  },
  {
    name: 'Isaac ROS',
    text: 'Hardware-accelerated perception and navigation.',
    weeks: 'Week 9',
  },
  {
    name: 'Python',
    text: 'The language of the code samples.',
    weeks: 'Every chapter',
  },
];

const FAQS = [
  {
    q: 'Do I need a robot to follow this book?',
    a: 'No. The first ten weeks run in simulation, in Gazebo and Isaac Sim.',
  },
  {
    q: 'What should I know before starting?',
    a: 'Basic Python. The rest is taught from the start.',
  },
  {
    q: 'Is the book really free?',
    a: 'Yes. All 13 chapters are free to read, with no account needed.',
  },
  {
    q: 'Does it include practice?',
    a: 'Yes. Every chapter ends with review questions and an exercise, and six quiz sets hold 15 questions.',
  },
  {
    q: 'Can I read it in Urdu, or simpler?',
    a: 'Yes. Sign in, then use the Urdu toggle or the Level selector.',
  },
];

function HomepageHeader() {
  return (
    <header className={styles.hero}>
      <div className={styles.container}>
        <div className={styles.heroGrid}>
          <div className={styles.heroText}>
            <Heading as="h1" className={styles.heroTitle}>
              Robots that see, think, and act.
            </Heading>
            <p className={styles.heroSentence}>
              A free 13-week textbook from ROS 2 to humanoid control.
            </p>
            <div className={styles.heroActions}>
              <Link
                className={styles.heroPrimaryButton}
                to="/docs/week1/intro-physical-ai">
                Start Week 1
              </Link>
              <Link
                className={styles.heroSecondaryButton}
                to="#course-journey">
                Browse the course
              </Link>
            </div>
            <div className={styles.heroSearch}>
              <SearchBar />
            </div>
            <dl className={styles.heroFacts}>
              <div className={styles.heroFact}>
                <dt>weeks</dt>
                <dd>13</dd>
              </div>
              <div className={styles.heroFact}>
                <dt>chapters</dt>
                <dd>13</dd>
              </div>
              <div className={styles.heroFact}>
                <dt>quiz questions</dt>
                <dd>15</dd>
              </div>
              <div className={styles.heroFact}>
                <dt>price</dt>
                <dd>Free</dd>
              </div>
            </dl>
          </div>
          <div className={styles.heroVisual}>
            <SenseThinkActLoop />
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
      description="A free 13-week textbook on Physical AI and humanoid robotics: ROS 2, simulation, NVIDIA Isaac, and humanoid control, with real code and quizzes.">
      <HomepageHeader />
      <main>
        <section className={styles.section}>
          <div className={styles.container}>
            <Heading as="h2" className={styles.sectionTitle}>
              What you will build
            </Heading>
            <p className={styles.sectionIntro}>
              Five things you can point at when the course is done.
            </p>
            <ol className={styles.outcomeList}>
              {OUTCOMES.map((outcome, idx) => (
                <li key={outcome.title} className={styles.outcomeRow}>
                  <span className={styles.outcomeNumber} aria-hidden="true">
                    {String(idx + 1).padStart(2, '0')}
                  </span>
                  <div className={styles.outcomeBody}>
                    <span className={styles.outcomeTitle}>{outcome.title}</span>
                    <span className={styles.outcomeText}>{outcome.text}</span>
                  </div>
                  <Link className={styles.outcomeLink} to={outcome.link}>
                    {outcome.week}
                  </Link>
                </li>
              ))}
            </ol>
          </div>
        </section>

        <section className={styles.sectionTint}>
          <div className={styles.container}>
            <Heading as="h2" id="course-journey" className={styles.sectionTitle}>
              The course, week by week
            </Heading>
            <p className={styles.sectionIntro}>
              Thirteen chapters in six phases, each one building on the last.
            </p>
            {PHASES.map((phase) => (
              <div key={phase.name} className={styles.phase}>
                <div className={styles.phaseHead}>
                  <span className={styles.phaseName}>{phase.name}</span>
                  <span className={styles.phaseRange}>{phase.range}</span>
                </div>
                <ul className={styles.phaseChapters}>
                  {phase.chapters.map((chapter) => (
                    <li key={chapter.week} className={styles.chapterRow}>
                      <Link className={styles.chapterLink} to={chapter.link}>
                        {chapter.title}
                      </Link>
                      <span className={styles.chapterWeek}>Week {chapter.week}</span>
                    </li>
                  ))}
                </ul>
              </div>
            ))}
          </div>
        </section>

        <section className={styles.section}>
          <div className={styles.container}>
            <Heading as="h2" className={styles.sectionTitle}>
              A look inside Week 1
            </Heading>
            <p className={styles.sectionIntro}>
              From &ldquo;Why Physical AI Is Hard&rdquo;, the first chapter.
            </p>
            <div className={styles.sampleBox}>
              <p className={styles.sampleLead}>
                Moving AI into the physical world adds problems that digital AI never faces.
              </p>
              <ul className={styles.sampleList}>
                <li>
                  <strong>The world is uncertain.</strong> A camera image is noisy, a lidar reading has gaps, and a joint encoder drifts.
                </li>
                <li>
                  <strong>Actions cannot be undone.</strong> A wrong step from a humanoid can damage the robot, the room, or a person nearby.
                </li>
                <li>
                  <strong>Time is strict.</strong> A robot that decides to catch a falling object one second too late has already failed.
                </li>
                <li>
                  <strong>Simulation is not reality.</strong> A policy trained in a simulator usually performs worse on the real robot.
                </li>
              </ul>
              <Link className={styles.sampleLink} to="/docs/week1/intro-physical-ai">
                Read Week 1
              </Link>
            </div>
          </div>
        </section>

        <section className={styles.sectionTint}>
          <div className={styles.container}>
            <Heading as="h2" className={styles.sectionTitle}>
              The tools
            </Heading>
            <ul className={styles.toolList}>
              {TOOLS.map((tool) => (
                <li key={tool.name} className={styles.toolRow}>
                  <span className={styles.toolName}>{tool.name}</span>
                  <span className={styles.toolText}>{tool.text}</span>
                  <span className={styles.toolWeeks}>{tool.weeks}</span>
                </li>
              ))}
            </ul>
          </div>
        </section>

        <section className={styles.section}>
          <div className={styles.container}>
            <Heading as="h2" className={styles.sectionTitle}>
              Questions
            </Heading>
            <div className={styles.faqList}>
              {FAQS.map((faq) => (
                <details key={faq.q} className={styles.faqItem}>
                  <summary className={styles.faqQuestion}>{faq.q}</summary>
                  <p className={styles.faqAnswer}>{faq.a}</p>
                </details>
              ))}
            </div>
          </div>
        </section>

        <section className={styles.sectionTint}>
          <div className={styles.container}>
            <Heading as="h2" className={styles.sectionTitle}>
              Start reading
            </Heading>
            <p className={styles.sectionIntro}>
              Week 1 is free, and so is the rest.
            </p>
            <Link
              className={styles.heroPrimaryButton}
              to="/docs/week1/intro-physical-ai">
              Start Week 1
            </Link>
            <div className={styles.authorBand}>
              <div className={styles.authorInfo}>
                <span className={styles.authorName}>Written by Muhammad Sarim</span>
                <span className={styles.authorRole}>
                  Software developer and teacher. School Admin at Islamic Science College (Boys).
                </span>
              </div>
              <div className={styles.authorLinks}>
                <a href="https://github.com/SarimArain99">GitHub</a>
                <a href="https://www.instagram.com/dev_sarim_arain/">Instagram</a>
                <a href="https://www.linkedin.com/in/muhammad-sarim-arain-b3795a2ba/">LinkedIn</a>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}
