import React from 'react';
import clsx from 'clsx';
import styles from './HomepageFeatures.module.css';

const FeatureList = [
  {
    title: 'Physical AI & Embodiment',
    Svg: require('@site/static/img/ai-robot.svg').default,
    description: (
      <>
        Explore the fundamentals of Physical AI, where intelligence emerges from 
        the interaction between body, environment, and control systems.
      </>
    ),
  },
  {
    title: 'Humanoid Robotics',
    Svg: require('@site/static/img/humanoid-robot.svg').default,
    description: (
      <>
        Master the complexities of humanoid robot development, including 
        locomotion, manipulation, and multi-limb coordination.
      </>
    ),
  },
  {
    title: 'Vision-Language-Action',
    Svg: require('@site/static/img/vla-system.svg').default,
    description: (
      <>
        Integrate perception, language understanding, and action planning 
        to create robots that understand and respond to natural commands.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        {Svg ? <Svg className={styles.featureSvg} role="img" /> : null}
      </div>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
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