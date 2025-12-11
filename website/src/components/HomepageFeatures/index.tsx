// import type { ReactNode } from "react";
// import clsx from "clsx";
// import Heading from "@theme/Heading";
// import styles from "./styles.module.css";

// type FeatureItem = {
//   title: string;
//   image: string;
//   description: ReactNode;
//   link: string;
// };

// const FeatureList: FeatureItem[] = [
//   {
//     title: "Physical AI & Humanoid Robotics",
//     image: "img/download.png",
//     link: "/modules//docs/intro",
//     description: (
//       <>
//         We&apos;ve been working on physical AI and humanoid robotics for over a
//         decade. Our expertise lies in creating lifelike robots that can interact
//         seamlessly with humans.
//       </>
//     ),
//   },
//   {
//     title: "The Robotic Nervous System",
//     image: "img/nervious.png",
//     link: "/modules/docs/modules/ros2/intro",
//     description: (
//       <>
//         Our proprietary Robotic Nervous System (RNS) integrates advanced AI
//         algorithms with cutting-edge hardware to deliver unparalleled
//         performance in robotics applications.
//       </>
//     ),
//   },
//   {
//     title: "Gazebo & Unity - The Digital Twin",
//     image: "img/twin.png",
//     link: "/modules/docs/modules/simulation/intro",
//     description: (
//       <>
//         We utilize Gazebo and Unity to create digital twins of our robotic
//         systems, allowing for precise simulations and testing in virtual
//         environments before deployment in the real world.
//       </>
//     ),
//   },
//   {
//     title: "NVIDIA Isaac - The AI-Robot Brain",
//     image: "img/robot.png",
//     link: "/modules/docs/modules/isaac/intro",
//     description: (
//       <>
//         Leveraging NVIDIA Isaac, we develop intelligent robotic brains that
//         enable autonomous decision-making and learning capabilities in our
//         humanoid robots.
//       </>
//     ),
//   },
//   {
//     title: "Vision-Language-Action (VLA) - The Complete System",
//     image: "img/vla.png",
//     link: "/modules/docs/modules/vla/intro",
//     description: (
//       <>
//         Our Vision-Language-Action (VLA) framework combines visual perception,
//         natural language understanding, and action execution to create robots
//         that can comprehend and respond to complex human instructions.
//       </>
//     ),
//   },
// ];

// function Feature({ title, image, description }: FeatureItem) {
//   return (
//     <div className={clsx("col col--4")}>
//       <div className="text--center">
//         <img src={image} className={styles.featureSvg} role="img" />
//       </div>
//       <div className="text--center padding-horiz--md">
//         <Heading as="h3">{title}</Heading>
//         <p>{description}</p>
//       </div>
//     </div>
//   );
// }

// export default function HomepageFeatures(): ReactNode {
//   return (
//     <section className={styles.features}>
//       <div className="container">
//         <div className="row">
//           {FeatureList.map((props, idx) => (
//             <Feature key={idx} {...props} />
//           ))}
//         </div>
//       </div>
//     </section>
//   );
// }

// =====================================
import React from "react";
import Link from "@docusaurus/Link";
import styles from "./styles.module.css"; // Use your HomepageFeatures/styles.module.css

function HomepageHeader() {
  return (
    <header className={styles.heroBanner}>
      <div className={styles.container}>
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
            <span className={styles.badge}>
              AI-NATIVE BOOK • Physical AI & Humanoid Robotics
            </span>

            <h1 className={styles.heroTitle}>
              Physical AI & Humanoid Robotics Course
            </h1>

            <p className={styles.heroSubtitle}>
              AI Systems in the Physical World. Embodied Intelligence. Goal:
              Bridging the gap between the digital brain and the physical body.
              Students apply their AI knowledge to control humanoid robots in
              simulated and real environments.
            </p>

            <div className={styles.badges}>
              <span className={styles.badgeItem}>
                <span className={styles.badgeIcon}>✨</span> Open Source
              </span>
              <span className={styles.badgeItem}>
                <span className={styles.badgeIcon}>🧡</span> Co-Learning with AI
              </span>
              <span className={styles.badgeItem}>
                <span className={styles.badgeIcon}>🔮</span> Spec-Driven
                Development
              </span>
            </div>

            <div className={styles.heroButtons}>
              <Link className={styles.buttonPrimary} to="/docs/intro">
                Start Learning →
              </Link>
            </div>
          </div>

          <div className={styles.heroImage}>
            <img
              src={require("@site/static/img/icon.png").default}
              alt="AI Course"
              className={styles.bookImage}
            />
          </div>
        </div>
      </div>
    </header>
  );
}

function FeatureSection() {
  return (
    <section className={styles.features}>
      <div className={styles.container}>
        <h2 className={styles.sectionTitle}>Course Modules</h2>

        <div className={styles.featureGrid}>
          <Link to="/docs/intro" className={styles.featureCard}>
            <div className={styles.featureIcon}>🤖</div>
            <h3>Physical AI & Humanoid Robotics (Intro)</h3>
            <p>
              Lifelike robots, embodied dynamics, and real-world AI system
              integration.
            </p>
          </Link>

          <Link to="/docs/modules/ros2/intro" className={styles.featureCard}>
            <div className={styles.featureIcon}>⚙️</div>
            <h3>The Robotic Nervous System (ROS2) (Module-1)</h3>
            <p>
              Real-time communication layer powering all humanoid robot actions.
            </p>
          </Link>

          <Link
            to="/docs/modules/simulation/intro"
            className={styles.featureCard}
          >
            <div className={styles.featureIcon}>🌐</div>
            <h3>Gazebo & Unity — Digital Twins (Module-2)</h3>
            <p>Create virtual worlds and simulate humanoid robot physics.</p>
          </Link>

          <Link to="/docs/modules/isaac/intro" className={styles.featureCard}>
            <div className={styles.featureIcon}>🧠</div>
            <h3>NVIDIA Isaac — AI Robot Brain (Module-3)</h3>
            <p>
              Build intelligent planning systems using NVIDIA’s robotics stack.
            </p>
          </Link>

          <Link to="/docs/modules/vla/intro" className={styles.featureCard}>
            <div className={styles.featureIcon}>👁️</div>
            <h3>Vision-Language-Action (VLA) (Module-4)</h3>
            <p>
              Voice → Vision → Plan → Action: The complete intelligent robot
              system.
            </p>
          </Link>
        </div>
      </div>
    </section>
  );
}

function TechStack() {
  const techItems = [
    { icon: "⚡", label: "ROS 2 Humble / Iron" },
    { icon: "🎮", label: "Gazebo & Unity Simulation" },
    { icon: "🎯", label: "NVIDIA Isaac Robotics" },
    { icon: "🗣️", label: "Whisper Speech AI" },
    { icon: "🤖", label: "Planning with GPT-4o" },
    { icon: "📊", label: "Nav2 Navigation Stack" },
  ];

  return (
    <section className={styles.techStack}>
      <div className={styles.container}>
        <h2 className={styles.sectionTitle}>What You Will Learn</h2>

        <div className={styles.techGrid}>
          {techItems.map((item, index) => (
            <div key={index} className={styles.techItem}>
              <span className={styles.techIcon}>{item.icon}</span>
              <span>{item.label}</span>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function HomepageFeatures() {
  return (
    <>
      <HomepageHeader />
      <main>
        <FeatureSection />
        <TechStack />
      </main>
    </>
  );
}
