import type { ReactNode } from "react";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import Layout from "@theme/Layout";
import HomepageFeatures from "@site/src/components/HomepageFeatures";

export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={siteConfig.title}
      description="AI-Native Physical AI & Humanoid Robotics Course"
    >
      {/* HomepageFeatures contains hero, modules, and tech stack */}
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
