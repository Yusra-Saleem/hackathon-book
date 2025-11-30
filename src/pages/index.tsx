import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import ChatInterface from '@site/src/components/ChatInterface';
import styles from './index.module.css';

import { FaRobot, FaEye, FaBrain, FaTheaterMasks, FaGlobe, FaBookOpen, FaCommentDots } from 'react-icons/fa';

function HeroSection() {
  const {siteConfig} = useDocusaurusContext();
  
  return (
    <header className={styles.heroSection}>
      <div className={styles.heroBackground}></div>
      <div className={styles.container}>
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
            <h1>
              The Future of <br />
              Embodied AI
            </h1>
            <p>
              From ROS2 control loops to Vision-Language-Action models. 
              The definitive open-source guide for the next generation of roboticists.
            </p>
            <div className={styles.heroButtons}>
              <Link className={styles.ctaButton + ' ' + styles.primaryBtn} to="/docs/Introduction/Foundations-Hardware">
                Start Learning
              </Link>
              <Link className={styles.ctaButton + ' ' + styles.secondaryBtn} to="https://github.com/your-org/my-ai-book">
                View Source
              </Link>
            </div>
          </div>
          
          <div className={styles.heroVisual}>
            <div className={styles.codeWindow}>
              <div className={styles.windowHeader}>
                <span className={styles.dot}></span>
                <span className={styles.dot}></span>
                <span className={styles.dot}></span>
              </div>
              <div className={styles.codeContent}>
                <div><span className={styles.keyword}>import</span> <span className={styles.string}>ros2</span></div>
                <div><span className={styles.keyword}>from</span> <span className={styles.string}>transformers</span> <span className={styles.keyword}>import</span> AutoModelForVLA</div>
                <br/>
                <div><span className={styles.function}>class</span> <span className={styles.keyword}>RobotController</span>:</div>
                <div style={{paddingLeft: '20px'}}>
                  <span className={styles.keyword}>def</span> <span className={styles.function}>__init__</span>(self):
                </div>
                <div style={{paddingLeft: '40px'}}>
                  self.model = <span className={styles.function}>load_policy</span>(<span className={styles.string}>"rt-2-x"</span>)
                </div>
                <div style={{paddingLeft: '40px'}}>
                  self.state = <span className={styles.string}>"INITIALIZING"</span><span className={styles.cursor}></span>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

function BentoFeatures() {
  return (
    <section className={styles.featuresSection}>
      <div className={styles.container}>
        <div className={styles.sectionHeader}>
          <span className={styles.sectionTag}>Curriculum</span>
          <h2 className={styles.sectionTitle}>Master the Stack</h2>
        </div>
        
        <div className={styles.bentoGrid}>
          {/* Card 1: Foundation (Large) */}
          <div className={styles.bentoCard + ' ' + styles.cardLarge}>
            <div className={styles.cardIcon}><FaRobot /></div>
            <div>
              <h3 className={styles.cardTitle}>Hardware & Control</h3>
              <p className={styles.cardDesc}>
                Understand the physics of actuation. Learn to write real-time controllers 
                in C++ and Python using the ROS2 middleware.
              </p>
            </div>
          </div>

          {/* Card 2: Perception */}
          <div className={styles.bentoCard}>
            <div className={styles.cardIcon}><FaEye /></div>
            <div>
              <h3 className={styles.cardTitle}>Perception</h3>
              <p className={styles.cardDesc}>
                3D Computer Vision, SLAM, and state estimation using LiDAR and Depth cameras.
              </p>
            </div>
          </div>

          {/* Card 3: AI Agents (Tall) */}
          <div className={styles.bentoCard + ' ' + styles.cardTall}>
            <div className={styles.cardIcon}><FaBrain /></div>
            <div>
              <h3 className={styles.cardTitle}>Agentic AI</h3>
              <p className={styles.cardDesc}>
                Build autonomous agents that can reason, plan, and execute complex tasks 
                using Large Language Models and RAG.
              </p>
            </div>
            <div style={{marginTop: '2rem', opacity: 0.5, fontSize: '3rem'}}>
              <FaRobot /> <FaCommentDots />
            </div>
          </div>

          {/* Card 4: Imitation Learning */}
          <div className={styles.bentoCard}>
            <div className={styles.cardIcon}><FaTheaterMasks /></div>
            <div>
              <h3 className={styles.cardTitle}>Imitation Learning</h3>
              <p className={styles.cardDesc}>
                Train policies from human demonstrations using Behavioral Cloning and Diffusion.
              </p>
            </div>
          </div>

          {/* Card 5: Sim2Real (Large) */}
          <div className={styles.bentoCard + ' ' + styles.cardLarge}>
            <div className={styles.cardIcon}><FaGlobe /></div>
            <div>
              <h3 className={styles.cardTitle}>Sim-to-Real Transfer</h3>
              <p className={styles.cardDesc}>
                Master physics simulators like Isaac Gym and MuJoCo to train robust policies 
                that work in the messy real world.
              </p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function AgentPreview() {
  return (
    <section className={styles.agentSection}>
      <div className={styles.container}>
        <div className={styles.agentContainer}>
          <h2 className={styles.sectionTitle}>Your Personal AI Tutor</h2>
          <p style={{color: '#8892b0', fontSize: '1.2rem', marginBottom: '2rem'}}>
            Stuck on a concept? Use the chat widget in the bottom right to ask questions. 
            Our RAG-powered agent has read the entire book and the source code.
          </p>
        </div>
      </div>
    </section>
  );
}

export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Home | ${siteConfig.title}`}
      description="The definitive guide to Physical AI and Humanoid Robotics">
      <main>
        <HeroSection />
        <BentoFeatures />
        <AgentPreview />
        <ChatInterface />
      </main>
    </Layout>
  );
}