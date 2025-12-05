import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import { FaRobot, FaEye, FaBrain, FaTheaterMasks, FaGlobe, FaCommentDots, FaCode, FaRocket, FaUsers, FaBook } from 'react-icons/fa';
import styles from './index.module.css';

function HeroSection() {
  return (
    <header className={styles.heroSection}>
      <div className={styles.heroBackground}>
        <div className={styles.heroGradient}></div>
        <div className={styles.heroPattern}></div>
        <div className={styles.heroGlow}></div>
        {/* Animated Nodes */}
        <div className={styles.nodesContainer}>
          <div className={styles.node + ' ' + styles.node1}></div>
          <div className={styles.node + ' ' + styles.node2}></div>
          <div className={styles.node + ' ' + styles.node3}></div>
          <div className={styles.node + ' ' + styles.node4}></div>
          <div className={styles.node + ' ' + styles.node5}></div>
          <div className={styles.nodeConnection}></div>
        </div>
      </div>
      <div className={styles.container}>
        <div className={styles.heroContent}>
          {/* Left Column - Text Content */}
          <div className={styles.heroLeft}>
            <div className={styles.heroBadge}>
              <span className={styles.heroKicker}>PHYSICAL AI · HUMANOID ROBOTICS</span>
            </div>
            <h1 className={styles.heroTitle}>
              Build the <span className={styles.heroTitleAccent}>robotics OS</span> <br />
              for your mind.
            </h1>
            <p className={styles.heroSubtitle}>
              A comprehensive, end‑to‑end playbook for ROS2, Isaac Sim, digital twins, and
              embodied AI agents. Learn the control loops, math, and code that matter.
            </p>
            <div className={styles.heroActions}>
              <Link
                className={`${styles.heroButton} ${styles.heroButtonPrimary}`}
                to="/docs/Introduction/Foundations-Hardware"
              >
                <span>Start the curriculum</span>
                <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
                  <path d="M6 12L10 8L6 4" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                </svg>
              </Link>
              <Link
                className={`${styles.heroButton} ${styles.heroButtonGhost}`}
                to="https://github.com/your-org/my-ai-book"
              >
                <span>Browse the repo</span>
              </Link>
            </div>
            <div className={styles.heroMetaRow}>
              <div className={styles.heroMetaItem}>
                <span className={styles.heroMetaIcon}>📚</span>
                <span>8+ modules</span>
              </div>
              <div className={styles.heroMetaItem}>
                <span className={styles.heroMetaIcon}>🤖</span>
                <span>Live RAG tutor</span>
              </div>
              <div className={styles.heroMetaItem}>
                <span className={styles.heroMetaIcon}>🚀</span>
                <span>Beginner → deployment</span>
              </div>
            </div>
          </div>
          
          {/* Right Column - Animated Robot */}
          <div className={styles.heroRight}>
            <div className={styles.robotContainer}>
              {/* Circular Nodes Around Robot */}
              <div className={styles.robotNodes}>
                <div className={styles.robotNodeRing + ' ' + styles.nodeRing1}>
                  <div className={styles.nodeDot + ' ' + styles.nodeDot1}></div>
                  <div className={styles.nodeDot + ' ' + styles.nodeDot2}></div>
                  <div className={styles.nodeDot + ' ' + styles.nodeDot3}></div>
                  <div className={styles.nodeDot + ' ' + styles.nodeDot4}></div>
                </div>
                <div className={styles.robotNodeRing + ' ' + styles.nodeRing2}>
                  <div className={styles.nodeDot + ' ' + styles.nodeDot5}></div>
                  <div className={styles.nodeDot + ' ' + styles.nodeDot6}></div>
                  <div className={styles.nodeDot + ' ' + styles.nodeDot7}></div>
                  <div className={styles.nodeDot + ' ' + styles.nodeDot8}></div>
                </div>
                <div className={styles.robotNodeRing + ' ' + styles.nodeRing3}>
                  <div className={styles.nodeDot + ' ' + styles.nodeDot9}></div>
                  <div className={styles.nodeDot + ' ' + styles.nodeDot10}></div>
                  <div className={styles.nodeDot + ' ' + styles.nodeDot11}></div>
                  <div className={styles.nodeDot + ' ' + styles.nodeDot12}></div>
                </div>
              </div>
              
              <div className={styles.robot}>
                {/* Robot Head */}
                <div className={styles.robotHead}>
                  <div className={styles.robotEyes}>
                    <div className={styles.robotEye}></div>
                    <div className={styles.robotEye}></div>
                  </div>
                </div>
                
                {/* Robot Body */}
                <div className={styles.robotBody}>
                  <div className={styles.robotChest}></div>
                </div>
                
                {/* Robot Arms */}
                <div className={styles.robotArms}>
                  <div className={styles.robotArm + ' ' + styles.robotArmLeft}>
                    <div className={styles.armUpper}></div>
                    <div className={styles.armLower}></div>
                    <div className={styles.robotHand + ' ' + styles.handLeft}></div>
                  </div>
                  <div className={styles.robotArm + ' ' + styles.robotArmRight}>
                    <div className={styles.armUpper}></div>
                    <div className={styles.armLower}></div>
                    <div className={styles.robotHand + ' ' + styles.handRight}></div>
                  </div>
                </div>
                
                {/* Robot Legs */}
                <div className={styles.robotLegs}>
                  <div className={styles.robotLeg}></div>
                  <div className={styles.robotLeg}></div>
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
          <h2 className={styles.sectionTitle}>Your Robotics Copilot</h2>
          <p className={styles.agentText}>
            Highlight any paragraph in the book and open the chat widget in the bottom right. 
            The RAG-powered tutor will ground its explanation directly in this textbook so you always stay on track.
          </p>
          <div className={styles.agentMetaRow}>
            <span className={styles.agentMetaPill}>Context-aware</span>
            <span className={styles.agentMetaPill}>Beginner-friendly</span>
            <span className={styles.agentMetaPill}>Textbook-grounded</span>
          </div>
        </div>
      </div>
    </section>
  );
}

function RoadmapStrip() {
  return (
    <section className={styles.roadmapSection}>
      <div className={styles.container}>
        <div className={styles.roadmapHeader}>
          <span className={styles.sectionTag}>Learning Path</span>
          <h2 className={styles.sectionTitle}>From Zero to Embodied AI</h2>
        </div>
        <div className={styles.roadmapTrack}>
          <div className={styles.roadmapStep}>
            <span className={styles.roadmapBadge}>01</span>
            <h3>Foundations & Hardware</h3>
            <p>Start with physical AI intuition, kinematics, and the core hardware stack.</p>
          </div>
          <div className={styles.roadmapStep}>
            <span className={styles.roadmapBadge}>02</span>
            <h3>ROS2 & Middleware</h3>
            <p>Model your robots as ROS2 nodes, topics, and services that scale cleanly.</p>
          </div>
          <div className={styles.roadmapStep}>
            <span className={styles.roadmapBadge}>03</span>
            <h3>Simulation & Digital Twins</h3>
            <p>Prototype everything in Isaac Sim before you risk real hardware.</p>
          </div>
          <div className={styles.roadmapStep}>
            <span className={styles.roadmapBadge}>04</span>
            <h3>Agents & VLA</h3>
            <p>Blend LLMs with perception and control to create full embodied agents.</p>
          </div>
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
        <RoadmapStrip />
      </main>
    </Layout>
  );
}