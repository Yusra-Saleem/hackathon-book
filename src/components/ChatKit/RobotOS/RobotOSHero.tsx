import React, { useEffect, useRef } from 'react';
import Link from '@docusaurus/Link';
import styles from './RobotOSHero.module.css';

export default function RobotOSHero() {
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    if (!canvasRef.current) return;

    // Simple wireframe robot using Canvas 2D (no Three.js dependency)
    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    canvas.width = 600;
    canvas.height = 700;

    let rotation = 0;
    const centerX = canvas.width / 2;
    const centerY = canvas.height / 2;

    function drawWireframeRobot() {
      if (!ctx) return;
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      
      ctx.save();
      ctx.translate(centerX, centerY);
      ctx.rotate(rotation);
      
      ctx.strokeStyle = '#00FFFF';
      ctx.lineWidth = 2;
      ctx.shadowBlur = 20;
      ctx.shadowColor = '#00FFFF';
      
      // Head (wireframe box)
      ctx.beginPath();
      ctx.rect(-30, -280, 60, 50);
      ctx.stroke();
      
      // Torso (main body)
      ctx.beginPath();
      ctx.rect(-50, -200, 100, 120);
      ctx.stroke();
      
      // Arms
      ctx.beginPath();
      ctx.moveTo(-50, -180);
      ctx.lineTo(-90, -120);
      ctx.moveTo(50, -180);
      ctx.lineTo(90, -120);
      ctx.stroke();
      
      // Legs
      ctx.beginPath();
      ctx.moveTo(-30, -80);
      ctx.lineTo(-40, 80);
      ctx.moveTo(30, -80);
      ctx.lineTo(40, 80);
      ctx.stroke();
      
      // Joints (glowing dots)
      const joints = [
        [-30, -280], [30, -280], // Head
        [-50, -180], [50, -180], // Shoulders
        [-90, -120], [90, -120], // Elbows
        [-30, -80], [30, -80], // Hips
        [-40, 80], [40, 80], // Feet
      ];
      
      ctx.fillStyle = '#00FFFF';
      joints.forEach(([x, y]) => {
        ctx.beginPath();
        ctx.arc(x, y, 4, 0, Math.PI * 2);
        ctx.fill();
      });
      
      ctx.restore();
    }

    function animate() {
      rotation += 0.005;
      drawWireframeRobot();
      requestAnimationFrame(animate);
    }

    animate();
  }, []);

  return (
    <div className={styles.robotOSHero}>
      <div className={styles.binaryStream}>
        {Array.from({ length: 50 }).map((_, i) => (
          <span key={i} className={styles.binaryChar}>
            {Math.random() > 0.5 ? '1' : '0'}
          </span>
        ))}
      </div>
      
      <div className={styles.systemStatusBar}>
        <div className={styles.statusItem}>
          <span className={styles.statusLabel}>SYSTEM</span>
          <span className={styles.statusValue}>ONLINE</span>
        </div>
        <div className={styles.statusItem}>
          <span className={styles.statusLabel}>AGENT_RAG</span>
          <span className={styles.statusValue}>READY</span>
        </div>
        <div className={styles.statusItem}>
          <span className={styles.statusLabel}>VECTOR_DB</span>
          <span className={styles.statusValue}>CONNECTED</span>
        </div>
      </div>

      <div className={styles.heroContent}>
        <div className={styles.heroCenter}>
          <div className={styles.titleWrapper}>
            <h1 className={styles.glitchTitle}>
              <span className={styles.glitchText} data-text="PHYSICAL AI">PHYSICAL AI</span>
              <br />
              <span className={styles.glitchText} data-text="NEURAL INTERFACE">NEURAL INTERFACE</span>
            </h1>
            <p className={styles.heroSubtitle}>
              Operating System for Embodied Intelligence
            </p>
          </div>

          <div className={styles.robotContainer}>
            <div className={styles.robotWrapper}>
              <canvas ref={canvasRef} className={styles.robotCanvas} />
              <div className={styles.robotGlow}></div>
              <div className={styles.energyRings}>
                <div className={styles.ring}></div>
                <div className={styles.ring}></div>
                <div className={styles.ring}></div>
              </div>
            </div>
          </div>

          <div className={styles.heroActions}>
            <Link to="/docs/Introduction/Foundations-Hardware" className={styles.hexButton}>
              <span>INITIALIZE CURRICULUM</span>
            </Link>
            <Link to="https://github.com/your-org/my-ai-book" className={styles.hexButtonSecondary}>
              <span>ACCESS REPOSITORY</span>
            </Link>
          </div>
        </div>
      </div>

      <div className={styles.resourceMonitor}>
        <div className={styles.monitorPanel}>
          <div className={styles.monitorHeader}>RESOURCE MONITOR</div>
          <div className={styles.monitorItem}>
            <span className={styles.monitorLabel}>CPU</span>
            <div className={styles.progressBar}>
              <div className={styles.progressFill} style={{ width: '68%' }}></div>
            </div>
            <span className={styles.monitorValue}>68%</span>
          </div>
          <div className={styles.monitorItem}>
            <span className={styles.monitorLabel}>RAM</span>
            <div className={styles.progressBar}>
              <div className={styles.progressFill} style={{ width: '45%' }}></div>
            </div>
            <span className={styles.monitorValue}>45%</span>
          </div>
          <div className={styles.monitorItem}>
            <span className={styles.monitorLabel}>VECTOR_DB</span>
            <div className={styles.progressBar}>
              <div className={styles.progressFill} style={{ width: '92%', backgroundColor: '#00FF88' }}></div>
            </div>
            <span className={styles.monitorValue}>92%</span>
          </div>
        </div>
      </div>
    </div>
  );
}

