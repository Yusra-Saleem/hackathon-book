import React, { useEffect } from 'react';
import Root from '@theme-original/Root'; // Import the original Docusaurus Root component
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

export default function RootWrapper(props) {
  useEffect(() => {
    if (ExecutionEnvironment.canUseDOM) {
      // Force dark theme
      document.documentElement.setAttribute('data-theme', 'dark');
      
      // Remove theme toggle button if it exists
      const themeToggle = document.querySelector('[class*="colorModeToggle"]');
      if (themeToggle) {
        themeToggle.remove();
      }
      
      // Ensure sidebar toggle button is visible and functional
      const ensureSidebarToggle = () => {
        const toggleButton = document.querySelector('.navbar__toggle');
        if (toggleButton) {
          toggleButton.style.display = 'inline-flex';
          toggleButton.style.visibility = 'visible';
          toggleButton.style.opacity = '1';
          toggleButton.style.pointerEvents = 'auto';
        }
      };
      
      // Run immediately and after a short delay
      ensureSidebarToggle();
      setTimeout(ensureSidebarToggle, 100);
      setTimeout(ensureSidebarToggle, 500);
      
      // Also check on window resize
      window.addEventListener('resize', ensureSidebarToggle);
      
      return () => {
        window.removeEventListener('resize', ensureSidebarToggle);
      };
    }
  }, []);

  return (
    <>
      <Root {...props} />

      {/* Global mobile textbook-style sidebar, shown when html has .book-sidebar-open */}
      <div className="book-mobile-sidebar">
          <nav className="book-mobile-sidebar__content">
            <div className="book-mobile-sidebar__header">
              <span className="book-mobile-sidebar__title">Textbook</span>
            </div>
            <ul className="book-mobile-sidebar__list">
              {/* Introduction */}
              <li className="book-mobile-sidebar__category">
                <div className="book-mobile-sidebar__category-label">
                  Introduction
                </div>
                <ul className="book-mobile-sidebar__chapter-list">
                  <li>
                    <a href="/docs/Introduction/Foundations-Hardware">
                      Week 1–2: Foundations &amp; Hardware
                    </a>
                  </li>
                </ul>
              </li>

              {/* Module 1 */}
              <li className="book-mobile-sidebar__category">
                <div className="book-mobile-sidebar__category-label">
                  Module 1: ROS2
                </div>
                <ul className="book-mobile-sidebar__chapter-list">
                  <li>
                    <a href="/docs/Module-1-ROS2/Week-3-Nodes-Topics">
                      Week 3: Nodes &amp; Topics
                    </a>
                  </li>
                  <li>
                    <a href="/docs/Module-1-ROS2/Week-4-Services-Actions">
                      Week 4: Services &amp; Actions
                    </a>
                  </li>
                  <li>
                    <a href="/docs/Module-1-ROS2/Week-5-URDF-rclpy">
                      Week 5: URDF &amp; rclpy
                    </a>
                  </li>
                </ul>
              </li>

              {/* Module 2 */}
              <li className="book-mobile-sidebar__category">
                <div className="book-mobile-sidebar__category-label">
                  Module 2: Digital Twin
                </div>
                <ul className="book-mobile-sidebar__chapter-list">
                  <li>
                    <a href="/docs/Module-2-DigitalTwin/Week-6-Gazebo-Setup">
                      Week 6: Gazebo Setup
                    </a>
                  </li>
                  <li>
                    <a href="/docs/Module-2-DigitalTwin/Week-7-Sensors-Unity">
                      Week 7: Sensors &amp; Unity
                    </a>
                  </li>
                </ul>
              </li>

              {/* Module 3 */}
              <li className="book-mobile-sidebar__category">
                <div className="book-mobile-sidebar__category-label">
                  Module 3: Isaac Brain
                </div>
                <ul className="book-mobile-sidebar__chapter-list">
                  <li>
                    <a href="/docs/Module-3-IsaacBrain/Week-8-Isaac-Sim-SDK">
                      Week 8: Isaac Sim SDK
                    </a>
                  </li>
                  <li>
                    <a href="/docs/Module-3-IsaacBrain/Week-9-VSLAM-Nav2">
                      Week 9: VSLAM &amp; Nav2
                    </a>
                  </li>
                  <li>
                    <a href="/docs/Module-3-IsaacBrain/Week-10-Reinforcement-Learning">
                      Week 10: Reinforcement Learning
                    </a>
                  </li>
                </ul>
              </li>

              {/* Module 4 */}
              <li className="book-mobile-sidebar__category">
                <div className="book-mobile-sidebar__category-label">
                  Module 4: VLA (Vision-Language-Action)
                </div>
                <ul className="book-mobile-sidebar__chapter-list">
                  <li>
                    <a href="/docs/Module-4-VLA/Week-11-Kinematics-Dynamics">
                      Week 11: Kinematics &amp; Dynamics
                    </a>
                  </li>
                  <li>
                    <a href="/docs/Module-4-VLA/Week-12-Manipulation-Interaction">
                      Week 12: Manipulation &amp; Interaction
                    </a>
                  </li>
                  <li>
                    <a href="/docs/Module-4-VLA/Week-13-Conversational-AI">
                      Week 13: Conversational AI
                    </a>
                  </li>
                </ul>
              </li>
            </ul>
          </nav>
        </div>
      </>
    );
}
