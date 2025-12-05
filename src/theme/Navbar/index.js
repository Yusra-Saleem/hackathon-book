import React, { useEffect } from 'react';
import NavbarOriginal from '@theme-original/Navbar';

export default function Navbar(props) {
  useEffect(() => {
    const root = document.documentElement;

    const handleToggleClick = (event) => {
      // Only react to actual navbar toggle clicks
      const toggle = event.target.closest('.navbar__toggle');
      if (!toggle) return;

      // Toggle custom class that CSS uses to slide the textbook sidebar
      root.classList.toggle('book-sidebar-open');
    };

    document.addEventListener('click', handleToggleClick);

    return () => {
      document.removeEventListener('click', handleToggleClick);
      root.classList.remove('book-sidebar-open');
    };
  }, []);

  return <NavbarOriginal {...props} />;
}

