import React, { useEffect, useState } from 'react';
import Link from '@docusaurus/Link';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import { useAuth } from '../contexts/AuthContext';
import AuthModal from './AuthModal';

export default function AuthButton() {
  const { user, login, logout } = useAuth();
  const [isModalOpen, setIsModalOpen] = useState(false);

  useEffect(() => {
    if (!ExecutionEnvironment.canUseDOM) return;
    const storedUserId = localStorage.getItem('user_id');
    const storedToken = localStorage.getItem('token');
    if (storedUserId && storedToken && !user) {
      login({
        user_id: storedUserId,
        token: storedToken,
      });
    }
  }, [user, login]);

  const handleOpenModal = () => setIsModalOpen(true);
  const handleCloseModal = () => setIsModalOpen(false);

  const handleAuthSuccess = (data) => {
    login(data);
    setIsModalOpen(false);
  };

  const handleLogout = () => {
    if (ExecutionEnvironment.canUseDOM) {
      localStorage.removeItem('user_id');
      localStorage.removeItem('token');
    }
    logout();
  };

  if (user) {
    return (
      <>
        <Link
          className="navbar__item navbar__link"
          to="/profile"
          aria-label="Profile"
        >
          Profile
        </Link>
        <button
          className="navbar__item navbar__link button button--link"
          onClick={handleLogout}
          aria-label="Logout"
        >
          Logout
        </button>
      </>
    );
  }

  return (
    <>
      <button
        className="navbar__item navbar__link button button--primary"
        onClick={handleOpenModal}
        aria-label="Sign in or sign up"
      >
        Sign In / Sign Up
      </button>
      <AuthModal
        isOpen={isModalOpen}
        onClose={handleCloseModal}
        onAuthSuccess={handleAuthSuccess}
      />
    </>
  );
}


