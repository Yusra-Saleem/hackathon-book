import React from 'react';
import { useHistory } from 'react-router-dom';
import { useAuth } from '../../contexts/AuthContext';

export default function AuthNavbarItem() {
  const { user, logout } = useAuth();
  const history = useHistory();

  const handleLogout = async () => {
      try {
      await logout();
    } catch (error) {
      console.error('Logout failed:', error);
    }
  };

  if (user) {
    return (
      <>
        <span className="navbar__item navbar__link">
          Welcome, {user.email || user.name}
        </span>
        <button
          className="navbar__item navbar__link button button--link"
          onClick={handleLogout}
          aria-label="Logout"
        >
          Logout
        </button>
      </>
    );
  } else {
    return (
      <>
        <button
          className="navbar__item navbar__link button button--link"
          onClick={() => history.push('/login')}
          aria-label="Login"
        >
          Login
        </button>
        <button
          className="navbar__item navbar__link button button--link"
          onClick={() => history.push('/signup')}
          aria-label="Sign Up"
        >
          Sign Up
        </button>
      </>
    );
  }
}
