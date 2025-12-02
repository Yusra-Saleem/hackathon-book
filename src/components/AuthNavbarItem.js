import React from 'react';
import { useHistory } from '@docusaurus/router';
import { useAuth } from '../contexts/AuthContext';
import Link from '@docusaurus/Link';

const AuthNavbarItem = () => {
  const { user, logout } = useAuth();
  const history = useHistory();

  const handleLogout = () => {
    logout();
    history.push('/login'); // Redirect to login page after logout
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
  } else {
    return (
      <>
        <Link
          className="navbar__item navbar__link"
          to="/login"
          aria-label="Login"
        >
          Login
        </Link>
        <Link
          className="navbar__item navbar__link"
          to="/signup"
          aria-label="Signup"
        >
          Signup
        </Link>
      </>
    );
  }
};

export default AuthNavbarItem;
