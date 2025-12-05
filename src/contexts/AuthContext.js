import React, { createContext, useState, useContext, useEffect } from 'react';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import { authClient } from '../auth';

const AuthContext = createContext(null);

export const AuthProvider = ({ children }) => {
  const [user, setUser] = useState(null);

  // Load user from localStorage on mount
  useEffect(() => {
    if (!ExecutionEnvironment.canUseDOM) return;
    try {
      const session = authClient.getSession && authClient.getSession();
      if (session && session.user) {
        setUser({
          token: session.token,
          user_id: session.user.id,
          email: session.user.email,
        });
      }
    } catch (e) {
      // fallback to previous localStorage keys
      const token = localStorage.getItem('token');
      const userId = localStorage.getItem('user_id');
      const email = localStorage.getItem('user_email');
      if (token && userId) {
        setUser({ token, user_id: userId, email: email || userId });
      }
    }
  }, []);

  const login = (userData) => {
    setUser(userData);
    // Persist session using authClient if available
    if (ExecutionEnvironment.canUseDOM) {
      try {
        if (authClient && authClient.setSession && userData && userData.token) {
          const session = {
            user: { id: userData.user_id || userData.id || Date.now().toString(), email: userData.email },
            token: userData.token,
          };
          authClient.setSession(session);
          return;
        }
      } catch (e) {
        // ignore and fallback to localStorage
      }

      if (userData.token) localStorage.setItem('token', userData.token);
      if (userData.user_id) localStorage.setItem('user_id', userData.user_id);
      if (userData.email) localStorage.setItem('user_email', userData.email);
    }
  };

  const logout = () => {
    setUser(null);
    if (ExecutionEnvironment.canUseDOM) {
      try {
        if (authClient && authClient.signOut) {
          authClient.signOut();
        }
      } catch (e) {
        // ignore
      }
      localStorage.removeItem('token');
      localStorage.removeItem('user_id');
      localStorage.removeItem('user_email');
    }
  };

  return (
    <AuthContext.Provider value={{ user, login, logout }}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = () => useContext(AuthContext);
