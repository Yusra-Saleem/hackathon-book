import React, { useState, useEffect } from 'react';
import { useHistory } from '@docusaurus/router';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import { useAuth } from '../contexts/AuthContext';
import AuthModal from '../components/AuthModal';
import Layout from '@theme/Layout'; 
export default function SignupPage() {
  const [showModal, setShowModal] = useState(true);
  const { user } = useAuth();
  const history = useHistory();

  useEffect(() => {
    if (user) {
      // Redirect to home if already logged in
      history.push('/');
    }
  }, [user, history]);

  const handleClose = () => {
    setShowModal(false);
    history.push('/');
  };

  const handleAuthSuccess = () => {
    setShowModal(false);
    history.push('/');
  };

  return (
    <Layout title="Sign Up" description="Create a new account">
      <div style={{ padding: '2rem', textAlign: 'center' }}>
        <h1>Sign Up</h1>
        <p>Please use the authentication modal to create an account.</p>
      </div>
      <AuthModal
        isOpen={showModal}
        onClose={handleClose}
        onAuthSuccess={handleAuthSuccess}
        defaultTab="sign_up"
      />
    </Layout>
  );
}
