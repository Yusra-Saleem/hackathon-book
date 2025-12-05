import React from 'react';
import Layout from '@theme/Layout';
import Login from '../components/Login';

function LoginPage() {
  return (
    <Layout title="Login" noFooter>
      <Login />
    </Layout>
  );
}

export default LoginPage;