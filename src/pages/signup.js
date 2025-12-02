import React from 'react';
import Layout from '@theme/Layout';
import Signup from '../components/Signup';

function SignupPage() {
  return (
    <Layout title="Signup">
      <div
        style={{
          display: 'flex',
          justifyContent: 'center',
          alignItems: 'center',
          height: '50vh',
          textAlign: 'center',
        }}
      >
        <Signup />
      </div>
    </Layout>
  );
}

export default SignupPage;
