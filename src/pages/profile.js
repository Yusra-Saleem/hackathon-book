import React from 'react';
import Layout from '@theme/Layout';
import Profile from '../components/Profile';

function ProfilePage() {
  return (
    <Layout title="User Profile">
      <div
        style={{
          display: 'flex',
          justifyContent: 'center',
          alignItems: 'center',
          height: '50vh',
          textAlign: 'center',
        }}
      >
        <Profile />
      </div>
    </Layout>
  );
}

export default ProfilePage;
