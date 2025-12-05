import React from 'react';
import Layout from '@theme/Layout';
import Profile from '../components/Profile';

function ProfilePage() {
  return (
    <Layout title="User Profile" noFooter>
      <Profile />
    </Layout>
  );
}

export default ProfilePage;
