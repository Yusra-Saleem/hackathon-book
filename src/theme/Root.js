import React from 'react';
import Root from '@theme-original/Root'; // Import the original Docusaurus Root component
import { AuthProvider } from '../contexts/AuthContext';

export default function RootWrapper(props) {
  return (
    <AuthProvider>
      <Root {...props} />
    </AuthProvider>
  );
}
