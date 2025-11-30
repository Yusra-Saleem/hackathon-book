import React from 'react';
import Root from '@theme-original/Root'; // Import the original Docusaurus Root component
import Chatbot from './Chatbot'; // Import your existing Chatbot component

export default function RootWrapper(props) {
  return (
    <>
      {/* Render the original Root component, which includes all Docusaurus content */}
      <Root {...props} />
      {/* Render your global chatbot component */}
      <Chatbot />
    </>
  );
}
