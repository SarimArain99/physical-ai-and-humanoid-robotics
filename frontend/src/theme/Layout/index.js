import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import ChatWidget from '@site/src/components/ChatWidget';
import { AuthProvider } from '@site/src/components/Auth/AuthProvider';

export default function Layout(props) {
  return (
    <AuthProvider>
      <OriginalLayout {...props} />
      <ChatWidget />
    </AuthProvider>
  );
}