import React, { useEffect, useState } from 'react';
import { SpeedInsights } from '@vercel/speed-insights/react';
import { AuthProvider } from '../components/Auth/AuthProvider';
import ChatbotWidget from '../components/Chatbot/ChatbotWidget';

export default function Root({ children }) {
  return (
    <AuthProvider>
      {children}
      <ChatbotWidget />
      <SpeedInsights />
    </AuthProvider>
  );
}
