import React, { useEffect, useState } from 'react';
import { SpeedInsights } from '@vercel/speed-insights/react';
import { AuthProvider } from '../components/Auth/AuthProvider';
import AuthModal from '../components/Auth/AuthModal';

export default function Root({ children }) {
  const [showAuthModal, setShowAuthModal] = useState(false);
  
  // Show auth modal on first visit
  useEffect(() => {
    const hasVisited = localStorage.getItem('hasVisited');
    if (!hasVisited) {
      setTimeout(() => {
        setShowAuthModal(true);
        localStorage.setItem('hasVisited', 'true');
      }, 2000); // Show after 2 seconds
    }
  }, []);
  useEffect(() => {
    const script = document.createElement('script');
    script.src = 'https://cdn.jsdelivr.net/npm/chatkit-sdk';
    script.onload = () => {
      ChatKit.init({
        endpoint: 'https://your-api.onrender.com/ask',
        selector: '#chatbot',
        title: 'Ask me about this book!'
      });
    };
    document.body.appendChild(script);
  }, []);
  return (
    <AuthProvider>
      {children}
      <SpeedInsights />
      <AuthModal isOpen={showAuthModal} onClose={() => setShowAuthModal(false)} />
      <div id="chatbot"></div>
    </AuthProvider>
  );
}
