import React, { useEffect, useState } from 'react';
import { SpeedInsights } from '@vercel/speed-insights/react';

// Temporarily disable auth until database is configured
// import { AuthProvider } from '../components/Auth/AuthProvider';
// import AuthModal from '../components/Auth/AuthModal';

export default function Root({ children }) {
  const [showAuthModal, setShowAuthModal] = useState(false);
  
  // Show auth modal on first visit (disabled until DB configured)
  // useEffect(() => {
  //   const hasVisited = localStorage.getItem('hasVisited');
  //   if (!hasVisited) {
  //     setTimeout(() => {
  //       setShowAuthModal(true);
  //       localStorage.setItem('hasVisited', 'true');
  //     }, 2000);
  //   }
  // }, []);
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
    <>
      {/* AuthProvider disabled until database is configured */}
      {/* <AuthProvider> */}
      {children}
      <SpeedInsights />
      {/* <AuthModal isOpen={showAuthModal} onClose={() => setShowAuthModal(false)} /> */}
      <div id="chatbot"></div>
      {/* </AuthProvider> */}
    </>
  );
}
