import React, { useEffect } from 'react';
import { injectSpeedInsights } from '@vercel/speed-insights';

export default function Root({ children }) {
  useEffect(() => {
    // Initialize Vercel Speed Insights
    injectSpeedInsights();

    // Initialize ChatKit
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
      {children}
      <div id="chatbot"></div>
    </>
  );
}
