import React, { useEffect } from 'react';

export default function Root({ children }) {
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
      {children}
      <div id="chatbot"></div>
    </>
  );
}
