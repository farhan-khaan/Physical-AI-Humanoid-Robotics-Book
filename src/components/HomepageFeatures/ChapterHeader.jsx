import React from 'react';
import axios from 'axios';

export default function ChapterHeader({ chapterId }) {
  const personalize = async () => {
    const res = await axios.post('https://your-api.onrender.com/personalize', { chapterId });
    alert(res.data.custom_text);
  };

  const translateUrdu = async () => {
    const res = await axios.post('https://your-api.onrender.com/translate', { chapterId });
    alert(res.data.translation);
  };

  return (
    <div>
      <button onClick={personalize}>🎯 Personalize Content</button>
      <button onClick={translateUrdu}>🌐 Translate to Urdu</button>
    </div>
  );
}
