import React, { useState } from 'react';
import { useAuth } from '../Auth/AuthProvider';
import styles from './ChapterActions.module.css';

interface PersonalizeButtonProps {
  chapterId: string;
  chapterTitle: string;
}

const PersonalizeButton: React.FC<PersonalizeButtonProps> = ({ chapterId, chapterTitle }) => {
  const { user } = useAuth();
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [loading, setLoading] = useState(false);

  if (!user) {
    return null; // Don't show button if not logged in
  }

  const handlePersonalize = async () => {
    setLoading(true);
    
    // Simulate personalization (in real app, this would adjust content)
    await new Promise(resolve => setTimeout(resolve, 1000));
    
    setIsPersonalized(!isPersonalized);
    setLoading(false);
    
    // Store preference
    localStorage.setItem(`personalized_${chapterId}`, (!isPersonalized).toString());
    
    // Show success message
    if (!isPersonalized) {
      showNotification('✨ Content personalized based on your profile!');
    } else {
      showNotification('📖 Showing standard content');
    }
  };

  const getPersonalizationMessage = () => {
    if (!user.softwareBackground) {
      return "Personalize content based on your profile";
    }
    
    const messages = {
      beginner: "Show beginner-friendly explanations",
      intermediate: "Adjust for intermediate level",
      advanced: "Show advanced content",
      expert: "Enable expert mode"
    };
    
    return messages[user.softwareBackground] || "Personalize content";
  };

  return (
    <button
      className={`${styles.actionButton} ${styles.personalizeButton} ${isPersonalized ? styles.active : ''}`}
      onClick={handlePersonalize}
      disabled={loading}
    >
      <svg 
        className={styles.icon} 
        width="20" 
        height="20" 
        viewBox="0 0 20 20"
        fill="none"
        xmlns="http://www.w3.org/2000/svg"
      >
        <path 
          d="M10 2C5.58 2 2 5.58 2 10s3.58 8 8 8 8-3.58 8-8-3.58-8-8-8zm0 14c-3.31 0-6-2.69-6-6s2.69-6 6-6 6 2.69 6 6-2.69 6-6 6z" 
          fill="currentColor"
        />
        <path 
          d="M10 6c-2.21 0-4 1.79-4 4s1.79 4 4 4 4-1.79 4-4-1.79-4-4-4zm0 6c-1.1 0-2-.9-2-2s.9-2 2-2 2 .9 2 2-.9 2-2 2z" 
          fill="currentColor"
        />
      </svg>
      
      <div className={styles.buttonContent}>
        <span className={styles.buttonTitle}>
          {isPersonalized ? '✨ Personalized' : '🎯 Personalize'}
        </span>
        <span className={styles.buttonSubtitle}>
          {getPersonalizationMessage()}
        </span>
      </div>
      
      {loading && (
        <div className={styles.spinner}></div>
      )}
    </button>
  );
};

function showNotification(message: string) {
  // Create notification element
  const notification = document.createElement('div');
  notification.className = styles.notification;
  notification.textContent = message;
  document.body.appendChild(notification);
  
  // Animate in
  setTimeout(() => notification.classList.add(styles.show), 10);
  
  // Remove after 3 seconds
  setTimeout(() => {
    notification.classList.remove(styles.show);
    setTimeout(() => notification.remove(), 300);
  }, 3000);
}

export default PersonalizeButton;
