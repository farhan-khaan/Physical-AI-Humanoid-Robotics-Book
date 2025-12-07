import React, { useState } from 'react';
import { useAuth } from '../Auth/AuthProvider';
import PersonalizeButton from './PersonalizeButton';
import TranslateButton from './TranslateButton';
import AuthModal from '../Auth/AuthModal';
import styles from './ChapterActions.module.css';

interface ChapterActionsProps {
  chapterId: string;
  chapterTitle: string;
  showPersonalize?: boolean;
  showTranslate?: boolean;
}

const ChapterActions: React.FC<ChapterActionsProps> = ({
  chapterId,
  chapterTitle,
  showPersonalize = true,
  showTranslate = true,
}) => {
  const { user } = useAuth();
  const [showAuthModal, setShowAuthModal] = useState(false);

  // If user is not logged in, show login prompt
  if (!user) {
    return (
      <>
        <div className={styles.loginPrompt}>
          <svg width="24" height="24" viewBox="0 0 24 24" fill="currentColor">
            <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm1 15h-2v-2h2v2zm0-4h-2V7h2v6z"/>
          </svg>
          
          <div className={styles.loginPromptContent}>
            <h4>🎯 Unlock Personalized Learning</h4>
            <p>
              Sign in to personalize this chapter based on your background and translate to Urdu instantly!
            </p>
          </div>
          
          <button 
            className={styles.loginButton}
            onClick={() => setShowAuthModal(true)}
          >
            Sign In
          </button>
        </div>
        
        <AuthModal isOpen={showAuthModal} onClose={() => setShowAuthModal(false)} />
      </>
    );
  }

  // If user is logged in, show action buttons
  return (
    <div className={styles.actionsContainer}>
      {showPersonalize && (
        <PersonalizeButton chapterId={chapterId} chapterTitle={chapterTitle} />
      )}
      {showTranslate && (
        <TranslateButton chapterId={chapterId} chapterTitle={chapterTitle} />
      )}
    </div>
  );
};

export default ChapterActions;
