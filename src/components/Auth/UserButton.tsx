import React, { useState } from 'react';
import { useAuth } from './AuthProvider';
import AuthModal from './AuthModal';
import styles from './UserButton.module.css';

const UserButton: React.FC = () => {
  const { user, loading, signOut } = useAuth();
  const [showAuthModal, setShowAuthModal] = useState(false);
  const [showDropdown, setShowDropdown] = useState(false);

  if (loading) {
    return <div className={styles.loading}>Loading...</div>;
  }

  if (!user) {
    return (
      <>
        <button 
          className={styles.signInButton}
          onClick={() => setShowAuthModal(true)}
        >
          Sign In
        </button>
        <AuthModal isOpen={showAuthModal} onClose={() => setShowAuthModal(false)} />
      </>
    );
  }

  return (
    <div className={styles.userMenu}>
      <button 
        className={styles.userButton}
        onClick={() => setShowDropdown(!showDropdown)}
      >
        {user.image ? (
          <img src={user.image} alt={user.name} className={styles.avatar} />
        ) : (
          <div className={styles.avatarPlaceholder}>
            {user.name?.charAt(0).toUpperCase() || user.email.charAt(0).toUpperCase()}
          </div>
        )}
        <span className={styles.userName}>{user.name || 'User'}</span>
        <svg 
          className={styles.chevron} 
          width="12" 
          height="12" 
          viewBox="0 0 12 12"
        >
          <path 
            fill="currentColor" 
            d="M2 4l4 4 4-4"
          />
        </svg>
      </button>

      {showDropdown && (
        <>
          <div 
            className={styles.overlay} 
            onClick={() => setShowDropdown(false)}
          />
          <div className={styles.dropdown}>
            <div className={styles.dropdownHeader}>
              <div className={styles.userInfo}>
                <strong>{user.name}</strong>
                <small>{user.email}</small>
              </div>
            </div>
            
            <div className={styles.dropdownSection}>
              <div className={styles.badge}>
                {user.experienceLevel && (
                  <span>Level: {user.experienceLevel}</span>
                )}
              </div>
              {user.preferredLanguage === 'ur' && (
                <div className={styles.languageBadge}>🇵🇰 اردو</div>
              )}
            </div>

            <div className={styles.divider} />

            <button className={styles.dropdownItem}>
              <svg width="16" height="16" viewBox="0 0 16 16">
                <path fill="currentColor" d="M8 0C3.6 0 0 3.6 0 8s3.6 8 8 8 8-3.6 8-8-3.6-8-8-8zm0 14c-3.3 0-6-2.7-6-6s2.7-6 6-6 6 2.7 6 6-2.7 6-6 6z"/>
                <path fill="currentColor" d="M8 4c-0.6 0-1 0.4-1 1v3c0 0.6 0.4 1 1 1s1-0.4 1-1V5c0-0.6-0.4-1-1-1z"/>
              </svg>
              My Profile
            </button>

            <button className={styles.dropdownItem}>
              <svg width="16" height="16" viewBox="0 0 16 16">
                <path fill="currentColor" d="M14 2H2C0.9 2 0 2.9 0 4v8c0 1.1 0.9 2 2 2h12c1.1 0 2-0.9 2-2V4c0-1.1-0.9-2-2-2zM8 11L2 7v5h12V7l-6 4z"/>
              </svg>
              Learning Progress
            </button>

            <button className={styles.dropdownItem}>
              <svg width="16" height="16" viewBox="0 0 16 16">
                <path fill="currentColor" d="M8 0C6.3 0 5 1.3 5 3s1.3 3 3 3 3-1.3 3-3-1.3-3-3-3zm0 8c-3.3 0-6 2.7-6 6v2h12v-2c0-3.3-2.7-6-6-6z"/>
              </svg>
              Settings
            </button>

            <div className={styles.divider} />

            <button 
              className={`${styles.dropdownItem} ${styles.danger}`}
              onClick={() => {
                signOut();
                setShowDropdown(false);
              }}
            >
              <svg width="16" height="16" viewBox="0 0 16 16">
                <path fill="currentColor" d="M3 0v16h10V0H3zm8 14H5V2h6v12z"/>
                <path fill="currentColor" d="M9 4H7v4H5l3 3 3-3H9z"/>
              </svg>
              Sign Out
            </button>
          </div>
        </>
      )}
    </div>
  );
};

export default UserButton;
