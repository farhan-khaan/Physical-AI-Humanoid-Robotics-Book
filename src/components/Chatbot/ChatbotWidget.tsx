import React, { useState, useEffect, useRef } from 'react';
import { useAuth } from '../Auth/AuthProvider';
import styles from './ChatbotWidget.module.css';

interface Message {
  role: 'user' | 'assistant';
  content: string;
  sources?: Array<{
    chapter: string;
    content: string;
    score: number;
  }>;
}

const ChatbotWidget: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [loading, setLoading] = useState(false);
  const [selectedText, setSelectedText] = useState<string>('');
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const { user } = useAuth();

  // Scroll to bottom when messages change
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Listen for text selection
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();
      
      if (text && text.length > 10 && text.length < 5000) {
        setSelectedText(text);
        
        // Show notification
        showSelectionNotification(text);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, []);

  const showSelectionNotification = (text: string) => {
    // Only show if chatbot is open
    if (!isOpen) return;
    
    const notification = document.createElement('div');
    notification.className = styles.selectionNotification;
    notification.innerHTML = `
      <div class="${styles.notificationContent}">
        <svg width="16" height="16" viewBox="0 0 16 16">
          <path fill="currentColor" d="M8 0C3.6 0 0 3.6 0 8s3.6 8 8 8 8-3.6 8-8-3.6-8-8-8zm1 12H7V7h2v5zm0-6H7V4h2v2z"/>
        </svg>
        <span>Text selected! Ask me about it.</span>
      </div>
    `;
    document.body.appendChild(notification);
    
    setTimeout(() => notification.classList.add(styles.show), 10);
    setTimeout(() => {
      notification.classList.remove(styles.show);
      setTimeout(() => notification.remove(), 300);
    }, 3000);
  };

  const getCurrentChapterId = () => {
    // Extract chapter ID from URL
    const path = window.location.pathname;
    const match = path.match(/docs\/physical-ai\/([^/]+)/);
    return match ? match[1] : null;
  };

  const sendMessage = async () => {
    if (!input.trim() || loading) return;

    const userMessage: Message = {
      role: 'user',
      content: input
    };

    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setLoading(true);

    try {
      // Get API URL from environment or use default
      const apiUrl = typeof window !== 'undefined' && (window as any).REACT_APP_API_URL 
        ? (window as any).REACT_APP_API_URL
        : process.env.REACT_APP_API_URL || 'http://localhost:8000';
      
      const response = await fetch(`${apiUrl}/api/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: input,
          conversation_history: messages,
          selected_text: selectedText || null,
          chapter_id: getCurrentChapterId(),
          user_id: user?.id || null
        }),
      });

      if (!response.ok) {
        throw new Error('Chat API failed');
      }

      const data = await response.json();

      const assistantMessage: Message = {
        role: 'assistant',
        content: data.answer,
        sources: data.sources
      };

      setMessages(prev => [...prev, assistantMessage]);
      
      // Clear selected text after using it
      if (selectedText) {
        setSelectedText('');
      }

    } catch (error) {
      console.error('Chat error:', error);
      
      const errorMessage: Message = {
        role: 'assistant',
        content: "Sorry, I'm having trouble connecting. Please try again later."
      };
      
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  const clearSelectedText = () => {
    setSelectedText('');
  };

  const quickQuestions = [
    "What is embodied intelligence?",
    "Explain the sense-think-act loop",
    "What sensors do humanoid robots use?",
    "How does a PID controller work?",
    "What is sim-to-real transfer?"
  ];

  return (
    <>
      {/* Chat Widget Button */}
      <button
        className={`${styles.chatButton} ${isOpen ? styles.open : ''}`}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Open chat"
      >
        {isOpen ? (
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none">
            <path d="M19 6.41L17.59 5 12 10.59 6.41 5 5 6.41 10.59 12 5 17.59 6.41 19 12 13.41 17.59 19 19 17.59 13.41 12z" fill="currentColor"/>
          </svg>
        ) : (
          <>
            <svg width="24" height="24" viewBox="0 0 24 24" fill="none">
              <path d="M20 2H4c-1.1 0-2 .9-2 2v18l4-4h14c1.1 0 2-.9 2-2V4c0-1.1-.9-2-2-2z" fill="currentColor"/>
            </svg>
            {messages.length > 0 && (
              <span className={styles.badge}>{messages.length}</span>
            )}
          </>
        )}
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <div className={styles.headerContent}>
              <div className={styles.botAvatar}>🤖</div>
              <div>
                <h3>Physical AI Assistant</h3>
                <p>Ask me anything about the book</p>
              </div>
            </div>
            <button onClick={() => setIsOpen(false)} className={styles.closeButton}>
              ×
            </button>
          </div>

          {/* Selected Text Banner */}
          {selectedText && (
            <div className={styles.selectedTextBanner}>
              <div className={styles.selectedTextContent}>
                <svg width="16" height="16" viewBox="0 0 16 16">
                  <path fill="currentColor" d="M14 2H2v12h12V2zM4 12V4h8v8H4z"/>
                </svg>
                <div>
                  <strong>Text selected</strong>
                  <p>{selectedText.substring(0, 80)}...</p>
                </div>
              </div>
              <button onClick={clearSelectedText} className={styles.clearButton}>
                ×
              </button>
            </div>
          )}

          {/* Messages */}
          <div className={styles.chatMessages}>
            {messages.length === 0 && (
              <div className={styles.welcomeMessage}>
                <div className={styles.welcomeIcon}>👋</div>
                <h4>Hi! I'm your Physical AI assistant</h4>
                <p>I can help you with:</p>
                <ul>
                  <li>📚 Explaining concepts from the book</li>
                  <li>💻 Understanding code examples</li>
                  <li>🎯 Finding specific topics</li>
                  <li>✨ Answering questions about selected text</li>
                </ul>
                
                <div className={styles.quickQuestions}>
                  <p><strong>Try asking:</strong></p>
                  {quickQuestions.map((q, i) => (
                    <button
                      key={i}
                      onClick={() => setInput(q)}
                      className={styles.quickQuestion}
                    >
                      {q}
                    </button>
                  ))}
                </div>
              </div>
            )}

            {messages.map((msg, idx) => (
              <div key={idx} className={`${styles.message} ${styles[msg.role]}`}>
                {msg.role === 'assistant' && (
                  <div className={styles.avatar}>🤖</div>
                )}
                
                <div className={styles.messageContent}>
                  <div className={styles.messageText}>
                    {msg.content}
                  </div>
                  
                  {msg.sources && msg.sources.length > 0 && (
                    <div className={styles.sources}>
                      <details>
                        <summary>📚 Sources ({msg.sources.length})</summary>
                        <div className={styles.sourcesList}>
                          {msg.sources.map((source, i) => (
                            <div key={i} className={styles.source}>
                              <strong>{source.chapter}</strong>
                              <p>{source.content}</p>
                              <span className={styles.relevance}>
                                Relevance: {Math.round(source.score * 100)}%
                              </span>
                            </div>
                          ))}
                        </div>
                      </details>
                    </div>
                  )}
                </div>
                
                {msg.role === 'user' && user?.image && (
                  <img src={user.image} alt={user.name} className={styles.avatar} />
                )}
              </div>
            ))}

            {loading && (
              <div className={`${styles.message} ${styles.assistant}`}>
                <div className={styles.avatar}>🤖</div>
                <div className={styles.messageContent}>
                  <div className={styles.typingIndicator}>
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Input */}
          <div className={styles.chatInput}>
            <textarea
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder={selectedText ? "Ask about the selected text..." : "Ask anything about Physical AI..."}
              rows={1}
              disabled={loading}
            />
            <button
              onClick={sendMessage}
              disabled={!input.trim() || loading}
              className={styles.sendButton}
            >
              <svg width="20" height="20" viewBox="0 0 20 20" fill="none">
                <path d="M2 10l16-8-8 16-2-8-6-2z" fill="currentColor"/>
              </svg>
            </button>
          </div>
        </div>
      )}
    </>
  );
};

export default ChatbotWidget;
