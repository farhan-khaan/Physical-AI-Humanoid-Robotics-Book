"""
Database module for storing conversation history in Neon Postgres
"""

from sqlalchemy import create_engine, Column, String, Text, DateTime, Integer
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from datetime import datetime
import os

DATABASE_URL = os.getenv("DATABASE_URL")

Base = declarative_base()

class Conversation(Base):
    """Store conversation history"""
    __tablename__ = "conversations"
    
    id = Column(String, primary_key=True)
    user_id = Column(String, nullable=True, index=True)
    created_at = Column(DateTime, default=datetime.utcnow, index=True)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

class Message(Base):
    """Store individual messages"""
    __tablename__ = "messages"
    
    id = Column(Integer, primary_key=True, autoincrement=True)
    conversation_id = Column(String, index=True)
    role = Column(String)  # 'user' or 'assistant'
    content = Column(Text)
    selected_text = Column(Text, nullable=True)
    chapter_id = Column(String, nullable=True)
    sources = Column(Text, nullable=True)  # JSON string
    timestamp = Column(DateTime, default=datetime.utcnow, index=True)

# Initialize database connection
engine = None
SessionLocal = None

def init_db():
    """Initialize database connection and create tables"""
    global engine, SessionLocal
    
    if not DATABASE_URL:
        print("⚠️  DATABASE_URL not set, conversation history disabled")
        return None
    
    try:
        engine = create_engine(DATABASE_URL)
        SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
        
        # Create tables
        Base.metadata.create_all(bind=engine)
        print("✅ Database initialized successfully")
        return engine
    except Exception as e:
        print(f"⚠️  Database initialization failed: {e}")
        return None

def get_db():
    """Get database session"""
    if SessionLocal is None:
        return None
    
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

def save_conversation(conversation_id: str, user_id: str = None):
    """Save a new conversation"""
    if SessionLocal is None:
        return
    
    db = SessionLocal()
    try:
        conv = Conversation(id=conversation_id, user_id=user_id)
        db.add(conv)
        db.commit()
    except Exception as e:
        print(f"Error saving conversation: {e}")
        db.rollback()
    finally:
        db.close()

def save_message(
    conversation_id: str,
    role: str,
    content: str,
    selected_text: str = None,
    chapter_id: str = None,
    sources: str = None
):
    """Save a message to the database"""
    if SessionLocal is None:
        return
    
    db = SessionLocal()
    try:
        message = Message(
            conversation_id=conversation_id,
            role=role,
            content=content,
            selected_text=selected_text,
            chapter_id=chapter_id,
            sources=sources
        )
        db.add(message)
        db.commit()
    except Exception as e:
        print(f"Error saving message: {e}")
        db.rollback()
    finally:
        db.close()

def get_conversation_history(conversation_id: str, limit: int = 10):
    """Retrieve conversation history"""
    if SessionLocal is None:
        return []
    
    db = SessionLocal()
    try:
        messages = db.query(Message).filter(
            Message.conversation_id == conversation_id
        ).order_by(Message.timestamp.desc()).limit(limit).all()
        
        return [
            {
                "role": msg.role,
                "content": msg.content,
                "timestamp": msg.timestamp.isoformat()
            }
            for msg in reversed(messages)
        ]
    except Exception as e:
        print(f"Error retrieving history: {e}")
        return []
    finally:
        db.close()
