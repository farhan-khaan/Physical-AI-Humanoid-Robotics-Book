"""
RAG Chatbot for Physical AI Book
Uses OpenAI, Qdrant, FastAPI, and Neon Postgres for intelligent Q&A
"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional
import os
from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct, Filter, FieldCondition, MatchValue
import hashlib
import json
from database import init_db, save_conversation, save_message, get_conversation_history

# Initialize FastAPI
app = FastAPI(title="Physical AI RAG Chatbot")

# CORS middleware for frontend
allowed_origins = os.getenv("ALLOWED_ORIGINS", "*").split(",")
app.add_middleware(
    CORSMiddleware,
    allow_origins=allowed_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize clients
openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# Initialize database
init_db()

COLLECTION_NAME = "physical_ai_book"
EMBEDDING_MODEL = "text-embedding-3-small"
CHAT_MODEL = "gpt-4o-mini"

# Pydantic models
class ChatMessage(BaseModel):
    role: str
    content: str

class ChatRequest(BaseModel):
    message: str
    conversation_history: List[ChatMessage] = []
    selected_text: Optional[str] = None
    chapter_id: Optional[str] = None
    user_id: Optional[str] = None

class ChatResponse(BaseModel):
    answer: str
    sources: List[dict]
    conversation_id: str

# Initialize collection
def init_collection():
    """Initialize Qdrant collection if it doesn't exist."""
    try:
        collections = qdrant_client.get_collections()
        if COLLECTION_NAME not in [c.name for c in collections.collections]:
            qdrant_client.create_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=VectorParams(size=1536, distance=Distance.COSINE)
            )
            print(f"✅ Created collection: {COLLECTION_NAME}")
        else:
            print(f"✅ Collection exists: {COLLECTION_NAME}")
    except Exception as e:
        print(f"⚠️  Collection initialization: {e}")

init_collection()

# Embedding functions
def get_embedding(text: str) -> List[float]:
    """Get embedding from OpenAI."""
    response = openai_client.embeddings.create(
        model=EMBEDDING_MODEL,
        input=text
    )
    return response.data[0].embedding

def chunk_text(text: str, chunk_size: int = 500, overlap: int = 50) -> List[str]:
    """Split text into overlapping chunks."""
    words = text.split()
    chunks = []
    
    for i in range(0, len(words), chunk_size - overlap):
        chunk = ' '.join(words[i:i + chunk_size])
        if chunk:
            chunks.append(chunk)
    
    return chunks

# Retrieval functions
def retrieve_relevant_chunks(query: str, chapter_id: Optional[str] = None, top_k: int = 5) -> List[dict]:
    """Retrieve relevant chunks from Qdrant."""
    query_embedding = get_embedding(query)
    
    # Build filter if chapter_id provided
    search_filter = None
    if chapter_id:
        search_filter = Filter(
            must=[
                FieldCondition(
                    key="chapter_id",
                    match=MatchValue(value=chapter_id)
                )
            ]
        )
    
    # Search
    results = qdrant_client.search(
        collection_name=COLLECTION_NAME,
        query_vector=query_embedding,
        limit=top_k,
        query_filter=search_filter
    )
    
    # Format results
    chunks = []
    for result in results:
        chunks.append({
            'content': result.payload.get('text', ''),
            'chapter': result.payload.get('chapter_title', 'Unknown'),
            'section': result.payload.get('section', ''),
            'score': result.score,
            'metadata': result.payload
        })
    
    return chunks

def build_context(chunks: List[dict], selected_text: Optional[str] = None) -> str:
    """Build context from retrieved chunks and selected text."""
    context_parts = []
    
    # Add selected text if provided (highest priority)
    if selected_text:
        context_parts.append(f"USER SELECTED TEXT:\n{selected_text}\n")
    
    # Add retrieved chunks
    context_parts.append("RELEVANT BOOK CONTENT:\n")
    for i, chunk in enumerate(chunks, 1):
        context_parts.append(
            f"[Source {i} - {chunk['chapter']}]\n"
            f"{chunk['content']}\n"
        )
    
    return "\n".join(context_parts)

# Chat endpoint
@app.post("/api/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """
    Main chat endpoint for RAG chatbot.
    """
    try:
        # If user selected text, prioritize that
        if request.selected_text:
            # Use selected text as primary context
            chunks = retrieve_relevant_chunks(
                request.message, 
                chapter_id=request.chapter_id,
                top_k=3
            )
            context = build_context(chunks, request.selected_text)
            
            system_prompt = f"""You are a helpful AI assistant for the Physical AI & Humanoid Robotics book.

The user has selected specific text from the book and is asking about it.

Context from the book:
{context}

Instructions:
- Answer based primarily on the SELECTED TEXT
- Use additional context if helpful
- If the question is about the selected text, focus on that
- Be specific and reference the content
- Use examples from the text when relevant
"""
        else:
            # Standard RAG retrieval
            chunks = retrieve_relevant_chunks(
                request.message,
                chapter_id=request.chapter_id,
                top_k=5
            )
            context = build_context(chunks)
            
            system_prompt = f"""You are a helpful AI assistant for the Physical AI & Humanoid Robotics book.

Context from the book:
{context}

Instructions:
- Answer based on the provided book content
- Be specific and cite chapters when relevant
- If unsure, say "I don't have information about that in the book"
- Use examples and code from the book when helpful
- Be educational and clear
"""
        
        # Build messages
        messages = [
            {"role": "system", "content": system_prompt}
        ]
        
        # Add conversation history
        for msg in request.conversation_history[-5:]:  # Last 5 messages
            messages.append({
                "role": msg.role,
                "content": msg.content
            })
        
        # Add current message
        messages.append({
            "role": "user",
            "content": request.message
        })
        
        # Get response from OpenAI
        response = openai_client.chat.completions.create(
            model=CHAT_MODEL,
            messages=messages,
            temperature=0.7,
            max_tokens=1000
        )
        
        answer = response.choices[0].message.content
        
        # Generate conversation ID
        conv_id = hashlib.md5(
            f"{request.user_id}{request.message}".encode()
        ).hexdigest()
        
        # Save conversation to database
        try:
            # Save conversation if new
            save_conversation(conv_id, request.user_id)
            
            # Save user message
            save_message(
                conversation_id=conv_id,
                role="user",
                content=request.message,
                selected_text=request.selected_text,
                chapter_id=request.chapter_id
            )
            
            # Save assistant response
            save_message(
                conversation_id=conv_id,
                role="assistant",
                content=answer,
                sources=json.dumps([
                    {
                        'chapter': chunk['chapter'],
                        'content': chunk['content'][:200],
                        'score': round(chunk['score'], 3)
                    }
                    for chunk in chunks[:3]
                ])
            )
        except Exception as db_error:
            print(f"Database save error: {db_error}")
        
        return ChatResponse(
            answer=answer,
            sources=[
                {
                    'chapter': chunk['chapter'],
                    'content': chunk['content'][:200] + '...',
                    'score': round(chunk['score'], 3)
                }
                for chunk in chunks[:3]
            ],
            conversation_id=conv_id
        )
    
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

# Health check
@app.get("/api/health")
async def health_check():
    """Health check endpoint."""
    return {
        "status": "healthy",
        "service": "Physical AI RAG Chatbot",
        "qdrant_connected": True,
        "openai_connected": True
    }

# Document embedding endpoint
@app.post("/api/embed-document")
async def embed_document(
    chapter_id: str,
    chapter_title: str,
    content: str,
    section: str = ""
):
    """Embed a document chunk into Qdrant."""
    try:
        # Chunk the content
        chunks = chunk_text(content)
        
        points = []
        for i, chunk in enumerate(chunks):
            # Generate embedding
            embedding = get_embedding(chunk)
            
            # Create point
            point_id = hashlib.md5(
                f"{chapter_id}{i}{chunk[:50]}".encode()
            ).hexdigest()
            
            point = PointStruct(
                id=point_id,
                vector=embedding,
                payload={
                    'text': chunk,
                    'chapter_id': chapter_id,
                    'chapter_title': chapter_title,
                    'section': section,
                    'chunk_index': i
                }
            )
            points.append(point)
        
        # Upload to Qdrant
        qdrant_client.upsert(
            collection_name=COLLECTION_NAME,
            points=points
        )
        
        return {
            "status": "success",
            "chunks_embedded": len(chunks),
            "chapter_id": chapter_id
        }
    
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

# Get collection stats
@app.get("/api/stats")
async def get_stats():
    """Get collection statistics."""
    try:
        collection_info = qdrant_client.get_collection(COLLECTION_NAME)
        return {
            "total_vectors": collection_info.points_count,
            "collection_name": COLLECTION_NAME
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
