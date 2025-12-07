# 🤖 Physical AI RAG Chatbot Backend

FastAPI-based RAG (Retrieval-Augmented Generation) chatbot for the Physical AI & Humanoid Robotics book.

## 🏗️ Architecture

```
┌─────────────┐      ┌──────────────┐      ┌─────────────┐
│  Frontend   │─────▶│    FastAPI   │─────▶│   OpenAI    │
│  (React)    │◀─────│   Backend    │◀─────│   GPT-4o    │
└─────────────┘      └──────────────┘      └─────────────┘
                            │  │
                            │  │
                     ┌──────┘  └──────┐
                     ▼                ▼
              ┌─────────────┐  ┌─────────────┐
              │   Qdrant    │  │    Neon     │
              │   Vectors   │  │  Postgres   │
              └─────────────┘  └─────────────┘
```

## 🎯 Features

- **RAG System**: Retrieves relevant book content using vector similarity
- **Selected Text Support**: Answer questions about user-selected text
- **Chapter Filtering**: Context-aware responses based on current chapter
- **Conversation History**: Stored in Neon Postgres
- **Source Citations**: Shows where answers come from
- **Streaming Support**: Ready for real-time responses (TODO)

## 📦 Components

### `rag_chatbot.py`
Main FastAPI application with:
- `/api/chat` - Chat endpoint
- `/api/embed-document` - Embed content endpoint
- `/api/stats` - Collection statistics
- `/api/health` - Health check

### `database.py`
SQLAlchemy models for Neon Postgres:
- `conversations` table
- `messages` table
- Helper functions for saving/retrieving

### `embed_all_content.py`
Script to embed all book content:
- Reads markdown files
- Chunks text (500 words, 50 overlap)
- Generates embeddings
- Uploads to Qdrant

### `test_chatbot.py`
Comprehensive test suite:
- Environment validation
- Backend health check
- Chat functionality
- Selected text queries
- Chapter filtering

## 🚀 Quick Start

### 1. Install Dependencies

```bash
pip install -r requirements.txt
```

### 2. Configure Environment

Create `.env` file:

```bash
OPENAI_API_KEY=sk-proj-xxxxx
QDRANT_URL=https://xxxxx.cloud.qdrant.io
QDRANT_API_KEY=your-key
DATABASE_URL=postgresql://user:pass@host.neon.tech/dbname
ALLOWED_ORIGINS=http://localhost:3000,https://your-site.vercel.app
```

### 3. Start Server

```bash
python rag_chatbot.py
```

Server runs at: http://localhost:8000

### 4. Embed Content

```bash
python embed_all_content.py
```

### 5. Test

```bash
python test_chatbot.py
```

## 🔌 API Endpoints

### POST `/api/chat`

Chat with the RAG system.

**Request:**
```json
{
  "message": "What is embodied intelligence?",
  "conversation_history": [],
  "selected_text": null,
  "chapter_id": "01-embodied-intelligence",
  "user_id": "user123"
}
```

**Response:**
```json
{
  "answer": "Embodied intelligence refers to...",
  "sources": [
    {
      "chapter": "Embodied Intelligence",
      "content": "...",
      "score": 0.892
    }
  ],
  "conversation_id": "abc123"
}
```

### POST `/api/embed-document`

Embed a document into the vector database.

**Request:**
```json
{
  "chapter_id": "01-embodied-intelligence",
  "chapter_title": "What is Embodied Intelligence?",
  "content": "Full chapter content...",
  "section": "Introduction"
}
```

### GET `/api/stats`

Get collection statistics.

**Response:**
```json
{
  "total_vectors": 156,
  "collection_name": "physical_ai_book"
}
```

### GET `/api/health`

Health check endpoint.

**Response:**
```json
{
  "status": "healthy",
  "service": "Physical AI RAG Chatbot",
  "qdrant_connected": true,
  "openai_connected": true
}
```

## 🗄️ Database Schema

### conversations
| Column | Type | Description |
|--------|------|-------------|
| id | String | UUID |
| user_id | String | User identifier |
| created_at | DateTime | Creation timestamp |
| updated_at | DateTime | Last update |

### messages
| Column | Type | Description |
|--------|------|-------------|
| id | Integer | Auto-increment |
| conversation_id | String | Foreign key |
| role | String | 'user' or 'assistant' |
| content | Text | Message content |
| selected_text | Text | Selected text (optional) |
| chapter_id | String | Chapter context |
| sources | Text | JSON sources |
| timestamp | DateTime | Message time |

## 🧪 Testing

Run the test suite:

```bash
python test_chatbot.py
```

Tests include:
- ✅ Environment variables
- ✅ Backend health
- ✅ Collection stats
- ✅ Chat endpoint
- ✅ Selected text queries
- ✅ Chapter filtering

## 🚀 Deployment

### Option 1: Render.com

1. Push to GitHub
2. Connect to Render
3. Use `render.yaml` config
4. Set environment variables
5. Deploy!

### Option 2: Docker

```bash
docker build -t rag-chatbot .
docker run -p 8000:8000 --env-file .env rag-chatbot
```

### Option 3: Vercel (Serverless)

Use `vercel.json` for serverless deployment.

## ⚙️ Configuration

### Chunk Settings

In `rag_chatbot.py`:

```python
def chunk_text(text: str, chunk_size: int = 500, overlap: int = 50):
    # Adjust chunk_size and overlap for your content
```

### Embedding Model

```python
EMBEDDING_MODEL = "text-embedding-3-small"  # 1536 dimensions
# Or use: "text-embedding-3-large" for better quality (3072 dimensions)
```

### Chat Model

```python
CHAT_MODEL = "gpt-4o-mini"  # Fast and cost-effective
# Or use: "gpt-4o" for better quality
```

### Retrieval Settings

```python
def retrieve_relevant_chunks(query: str, chapter_id: Optional[str] = None, top_k: int = 5):
    # Adjust top_k for more/fewer chunks
```

## 📊 Cost Estimates

### OpenAI API Costs

**Embeddings** (`text-embedding-3-small`):
- $0.02 per 1M tokens
- Full book (~50k words) ≈ $0.10 one-time

**Chat** (`gpt-4o-mini`):
- Input: $0.15 per 1M tokens
- Output: $0.60 per 1M tokens
- Average conversation: $0.001 - $0.005

**Monthly estimate**: $10-50 for moderate usage (500-2000 conversations)

### Storage Costs

- **Qdrant**: Free (1GB)
- **Neon**: Free (0.5GB, 100hrs compute)

## 🐛 Troubleshooting

### "OPENAI_API_KEY is not set"
- Check `.env` file exists
- Verify key format: `sk-proj-...`
- Restart server

### "Collection not found"
- Run `embed_all_content.py`
- Check Qdrant connection
- Verify collection name

### "Database connection failed"
- DATABASE_URL is optional
- Check Neon connection string
- Verify SSL mode: `?sslmode=require`

### Slow responses
- Check OpenAI API status
- Reduce `top_k` in retrieval
- Use faster model (`gpt-4o-mini`)

### CORS errors
- Update `ALLOWED_ORIGINS` in `.env`
- Restart backend server
- Check frontend URL matches

## 📚 Development

### Add new endpoint

```python
@app.post("/api/new-endpoint")
async def new_endpoint(request: Request):
    # Your logic here
    return {"status": "success"}
```

### Add database table

In `database.py`:

```python
class NewTable(Base):
    __tablename__ = "new_table"
    id = Column(Integer, primary_key=True)
    # Add columns
```

### Customize prompts

In `rag_chatbot.py`, edit the system prompts:

```python
system_prompt = f"""You are a helpful AI assistant...
Custom instructions here...
"""
```

## 🔐 Security

- Never commit `.env` files
- Use environment variables for secrets
- Implement rate limiting (see `slowapi`)
- Validate user inputs
- Use HTTPS in production

## 📖 Resources

- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [OpenAI API Reference](https://platform.openai.com/docs)
- [Qdrant Documentation](https://qdrant.tech/documentation/)
- [Neon Documentation](https://neon.tech/docs)

## 🤝 Contributing

Improvements welcome! Areas for contribution:
- Streaming responses
- Rate limiting
- Caching
- Advanced retrieval strategies
- Multi-language support

## 📄 License

MIT License - See main project LICENSE file
