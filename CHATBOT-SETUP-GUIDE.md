# 🤖 RAG Chatbot Setup Guide

Complete guide to set up and deploy the integrated RAG chatbot for the Physical AI book.

## 📋 Overview

This chatbot uses:
- **OpenAI GPT-4o-mini** for natural language understanding
- **OpenAI text-embedding-3-small** for vector embeddings
- **Qdrant Cloud (Free Tier)** for vector storage and similarity search
- **Neon Serverless Postgres** for conversation history
- **FastAPI** for the backend API
- **React/TypeScript** for the frontend widget

## 🎯 Features

✅ Answer questions about book content using RAG  
✅ Support for selected text queries  
✅ Chapter-specific context filtering  
✅ Conversation history with Neon Postgres  
✅ Source citations with relevance scores  
✅ Real-time streaming responses  
✅ User authentication integration  

---

## 🚀 Step 1: Set Up Qdrant Cloud (Vector Database)

### 1.1 Create Qdrant Account
1. Go to [Qdrant Cloud](https://cloud.qdrant.io/)
2. Sign up for a free account
3. Create a new cluster (Free Tier: 1GB storage)

### 1.2 Get Credentials
1. Note your **Cluster URL**: `https://xxxxx.cloud.qdrant.io`
2. Generate an **API Key** from the dashboard
3. Save these for later

---

## 🗄️ Step 2: Set Up Neon Postgres (Conversation History)

### 2.1 Create Neon Account
1. Go to [Neon](https://neon.tech/)
2. Sign up for a free account
3. Create a new project: `physical-ai-chatbot`

### 2.2 Get Connection String
1. Go to your project dashboard
2. Copy the **Connection String**:
   ```
   postgresql://user:password@ep-xxxxx.us-east-2.aws.neon.tech/physical_ai_chatbot?sslmode=require
   ```
3. Save this for later

### 2.3 Initialize Database
The database tables will be created automatically when you first run the backend.

---

## 🔑 Step 3: Get OpenAI API Key

1. Go to [OpenAI Platform](https://platform.openai.com/)
2. Create an account or sign in
3. Navigate to **API Keys**
4. Create a new API key
5. **Important**: Copy it immediately (you won't see it again)
6. Save it securely

---

## ⚙️ Step 4: Configure Environment Variables

### 4.1 Backend Configuration

Create `backend/.env`:

```bash
# OpenAI Configuration
OPENAI_API_KEY=sk-proj-xxxxxxxxxxxxxxxxxxxxx

# Qdrant Configuration
QDRANT_URL=https://xxxxx.cloud.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key

# Neon Database
DATABASE_URL=postgresql://user:password@ep-xxxxx.us-east-2.aws.neon.tech/physical_ai_chatbot?sslmode=require

# CORS Settings
ALLOWED_ORIGINS=https://your-site.vercel.app,http://localhost:3000

# API Configuration
PORT=8000
```

### 4.2 Frontend Configuration

Create `.env.local`:

```bash
# Development
REACT_APP_API_URL=http://localhost:8000

# For production (set in Vercel):
# REACT_APP_API_URL=https://your-backend.onrender.com
```

---

## 📚 Step 5: Embed Book Content

### 5.1 Install Backend Dependencies

```bash
cd backend
pip install -r requirements.txt
```

### 5.2 Start Backend Server

```bash
# From backend directory
python rag_chatbot.py
```

Server will start at `http://localhost:8000`

### 5.3 Verify Backend Health

```bash
curl http://localhost:8000/api/health
```

Expected response:
```json
{
  "status": "healthy",
  "service": "Physical AI RAG Chatbot",
  "qdrant_connected": true,
  "openai_connected": true
}
```

### 5.4 Embed All Content

```bash
# From backend directory
python embed_all_content.py
```

This will:
- Read all markdown files from `docs/physical-ai/`
- Chunk the content (500 words with 50-word overlap)
- Generate embeddings using OpenAI
- Upload to Qdrant

Expected output:
```
🚀 Starting to embed Physical AI book content...

Found 20 markdown files

Processing: docs/physical-ai/01-embodied-intelligence/intro.md
  ✅ Embedded 5 chunks
Processing: docs/physical-ai/01-embodied-intelligence/what-is-embodied-intelligence.md
  ✅ Embedded 8 chunks
...

✅ Completed! Embedded 20 files

📊 Collection Stats:
   Total vectors: 156
   Collection: physical_ai_book
```

---

## 🖥️ Step 6: Test Locally

### 6.1 Start Backend

```bash
cd backend
python rag_chatbot.py
```

### 6.2 Start Frontend

```bash
# From project root
npm start
```

### 6.3 Test the Chatbot

1. Open `http://localhost:3000`
2. Navigate to any chapter
3. Click the chatbot button (bottom right)
4. Try these questions:
   - "What is embodied intelligence?"
   - "Explain the sense-think-act loop"
   - "What sensors do humanoid robots use?"
5. Select text on the page and ask about it

---

## ☁️ Step 7: Deploy Backend

### Option A: Deploy to Render.com (Recommended)

1. Go to [Render](https://render.com/)
2. Sign up/Login
3. Click **New** → **Web Service**
4. Connect your GitHub repository
5. Configure:
   - **Name**: `physical-ai-chatbot`
   - **Environment**: `Python 3`
   - **Build Command**: `pip install -r requirements.txt`
   - **Start Command**: `uvicorn rag_chatbot:app --host 0.0.0.0 --port $PORT`
   - **Root Directory**: `backend`
6. Add environment variables (from Step 4.1)
7. Deploy!

Your backend URL: `https://physical-ai-chatbot.onrender.com`

### Option B: Deploy with Docker

```bash
cd backend
docker build -t physical-ai-chatbot .
docker run -p 8000:8000 --env-file .env physical-ai-chatbot
```

---

## 🌐 Step 8: Deploy Frontend to Vercel

### 8.1 Set Environment Variables in Vercel

1. Go to your project in Vercel
2. Settings → Environment Variables
3. Add:
   ```
   REACT_APP_API_URL=https://your-backend.onrender.com
   ```

### 8.2 Deploy

```bash
# From project root
npm run build
vercel --prod
```

Or push to GitHub (auto-deploys if connected)

---

## 🧪 Step 9: Test Production

1. Visit your deployed site
2. Open the chatbot
3. Ask questions
4. Select text and query
5. Check conversation history in Neon dashboard

---

## 📊 Monitoring & Maintenance

### Check Qdrant Stats

```bash
curl https://your-backend.onrender.com/api/stats
```

### View Conversations in Neon

1. Go to Neon dashboard
2. Open SQL Editor
3. Run:
```sql
-- View recent conversations
SELECT * FROM conversations ORDER BY created_at DESC LIMIT 10;

-- View messages
SELECT * FROM messages ORDER BY timestamp DESC LIMIT 20;

-- Count total conversations
SELECT COUNT(*) FROM conversations;
```

### Re-embed Content (After Updates)

```bash
cd backend
python embed_all_content.py
```

---

## 🔧 Troubleshooting

### Chatbot Not Appearing

**Problem**: Widget doesn't show on site  
**Solution**: 
1. Check `src/theme/Root.js` imports ChatbotWidget
2. Clear browser cache
3. Check console for errors

### "Connection Failed" Error

**Problem**: Can't reach backend API  
**Solution**:
1. Verify backend is running
2. Check `REACT_APP_API_URL` is set correctly
3. Check CORS settings in backend
4. Verify network connectivity

### Empty Responses

**Problem**: Chatbot returns no answers  
**Solution**:
1. Check if content is embedded: `curl http://localhost:8000/api/stats`
2. Verify Qdrant connection
3. Check OpenAI API key is valid
4. Review backend logs

### Database Connection Error

**Problem**: "DATABASE_URL not set" warning  
**Solution**:
1. Verify DATABASE_URL in backend/.env
2. Check Neon database is active
3. Test connection string
4. **Note**: Chatbot works without DB, just no history saved

### Embedding Fails

**Problem**: `embed_all_content.py` errors  
**Solution**:
1. Check OpenAI API key
2. Verify Qdrant credentials
3. Check markdown files exist in `docs/physical-ai/`
4. Review file permissions

---

## 💡 Usage Tips

### For Users

1. **Ask Specific Questions**: "How does PID control work?" vs "Tell me about control"
2. **Use Selected Text**: Highlight text → Ask "Explain this"
3. **Reference Chapters**: "In Chapter 2, what sensors are mentioned?"
4. **Follow Up**: Continue conversation for deeper understanding

### For Admins

1. **Update Content**: Re-run `embed_all_content.py` after changes
2. **Monitor Costs**: Track OpenAI API usage
3. **Review Conversations**: Check Neon DB for user insights
4. **Optimize Chunks**: Adjust chunk size in `rag_chatbot.py` (line 83)

---

## 📈 Cost Estimates (Monthly)

### Free Tier
- **Qdrant**: Free (1GB)
- **Neon**: Free (0.5GB storage, 100hrs compute)
- **OpenAI**: Pay-as-you-go
  - Embeddings: ~$0.02 per 1M tokens (~$0.10 for full book)
  - Chat: ~$0.15 per 1M tokens input, ~$0.60 per 1M tokens output
  - Estimated: $5-20/month for moderate usage

### Recommended Tier (Growing)
- **Qdrant**: Free → $25/month (10GB)
- **Neon**: Free → $19/month (10GB storage)
- **OpenAI**: $50/month budget
- **Total**: ~$100/month for 1000+ conversations

---

## 🎓 Advanced Features

### Add Streaming Responses

Update `ChatbotWidget.tsx` to use `fetch` with streaming:

```typescript
const response = await fetch(`${API_URL}/api/chat-stream`, {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({ message: input })
});

const reader = response.body.getReader();
// Handle streaming...
```

### Add Rate Limiting

Install: `pip install slowapi`

```python
from slowapi import Limiter
from slowapi.util import get_remote_address

limiter = Limiter(key_func=get_remote_address)

@app.post("/api/chat")
@limiter.limit("10/minute")
async def chat(request: Request):
    # ...
```

### Add User Feedback

Track thumbs up/down in Neon:

```sql
ALTER TABLE messages ADD COLUMN feedback VARCHAR(10);
```

---

## 📞 Support

- **Issues**: [GitHub Issues](https://github.com/your-repo/issues)
- **Docs**: See README.md
- **OpenAI**: [platform.openai.com/docs](https://platform.openai.com/docs)
- **Qdrant**: [qdrant.tech/documentation](https://qdrant.tech/documentation/)
- **Neon**: [neon.tech/docs](https://neon.tech/docs)

---

## ✅ Checklist

- [ ] Qdrant Cloud account created
- [ ] Neon Postgres database created
- [ ] OpenAI API key obtained
- [ ] Backend `.env` configured
- [ ] Frontend `.env.local` configured
- [ ] Backend dependencies installed
- [ ] Backend running locally
- [ ] Content embedded to Qdrant
- [ ] Frontend tested locally
- [ ] Backend deployed to Render
- [ ] Frontend deployed to Vercel
- [ ] Production chatbot tested
- [ ] Monitoring set up

---

**🎉 Congratulations! Your RAG chatbot is now live!**

Users can now ask questions about the Physical AI book and get intelligent, context-aware answers with source citations.
