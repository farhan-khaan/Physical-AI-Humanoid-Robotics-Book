# 🚀 Quick Start: RAG Chatbot in 15 Minutes

Get your RAG chatbot running locally in 15 minutes!

## Prerequisites

- Python 3.11+
- Node.js 20+
- OpenAI API key
- Qdrant Cloud account (free)
- Neon Postgres account (free, optional)

---

## Step 1: Get API Keys (5 minutes)

### OpenAI API Key
1. Go to https://platform.openai.com/api-keys
2. Click "Create new secret key"
3. Copy the key (starts with `sk-proj-...`)

### Qdrant Cloud
1. Go to https://cloud.qdrant.io/
2. Sign up → Create cluster (Free tier)
3. Copy your cluster URL and API key

### Neon Postgres (Optional)
1. Go to https://neon.tech/
2. Sign up → Create project
3. Copy connection string

---

## Step 2: Configure Environment (2 minutes)

### Backend Configuration

Create `backend/.env`:

```bash
OPENAI_API_KEY=sk-proj-your-key-here
QDRANT_URL=https://your-cluster.cloud.qdrant.io
QDRANT_API_KEY=your-qdrant-key
DATABASE_URL=postgresql://user:pass@host.neon.tech/dbname
ALLOWED_ORIGINS=http://localhost:3000
```

### Frontend Configuration

Create `.env.local`:

```bash
REACT_APP_API_URL=http://localhost:8000
```

---

## Step 3: Install & Run Backend (3 minutes)

```bash
# Navigate to backend
cd backend

# Install dependencies
pip install -r requirements.txt

# Start the server
python rag_chatbot.py
```

You should see:
```
✅ Collection exists: physical_ai_book
✅ Database initialized successfully
INFO:     Started server process
INFO:     Uvicorn running on http://0.0.0.0:8000
```

---

## Step 4: Embed Content (3 minutes)

Open a **new terminal**:

```bash
cd backend
python embed_all_content.py
```

This will:
- Read all markdown files
- Generate embeddings
- Upload to Qdrant

Output:
```
🚀 Starting to embed Physical AI book content...
Found 20 markdown files
✅ Embedded 156 chunks
```

---

## Step 5: Run Frontend (2 minutes)

Open a **new terminal**:

```bash
# From project root
npm install
npm start
```

Site opens at http://localhost:3000

---

## Step 6: Test the Chatbot! 🎉

1. Navigate to any chapter page
2. Click the chatbot button (bottom right) 🤖
3. Try asking:
   - "What is embodied intelligence?"
   - "Explain the sense-think-act loop"
   - "What sensors do humanoid robots use?"
4. **Select text** on the page and ask about it!

---

## ✅ Success Checklist

- [ ] Backend running at http://localhost:8000
- [ ] Frontend running at http://localhost:3000
- [ ] Content embedded (156+ vectors in Qdrant)
- [ ] Chatbot widget appears on site
- [ ] Can ask questions and get answers
- [ ] Selected text queries work

---

## 🐛 Troubleshooting

### Backend won't start
- Check Python version: `python --version` (need 3.11+)
- Verify API keys in `backend/.env`
- Check ports: `lsof -i :8000` (macOS/Linux)

### No answers from chatbot
- Verify content is embedded: `curl http://localhost:8000/api/stats`
- Check browser console for errors
- Verify `REACT_APP_API_URL` in `.env.local`

### CORS errors
- Add `http://localhost:3000` to `ALLOWED_ORIGINS` in `backend/.env`
- Restart backend

---

## 🚀 Next Steps

1. **Deploy Backend**: See `CHATBOT-SETUP-GUIDE.md` for Render deployment
2. **Deploy Frontend**: Push to GitHub → Vercel auto-deploys
3. **Customize**: Edit `ChatbotWidget.tsx` for styling
4. **Monitor**: Check Neon DB for conversation history

---

## 📊 Verify Everything Works

### Check Backend Health
```bash
curl http://localhost:8000/api/health
```

### Check Embedded Content
```bash
curl http://localhost:8000/api/stats
```

### Test Chat API
```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What is embodied intelligence?",
    "conversation_history": [],
    "selected_text": null,
    "chapter_id": null,
    "user_id": "test"
  }'
```

---

## 💡 Tips

- **Cost**: First 1M tokens free on OpenAI, then ~$0.02/1k
- **Performance**: Embeddings cached, chat responses ~1-2 seconds
- **Updates**: Re-run `embed_all_content.py` after content changes
- **Database**: Works without Neon, just won't save history

---

**🎉 Congratulations! Your RAG chatbot is running!**

For full deployment guide, see: `CHATBOT-SETUP-GUIDE.md`
