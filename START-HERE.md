# 🚀 START HERE - Deployment Checklist

## ⚠️ BEFORE YOU BEGIN

### Critical Security Step
**You MUST revoke the exposed API key before deploying!**

1. Go to: https://platform.openai.com/api-keys
2. Find the key you posted in chat
3. Click DELETE/REVOKE
4. Create a NEW key for production
5. Keep it secure (don't post it anywhere)

---

## 📋 Deployment Steps

### Step 1: Push to GitHub ✅
Your repo: https://github.com/farhan-khaan/Physical-AI-Humanoid-Robotics-Book.git

**Automated:**
```powershell
.\tmp_rovodev_push_to_github.ps1
```

**Manual:**
```powershell
git add .
git commit -m "Add RAG chatbot integration"
git push -u origin main
```

---

### Step 2: Deploy Frontend (Vercel) ⏱️ 5 min

1. https://vercel.com/
2. Sign in with GitHub
3. Import: `Physical-AI-Humanoid-Robotics-Book`
4. Add environment variable:
   - `REACT_APP_API_URL` = `https://placeholder.com`
5. Deploy
6. Save your URL: `https://______.vercel.app`

---

### Step 3: Deploy Backend (Render) ⏱️ 5 min

1. https://render.com/
2. Sign in with GitHub
3. New Web Service → Select your repo
4. Configure:
   - Root directory: `backend`
   - Build: `pip install -r requirements.txt`
   - Start: `uvicorn rag_chatbot:app --host 0.0.0.0 --port $PORT`
5. Add environment variables:
   - `OPENAI_API_KEY` = (your NEW key)
   - `QDRANT_URL` = (your Qdrant URL)
   - `QDRANT_API_KEY` = (your Qdrant key)
   - `ALLOWED_ORIGINS` = `https://your-vercel-url.vercel.app`
   - `PORT` = `10000`
6. Deploy
7. Save your backend URL: `https://______.onrender.com`

---

### Step 4: Connect Frontend to Backend ⏱️ 2 min

1. Go back to Vercel dashboard
2. Your project → Settings → Environment Variables
3. Update `REACT_APP_API_URL`:
   - New value: `https://your-backend.onrender.com`
4. Save
5. Deployments → Redeploy

---

### Step 5: Embed Content ⏱️ 3 min

```powershell
cd backend
python embed_all_content.py
```

Wait for: "✅ Completed! Embedded 20 files"

---

### Step 6: Test Your Site! 🎉

1. Visit: `https://your-project.vercel.app`
2. Look for chatbot button (bottom right)
3. Click and ask: "What is embodied intelligence?"
4. Select text on page → Ask about it
5. Verify sources appear

---

## ✅ Success Checklist

- [ ] Exposed API key revoked
- [ ] New production keys created
- [ ] Code pushed to GitHub
- [ ] Frontend deployed (Vercel)
- [ ] Backend deployed (Render)
- [ ] Frontend updated with backend URL
- [ ] Content embedded
- [ ] Chatbot working on live site

---

## 🐛 Quick Troubleshooting

**Chatbot not appearing?**
- Check browser console (F12)
- Verify `REACT_APP_API_URL` in Vercel
- Redeploy after adding env var

**CORS errors?**
- Check `ALLOWED_ORIGINS` in Render
- Must include your Vercel URL
- Restart backend

**No answers?**
- Verify content embedded: `/api/stats`
- Check OpenAI API key is valid
- Check backend logs in Render

---

## 📖 Detailed Guides

- **DEPLOY.md** - Quick reference
- **tmp_rovodev_deployment_guide.md** - Complete guide
- **CHATBOT-SETUP-GUIDE.md** - Full setup
- **tmp_rovodev_URGENT_SECURITY.md** - Security info

---

## 🎯 You Are Here

```
[ ] Step 1: Revoke exposed key ← DO THIS FIRST!
[ ] Step 2: Push to GitHub
[ ] Step 3: Deploy to Vercel
[ ] Step 4: Deploy to Render
[ ] Step 5: Connect frontend to backend
[ ] Step 6: Embed content
[ ] Step 7: Test & launch! 🎉
```

---

## 💰 Expected Costs

- Vercel: Free
- Render: Free (sleeps) or $7/month
- Qdrant: Free (1GB)
- Neon: Free (0.5GB)
- OpenAI: ~$10-20/month

**Total: $10-30/month**

---

## 🆘 Need Help?

Ask questions at any step. I'm here to help!

---

**Ready? Start with Step 1: Revoke that API key! 🔒**
