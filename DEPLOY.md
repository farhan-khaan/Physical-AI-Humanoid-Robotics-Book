# 🚀 Deployment Instructions

## Quick Deploy to GitHub → Vercel

Your repository: https://github.com/farhan-khaan/Physical-AI-Humanoid-Robotics-Book.git

---

## Step 1: Push to GitHub

### Automated (Recommended)
```powershell
.\tmp_rovodev_push_to_github.ps1
```

### Manual Commands
```bash
# Clean up temporary files
Remove-Item tmp_rovodev_*.* -Force

# Stage all changes
git add .

# Commit
git commit -m "Add RAG chatbot with OpenAI, Qdrant, and Neon integration"

# Push to GitHub
git push -u origin main
```

---

## Step 2: Deploy Frontend to Vercel

1. **Go to Vercel**: https://vercel.com/
2. **Sign in** with GitHub
3. **Import Project**:
   - Click "Add New Project"
   - Select: `Physical-AI-Humanoid-Robotics-Book`
4. **Configure**:
   - Framework: Docusaurus (auto-detected)
   - Build Command: `npm run build`
   - Output Directory: `build`
5. **Add Environment Variable**:
   ```
   Name: REACT_APP_API_URL
   Value: (Add backend URL after Step 3)
   ```
   For now, use: `https://placeholder.com`
6. **Deploy** → Wait 2-3 minutes

Your site will be live at: `https://your-project.vercel.app`

---

## Step 3: Deploy Backend to Render.com

1. **Go to Render**: https://render.com/
2. **Sign up/Login** with GitHub
3. **Create Web Service**:
   - Click "New +" → "Web Service"
   - Connect your repository
4. **Configure**:
   ```
   Name: physical-ai-chatbot
   Region: (Choose closest)
   Branch: main
   Root Directory: backend
   Runtime: Python 3
   Build Command: pip install -r requirements.txt
   Start Command: uvicorn rag_chatbot:app --host 0.0.0.0 --port $PORT
   ```
5. **Add Environment Variables**:
   ```
   OPENAI_API_KEY=your-new-key-here
   QDRANT_URL=your-qdrant-url
   QDRANT_API_KEY=your-qdrant-key
   DATABASE_URL=your-neon-url (optional)
   ALLOWED_ORIGINS=https://your-project.vercel.app
   PORT=10000
   ```
6. **Deploy** → Wait 3-5 minutes

Copy your backend URL: `https://physical-ai-chatbot.onrender.com`

---

## Step 4: Update Frontend with Backend URL

1. Go back to **Vercel Dashboard**
2. Select your project
3. Go to **Settings** → **Environment Variables**
4. Edit `REACT_APP_API_URL`:
   ```
   REACT_APP_API_URL=https://physical-ai-chatbot.onrender.com
   ```
5. **Save**
6. Go to **Deployments** → Click "..." → **Redeploy**

---

## Step 5: Embed Content

### Option A: Local (Recommended)
```powershell
cd backend
python embed_all_content.py
```

### Option B: Render Shell
1. Go to Render dashboard
2. Click on your service
3. Go to "Shell" tab
4. Run: `python embed_all_content.py`

---

## Step 6: Test Your Live Site! 🎉

1. Visit: `https://your-project.vercel.app`
2. Look for chatbot button (bottom right)
3. Click and ask: "What is embodied intelligence?"
4. Select text on page → Ask about it
5. Check sources are displayed

### Verify Backend
```
https://physical-ai-chatbot.onrender.com/api/health
```

Should return:
```json
{
  "status": "healthy",
  "qdrant_connected": true,
  "openai_connected": true
}
```

---

## ✅ Success Checklist

- [ ] Code pushed to GitHub
- [ ] Frontend deployed on Vercel
- [ ] Backend deployed on Render
- [ ] Environment variables configured
- [ ] Backend URL updated in Vercel
- [ ] Content embedded to Qdrant
- [ ] Chatbot working on live site
- [ ] Selected text feature working

---

## 🐛 Troubleshooting

### "Chatbot not appearing"
- Check browser console (F12)
- Verify `REACT_APP_API_URL` in Vercel
- Redeploy frontend after adding env var

### "CORS errors"
- Check `ALLOWED_ORIGINS` in Render backend
- Should include your Vercel URL
- Restart backend service

### "Backend 503 errors"
- Render free tier sleeps after 15min inactivity
- First request takes 30-60s to wake up
- Consider paid tier ($7/month) for production

### "No answers from chatbot"
- Verify content is embedded
- Check: `/api/stats` endpoint
- Should show 150+ vectors
- Re-run `embed_all_content.py`

---

## 💰 Cost Summary

**Free Tier (Testing)**
- Vercel: Free
- Render: Free (with sleep)
- Qdrant: Free (1GB)
- Neon: Free (0.5GB)
- OpenAI: ~$10-20/month
**Total: ~$10-20/month**

**Production Tier**
- Vercel: Free
- Render: $7/month (no sleep)
- Qdrant: Free
- Neon: Free
- OpenAI: ~$50/month
**Total: ~$60/month**

---

## 📞 Need Help?

- **Full Guide**: See `tmp_rovodev_deployment_guide.md`
- **Vercel Docs**: https://vercel.com/docs
- **Render Docs**: https://render.com/docs
- **Troubleshooting**: See CHATBOT-SETUP-GUIDE.md

---

## 🎊 You're Live!

Once deployed, share your site:
- Live URL: `https://your-project.vercel.app`
- GitHub: `https://github.com/farhan-khaan/Physical-AI-Humanoid-Robotics-Book`

Update your README.md with the live demo link!

---

**Made with ❤️ - Happy Deploying! 🚀**
