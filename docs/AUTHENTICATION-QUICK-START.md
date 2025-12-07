# 🔐 Authentication Quick Start Guide

**Time to Complete**: 20 minutes  
**Difficulty**: Beginner-friendly

---

## 🎯 Overview

This guide will help you enable the authentication system step-by-step. By the end, users will be able to:
- Sign up with email/password
- Sign in with Google or GitHub
- Complete a 6-step personalization questionnaire
- Get personalized content recommendations

---

## 📋 Prerequisites Checklist

Before starting, ensure you have:
- [ ] Node.js 18+ installed
- [ ] Git installed
- [ ] A GitHub account (you already have this!)
- [ ] A Google account
- [ ] 20 minutes of focused time

---

## 🚀 Step 1: Choose Your Database (5 minutes)

### Option A: Supabase (Recommended - Free & Easy)

**Why Supabase?**
- ✅ Free tier (500MB database)
- ✅ No credit card required
- ✅ PostgreSQL (production-ready)
- ✅ Easy setup
- ✅ Automatic backups

**Setup Steps:**

1. **Go to** [Supabase](https://supabase.com)
2. **Click** "Start your project" → Sign in with GitHub
3. **Create New Project**:
   ```
   Name: physical-ai-auth
   Database Password: [choose strong password]
   Region: [closest to you]
   ```
4. **Wait** 2 minutes for database to initialize
5. **Get Connection String**:
   - Click "Project Settings" (gear icon)
   - Click "Database"
   - Copy "Connection string" under "Connection pooling"
   - It looks like: `postgresql://postgres:[password]@[host]:5432/postgres`

6. **Save to .env**:
   ```bash
   DATABASE_URL="your_connection_string_here"
   ```

### Option B: SQLite (Testing Only)

**Good for**: Local development, testing  
**Not recommended for**: Production deployment

**Setup Steps:**

1. **Edit** `prisma/schema.prisma`:
   ```prisma
   datasource db {
     provider = "sqlite"  // Change from "postgresql"
     url      = env("DATABASE_URL")
   }
   ```

2. **Update .env**:
   ```bash
   DATABASE_URL="file:./dev.db"
   ```

---

## 🔑 Step 2: Set Up Google OAuth (5 minutes)

### Why Google OAuth?
- Most users have Google accounts
- Trusted authentication
- One-click sign-in

### Setup Steps:

1. **Go to** [Google Cloud Console](https://console.cloud.google.com/)

2. **Create Project** (or use existing):
   - Click project dropdown → "New Project"
   - Name: "Physical AI Book"
   - Click "Create"

3. **Enable Google+ API**:
   - Go to "APIs & Services" → "Library"
   - Search "Google+ API"
   - Click "Enable"

4. **Create OAuth Credentials**:
   - Go to "APIs & Services" → "Credentials"
   - Click "Create Credentials" → "OAuth client ID"
   - If prompted, configure consent screen:
     ```
     App name: Physical AI Book
     User support email: [your email]
     Developer email: [your email]
     ```
   - Choose "Web application"
   - Name: "Physical AI Web Client"

5. **Add Authorized Redirect URIs**:
   ```
   http://localhost:3000/api/auth/callback/google
   https://your-vercel-url.vercel.app/api/auth/callback/google
   ```
   ⚠️ Replace `your-vercel-url` with your actual Vercel URL

6. **Copy Credentials**:
   - Client ID: `123456789.apps.googleusercontent.com`
   - Client Secret: `GOCSPX-abc123...`

7. **Add to .env**:
   ```bash
   GOOGLE_CLIENT_ID="your_client_id_here"
   GOOGLE_CLIENT_SECRET="your_client_secret_here"
   ```

---

## 🐙 Step 3: Set Up GitHub OAuth (3 minutes)

### Why GitHub OAuth?
- Developers love GitHub
- Easy setup
- Professional user base

### Setup Steps:

1. **Go to** [GitHub Developer Settings](https://github.com/settings/developers)

2. **Click** "New OAuth App"

3. **Fill Details**:
   ```
   Application name: Physical AI Book
   Homepage URL: http://localhost:3000
   Application description: Educational platform for Physical AI
   Authorization callback URL: http://localhost:3000/api/auth/callback/github
   ```

4. **Create Application**

5. **Generate Client Secret**:
   - Click "Generate a new client secret"
   - Copy it immediately (won't be shown again!)

6. **Copy Credentials**:
   - Client ID: `Iv1.abc123...`
   - Client Secret: `ghp_abc123...`

7. **Add to .env**:
   ```bash
   GITHUB_CLIENT_ID="your_client_id_here"
   GITHUB_CLIENT_SECRET="your_client_secret_here"
   ```

8. **For Production** (after deploying):
   - Click "Edit" on your OAuth app
   - Add production callback:
     ```
     https://your-vercel-url.vercel.app/api/auth/callback/github
     ```

---

## ⚙️ Step 4: Configure Environment Variables (2 minutes)

### Your Complete .env File:

```bash
# Database
DATABASE_URL="postgresql://postgres:[password]@[host]:5432/postgres"

# Google OAuth
GOOGLE_CLIENT_ID="123456789.apps.googleusercontent.com"
GOOGLE_CLIENT_SECRET="GOCSPX-abc123..."

# GitHub OAuth
GITHUB_CLIENT_ID="Iv1.abc123..."
GITHUB_CLIENT_SECRET="ghp_abc123..."

# App Configuration
NEXTAUTH_URL="http://localhost:3000"
NEXTAUTH_SECRET="generate_random_string_here"

# Optional: For RAG Chatbot (future)
OPENAI_API_KEY="sk-..."
QDRANT_URL="https://..."
```

### Generate NEXTAUTH_SECRET:

**Option 1**: Use online generator
- Go to: https://generate-secret.vercel.app/32

**Option 2**: Use terminal
```bash
node -e "console.log(require('crypto').randomBytes(32).toString('base64'))"
```

---

## 🗄️ Step 5: Initialize Database (2 minutes)

### Run These Commands:

```bash
# 1. Generate Prisma client
npx prisma generate

# 2. Create database tables
npx prisma db push

# 3. Verify success
npx prisma studio
```

**What `prisma studio` shows:**
- User table with custom fields ✅
- Session table ✅
- Account table ✅
- All ready to use! ✅

---

## 🎨 Step 6: Enable Authentication UI (2 minutes)

### Uncomment Code in `src/theme/Root.js`:

**Find these commented lines:**
```javascript
// import { AuthProvider } from '../components/Auth/AuthProvider';
// import AuthModal from '../components/Auth/AuthModal';
```

**Change to:**
```javascript
import { AuthProvider } from '../components/Auth/AuthProvider';
import AuthModal from '../components/Auth/AuthModal';
```

**Find this commented block:**
```javascript
// useEffect(() => {
//   const hasVisited = localStorage.getItem('hasVisited');
//   if (!hasVisited) {
//     setTimeout(() => {
//       setShowAuthModal(true);
//       localStorage.setItem('hasVisited', 'true');
//     }, 2000);
//   }
// }, []);
```

**Change to:**
```javascript
useEffect(() => {
  const hasVisited = localStorage.getItem('hasVisited');
  if (!hasVisited) {
    setTimeout(() => {
      setShowAuthModal(true);
      localStorage.setItem('hasVisited', 'true');
    }, 2000);
  }
}, []);
```

**Find this commented JSX:**
```javascript
{/* <AuthProvider> */}
{children}
<SpeedInsights />
{/* <AuthModal isOpen={showAuthModal} onClose={() => setShowAuthModal(false)} /> */}
{/* </AuthProvider> */}
```

**Change to:**
```javascript
<AuthProvider>
  {children}
  <SpeedInsights />
  <AuthModal isOpen={showAuthModal} onClose={() => setShowAuthModal(false)} />
</AuthProvider>
```

---

## 🧪 Step 7: Test Locally (3 minutes)

### Start Development Server:

```bash
npm start
```

### Test Checklist:

1. **Open** http://localhost:3000
2. **Wait** 2 seconds → Auth modal should appear ✅
3. **Try Sign Up**:
   - Enter email and password
   - Click "Continue"
   - Complete 6-step questionnaire
   - Should create account ✅
4. **Try Google Sign In**:
   - Click "Continue with Google"
   - Authorize
   - Should redirect back and sign in ✅
5. **Try GitHub Sign In**:
   - Click "Continue with GitHub"
   - Authorize
   - Should redirect back and sign in ✅
6. **Check User Menu**:
   - See your avatar/name in navbar ✅
   - Click to see dropdown ✅
   - Verify profile info shown ✅
   - Sign out works ✅

---

## 🚀 Step 8: Deploy to Production (2 minutes)

### Add Environment Variables to Vercel:

1. **Go to** [Vercel Dashboard](https://vercel.com/)
2. **Select** your project
3. **Go to** "Settings" → "Environment Variables"
4. **Add each variable**:
   ```
   DATABASE_URL
   GOOGLE_CLIENT_ID
   GOOGLE_CLIENT_SECRET
   GITHUB_CLIENT_ID
   GITHUB_CLIENT_SECRET
   NEXTAUTH_URL (use your vercel URL)
   NEXTAUTH_SECRET
   ```

### Update OAuth Redirect URLs:

**Google:**
- Add: `https://your-vercel-url.vercel.app/api/auth/callback/google`

**GitHub:**
- Add: `https://your-vercel-url.vercel.app/api/auth/callback/github`

### Push to GitHub:

```bash
git add .
git commit -m "Enable authentication with full configuration"
git push origin main
```

**Vercel will auto-deploy!** ✅

---

## ✅ Success Checklist

After deployment, verify:

- [ ] Modal appears on first visit
- [ ] Email/password sign up works
- [ ] Questionnaire collects all 6 data points
- [ ] Google sign in works
- [ ] GitHub sign in works
- [ ] User menu appears in navbar
- [ ] Profile shows correct info
- [ ] Sign out works
- [ ] Sign in again works

---

## 🐛 Troubleshooting

### Issue: OAuth redirect fails

**Solution:**
- Verify callback URLs match exactly (no trailing slash)
- Check environment variables are set
- Ensure OAuth apps are enabled

### Issue: Database connection error

**Solution:**
```bash
# Verify DATABASE_URL is correct
echo $DATABASE_URL

# Regenerate Prisma client
npx prisma generate

# Push schema again
npx prisma db push
```

### Issue: Modal doesn't show

**Solution:**
- Check browser console for errors
- Verify all imports are uncommented
- Clear localStorage: `localStorage.clear()`

### Issue: "Failed to fetch" error

**Solution:**
- Ensure API routes exist in `api/auth/[...all].ts`
- Check Better Auth configuration in `lib/auth.ts`
- Verify all dependencies installed

---

## 🎉 You're Done!

Your authentication system is now:
- ✅ Fully functional
- ✅ Collecting user profiles
- ✅ Supporting 3 auth methods
- ✅ Ready for personalization
- ✅ Production-deployed

**Next**: See CONTENT-PERSONALIZATION-GUIDE.md to use this data!

---

## 📞 Quick Reference

### Useful Commands:
```bash
# View database
npx prisma studio

# Reset database
npx prisma db push --force-reset

# Generate new client
npx prisma generate

# Check environment
npm run env:check
```

### Important Files:
```
src/theme/Root.js              - Enable/disable auth
lib/auth.ts                    - Auth configuration
prisma/schema.prisma           - Database schema
.env                          - Environment variables
api/auth/[...all].ts          - Auth API routes
```

---

**Total Time**: ~20 minutes  
**Difficulty**: ✅ Beginner-friendly  
**Status**: Ready to implement!
