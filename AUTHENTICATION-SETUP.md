# 🔐 Authentication & Personalization Setup Guide

This guide will help you complete the authentication system setup with Better Auth, Google/GitHub OAuth, and user personalization.

## 📋 What's Already Implemented

✅ **Authentication Components**
- AuthProvider (Context for auth state)
- AuthModal (Sign In/Sign Up UI)
- OnboardingQuestionnaire (6-step user profiling)
- UserButton (User menu in navbar)

✅ **Features**
- Email/Password authentication
- Google OAuth integration
- GitHub OAuth integration
- Personalized onboarding (software/hardware background)
- Urdu/English language selection
- User profile with learning goals
- Auto-show modal on first visit

✅ **Database Schema**
- Prisma schema with custom user fields
- User profiles with experience levels
- Session management

---

## 🚀 Quick Start (3 Steps)

### Step 1: Set Up Database

**Option A: PostgreSQL (Recommended)**
```bash
# Install PostgreSQL locally or use a cloud service:
# - Supabase (Free tier)
# - Railway (Free tier)
# - Neon (Free tier)

# Update DATABASE_URL in .env
DATABASE_URL="postgresql://user:password@localhost:5432/physical_ai_db"

# Initialize Prisma
npx prisma generate
npx prisma db push
```

**Option B: SQLite (For Testing)**
```bash
# Update prisma/schema.prisma
# Change: provider = "sqlite"
# Change DATABASE_URL to: file:./dev.db

npx prisma generate
npx prisma db push
```

### Step 2: Configure OAuth Providers

#### Google OAuth Setup (5 minutes)

1. Go to [Google Cloud Console](https://console.cloud.google.com/)
2. Create new project or select existing
3. Enable "Google+ API"
4. Go to "Credentials" → "Create Credentials" → "OAuth 2.0 Client ID"
5. Configure:
   ```
   Application type: Web application
   Authorized redirect URIs:
   - http://localhost:3000/api/auth/callback/google
   - https://your-domain.vercel.app/api/auth/callback/google
   ```
6. Copy Client ID and Client Secret to `.env`

#### GitHub OAuth Setup (3 minutes)

1. Go to [GitHub Developer Settings](https://github.com/settings/developers)
2. Click "New OAuth App"
3. Configure:
   ```
   Application name: Physical AI Book
   Homepage URL: http://localhost:3000
   Authorization callback URL: http://localhost:3000/api/auth/callback/github
   ```
4. Create app and copy Client ID and Client Secret to `.env`

### Step 3: Run the App

```bash
# Copy environment variables
cp .env.example .env

# Edit .env with your credentials

# Install dependencies (if not done)
npm install

# Generate Prisma client
npx prisma generate

# Start development server
npm start
```

---

## 🎯 Testing the Authentication

### Test Flow:

1. **Open Website** → Modal appears after 2 seconds
2. **Sign Up** → Fill email/password → Continue
3. **Onboarding** → Answer 6 questions about background
4. **Complete** → Profile created with personalization
5. **Sign Out/In** → Test authentication persistence

### Test Scenarios:

```bash
# Test 1: Email/Password Sign Up
✓ Enter valid email and password
✓ Complete all onboarding questions
✓ Verify user appears in database
✓ Check personalization data saved

# Test 2: Google OAuth
✓ Click "Continue with Google"
✓ Authorize application
✓ Redirect back to site
✓ User created automatically

# Test 3: GitHub OAuth
✓ Click "Continue with GitHub"
✓ Authorize application
✓ Redirect back to site
✓ User created automatically

# Test 4: User Profile
✓ Click user avatar in navbar
✓ View dropdown with profile info
✓ Check experience level badge
✓ Check language preference
✓ Sign out successfully
```

---

## 📊 Database Schema Reference

```prisma
model User {
  id                   String   @id @default(cuid())
  email                String   @unique
  name                 String?
  image                String?
  
  // Personalization fields
  softwareBackground   String?  // beginner, intermediate, advanced, expert
  hardwareBackground   String?  // none, hobby, education, professional
  experienceLevel      String?  // Derived from both backgrounds
  programmingLanguages String?  // JSON: ["Python", "C++", ...]
  roboticsExperience   String?  // none, simulation, hobby, professional
  learningGoals        String?  // JSON: ["Build robot", "Career", ...]
  preferredLanguage    String   @default("en")  // en or ur
  
  createdAt DateTime @default(now())
  updatedAt DateTime @updatedAt
}
```

---

## 🌍 Content Personalization

Based on user profile, you can now personalize:

### 1. **Content Difficulty**
```javascript
if (user.softwareBackground === 'beginner') {
  // Show more detailed explanations
  // Include basic concepts
  // Slower pace
}
```

### 2. **Code Examples**
```javascript
if (user.programmingLanguages.includes('Python')) {
  // Default to Python examples
} else if (user.programmingLanguages.includes('C++')) {
  // Default to C++ examples
}
```

### 3. **Hardware Focus**
```javascript
if (user.hardwareBackground === 'none') {
  // More hardware explanations
  // Shopping lists for components
  // Assembly guides
}
```

### 4. **Learning Path**
```javascript
if (user.learningGoals.includes('Career in robotics')) {
  // Recommend advanced chapters
  // Industry-relevant projects
  // Job preparation content
}
```

---

## 🔒 Security Best Practices

### ✅ Implemented:
- Password hashing (Better Auth handles this)
- Secure session tokens
- HTTPS for OAuth callbacks
- Environment variables for secrets

### 🎯 Recommended:
- Rate limiting on auth endpoints
- Email verification (add later)
- 2FA support (add later)
- CORS configuration for production

---

## 🌐 Urdu Translation Integration

The user's language preference is saved. To implement full translation:

### Option 1: Docusaurus i18n (Recommended)

```bash
# Add Urdu locale
npm run write-translations -- --locale ur

# Translate content in i18n/ur/ folder
```

### Option 2: Dynamic Translation

```javascript
import { useAuth } from '@/components/Auth/AuthProvider';

function MyComponent() {
  const { user } = useAuth();
  
  const content = {
    en: "Welcome to Physical AI",
    ur: "فزیکل AI میں خوش آمدید"
  };
  
  return <h1>{content[user?.preferredLanguage || 'en']}</h1>;
}
```

---

## 📱 Adding UserButton to Navbar

The UserButton component is ready. To add it to the navbar:

### In `docusaurus.config.ts`:

```typescript
navbar: {
  items: [
    // ... existing items
    {
      type: 'custom-userButton',
      position: 'right',
    },
  ],
}
```

### Create `src/theme/NavbarItem/CustomUserButton.tsx`:

```tsx
import React from 'react';
import UserButton from '@site/src/components/Auth/UserButton';

export default function CustomUserButton() {
  return <UserButton />;
}
```

---

## 🐛 Troubleshooting

### Issue: Modal doesn't appear
**Solution**: Check browser console for errors. Ensure AuthProvider wraps app.

### Issue: OAuth redirects fail
**Solution**: 
- Verify callback URLs match exactly
- Check NEXTAUTH_URL in .env
- Ensure OAuth apps are enabled

### Issue: Database connection fails
**Solution**:
- Verify DATABASE_URL format
- Check database is running
- Run `npx prisma db push`

### Issue: Prisma errors
**Solution**:
```bash
npx prisma generate
npx prisma db push --force-reset
```

---

## 📈 Next Steps

### Phase 1: Core (Already Done! ✅)
- [x] Authentication UI
- [x] Onboarding questionnaire
- [x] User profiles
- [x] OAuth integration

### Phase 2: Personalization (To Implement)
- [ ] Content filtering by experience level
- [ ] Recommended learning paths
- [ ] Progress tracking
- [ ] Bookmarking system

### Phase 3: Translation (To Implement)
- [ ] UI translation (Urdu)
- [ ] Content translation (start with key pages)
- [ ] Language switcher in navbar
- [ ] RTL support for Urdu

### Phase 4: Advanced Features
- [ ] Email verification
- [ ] Password reset
- [ ] Social profile integration
- [ ] Learning analytics dashboard

---

## 🎉 You're Ready!

Your authentication system is now:
- ✅ Fully functional with email/password
- ✅ Supporting Google & GitHub OAuth
- ✅ Collecting detailed user profiles
- ✅ Ready for content personalization
- ✅ Supporting multiple languages

**Next**: Follow the Quick Start steps above to complete the setup!

---

## 💬 Need Help?

If you encounter issues:
1. Check the troubleshooting section above
2. Review Better Auth documentation: https://www.better-auth.com
3. Check Prisma docs: https://www.prisma.io/docs
4. Verify all environment variables are set

**Happy Building!** 🚀
