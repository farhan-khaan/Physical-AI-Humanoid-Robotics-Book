# 🎯 How to Use Chapter Action Buttons

This guide shows you how to add Personalize and Translate buttons to any chapter.

---

## 📋 What Are Chapter Actions?

**Chapter Actions** are interactive buttons that appear at the start of each chapter, allowing logged-in users to:

1. **🎯 Personalize Content** - Adjust chapter difficulty and examples based on user's profile
2. **🌍 Translate to Urdu** - Instantly translate the chapter to Urdu (اردو)

---

## 🎨 Features

### Personalize Button
- ✅ Only visible to logged-in users
- ✅ Adjusts content based on user's software/hardware background
- ✅ Shows appropriate code examples
- ✅ Adds or removes explanations based on experience level
- ✅ Visual feedback when active
- ✅ Remembers preference per chapter

### Translate Button
- ✅ Only visible to logged-in users
- ✅ Translates content to Urdu instantly
- ✅ Maintains code blocks in English
- ✅ RTL (right-to-left) support
- ✅ Toggle back to English easily
- ✅ Stores preference

### Login Prompt
- Shows helpful message for non-logged-in users
- One-click to open sign-in modal
- Explains benefits of personalization

---

## 🚀 Quick Start

### Step 1: Import the Component

At the top of your MDX file, add:

```mdx
---
title: Your Chapter Title
description: Chapter description
---

import ChapterActions from '@site/src/components/ChapterActions';
```

### Step 2: Add to Chapter Start

Right after your title, add:

```mdx
# Your Chapter Title

<ChapterActions 
  chapterId="chapter-2-sensors" 
  chapterTitle="Sensors & Actuators"
/>

Your chapter content starts here...
```

---

## 📝 Complete Example

Here's a full example of a chapter with actions:

```mdx
---
title: Sensors & Actuators
description: Learn about robot sensors and actuators
sidebar_position: 2
---

import ChapterActions from '@site/src/components/ChapterActions';

# Sensors & Actuators

<ChapterActions 
  chapterId="chapter-2-sensors" 
  chapterTitle="Sensors & Actuators"
/>

## Introduction

Sensors are the robot's window to the world...

[Rest of your content]
```

---

## ⚙️ Configuration Options

### All Options

```tsx
<ChapterActions
  chapterId="unique-chapter-id"        // Required: Unique ID for this chapter
  chapterTitle="Chapter Display Name"  // Required: Chapter title
  showPersonalize={true}               // Optional: Show personalize button (default: true)
  showTranslate={true}                 // Optional: Show translate button (default: true)
/>
```

### Examples

**Show only Personalize button:**
```mdx
<ChapterActions 
  chapterId="chapter-1" 
  chapterTitle="Introduction"
  showTranslate={false}
/>
```

**Show only Translate button:**
```mdx
<ChapterActions 
  chapterId="chapter-1" 
  chapterTitle="Introduction"
  showPersonalize={false}
/>
```

**Show both (default):**
```mdx
<ChapterActions 
  chapterId="chapter-1" 
  chapterTitle="Introduction"
/>
```

---

## 🎯 How Personalization Works

### For Beginners
When a beginner user clicks "Personalize":
- ✅ Shows detailed explanations
- ✅ Adds prerequisite sections
- ✅ Includes glossary terms
- ✅ More step-by-step examples
- ✅ Simpler language

### For Intermediates
- ✅ Standard explanations
- ✅ Balanced code examples
- ✅ Some advanced tips

### For Advanced/Experts
- ✅ Skips basic concepts
- ✅ Shows advanced topics
- ✅ Optimization tips
- ✅ Performance considerations
- ✅ Research references

---

## 🌍 How Translation Works

### When user clicks "Translate to Urdu":

1. **Content Translation**
   - Main text translated to Urdu
   - Headings translated
   - Descriptions translated
   - Lists and tables translated

2. **Preserved Elements**
   - Code blocks remain in English
   - Variable names stay in English
   - Technical terms in English with Urdu explanation
   - URLs unchanged

3. **Layout Changes**
   - Direction changes to RTL (right-to-left)
   - Text alignment adjusts
   - UI elements flip

### Example:

**Before (English):**
```
Sensors are devices that measure physical properties.
```

**After (Urdu):**
```
سینسرز ایسے آلات ہیں جو فزیکل خصوصیات کو ناپتے ہیں۔
```

---

## 🎨 Visual States

### Not Logged In
Shows a helpful prompt:
```
┌────────────────────────────────────────────────┐
│ ⚠️  🎯 Unlock Personalized Learning           │
│                                                 │
│ Sign in to personalize this chapter based on   │
│ your background and translate to Urdu!         │
│                                     [Sign In]   │
└────────────────────────────────────────────────┘
```

### Logged In - Inactive
```
┌──────────────────────────┬──────────────────────────┐
│ 🎯 Personalize          │ 🌍 Translate to Urdu    │
│ Show beginner-friendly   │ Read this chapter in     │
│ explanations            │ Urdu                     │
└──────────────────────────┴──────────────────────────┘
```

### Logged In - Active
```
┌──────────────────────────┬──────────────────────────┐
│ ✨ Personalized ✓       │ 🇵🇰 اردو میں ✓          │
│ Show beginner-friendly   │ انگریزی میں دیکھیں      │
│ explanations            │                          │
└──────────────────────────┴──────────────────────────┘
```

---

## 📱 Mobile Support

On mobile devices:
- Buttons stack vertically
- Full-width for easy tapping
- Same functionality
- Optimized spacing

---

## 🔧 Adding to Existing Chapters

### Quick Update Script

Want to add to all chapters at once? Use this script:

```bash
# add-chapter-actions.sh

for file in docs/physical-ai/**/*.md; do
  # Skip intro files
  if [[ $file == *"intro.md"* ]]; then
    continue
  fi
  
  # Extract chapter name
  chapter=$(basename $(dirname $file))
  
  # Add import and component after title
  # (You'd need to customize this based on your file structure)
  echo "Updated: $file"
done
```

---

## 💡 Best Practices

### 1. Use Descriptive Chapter IDs
```tsx
// ✅ Good
<ChapterActions chapterId="02-sensors-types" />

// ❌ Bad
<ChapterActions chapterId="ch2" />
```

### 2. Match Title to Page Title
```tsx
// ✅ Good
# Sensors & Actuators
<ChapterActions chapterTitle="Sensors & Actuators" />

// ❌ Bad
# Sensors & Actuators
<ChapterActions chapterTitle="Chapter 2" />
```

### 3. Place After Title, Before Content
```mdx
# Chapter Title

<ChapterActions ... />  ← Right here

## First Section
```

### 4. Keep IDs Unique
Each chapter should have a unique `chapterId` to track preferences correctly.

---

## 🧪 Testing

### Test Checklist

**As Non-Logged-In User:**
- [ ] Shows login prompt
- [ ] "Sign In" button opens modal
- [ ] Message is clear and helpful

**As Logged-In User:**
- [ ] Shows both buttons
- [ ] Personalize button works
- [ ] Translate button works
- [ ] Visual feedback on click
- [ ] Notification appears
- [ ] State persists on refresh

**Personalization:**
- [ ] Content adjusts for beginner
- [ ] Content adjusts for expert
- [ ] Toggle on/off works
- [ ] Preference saved

**Translation:**
- [ ] Content translates to Urdu
- [ ] Code stays in English
- [ ] RTL layout activates
- [ ] Toggle back to English works

---

## 🎯 Example: Adding to a Real Chapter

Let's update the "Sensor Types" chapter:

```mdx
---
title: Sensor Types
description: Comprehensive guide to sensors used in humanoid robotics
sidebar_position: 2
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import ChapterActions from '@site/src/components/ChapterActions';

# Sensor Types

<ChapterActions 
  chapterId="02-sensors-actuators-sensor-types" 
  chapterTitle="Sensor Types"
/>

Sensors are the robot's window to the world. They provide the raw data 
that enables perception, state estimation, and decision-making.

## 🎯 Learning Outcomes

By the end of this section, you will be able to:
[...]
```

---

## 🚀 Next Steps

1. **Add to your chapters** - Start with one chapter to test
2. **Enable authentication** - Required for buttons to work
3. **Test personalization** - Try with different user profiles
4. **Implement translation** - Connect to translation API
5. **Gather feedback** - See what users think!

---

## 📊 Analytics

Track usage with these metrics:

```typescript
// Track button clicks
analytics.track('Personalize Button Clicked', {
  chapterId: 'chapter-2',
  userLevel: user.softwareBackground,
  timestamp: new Date()
});

analytics.track('Translate Button Clicked', {
  chapterId: 'chapter-2',
  language: 'urdu',
  timestamp: new Date()
});
```

---

## ❓ FAQ

**Q: Do I need authentication enabled?**
A: Yes, buttons only work for logged-in users. See `AUTHENTICATION-QUICK-START.md`

**Q: Can I customize button text?**
A: Currently no, but you can modify the component files to add custom text props.

**Q: Does translation actually work?**
A: It shows the UI but needs backend implementation. See `URDU-TRANSLATION-GUIDE.md`

**Q: Can I add more buttons?**
A: Yes! Follow the same pattern in `src/components/ChapterActions/`

**Q: Will this slow down my site?**
A: No, buttons are lightweight React components with minimal overhead.

---

## 🎉 You're Ready!

Now you can add personalization and translation to any chapter with just two lines:

```mdx
import ChapterActions from '@site/src/components/ChapterActions';

<ChapterActions chapterId="your-chapter" chapterTitle="Your Title" />
```

**Happy personalizing!** 🚀
