# 🎯 Content Personalization Guide

**Goal**: Use the user profile data from onboarding to deliver personalized learning experiences.

---

## 📊 Available User Data

After onboarding, you have access to:

```typescript
interface UserProfile {
  // Basic info
  id: string;
  email: string;
  name: string;
  
  // Background (from questionnaire)
  softwareBackground: 'beginner' | 'intermediate' | 'advanced' | 'expert';
  hardwareBackground: 'none' | 'hobby' | 'education' | 'professional';
  experienceLevel: string;  // Computed from above
  
  // Skills
  programmingLanguages: string[];  // ['Python', 'C++', 'JavaScript', ...]
  roboticsExperience: 'none' | 'simulation' | 'hobby' | 'professional';
  
  // Goals
  learningGoals: string[];  // ['Build a robot', 'Career in robotics', ...]
  
  // Preferences
  preferredLanguage: 'en' | 'ur';
}
```

---

## 🎨 Personalization Strategies

### 1. **Content Difficulty Adjustment**

#### Beginner Users
```typescript
// Show more detailed explanations
// Include basic prerequisites
// Add glossary definitions
// Slower pacing

if (user.softwareBackground === 'beginner') {
  return (
    <>
      <AdditionalContext>
        <h4>📚 Background: What is a PID Controller?</h4>
        <p>A PID controller is like a thermostat for your heating system...</p>
      </AdditionalContext>
      
      <MainContent>
        {/* Regular content with extra explanations */}
      </MainContent>
      
      <Glossary terms={['controller', 'feedback', 'setpoint']} />
    </>
  );
}
```

#### Expert Users
```typescript
// Skip basics
// Show advanced topics
// Focus on optimization
// Faster pacing

if (user.softwareBackground === 'expert') {
  return (
    <>
      <MainContent>
        {/* Skip prerequisites, jump to advanced */}
      </MainContent>
      
      <AdvancedTopics>
        <h4>🚀 Advanced: Adaptive Control</h4>
        <p>Consider model-predictive control for...</p>
      </AdvancedTopics>
    </>
  );
}
```

### 2. **Code Example Preferences**

#### Based on Programming Languages
```typescript
function CodeExample({ pythonCode, cppCode, user }) {
  // Default tab based on user preference
  const defaultTab = user.programmingLanguages.includes('Python') 
    ? 'python' 
    : user.programmingLanguages.includes('C++')
    ? 'cpp'
    : 'python';  // fallback
  
  return (
    <Tabs groupId="code" defaultValue={defaultTab}>
      <TabItem value="python" label="Python">
        {pythonCode}
      </TabItem>
      <TabItem value="cpp" label="C++">
        {cppCode}
      </TabItem>
    </Tabs>
  );
}
```

#### Single Language Users
```typescript
// If user only knows Python, hide C++ tabs
if (user.programmingLanguages.length === 1 && 
    user.programmingLanguages[0] === 'Python') {
  return (
    <CodeBlock language="python">
      {pythonCode}
    </CodeBlock>
  );
}
```

### 3. **Hardware-Focused Content**

#### No Hardware Experience
```typescript
if (user.hardwareBackground === 'none') {
  return (
    <>
      <HardwareIntro>
        <h4>🔧 Hardware You'll Need</h4>
        <ShoppingList>
          <li>Arduino Uno ($25) - Where to buy →</li>
          <li>Servo motors (pack of 5, $20)</li>
          <li>Breadboard and jumper wires ($15)</li>
        </ShoppingList>
        <AssemblyGuide />
      </HardwareIntro>
      
      <MainContent />
    </>
  );
}
```

#### Professional Hardware Background
```typescript
if (user.hardwareBackground === 'professional') {
  return (
    <>
      <MainContent />
      
      <ProductionNotes>
        <h4>⚙️ Production Considerations</h4>
        <ul>
          <li>EMI compliance</li>
          <li>Thermal management</li>
          <li>Safety certifications</li>
        </ul>
      </ProductionNotes>
    </>
  );
}
```

### 4. **Learning Path Recommendations**

#### Based on Robotics Experience
```typescript
function RecommendedPath({ user }) {
  if (user.roboticsExperience === 'none') {
    return (
      <LearningPath>
        <h3>📚 Your Recommended Learning Path</h3>
        <Step number={1} completed>
          Chapter 1: Embodied Intelligence
        </Step>
        <Step number={2} current>
          Chapter 2: Sensors & Actuators ← Start here!
        </Step>
        <Step number={3}>
          Chapter 3: Simulation
        </Step>
        <Tip>💡 Focus on understanding concepts before implementation</Tip>
      </LearningPath>
    );
  }
  
  if (user.roboticsExperience === 'professional') {
    return (
      <LearningPath>
        <h3>🚀 Fast Track for Professionals</h3>
        <p>Skip to advanced topics:</p>
        <QuickLinks>
          <li><Link to="/control-strategies/learned-control">RL Control</Link></li>
          <li><Link to="/simulation/sim-to-real">Sim-to-Real</Link></li>
          <li><Link to="/capstone/examples">Advanced Projects</Link></li>
        </QuickLinks>
      </LearningPath>
    );
  }
}
```

#### Based on Learning Goals
```typescript
function PersonalizedProjects({ user }) {
  const projects = {
    'Build a humanoid robot': [
      'Bipedal Walking',
      'Object Manipulation',
      'Full Humanoid Task Execution'
    ],
    'Career in robotics': [
      'RL-Based Locomotion',
      'Multi-Robot Coordination',
      'Vision-based Navigation'
    ],
    'Academic research': [
      'Dynamic Reaching',
      'Sensor Fusion System',
      'Advanced Control Architectures'
    ]
  };
  
  const recommended = user.learningGoals
    .flatMap(goal => projects[goal] || [])
    .filter((v, i, a) => a.indexOf(v) === i);  // unique
  
  return (
    <div>
      <h3>🎯 Recommended Projects for You</h3>
      {recommended.map(project => (
        <ProjectCard key={project} name={project} />
      ))}
    </div>
  );
}
```

---

## 💡 Implementation Examples

### Example 1: Smart Content Component

```typescript
// src/components/PersonalizedContent.tsx

import React from 'react';
import { useAuth } from '@/components/Auth/AuthProvider';

interface Props {
  beginnerContent?: React.ReactNode;
  intermediateContent?: React.ReactNode;
  advancedContent?: React.ReactNode;
  expertContent?: React.ReactNode;
}

export function PersonalizedContent({
  beginnerContent,
  intermediateContent,
  advancedContent,
  expertContent
}: Props) {
  const { user } = useAuth();
  
  if (!user) return beginnerContent || null;
  
  const contentMap = {
    beginner: beginnerContent,
    intermediate: intermediateContent || beginnerContent,
    advanced: advancedContent || intermediateContent || beginnerContent,
    expert: expertContent || advancedContent || intermediateContent
  };
  
  return <>{contentMap[user.softwareBackground] || beginnerContent}</>;
}

// Usage in MDX:
// <PersonalizedContent
//   beginnerContent={<BeginnerExplanation />}
//   expertContent={<AdvancedTopics />}
// />
```

### Example 2: Difficulty Badge

```typescript
// src/components/DifficultyBadge.tsx

export function DifficultyBadge({ level }: { level: string }) {
  const { user } = useAuth();
  
  // Show if content matches or is one level above user
  const shouldShow = !user || 
    level === user.softwareBackground ||
    getNextLevel(user.softwareBackground) === level;
  
  if (!shouldShow) return null;
  
  const colors = {
    beginner: 'green',
    intermediate: 'blue',
    advanced: 'orange',
    expert: 'red'
  };
  
  return (
    <span className={`badge badge-${colors[level]}`}>
      {level.toUpperCase()}
    </span>
  );
}
```

### Example 3: Personalized Sidebar

```typescript
// src/components/PersonalizedSidebar.tsx

export function PersonalizedSidebar() {
  const { user } = useAuth();
  
  const chapters = [
    { 
      id: 1, 
      name: 'Embodied Intelligence', 
      difficulty: 'beginner',
      recommended: true  // Always recommended
    },
    { 
      id: 2, 
      name: 'Sensors & Actuators', 
      difficulty: 'beginner',
      recommended: user?.hardwareBackground === 'none'
    },
    { 
      id: 3, 
      name: 'Simulation', 
      difficulty: 'intermediate',
      recommended: user?.roboticsExperience === 'simulation'
    },
    { 
      id: 4, 
      name: 'Control Strategies', 
      difficulty: 'advanced',
      recommended: user?.learningGoals?.includes('Career in robotics')
    },
    { 
      id: 5, 
      name: 'Capstone Project', 
      difficulty: 'advanced',
      recommended: true
    }
  ];
  
  return (
    <nav>
      {chapters.map(chapter => (
        <div key={chapter.id}>
          <Link to={`/chapter-${chapter.id}`}>
            {chapter.name}
            {chapter.recommended && <span>⭐</span>}
          </Link>
          <DifficultyBadge level={chapter.difficulty} />
        </div>
      ))}
    </nav>
  );
}
```

### Example 4: Progress Tracking

```typescript
// src/components/ProgressTracker.tsx

export function ProgressTracker() {
  const { user } = useAuth();
  const [progress, setProgress] = useState<Record<string, number>>({});
  
  useEffect(() => {
    if (user) {
      loadProgress(user.id).then(setProgress);
    }
  }, [user]);
  
  const chapters = [1, 2, 3, 4, 5];
  const totalProgress = Object.values(progress).reduce((a, b) => a + b, 0) / chapters.length;
  
  return (
    <div className="progress-tracker">
      <h3>Your Progress</h3>
      <ProgressBar value={totalProgress} />
      
      <div className="chapters">
        {chapters.map(ch => (
          <ChapterProgress
            key={ch}
            chapter={ch}
            progress={progress[ch] || 0}
          />
        ))}
      </div>
      
      <NextRecommendation user={user} progress={progress} />
    </div>
  );
}
```

---

## 🎯 Personalization Scenarios

### Scenario 1: Complete Beginner

**Profile:**
- Software: Beginner
- Hardware: None
- Languages: None
- Robotics: None

**Personalization:**
```typescript
{
  // Show basic prerequisites
  showPrerequisites: true,
  
  // Recommend starting with Python
  defaultCodeLanguage: 'python',
  
  // Include hardware shopping lists
  includeHardwareGuides: true,
  
  // Suggest beginner projects
  recommendedProjects: ['Balance Controller', 'Object Tracking'],
  
  // Slower pacing
  chaptersPerWeek: 1,
  
  // Extra help
  showGlossary: true,
  showTutorialVideos: true
}
```

### Scenario 2: Software Developer (New to Robotics)

**Profile:**
- Software: Advanced
- Hardware: Hobby
- Languages: Python, JavaScript
- Robotics: None

**Personalization:**
```typescript
{
  // Skip basic programming concepts
  showPrerequisites: false,
  
  // Focus on robotics-specific concepts
  emphasize: ['sensors', 'control', 'ROS'],
  
  // Use Python by default
  defaultCodeLanguage: 'python',
  
  // Intermediate projects
  recommendedProjects: ['Autonomous Navigation', 'Object Manipulation'],
  
  // Faster pacing
  chaptersPerWeek: 2,
  
  // Show real-world applications
  includeIndustryExamples: true
}
```

### Scenario 3: Robotics Professional

**Profile:**
- Software: Expert
- Hardware: Professional
- Languages: Python, C++, ROS
- Robotics: Professional

**Personalization:**
```typescript
{
  // Skip all basics
  showPrerequisites: false,
  skipChapter1: true,
  
  // Advanced topics only
  emphasize: ['learned-control', 'sim-to-real', 'production'],
  
  // C++ by default (more professional)
  defaultCodeLanguage: 'cpp',
  
  // Expert projects
  recommendedProjects: ['RL Locomotion', 'Multi-Robot Systems'],
  
  // Fast track
  chaptersPerWeek: 3,
  
  // Industry focus
  showProductionConsiderations: true,
  showScalabilityNotes: true
}
```

---

## 🔧 Implementation Checklist

### Phase 1: Basic Personalization (Quick Wins)
- [ ] Default code language based on user preference
- [ ] Show/hide beginner explanations
- [ ] Recommended projects based on goals
- [ ] Learning path suggestions

### Phase 2: Content Filtering
- [ ] Filter chapters by difficulty
- [ ] Show relevant exercises only
- [ ] Hardware guides for beginners
- [ ] Advanced topics for experts

### Phase 3: Progress Tracking
- [ ] Track completed chapters
- [ ] Track completed exercises
- [ ] Calculate overall progress
- [ ] Suggest next chapter

### Phase 4: Smart Recommendations
- [ ] "Next for you" suggestions
- [ ] Related content recommendations
- [ ] Project difficulty matching
- [ ] Time estimates based on level

### Phase 5: Advanced Features
- [ ] Adaptive difficulty (changes based on performance)
- [ ] Learning style detection
- [ ] Personalized assessments
- [ ] Certificate generation

---

## 📊 Analytics to Track

### User Engagement
```typescript
interface UserAnalytics {
  // Time spent
  timeOnSite: number;
  timePerChapter: Record<string, number>;
  
  // Completion
  chaptersCompleted: number[];
  exercisesCompleted: number[];
  projectsCompleted: number[];
  
  // Interaction
  codeExamplesCopied: number;
  tabSwitches: number;
  searchQueries: string[];
  
  // Preferences
  preferredCodeLanguage: string;
  darkModeUsage: number;
  mobileVsDesktop: number;
}
```

### Personalization Effectiveness
```typescript
interface PersonalizationMetrics {
  // Before personalization
  averageTimeToComplete: number;
  dropoffRate: number;
  satisfactionScore: number;
  
  // After personalization
  personalizedTimeToComplete: number;
  personalizedDropoffRate: number;
  personalizedSatisfactionScore: number;
  
  // Impact
  improvement: {
    timeReduction: number;  // e.g., -30%
    dropoffReduction: number;  // e.g., -50%
    satisfactionIncrease: number;  // e.g., +40%
  };
}
```

---

## 🎨 UI Components Library

### 1. Personalized Welcome Message

```typescript
function WelcomeMessage({ user }) {
  const messages = {
    beginner: "Welcome! We'll start with the basics and build up gradually.",
    intermediate: "Great to have you! We'll focus on robotics concepts.",
    advanced: "Welcome! Let's dive into advanced topics.",
    expert: "Welcome back! Fast-track to cutting-edge robotics."
  };
  
  return (
    <div className="welcome-banner">
      <h2>Hello, {user.name}! 👋</h2>
      <p>{messages[user.softwareBackground]}</p>
      <ProgressSummary user={user} />
    </div>
  );
}
```

### 2. Skill-Based Navigation

```typescript
function SkillBasedNav({ user }) {
  const showChapter = (difficulty) => {
    const levels = ['beginner', 'intermediate', 'advanced', 'expert'];
    const userLevel = levels.indexOf(user.softwareBackground);
    const chapterLevel = levels.indexOf(difficulty);
    
    return chapterLevel <= userLevel + 1;  // Show current + 1 level ahead
  };
  
  return (
    <nav>
      {chapters.map(ch => 
        showChapter(ch.difficulty) && <ChapterLink chapter={ch} />
      )}
    </nav>
  );
}
```

### 3. Dynamic Exercise Difficulty

```typescript
function Exercise({ exercise, user }) {
  const difficulty = calculateDifficulty(exercise, user);
  
  return (
    <div className={`exercise difficulty-${difficulty}`}>
      <h4>{exercise.title}</h4>
      <DifficultyIndicator level={difficulty} />
      
      {difficulty === 'hard' && user.softwareBackground === 'beginner' && (
        <HintSection hints={exercise.hints} />
      )}
      
      <ExerciseContent {...exercise} />
      
      {difficulty === 'easy' && user.softwareBackground === 'expert' && (
        <BonusChallenge challenge={exercise.bonusChallenge} />
      )}
    </div>
  );
}
```

---

## 🚀 Quick Implementation Guide

### Step 1: Add PersonalizedContent Component (10 min)

```typescript
// src/components/PersonalizedContent.tsx
// Copy code from Example 1 above
```

### Step 2: Use in MDX Files (5 min)

```mdx
---
title: PID Control
---

import { PersonalizedContent } from '@site/src/components/PersonalizedContent';

<PersonalizedContent
  beginnerContent={
    <>
      <h3>What is PID Control?</h3>
      <p>Think of it like a thermostat...</p>
    </>
  }
  expertContent={
    <>
      <h3>Advanced PID Techniques</h3>
      <p>Consider cascade control for...</p>
    </>
  }
/>
```

### Step 3: Test Personalization (5 min)

```bash
# Sign in with different profiles
# Verify content changes based on profile
```

---

## ✅ Benefits Summary

### For Beginners:
- ✅ Less overwhelming
- ✅ Clear starting point
- ✅ Extra support
- ✅ Confidence building

### For Experts:
- ✅ Skip redundant content
- ✅ Focus on advanced topics
- ✅ Faster progress
- ✅ Challenge appropriate

### For All Users:
- ✅ Relevant content
- ✅ Better engagement
- ✅ Higher completion rate
- ✅ Personalized experience

---

**Next Steps:**
1. Review authentication setup (AUTHENTICATION-QUICK-START.md)
2. Enable authentication
3. Start implementing personalization features
4. Track analytics to measure impact

---

**Time to Implement**: 1-2 hours for basic personalization  
**Impact**: 40-60% improvement in user engagement  
**Difficulty**: Medium
