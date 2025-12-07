import React, { useState } from 'react';
import styles from './OnboardingQuestionnaire.module.css';

interface QuestionnaireData {
  softwareBackground: string;
  hardwareBackground: string;
  experienceLevel: string;
  programmingLanguages: string[];
  roboticsExperience: string;
  learningGoals: string[];
  preferredLanguage: string;
}

interface Props {
  onComplete: (data: QuestionnaireData) => void;
}

const OnboardingQuestionnaire: React.FC<Props> = ({ onComplete }) => {
  const [step, setStep] = useState(1);
  const [data, setData] = useState<QuestionnaireData>({
    softwareBackground: '',
    hardwareBackground: '',
    experienceLevel: '',
    programmingLanguages: [],
    roboticsExperience: '',
    learningGoals: [],
    preferredLanguage: 'en',
  });

  const totalSteps = 6;

  const handleNext = () => {
    if (step < totalSteps) {
      setStep(step + 1);
    } else {
      onComplete(data);
    }
  };

  const handleBack = () => {
    if (step > 1) {
      setStep(step - 1);
    }
  };

  const toggleArrayItem = (array: string[], item: string) => {
    return array.includes(item)
      ? array.filter(i => i !== item)
      : [...array, item];
  };

  return (
    <div className={styles.questionnaire}>
      <div className={styles.header}>
        <h2>Personalize Your Learning Experience</h2>
        <p>Tell us about your background to get customized content</p>
        <div className={styles.progress}>
          <div 
            className={styles.progressBar} 
            style={{ width: `${(step / totalSteps) * 100}%` }}
          />
        </div>
        <p className={styles.stepIndicator}>Step {step} of {totalSteps}</p>
      </div>

      <div className={styles.questionContainer}>
        {/* Step 1: Software Background */}
        {step === 1 && (
          <div className={styles.question}>
            <h3>What is your software development background?</h3>
            <div className={styles.options}>
              <button
                className={data.softwareBackground === 'beginner' ? styles.selected : ''}
                onClick={() => setData({ ...data, softwareBackground: 'beginner' })}
              >
                <span className={styles.emoji}>🌱</span>
                <strong>Beginner</strong>
                <small>Just starting with programming</small>
              </button>
              <button
                className={data.softwareBackground === 'intermediate' ? styles.selected : ''}
                onClick={() => setData({ ...data, softwareBackground: 'intermediate' })}
              >
                <span className={styles.emoji}>📚</span>
                <strong>Intermediate</strong>
                <small>1-3 years of coding experience</small>
              </button>
              <button
                className={data.softwareBackground === 'advanced' ? styles.selected : ''}
                onClick={() => setData({ ...data, softwareBackground: 'advanced' })}
              >
                <span className={styles.emoji}>🚀</span>
                <strong>Advanced</strong>
                <small>3+ years, comfortable with complex systems</small>
              </button>
              <button
                className={data.softwareBackground === 'expert' ? styles.selected : ''}
                onClick={() => setData({ ...data, softwareBackground: 'expert' })}
              >
                <span className={styles.emoji}>⭐</span>
                <strong>Expert</strong>
                <small>Professional software engineer</small>
              </button>
            </div>
          </div>
        )}

        {/* Step 2: Hardware Background */}
        {step === 2 && (
          <div className={styles.question}>
            <h3>What is your hardware/electronics background?</h3>
            <div className={styles.options}>
              <button
                className={data.hardwareBackground === 'none' ? styles.selected : ''}
                onClick={() => setData({ ...data, hardwareBackground: 'none' })}
              >
                <span className={styles.emoji}>❓</span>
                <strong>No Experience</strong>
                <small>Never worked with hardware</small>
              </button>
              <button
                className={data.hardwareBackground === 'hobby' ? styles.selected : ''}
                onClick={() => setData({ ...data, hardwareBackground: 'hobby' })}
              >
                <span className={styles.emoji}>🔧</span>
                <strong>Hobby Projects</strong>
                <small>Arduino, Raspberry Pi, etc.</small>
              </button>
              <button
                className={data.hardwareBackground === 'education' ? styles.selected : ''}
                onClick={() => setData({ ...data, hardwareBackground: 'education' })}
              >
                <span className={styles.emoji}>🎓</span>
                <strong>Academic</strong>
                <small>Studied electronics/EE</small>
              </button>
              <button
                className={data.hardwareBackground === 'professional' ? styles.selected : ''}
                onClick={() => setData({ ...data, hardwareBackground: 'professional' })}
              >
                <span className={styles.emoji}>⚡</span>
                <strong>Professional</strong>
                <small>Work with hardware regularly</small>
              </button>
            </div>
          </div>
        )}

        {/* Step 3: Programming Languages */}
        {step === 3 && (
          <div className={styles.question}>
            <h3>Which programming languages are you familiar with?</h3>
            <small className={styles.hint}>Select all that apply</small>
            <div className={styles.checkboxGrid}>
              {['Python', 'C++', 'C', 'JavaScript', 'Java', 'MATLAB', 'ROS', 'None'].map(lang => (
                <label key={lang} className={styles.checkbox}>
                  <input
                    type="checkbox"
                    checked={data.programmingLanguages.includes(lang)}
                    onChange={() => setData({
                      ...data,
                      programmingLanguages: toggleArrayItem(data.programmingLanguages, lang)
                    })}
                  />
                  <span>{lang}</span>
                </label>
              ))}
            </div>
          </div>
        )}

        {/* Step 4: Robotics Experience */}
        {step === 4 && (
          <div className={styles.question}>
            <h3>What is your robotics experience?</h3>
            <div className={styles.options}>
              <button
                className={data.roboticsExperience === 'none' ? styles.selected : ''}
                onClick={() => setData({ ...data, roboticsExperience: 'none' })}
              >
                <span className={styles.emoji}>🤖</span>
                <strong>Curious Beginner</strong>
                <small>No robotics experience</small>
              </button>
              <button
                className={data.roboticsExperience === 'simulation' ? styles.selected : ''}
                onClick={() => setData({ ...data, roboticsExperience: 'simulation' })}
              >
                <span className={styles.emoji}>💻</span>
                <strong>Simulation Only</strong>
                <small>Used PyBullet, Gazebo, etc.</small>
              </button>
              <button
                className={data.roboticsExperience === 'hobby' ? styles.selected : ''}
                onClick={() => setData({ ...data, roboticsExperience: 'hobby' })}
              >
                <span className={styles.emoji}>🛠️</span>
                <strong>Hobby Robotics</strong>
                <small>Built simple robots</small>
              </button>
              <button
                className={data.roboticsExperience === 'professional' ? styles.selected : ''}
                onClick={() => setData({ ...data, roboticsExperience: 'professional' })}
              >
                <span className={styles.emoji}>🏭</span>
                <strong>Professional</strong>
                <small>Work in robotics field</small>
              </button>
            </div>
          </div>
        )}

        {/* Step 5: Learning Goals */}
        {step === 5 && (
          <div className={styles.question}>
            <h3>What are your learning goals?</h3>
            <small className={styles.hint}>Select all that apply</small>
            <div className={styles.checkboxGrid}>
              {[
                'Understand Physical AI concepts',
                'Build a humanoid robot',
                'Learn control algorithms',
                'Work with sensors and actuators',
                'Master simulation tools',
                'Career in robotics',
                'Academic research',
                'Personal projects'
              ].map(goal => (
                <label key={goal} className={styles.checkbox}>
                  <input
                    type="checkbox"
                    checked={data.learningGoals.includes(goal)}
                    onChange={() => setData({
                      ...data,
                      learningGoals: toggleArrayItem(data.learningGoals, goal)
                    })}
                  />
                  <span>{goal}</span>
                </label>
              ))}
            </div>
          </div>
        )}

        {/* Step 6: Language Preference */}
        {step === 6 && (
          <div className={styles.question}>
            <h3>Preferred Language / ترجیحی زبان</h3>
            <div className={styles.options}>
              <button
                className={data.preferredLanguage === 'en' ? styles.selected : ''}
                onClick={() => setData({ ...data, preferredLanguage: 'en' })}
              >
                <span className={styles.emoji}>🇬🇧</span>
                <strong>English</strong>
                <small>All content in English</small>
              </button>
              <button
                className={data.preferredLanguage === 'ur' ? styles.selected : ''}
                onClick={() => setData({ ...data, preferredLanguage: 'ur' })}
              >
                <span className={styles.emoji}>🇵🇰</span>
                <strong>اردو (Urdu)</strong>
                <small>تمام مواد اردو میں</small>
              </button>
            </div>
          </div>
        )}
      </div>

      <div className={styles.footer}>
        {step > 1 && (
          <button onClick={handleBack} className={styles.backButton}>
            ← Back
          </button>
        )}
        <button 
          onClick={handleNext} 
          className={styles.nextButton}
          disabled={
            (step === 1 && !data.softwareBackground) ||
            (step === 2 && !data.hardwareBackground) ||
            (step === 3 && data.programmingLanguages.length === 0) ||
            (step === 4 && !data.roboticsExperience) ||
            (step === 5 && data.learningGoals.length === 0) ||
            (step === 6 && !data.preferredLanguage)
          }
        >
          {step === totalSteps ? 'Complete ✓' : 'Next →'}
        </button>
      </div>
    </div>
  );
};

export default OnboardingQuestionnaire;
