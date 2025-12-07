# 🤖 Physical AI & Humanoid Robotics

> A comprehensive educational resource covering the fundamentals of Physical AI, sensors, simulation, control strategies, and real-world robotics implementation.

[![Built with Docusaurus](https://img.shields.io/badge/Built%20with-Docusaurus-success)](https://docusaurus.io/)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)](CONTRIBUTING.md)

## 📚 About This Book

This interactive educational book provides a complete learning path from foundational concepts to advanced implementations in Physical AI and humanoid robotics. Perfect for students, researchers, and robotics enthusiasts.

### 🎯 What You'll Learn

- **Embodied Intelligence**: Understanding the sense-think-act loop
- **Sensors & Actuators**: Hardware fundamentals and integration
- **Simulation**: Digital twins, PyBullet, Gazebo, Isaac Sim
- **Control Strategies**: Reactive, deliberative, hybrid, and learned control
- **Capstone Projects**: Hands-on implementation from beginner to expert

## 📖 Book Structure

### Chapter 1: Embodied Intelligence
Introduction to Physical AI concepts, the sense-think-act loop, and real-world applications.

### Chapter 2: Sensors & Actuators
Comprehensive coverage of sensor types (cameras, LIDAR, IMU), actuators (motors, servos), and sensor fusion techniques.

### Chapter 3: Simulation
Digital twins, simulation platforms comparison, setup guides, and sim-to-real transfer strategies.

### Chapter 4: Control Strategies
From PID controllers to reinforcement learning - reactive control, path planning, behavior trees, and more.

### Chapter 5: Capstone Project
Complete project guide with 8+ project ideas, requirements, evaluation rubrics, and detailed examples.

## 🚀 Quick Start

### Prerequisites

- Node.js 18.0 or higher
- npm or yarn

### Installation

```bash
# Clone the repository
git clone https://github.com/YOUR-USERNAME/physical-ai-robotics-book.git
cd physical-ai-robotics-book

# Install dependencies
npm install

# Start development server
npm start
```

Open [http://localhost:3000](http://localhost:3000) to view the book.

### Build for Production

```bash
npm run build
npm run serve
```

## 🤖 RAG Chatbot Setup

The book includes an AI-powered chatbot that can answer questions about the content using Retrieval-Augmented Generation (RAG).

### Quick Setup (15 minutes)

See **[QUICK-START-CHATBOT.md](QUICK-START-CHATBOT.md)** for a 15-minute setup guide.

### Full Setup Guide

See **[CHATBOT-SETUP-GUIDE.md](CHATBOT-SETUP-GUIDE.md)** for comprehensive deployment instructions.

### Features

- 💬 Ask questions about any chapter
- ✂️ Select text and get instant explanations
- 📚 Automatic source citations
- 💾 Conversation history saved to database
- 🎯 Chapter-aware context filtering
- 🔐 User authentication integration

### Test the Chatbot

```bash
# Start backend
cd backend
python rag_chatbot.py

# In another terminal, embed content
cd backend
python embed_all_content.py

# In another terminal, start frontend
npm start
```

Open http://localhost:3000 and click the chatbot icon (bottom right)!

## 📊 Content Statistics

- **📄 Pages**: 29 comprehensive chapters
- **📝 Words**: 56,000+ words of content
- **💻 Code Examples**: 100+ working examples
- **🐍 Languages**: Python and C++
- **🎯 Exercises**: Multiple per chapter
- **🏆 Projects**: 8 capstone project ideas

## 🛠️ Technologies Used

### Frontend
- **Framework**: [Docusaurus](https://docusaurus.io/)
- **Language**: TypeScript, React
- **Deployment**: Vercel
- **Authentication**: Better Auth (Google/GitHub OAuth)

### Backend (RAG Chatbot)
- **API**: FastAPI (Python)
- **AI**: OpenAI GPT-4o-mini, text-embedding-3-small
- **Vector DB**: Qdrant Cloud (Free Tier)
- **Database**: Neon Serverless Postgres
- **Deployment**: Render.com / Docker

## 🎓 Learning Path

```
Week 1-2  → Chapter 1: Embodied Intelligence
Week 3-4  → Chapter 2: Sensors & Actuators
Week 5-6  → Chapter 3: Simulation
Week 7-9  → Chapter 4: Control Strategies
Week 10+  → Chapter 5: Capstone Project
```

## 💡 Features

- ✅ **Interactive Code Examples** with syntax highlighting
- ✅ **Multi-language Support** (Python/C++ tabs)
- ✅ **Responsive Design** for mobile and desktop
- ✅ **Dark Mode** support
- ✅ **Search Functionality**
- ✅ **Progressive Learning** from basics to advanced
- ✅ **Hands-on Exercises** with solutions
- ✅ **Real-world Projects** with detailed guidance
- ✅ **🤖 AI-Powered RAG Chatbot** - Ask questions about book content
- ✅ **Selected Text Queries** - Highlight text and ask for explanations
- ✅ **User Authentication** - Google/GitHub OAuth integration
- ✅ **Content Personalization** - Tailored learning experiences

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request. For major changes, please open an issue first to discuss what you would like to change.

### How to Contribute

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🌟 Acknowledgments

- Built with [Docusaurus](https://docusaurus.io/)
- Inspired by the Physical AI and robotics community
- Code examples tested with PyBullet, ROS, and MuJoCo

## 📧 Contact

**Author**: Your Name  
**Email**: your.email@example.com  
**GitHub**: [@yourusername](https://github.com/yourusername)

## 🔗 Links

- **Live Demo**: [https://your-site.vercel.app](https://your-site.vercel.app)
- **Documentation**: [https://your-site.vercel.app/docs/intro](https://your-site.vercel.app/docs/intro)
- **Issues**: [GitHub Issues](https://github.com/YOUR-USERNAME/physical-ai-robotics-book/issues)

## ⭐ Star History

If you find this resource helpful, please consider giving it a star! ⭐

---

**Made with ❤️ for the robotics community**
