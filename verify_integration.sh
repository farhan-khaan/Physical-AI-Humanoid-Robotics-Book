#!/bin/bash

# Verification script for RAG Chatbot Integration
# Run this to check if everything is properly set up

echo "====================================="
echo "🔍 RAG Chatbot Integration Verification"
echo "====================================="
echo ""

all_good=true

# Check Python
echo "Checking Python..."
if command -v python3 &> /dev/null; then
    python_version=$(python3 --version)
    if [[ $python_version =~ Python\ 3\.(1[1-9]|[2-9][0-9]) ]]; then
        echo "✅ Python: $python_version"
    else
        echo "⚠️  Python: $python_version (Need 3.11+)"
        all_good=false
    fi
else
    echo "❌ Python not found"
    all_good=false
fi

# Check Node.js
echo "Checking Node.js..."
if command -v node &> /dev/null; then
    node_version=$(node --version)
    if [[ $node_version =~ v(1[8-9]|2[0-9]) ]]; then
        echo "✅ Node.js: $node_version"
    else
        echo "⚠️  Node.js: $node_version (Need 18+)"
        all_good=false
    fi
else
    echo "❌ Node.js not found"
    all_good=false
fi

echo ""
echo "Checking Files..."

# Check backend files
backend_files=(
    "backend/rag_chatbot.py"
    "backend/database.py"
    "backend/embed_all_content.py"
    "backend/test_chatbot.py"
    "backend/requirements.txt"
    "backend/.env.example"
    "backend/Dockerfile"
)

for file in "${backend_files[@]}"; do
    if [ -f "$file" ]; then
        echo "✅ $file"
    else
        echo "❌ $file - MISSING"
        all_good=false
    fi
done

# Check frontend files
frontend_files=(
    "src/components/Chatbot/ChatbotWidget.tsx"
    "src/components/Chatbot/ChatbotWidget.module.css"
    "src/theme/Root.js"
)

for file in "${frontend_files[@]}"; do
    if [ -f "$file" ]; then
        echo "✅ $file"
    else
        echo "❌ $file - MISSING"
        all_good=false
    fi
done

# Check documentation
doc_files=(
    "CHATBOT-SETUP-GUIDE.md"
    "QUICK-START-CHATBOT.md"
    "CHATBOT-INTEGRATION-CHECKLIST.md"
    "RAG-CHATBOT-SUMMARY.md"
)

for file in "${doc_files[@]}"; do
    if [ -f "$file" ]; then
        echo "✅ $file"
    else
        echo "❌ $file - MISSING"
        all_good=false
    fi
done

echo ""
echo "Checking Configuration..."

# Check if backend .env exists
if [ -f "backend/.env" ]; then
    echo "✅ backend/.env exists"
    
    # Check for required keys
    if grep -q "OPENAI_API_KEY=sk-" "backend/.env"; then
        echo "✅ OPENAI_API_KEY configured"
    else
        echo "⚠️  OPENAI_API_KEY needs to be set in backend/.env"
        all_good=false
    fi
    
    if grep -q "QDRANT_URL=https://" "backend/.env"; then
        echo "✅ QDRANT_URL configured"
    else
        echo "⚠️  QDRANT_URL needs to be set in backend/.env"
        all_good=false
    fi
else
    echo "⚠️  backend/.env not found (copy from .env.example)"
    all_good=false
fi

# Check if frontend .env.local exists
if [ -f ".env.local" ]; then
    echo "✅ .env.local exists"
else
    echo "⚠️  .env.local not found (needed for frontend)"
fi

echo ""
echo "Checking Dependencies..."

# Check if node_modules exists
if [ -d "node_modules" ]; then
    echo "✅ Frontend dependencies installed"
else
    echo "⚠️  Run 'npm install' to install frontend dependencies"
    all_good=false
fi

# Check if backend venv exists
if [ -d "backend/venv" ]; then
    echo "✅ Backend virtual environment exists"
else
    echo "ℹ️  Backend venv not found (optional, can use global Python)"
fi

echo ""
echo "Checking Integration..."

# Check if ChatbotWidget is imported in Root.js
if grep -q "import ChatbotWidget" "src/theme/Root.js"; then
    echo "✅ ChatbotWidget imported in Root.js"
else
    echo "❌ ChatbotWidget NOT imported in Root.js"
    all_good=false
fi

if grep -q "<ChatbotWidget" "src/theme/Root.js"; then
    echo "✅ ChatbotWidget rendered in Root.js"
else
    echo "❌ ChatbotWidget NOT rendered in Root.js"
    all_good=false
fi

echo ""
echo "====================================="

if [ "$all_good" = true ]; then
    echo "🎉 All checks passed!"
    echo ""
    echo "Next Steps:"
    echo "1. Configure backend/.env with your API keys"
    echo "2. Start backend: cd backend && python rag_chatbot.py"
    echo "3. Embed content: cd backend && python embed_all_content.py"
    echo "4. Start frontend: npm start"
    echo "5. Test chatbot at http://localhost:3000"
    echo ""
    echo "See QUICK-START-CHATBOT.md for detailed instructions."
else
    echo "⚠️  Some checks failed. Please review the issues above."
    echo ""
    echo "For help, see:"
    echo "- CHATBOT-SETUP-GUIDE.md"
    echo "- RAG-CHATBOT-SUMMARY.md"
fi

echo "====================================="
