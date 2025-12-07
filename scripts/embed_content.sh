#!/bin/bash

# Script to embed all book content into Qdrant
# Run this after setting up your backend and environment variables

echo "🚀 Starting content embedding process..."
echo ""

# Check if backend directory exists
if [ ! -d "backend" ]; then
    echo "❌ Error: backend directory not found"
    echo "Please run this script from the project root"
    exit 1
fi

# Check if .env exists
if [ ! -f "backend/.env" ]; then
    echo "❌ Error: backend/.env file not found"
    echo "Please create backend/.env with your API keys"
    echo "See backend/.env.example for template"
    exit 1
fi

# Change to backend directory
cd backend

# Check if virtual environment exists, create if not
if [ ! -d "venv" ]; then
    echo "📦 Creating virtual environment..."
    python3 -m venv venv
fi

# Activate virtual environment
echo "🔧 Activating virtual environment..."
source venv/bin/activate

# Install requirements
echo "📥 Installing requirements..."
pip install -q -r requirements.txt

# Start the backend in background
echo "🚀 Starting backend server..."
python rag_chatbot.py &
BACKEND_PID=$!

# Wait for backend to start
echo "⏳ Waiting for backend to start..."
sleep 5

# Check if backend is running
if curl -s http://localhost:8000/api/health > /dev/null; then
    echo "✅ Backend is running"
else
    echo "❌ Backend failed to start"
    kill $BACKEND_PID 2>/dev/null
    exit 1
fi

# Run embedding script
echo ""
echo "📚 Embedding book content..."
python embed_all_content.py

# Stop backend
echo ""
echo "🛑 Stopping backend server..."
kill $BACKEND_PID 2>/dev/null

# Deactivate virtual environment
deactivate

echo ""
echo "✅ Content embedding complete!"
echo ""
echo "Next steps:"
echo "1. Start your backend: cd backend && python rag_chatbot.py"
echo "2. Start your frontend: npm start"
echo "3. Test the chatbot at http://localhost:3000"
