# Verification script for RAG Chatbot Integration
# Run this to check if everything is properly set up

Write-Host "=====================================" -ForegroundColor Cyan
Write-Host "🔍 RAG Chatbot Integration Verification" -ForegroundColor Cyan
Write-Host "=====================================" -ForegroundColor Cyan
Write-Host ""

$allGood = $true

# Check Python
Write-Host "Checking Python..." -ForegroundColor Yellow
try {
    $pythonVersion = python --version 2>&1
    if ($pythonVersion -match "Python 3\.1[1-9]") {
        Write-Host "✅ Python: $pythonVersion" -ForegroundColor Green
    } else {
        Write-Host "⚠️  Python: $pythonVersion (Need 3.11+)" -ForegroundColor Yellow
        $allGood = $false
    }
} catch {
    Write-Host "❌ Python not found" -ForegroundColor Red
    $allGood = $false
}

# Check Node.js
Write-Host "Checking Node.js..." -ForegroundColor Yellow
try {
    $nodeVersion = node --version 2>&1
    if ($nodeVersion -match "v(1[8-9]|2[0-9])") {
        Write-Host "✅ Node.js: $nodeVersion" -ForegroundColor Green
    } else {
        Write-Host "⚠️  Node.js: $nodeVersion (Need 18+)" -ForegroundColor Yellow
        $allGood = $false
    }
} catch {
    Write-Host "❌ Node.js not found" -ForegroundColor Red
    $allGood = $false
}

Write-Host ""
Write-Host "Checking Files..." -ForegroundColor Yellow

# Check backend files
$backendFiles = @(
    "backend/rag_chatbot.py",
    "backend/database.py",
    "backend/embed_all_content.py",
    "backend/test_chatbot.py",
    "backend/requirements.txt",
    "backend/.env.example",
    "backend/Dockerfile"
)

foreach ($file in $backendFiles) {
    if (Test-Path $file) {
        Write-Host "✅ $file" -ForegroundColor Green
    } else {
        Write-Host "❌ $file - MISSING" -ForegroundColor Red
        $allGood = $false
    }
}

# Check frontend files
$frontendFiles = @(
    "src/components/Chatbot/ChatbotWidget.tsx",
    "src/components/Chatbot/ChatbotWidget.module.css",
    "src/theme/Root.js"
)

foreach ($file in $frontendFiles) {
    if (Test-Path $file) {
        Write-Host "✅ $file" -ForegroundColor Green
    } else {
        Write-Host "❌ $file - MISSING" -ForegroundColor Red
        $allGood = $false
    }
}

# Check documentation
$docFiles = @(
    "CHATBOT-SETUP-GUIDE.md",
    "QUICK-START-CHATBOT.md",
    "CHATBOT-INTEGRATION-CHECKLIST.md",
    "RAG-CHATBOT-SUMMARY.md"
)

foreach ($file in $docFiles) {
    if (Test-Path $file) {
        Write-Host "✅ $file" -ForegroundColor Green
    } else {
        Write-Host "❌ $file - MISSING" -ForegroundColor Red
        $allGood = $false
    }
}

Write-Host ""
Write-Host "Checking Configuration..." -ForegroundColor Yellow

# Check if backend .env exists
if (Test-Path "backend/.env") {
    Write-Host "✅ backend/.env exists" -ForegroundColor Green
    
    # Check for required keys
    $envContent = Get-Content "backend/.env" -Raw
    if ($envContent -match "OPENAI_API_KEY=sk-") {
        Write-Host "✅ OPENAI_API_KEY configured" -ForegroundColor Green
    } else {
        Write-Host "⚠️  OPENAI_API_KEY needs to be set in backend/.env" -ForegroundColor Yellow
        $allGood = $false
    }
    
    if ($envContent -match "QDRANT_URL=https://") {
        Write-Host "✅ QDRANT_URL configured" -ForegroundColor Green
    } else {
        Write-Host "⚠️  QDRANT_URL needs to be set in backend/.env" -ForegroundColor Yellow
        $allGood = $false
    }
} else {
    Write-Host "⚠️  backend/.env not found (copy from .env.example)" -ForegroundColor Yellow
    $allGood = $false
}

# Check if frontend .env.local exists
if (Test-Path ".env.local") {
    Write-Host "✅ .env.local exists" -ForegroundColor Green
} else {
    Write-Host "⚠️  .env.local not found (needed for frontend)" -ForegroundColor Yellow
}

Write-Host ""
Write-Host "Checking Dependencies..." -ForegroundColor Yellow

# Check if node_modules exists
if (Test-Path "node_modules") {
    Write-Host "✅ Frontend dependencies installed" -ForegroundColor Green
} else {
    Write-Host "⚠️  Run 'npm install' to install frontend dependencies" -ForegroundColor Yellow
    $allGood = $false
}

# Check if backend venv exists
if (Test-Path "backend/venv") {
    Write-Host "✅ Backend virtual environment exists" -ForegroundColor Green
} else {
    Write-Host "ℹ️  Backend venv not found (optional, can use global Python)" -ForegroundColor Cyan
}

Write-Host ""
Write-Host "Checking Integration..." -ForegroundColor Yellow

# Check if ChatbotWidget is imported in Root.js
$rootContent = Get-Content "src/theme/Root.js" -Raw
if ($rootContent -match "import ChatbotWidget") {
    Write-Host "✅ ChatbotWidget imported in Root.js" -ForegroundColor Green
} else {
    Write-Host "❌ ChatbotWidget NOT imported in Root.js" -ForegroundColor Red
    $allGood = $false
}

if ($rootContent -match "<ChatbotWidget") {
    Write-Host "✅ ChatbotWidget rendered in Root.js" -ForegroundColor Green
} else {
    Write-Host "❌ ChatbotWidget NOT rendered in Root.js" -ForegroundColor Red
    $allGood = $false
}

Write-Host ""
Write-Host "=====================================" -ForegroundColor Cyan

if ($allGood) {
    Write-Host "🎉 All checks passed!" -ForegroundColor Green
    Write-Host ""
    Write-Host "Next Steps:" -ForegroundColor Cyan
    Write-Host "1. Configure backend/.env with your API keys" -ForegroundColor White
    Write-Host "2. Start backend: cd backend && python rag_chatbot.py" -ForegroundColor White
    Write-Host "3. Embed content: cd backend && python embed_all_content.py" -ForegroundColor White
    Write-Host "4. Start frontend: npm start" -ForegroundColor White
    Write-Host "5. Test chatbot at http://localhost:3000" -ForegroundColor White
    Write-Host ""
    Write-Host "See QUICK-START-CHATBOT.md for detailed instructions." -ForegroundColor Cyan
} else {
    Write-Host "⚠️  Some checks failed. Please review the issues above." -ForegroundColor Yellow
    Write-Host ""
    Write-Host "For help, see:" -ForegroundColor Cyan
    Write-Host "- CHATBOT-SETUP-GUIDE.md" -ForegroundColor White
    Write-Host "- RAG-CHATBOT-SUMMARY.md" -ForegroundColor White
}

Write-Host "=====================================" -ForegroundColor Cyan
