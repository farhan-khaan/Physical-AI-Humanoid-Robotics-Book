# PowerShell script to embed all book content into Qdrant
# Run this after setting up your backend and environment variables

Write-Host "🚀 Starting content embedding process..." -ForegroundColor Cyan
Write-Host ""

# Check if backend directory exists
if (-not (Test-Path "backend")) {
    Write-Host "❌ Error: backend directory not found" -ForegroundColor Red
    Write-Host "Please run this script from the project root" -ForegroundColor Yellow
    exit 1
}

# Check if .env exists
if (-not (Test-Path "backend/.env")) {
    Write-Host "❌ Error: backend/.env file not found" -ForegroundColor Red
    Write-Host "Please create backend/.env with your API keys" -ForegroundColor Yellow
    Write-Host "See backend/.env.example for template" -ForegroundColor Yellow
    exit 1
}

# Change to backend directory
Set-Location backend

# Check if virtual environment exists, create if not
if (-not (Test-Path "venv")) {
    Write-Host "📦 Creating virtual environment..." -ForegroundColor Cyan
    python -m venv venv
}

# Activate virtual environment
Write-Host "🔧 Activating virtual environment..." -ForegroundColor Cyan
& ".\venv\Scripts\Activate.ps1"

# Install requirements
Write-Host "📥 Installing requirements..." -ForegroundColor Cyan
pip install -q -r requirements.txt

# Start the backend in background
Write-Host "🚀 Starting backend server..." -ForegroundColor Cyan
$backendJob = Start-Job -ScriptBlock {
    Set-Location $using:PWD
    & ".\venv\Scripts\python.exe" rag_chatbot.py
}

# Wait for backend to start
Write-Host "⏳ Waiting for backend to start..." -ForegroundColor Yellow
Start-Sleep -Seconds 5

# Check if backend is running
try {
    $response = Invoke-WebRequest -Uri "http://localhost:8000/api/health" -UseBasicParsing -ErrorAction Stop
    Write-Host "✅ Backend is running" -ForegroundColor Green
}
catch {
    Write-Host "❌ Backend failed to start" -ForegroundColor Red
    Stop-Job -Job $backendJob
    Remove-Job -Job $backendJob
    exit 1
}

# Run embedding script
Write-Host ""
Write-Host "📚 Embedding book content..." -ForegroundColor Cyan
python embed_all_content.py

# Stop backend
Write-Host ""
Write-Host "🛑 Stopping backend server..." -ForegroundColor Yellow
Stop-Job -Job $backendJob
Remove-Job -Job $backendJob

# Deactivate virtual environment
deactivate

Write-Host ""
Write-Host "✅ Content embedding complete!" -ForegroundColor Green
Write-Host ""
Write-Host "Next steps:" -ForegroundColor Cyan
Write-Host "1. Start your backend: cd backend && python rag_chatbot.py"
Write-Host "2. Start your frontend: npm start"
Write-Host "3. Test the chatbot at http://localhost:3000"
