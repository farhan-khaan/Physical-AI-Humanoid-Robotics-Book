"""
Test script for RAG Chatbot
Validates all components are working correctly
"""

import requests
import os
from dotenv import load_dotenv
import time

load_dotenv()

# Configuration
API_URL = "http://localhost:8000"
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
DATABASE_URL = os.getenv("DATABASE_URL")

def print_test(name, passed, message=""):
    """Print test result"""
    symbol = "✅" if passed else "❌"
    print(f"{symbol} {name}")
    if message:
        print(f"   {message}")

def test_environment():
    """Test environment variables"""
    print("\n🔧 Testing Environment Variables...")
    
    tests = [
        ("OPENAI_API_KEY", OPENAI_API_KEY is not None),
        ("QDRANT_URL", QDRANT_URL is not None),
        ("DATABASE_URL (optional)", True)  # Optional
    ]
    
    for name, passed in tests:
        print_test(name, passed)
    
    return all([t[1] for t in tests[:2]])  # Only first 2 are required

def test_backend_health():
    """Test backend health endpoint"""
    print("\n🏥 Testing Backend Health...")
    
    try:
        response = requests.get(f"{API_URL}/api/health", timeout=5)
        passed = response.status_code == 200
        
        if passed:
            data = response.json()
            print_test("Backend Health", True, f"Status: {data.get('status')}")
            print_test("Qdrant Connected", data.get('qdrant_connected', False))
            print_test("OpenAI Connected", data.get('openai_connected', False))
        else:
            print_test("Backend Health", False, f"Status code: {response.status_code}")
        
        return passed
    except requests.exceptions.RequestException as e:
        print_test("Backend Health", False, f"Connection failed: {e}")
        return False

def test_collection_stats():
    """Test collection statistics"""
    print("\n📊 Testing Collection Stats...")
    
    try:
        response = requests.get(f"{API_URL}/api/stats", timeout=5)
        passed = response.status_code == 200
        
        if passed:
            data = response.json()
            total_vectors = data.get('total_vectors', 0)
            collection_name = data.get('collection_name', 'unknown')
            
            print_test("Collection Stats", True, 
                      f"{total_vectors} vectors in '{collection_name}'")
            
            if total_vectors == 0:
                print("   ⚠️  Warning: No vectors found. Run embed_all_content.py")
                return False
            elif total_vectors < 50:
                print("   ⚠️  Warning: Low vector count. Expected 100+")
        else:
            print_test("Collection Stats", False, f"Status code: {response.status_code}")
        
        return passed
    except requests.exceptions.RequestException as e:
        print_test("Collection Stats", False, f"Connection failed: {e}")
        return False

def test_chat_endpoint():
    """Test chat endpoint"""
    print("\n💬 Testing Chat Endpoint...")
    
    test_message = "What is embodied intelligence?"
    
    try:
        response = requests.post(
            f"{API_URL}/api/chat",
            json={
                "message": test_message,
                "conversation_history": [],
                "selected_text": None,
                "chapter_id": None,
                "user_id": "test_user"
            },
            timeout=30
        )
        
        passed = response.status_code == 200
        
        if passed:
            data = response.json()
            answer = data.get('answer', '')
            sources = data.get('sources', [])
            
            print_test("Chat Response", True, f"Got {len(answer)} chars")
            print_test("Sources Retrieved", len(sources) > 0, 
                      f"Found {len(sources)} sources")
            
            print(f"\n   Question: {test_message}")
            print(f"   Answer: {answer[:200]}...")
            
            if sources:
                print(f"\n   Top Source:")
                print(f"   - Chapter: {sources[0].get('chapter', 'Unknown')}")
                print(f"   - Relevance: {sources[0].get('score', 0):.1%}")
        else:
            print_test("Chat Response", False, f"Status code: {response.status_code}")
            print(f"   Error: {response.text}")
        
        return passed
    except requests.exceptions.RequestException as e:
        print_test("Chat Response", False, f"Connection failed: {e}")
        return False

def test_selected_text_query():
    """Test selected text functionality"""
    print("\n✂️  Testing Selected Text Query...")
    
    selected_text = """
    Embodied intelligence refers to AI systems that interact with the physical world
    through sensors and actuators, following a sense-think-act loop.
    """
    
    test_message = "Explain this concept in simple terms"
    
    try:
        response = requests.post(
            f"{API_URL}/api/chat",
            json={
                "message": test_message,
                "conversation_history": [],
                "selected_text": selected_text.strip(),
                "chapter_id": "01-embodied-intelligence",
                "user_id": "test_user"
            },
            timeout=30
        )
        
        passed = response.status_code == 200
        
        if passed:
            data = response.json()
            answer = data.get('answer', '')
            
            print_test("Selected Text Query", True, f"Got {len(answer)} chars")
            print(f"\n   Selected: {selected_text.strip()[:100]}...")
            print(f"   Question: {test_message}")
            print(f"   Answer: {answer[:200]}...")
        else:
            print_test("Selected Text Query", False, f"Status code: {response.status_code}")
        
        return passed
    except requests.exceptions.RequestException as e:
        print_test("Selected Text Query", False, f"Connection failed: {e}")
        return False

def test_chapter_filtering():
    """Test chapter-specific filtering"""
    print("\n📖 Testing Chapter Filtering...")
    
    try:
        response = requests.post(
            f"{API_URL}/api/chat",
            json={
                "message": "What are the main topics covered?",
                "conversation_history": [],
                "selected_text": None,
                "chapter_id": "02-sensors-actuators",
                "user_id": "test_user"
            },
            timeout=30
        )
        
        passed = response.status_code == 200
        
        if passed:
            data = response.json()
            sources = data.get('sources', [])
            
            # Check if sources are from the specified chapter
            chapter_match = any('sensor' in s.get('chapter', '').lower() 
                              for s in sources)
            
            print_test("Chapter Filtering", chapter_match, 
                      f"Sources filtered to chapter")
        else:
            print_test("Chapter Filtering", False, f"Status code: {response.status_code}")
        
        return passed
    except requests.exceptions.RequestException as e:
        print_test("Chapter Filtering", False, f"Connection failed: {e}")
        return False

def run_all_tests():
    """Run all tests"""
    print("=" * 60)
    print("🧪 RAG Chatbot Test Suite")
    print("=" * 60)
    
    results = {}
    
    # Run tests
    results['environment'] = test_environment()
    
    if not results['environment']:
        print("\n❌ Environment variables not set. Please configure backend/.env")
        print("   See backend/.env.example for template")
        return
    
    results['health'] = test_backend_health()
    
    if not results['health']:
        print("\n❌ Backend is not running. Please start it:")
        print("   cd backend && python rag_chatbot.py")
        return
    
    results['stats'] = test_collection_stats()
    results['chat'] = test_chat_endpoint()
    results['selected_text'] = test_selected_text_query()
    results['chapter_filter'] = test_chapter_filtering()
    
    # Summary
    print("\n" + "=" * 60)
    print("📊 Test Summary")
    print("=" * 60)
    
    total = len(results)
    passed = sum(results.values())
    
    for test_name, passed_flag in results.items():
        symbol = "✅" if passed_flag else "❌"
        print(f"{symbol} {test_name.replace('_', ' ').title()}")
    
    print("\n" + "=" * 60)
    print(f"Results: {passed}/{total} tests passed")
    
    if passed == total:
        print("🎉 All tests passed! Your chatbot is ready!")
    else:
        print("⚠️  Some tests failed. Please review the errors above.")
    
    print("=" * 60)

if __name__ == "__main__":
    run_all_tests()
