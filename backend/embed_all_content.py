"""
Script to embed all Physical AI book content into Qdrant
"""

import os
import glob
import requests
from pathlib import Path
import re

# API endpoint (local or deployed)
API_URL = os.getenv("API_URL", "http://localhost:8000")

def extract_metadata_from_markdown(content: str):
    """Extract frontmatter metadata from markdown."""
    metadata = {}
    
    # Extract frontmatter
    if content.startswith('---'):
        parts = content.split('---', 2)
        if len(parts) >= 3:
            frontmatter = parts[1]
            for line in frontmatter.split('\n'):
                if ':' in line:
                    key, value = line.split(':', 1)
                    metadata[key.strip()] = value.strip().strip('"').strip("'")
    
    return metadata

def clean_markdown(content: str) -> str:
    """Remove markdown syntax but keep content."""
    # Remove frontmatter
    if content.startswith('---'):
        parts = content.split('---', 2)
        if len(parts) >= 3:
            content = parts[2]
    
    # Remove import statements
    content = re.sub(r'import .+ from .+;?\n', '', content)
    
    # Remove JSX components (keep content inside)
    content = re.sub(r'<[A-Z]\w+[^>]*>', '', content)
    content = re.sub(r'</[A-Z]\w+>', '', content)
    
    # Remove code block markers but keep code
    content = re.sub(r'```\w*\n', '', content)
    content = re.sub(r'```', '', content)
    
    return content.strip()

def embed_file(file_path: str):
    """Embed a single markdown file."""
    print(f"Processing: {file_path}")
    
    # Read file
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Extract metadata
    metadata = extract_metadata_from_markdown(content)
    title = metadata.get('title', os.path.basename(file_path))
    
    # Generate chapter ID from path
    chapter_id = file_path.replace('\\', '/').replace('docs/', '').replace('.md', '')
    
    # Clean content
    clean_content = clean_markdown(content)
    
    # Skip if too short
    if len(clean_content) < 100:
        print(f"  ⚠️  Skipped (too short)")
        return
    
    # Send to API
    try:
        response = requests.post(
            f"{API_URL}/api/embed-document",
            json={
                "chapter_id": chapter_id,
                "chapter_title": title,
                "content": clean_content,
                "section": metadata.get('description', '')
            }
        )
        
        if response.status_code == 200:
            data = response.json()
            print(f"  ✅ Embedded {data['chunks_embedded']} chunks")
        else:
            print(f"  ❌ Error: {response.status_code}")
    
    except Exception as e:
        print(f"  ❌ Error: {e}")

def embed_all_content():
    """Embed all Physical AI book content."""
    print("🚀 Starting to embed Physical AI book content...\n")
    
    # Find all markdown files in physical-ai directory
    markdown_files = glob.glob('docs/physical-ai/**/*.md', recursive=True)
    
    print(f"Found {len(markdown_files)} markdown files\n")
    
    total_embedded = 0
    for file_path in markdown_files:
        embed_file(file_path)
        total_embedded += 1
    
    print(f"\n✅ Completed! Embedded {total_embedded} files")
    
    # Get stats
    try:
        response = requests.get(f"{API_URL}/api/stats")
        if response.status_code == 200:
            stats = response.json()
            print(f"\n📊 Collection Stats:")
            print(f"   Total vectors: {stats['total_vectors']}")
            print(f"   Collection: {stats['collection_name']}")
    except:
        pass

if __name__ == "__main__":
    embed_all_content()
