from fastapi import FastAPI, Request
from openai import OpenAI
from qdrant_client import QdrantClient
import os

app = FastAPI()

api_key = os.getenv("OPENAI_API_KEY")
qdrant_url = os.getenv("QDRANT_URL")

if not api_key:
    raise ValueError("OPENAI_API_KEY is not set")
if not qdrant_url:
    raise ValueError("QDRANT_URL is not set")

client = OpenAI(api_key=api_key)
qdrant = QdrantClient(url=qdrant_url)

@app.get("/")
def home():
    return {"status": "ok"}

@app.post("/ask")
async def ask(request: Request):
    data = await request.json()
    query = data["query"]
    selected_text = data.get("selected_text")

    context = selected_text or "Book context from Qdrant search goes here."

    answer = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[
            {"role": "system", "content": f"Use this context to answer accurately:\n{context}"},
            {"role": "user", "content": query}
        ]
    )

    return {"answer": answer.choices[0].message.content}