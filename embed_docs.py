from openai import OpenAI
from qdrant_client import QdrantClient
import glob, os

# Use environment variables for security
client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
qdrant = QdrantClient(url=os.getenv("QDRANT_URL"))
files = glob.glob("../docs/*.md")

for f in files:
    with open(f, 'r') as file:
        text = file.read()
    emb = client.embeddings.create(model="text-embedding-3-small", input=text)
    qdrant.upsert(collection_name="book", points=[{"id": f, "vector": emb.data[0].embedding, "payload": {"file": f}}])