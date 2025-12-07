from fastapi import FastAPI, Request
from openai import OpenAI
import os

app = FastAPI()
client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

@app.post("/translate")
async def translate(request: Request):
    # Get JSON data from frontend
    data = await request.json()
    text_to_translate = data.get("text", "")

    if not text_to_translate:
        return {"error": "Missing 'text' field in request."}

    # Ask OpenAI to translate to Urdu
    response = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[
            {"role": "system", "content": "Translate the following English text to Urdu, keeping technical accuracy."},
            {"role": "user", "content": text_to_translate}
        ]
    )

    translated = response.choices[0].message.content.strip()
    return {"translation": translated}
