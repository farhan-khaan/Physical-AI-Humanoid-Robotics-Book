from fastapi import FastAPI, Request
from openai import OpenAI
from dotenv import load_dotenv

load_dotenv()
app = FastAPI()
client = OpenAI(api_key="hHVmPdfWiiFRo96RaLv9mAV32Q9nsOKvidocUKdLeTTsRSwSppHAhQSe1JCHH_3sjJIxi2Sqx0T3BlbkFJhz18rRqyyngR8s6q1d39QMy-9kYz2sxn5WHZ0pWnfxt_h46dJCOAATVycQyxEOQZykfW1z2woA")

@app.post("/personalize")
async def personalize(request: Request):
    data = await request.json()
    chapter_id = data["chapterId"]
    user_background = "user data from DB"

    response = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[
            {"role": "system", "content": f"Rewrite this chapter for a {user_background} learner."},
            {"role": "user", "content": f"Chapter ID: {chapter_id}"}
        ]
    )
    return {"custom_text": response.choices[0].message.content}
