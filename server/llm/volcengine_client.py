from openai import OpenAI
import os
from typing import Optional, Dict, Any

class VolcengineClient:
    def __init__(self, api_key: str):
        self.client = OpenAI(
            api_key=api_key,
            base_url="https://ark.cn-beijing.volces.com/api/v3"
        )
        self.model_id = "ep-20241226171735-z9tvd"

    async def get_response(self, text: str) -> str:
        """调用火山云大语言模型获取回复"""
        try:
            completion = self.client.chat.completions.create(
                model=self.model_id,
                messages=[
                    {"role": "system", "content": "你是一个智能助手，请用简短的语言回答问题。"},
                    {"role": "user", "content": text}
                ]
            )
            return completion.choices[0].message.content
            
        except Exception as e:
            print(f"调用大语言模型出错: {e}")
            return "抱歉，我现在无法回答这个问题。" 