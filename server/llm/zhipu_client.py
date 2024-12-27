import httpx
import os
from typing import Optional, Dict, Any

class ZhipuClient:
    def __init__(self, server_host: str, api_key: str):
        self.server_host = server_host
        self.api_key = api_key
        self.base_url = f"http://{server_host}"

    async def get_response(self, text: str) -> str:
        """调用智谱AI大语言模型获取回复"""
        try:
            headers = {"X-API-Key": self.api_key}
            
            chat_request = {
                "prompt": text,
                "provider": "zhipu",
                "model": "glm-4-flash",
                "temperature": 0.7,
                "top_p": 0.9
            }
            
            async with httpx.AsyncClient(timeout=30.0) as client:
                response = await client.post(
                    f"{self.base_url}/chat",
                    headers=headers,
                    json=chat_request
                )
                
                if response.status_code == 200:
                    result = response.json()
                    return result.get('response', '抱歉，我现在无法回答这个问题。')
                else:
                    print(f"调用API失败: {response.status_code} - {response.text}")
                    return "抱歉，服务出现错误。"
                    
        except Exception as e:
            print(f"调用大语言模型出错: {e}")
            return "抱歉，我现在无法回答这个问题。" 