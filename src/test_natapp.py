import httpx
import asyncio
import logging
from config import settings

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

async def test_natapp_connection():
    base_url = f"http://{settings.server_host}"
    headers = {"X-API-Key": settings.server_api_key}
    
    try:
        async with httpx.AsyncClient(timeout=20.0) as client:
            # 1. 测试健康检查
            health_response = await client.get(f"{base_url}/health")
            health_data = health_response.json()
            logger.info("Health check response:")
            logger.info(f"  Status: {health_data.get('status')}")
            logger.info(f"  Version: {health_data.get('version')}")
            
            if health_response.status_code != 200:
                logger.error(f"Health check failed with status: {health_response.status_code}")
                return False

            # 2. 获取可用的供应商列表
            providers_response = await client.get(f"{base_url}/providers", headers=headers)
            if providers_response.status_code == 200:
                providers = providers_response.json()
                logger.info("\nAvailable providers:")
                for provider in providers:
                    logger.info(f"  - {provider}")
            else:
                logger.error(f"Failed to get providers: {providers_response.text}")

            # 3. 获取可用的模型列表
            models_response = await client.get(f"{base_url}/models", headers=headers)
            if models_response.status_code == 200:
                models = models_response.json()
                logger.info("\nAvailable models:")
                for provider, provider_models in models.items():
                    logger.info(f"  {provider}:")
                    for model in provider_models:
                        logger.info(f"    - {model}")
            else:
                logger.error(f"Failed to get models: {models_response.text}")

            # 4. 测试聊天功能
            chat_requests = [
                {
                    "prompt": "小黑子是指什么意思？",
                    "provider": "openai",
                    "model": "gpt-3.5-turbo",
                    "temperature": 0.7,
                    "top_p": 0.9
                },
                {
                    "prompt": "请介绍一下你自己",
                    "provider": "zhipu",
                    "model": "glm-4",
                    "temperature": 0.7,
                    "top_p": 0.9
                }
            ]

            for chat_request in chat_requests:
                logger.info(f"\nTesting chat:")
                logger.info(f"  Provider: {chat_request['provider']}")
                logger.info(f"  Model: {chat_request['model']}")
                logger.info(f"  Prompt: {chat_request['prompt']}")
                
                try:
                    response = await client.post(
                        f"{base_url}/chat", 
                        headers=headers, 
                        json=chat_request,
                        timeout=30.0
                    )
                    
                    if response.status_code == 200:
                        result = response.json()
                        logger.info("\nResponse:")
                        logger.info("  Content:")
                        content = result.get('response', '')
                        if content:
                            for line in content.split('\n'):
                                logger.info(f"    {line}")
                        
                        usage = result.get('usage', {})
                        if usage:
                            logger.info("  Usage:")
                            logger.info(f"    Prompt tokens: {usage.get('prompt_tokens')}")
                            logger.info(f"    Completion tokens: {usage.get('completion_tokens')}")
                            logger.info(f"    Total tokens: {usage.get('total_tokens')}")
                    else:
                        logger.error(f"Chat request failed with status: {response.status_code}")
                        logger.error(f"Error response: {response.text}")
                        
                except Exception as e:
                    logger.error(f"Request failed: {str(e)}")

            return True

    except Exception as e:
        logger.error(f"Connection test failed: {str(e)}")
        return False

async def main():
    logger.info(f"Starting connection test to {settings.server_host}")
    success = await test_natapp_connection()
    if success:
        logger.info("All tests completed!")
    else:
        logger.error("Tests failed!")

if __name__ == "__main__":
    asyncio.run(main())