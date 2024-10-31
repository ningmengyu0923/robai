import httpx as requests

# Sider API 的基本信息  
API_URL = "https://api.sider.example.com/chat"  # 替换为实际的 API URL  
USERNAME = "spirit.ai.it@gmail.com"  # 替换为你的用户名
PASSWORD = "cXfa5}2;N4=_"  # 替换为你的密码


def authenticate(username, password):
    # 进行认证获取 token 或 session ID，具体实现依据 Sider 的 API 文档  
    response = requests.post(f"{API_URL}/login", json={"username": username, "password": password})
    if response.status_code == 200:
        return response.json().get("token")  # 假设 API 返回 token  
    else:
        print("Authentication failed:", response.text)
        return None


def chat_with_sider(token, message):
    # 发送消息进行对话  
    headers = {"Authorization": f"Bearer {token}"}
    response = requests.post(f"{API_URL}/message", headers=headers, json={"message": message})

    if response.status_code == 200:
        return response.json()  # 返回 API 响应  
    else:
        print("Chat failed:", response.text)
        return None


if __name__ == "__main__":
    # 认证用户  
    token = authenticate(USERNAME, PASSWORD)
    if token:
        # 与 Sider 进行对话  
        user_message = "你好，请问今天的天气怎么样？"
        reply = chat_with_sider(token, user_message)

        if reply:
            print("Sider 回复:", reply)