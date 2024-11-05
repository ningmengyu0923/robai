FROM ros2:1010  

# 将工作目录设置为 /app
WORKDIR /app

# 将 requirements.txt 复制到工作目录
COPY requirements.txt .

# 安装 Python 依赖包
RUN pip install --no-cache-dir -i https://pypi.tuna.tsinghua.edu.cn/simple -r requirements.txt

