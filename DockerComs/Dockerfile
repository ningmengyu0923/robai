FROM ros2:1010  

COPY requirements.txt .  
RUN pip install -r requirements.txt  

COPY DockerComs/data_zset.py .  
COPY DockerComs/data_zset_dash.py .  

ENTRYPOINT ["bash", "-c"]  

IMAGE dashrob:v1
# 指定新的镜像名称为 dashrob:v1  
# 格式为 REPOSITORY:TAG  
# 这里使用 dashrob 作为 repository 名称, v1 作为 tag  
# 您可以根据需要修改这个名称  
# 例如: dashrob:latest 或者 dashrob:dev 等  
# 只要与 docker-compose.yml 文件中的 image 字段保持一致即可  
# 此外,如果需要多个版本的镜像,可以修改 TAG 部分,例如 v2, v3 等  
# 而 REPOSITORY 通常保持一致  
# 这里我们将名称设置为 dashrob:v1  
# 如果不需要指定 TAG,可以直接使用 dashrob 作为镜像名称  
# 即: REPOSITORY  DASHROB:TAG  V1  
# 这样 docker-compose.yml 中的 image 字段也需要相应修改  
