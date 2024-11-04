FROM ros2:1010  

COPY requirements.txt .  
RUN pip install -r requirements.txt  

COPY DockerComs/data_zset.py .  
COPY DockerComs/data_zset_dash.py .  

ENTRYPOINT ["bash", "-c"]