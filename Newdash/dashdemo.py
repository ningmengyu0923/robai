import asyncio
import websockets
import redis
import json
import time

# Redis 连接
redis_client = redis.Redis(host='192.168.10.45', port=6379, db=1)


# 从 Redis 获取数据
def fetch_latest_data():
    """从 Redis 有序集合获取最新的 1000 条数据，按 Timestamp 顺序"""
    latest_data = redis_client.zrange('sensor_data_sorted', -1000, -1)  # 获取最新的 1000 条数据
    latest_data = [json.loads(data.decode('utf-8')) for data in latest_data]

    # 提取 Timestamps 和各个度量
    timestamps = [data['Timestamp'] for data in latest_data]
    metrics = {key: [data[key] for data in latest_data] for key in
               ['cur_pos', 'cur_vel', 'cur_trq', 'cur_following_err', 'servo_err',
                'servo_temperature', 'pos_cmd', 'vel_cmd']}
    return timestamps, metrics


# WebSocket 处理函数
async def data_stream(websocket, path):
    while True:
        # 每秒获取数据
        timestamps, metrics = fetch_latest_data()

        # 将数据打包为JSON格式发送到客户端
        data = {'timestamps': timestamps, 'metrics': metrics}
        await websocket.send(json.dumps(data))

        # 等待一秒后继续发送数据
        await asyncio.sleep(0.001)  # 1ms 间隔，每秒 1000 次


# 启动 WebSocket 服务器
async def start_server():
    async with websockets.serve(data_stream, "0.0.0.0", 8765):
        await asyncio.Future()  # 保持服务器运行


if __name__ == "__main__":
    asyncio.run(start_server())
