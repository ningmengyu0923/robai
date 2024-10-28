import rclpy
from rclpy.node import Node
from mc_core_interface.msg import ServoFeedback
import json
import websockets
import asyncio
from multiprocessing import Process, Queue

class ROSSubscriber(Node):
    def __init__(self, queue):
        super().__init__('ros_subscriber')
        self.queue = queue
        self.k = 1
        # 创建 ROS 订阅
        self.subscription = self.create_subscription(
            msg_type=ServoFeedback,
            topic='servo_data_msg',
            callback=self.listener_callback,
            qos_profile=10
        )
        self.get_logger().info("ROS subscriber initialized")

    def listener_callback(self, msg):
        """处理接收到的消息并将其放入队列"""
        data = {
            "Timestamp": int(msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec / 1000000),
            "cur_pos": list(msg.cur_pos),
            "cur_vel": list(msg.cur_vel),
            "cur_trq": list(msg.cur_trq),
            "cur_following_err": list(msg.cur_following_err),
            "servo_err": list(msg.servo_err),
            "servo_temperature": list(msg.servo_temperature),
            "pos_cmd": list(msg.pos_cmd),
            "vel_cmd": list(msg.vel_cmd),
        }
        self.k+=1
        message = json.dumps(data)
        self.queue.put(message)  # 将消息放入队列

async def websocket_handler(websocket, path, queue):
    print("New client connected")
    try:
        # # 等待客户端发送连接确认信息
        # await websocket.recv()  # 接收连接信息
        # await websocket.send("Connection established, waiting for data...")  # 发送确认信息

        while True:
            # 从队列中获取消息并发送
            if not queue.empty():
                message = queue.get()
                await websocket.send(message)  # 发送消息给客户端
    except websockets.exceptions.ConnectionClosed:
        print("Client disconnected")
    except Exception as e:
        print(f"WebSocket error: {e}")

async def run_websocket_server(queue):
    """运行 WebSocket 服务器"""
    start_server = websockets.serve(lambda ws, path: websocket_handler(ws, path, queue), "192.168.10.134", 8765)
    await start_server
    print("WebSocket server started on ws://192.168.10.134:8765")
    await asyncio.Future()  # 永远运行

def websocket_process(queue):
    asyncio.run(run_websocket_server(queue))

def ros_process_function(queue):
    rclpy.init()
    subscriber = ROSSubscriber(queue)
    rclpy.spin(subscriber)
    rclpy.shutdown()

def main():
    # 创建队列用于进程间通信
    queue = Queue()

    # 启动 ROS 订阅进程
    ros_process = Process(target=ros_process_function, args=(queue,))
    ros_process.start()

    # 启动 WebSocket 进程
    websocket_proc = Process(target=websocket_process, args=(queue,))
    websocket_proc.start()

    try:
        while True:
            pass  # 主进程保持运行
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        ros_process.terminate()
        websocket_proc.terminate()

if __name__ == '__main__':
    main()
