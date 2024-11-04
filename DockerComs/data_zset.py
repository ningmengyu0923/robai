import os
import json
import time

import rclpy
from rclpy.node import Node
from mc_core_interface.msg import ServoFeedback
import redis

class RedisWriter(Node):
    def __init__(self):
        super().__init__('redis_writer')

        # 从环境变量中读取 Redis 配置
        redis_host = os.getenv('REDIS_HOST', 'localhost')  # 默认值为 localhost
        redis_port = int(os.getenv('REDIS_PORT', 6379))  # 默认值为 6379

        # 连接到 Redis
        self.redis_client = self.connect_to_redis(redis_host, redis_port)

        self.subscription = self.create_subscription(
            msg_type=ServoFeedback,
            topic='servo_data_msg',
            callback=self.listener_callback,
            qos_profile=10
        )

        self.k = 0
        self.batch_size = 10  # 批量插入的大小
        self.buffer = []  # 缓存消息

    def connect_to_redis(self, host, port):
        """尝试连接到 Redis，失败时记录日志并退出"""
        try:
            client = redis.StrictRedis(host=host, port=port, db=1)
            client.ping()  # 测试连接
            return client
        except redis.RedisError as e:
            self.get_logger().error(f"Could not connect to Redis: {e}")
            rclpy.shutdown()  # 关闭 ROS 节点
            raise

    def listener_callback(self, msg):
        """处理接收到的消息并写入 Redis"""
        timestamp_ms = int(msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec / 1000000)

        req_msg = {
            "Timestamp": timestamp_ms,
            "count": len(msg.cur_pos),
            "cur_pos": list(msg.cur_pos),
            "cur_vel": list(msg.cur_vel),
            "cur_trq": list(msg.cur_trq),
            "cur_following_err": list(msg.cur_following_err),
            "servo_err": list(msg.servo_err),
            "servo_temperature": list(msg.servo_temperature),
            "pos_cmd": list(msg.pos_cmd),
            "vel_cmd": list(msg.vel_cmd),
        }

        self.k += 1
        if self.k % 50 == 0:
            self.buffer.append((timestamp_ms, req_msg))

        if len(self.buffer) >= self.batch_size:
            self.bulk_insert()

    def bulk_insert(self):
        """将缓存的消息批量写入 Redis"""
        if not self.buffer:
            return

        try:
            pipeline = self.redis_client.pipeline()
            start = time.time()
            for timestamp_ms, msg in self.buffer:
                pipeline.zadd('sensor_data_sorted', {json.dumps(msg): timestamp_ms})
            print(time.time() - start)
            pipeline.zremrangebyrank('sensor_data_sorted', 0, -30000)
            pipeline.execute()

            self.get_logger().info(f"Inserted {len(self.buffer)} messages to Redis")
            self.buffer.clear()
        except redis.RedisError as e:
            self.get_logger().error(f"Redis error during bulk insert: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

    def __del__(self):
        """在节点销毁时进行清理"""
        try:
            if self.buffer:
                self.bulk_insert()
        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {e}")

if __name__ == '__main__':
    rclpy.init(args=None)
    redis_writer = RedisWriter()

    try:
        rclpy.spin(redis_writer)
    except KeyboardInterrupt:
        redis_writer.get_logger().info("Node interrupted by user")
    finally:
        redis_writer.bulk_insert()
        redis_writer.destroy_node()
        rclpy.shutdown()
