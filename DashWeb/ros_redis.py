import json
import rclpy
from rclpy.node import Node
from mc_core_interface.msg import ServoFeedback
import redis
import datetime


class RedisWriter(Node):
    def __init__(self):
        super().__init__('redis_writer')

        # 连接到 Redis
        self.redis_client = redis.StrictRedis(host='192.168.10.45', port=6379, db=0)

        self.subscription = self.create_subscription(
            msg_type=ServoFeedback,
            topic='servo_data_msg',
            callback=self.listener_callback,
            qos_profile=10
        )

        self.k = 0
        self.batch_size = 200  # 批量插入的大小
        self.buffer = []  # 缓存消息

    def listener_callback(self, msg):
        """处理接收到的消息并写入 Redis"""
        timestamp_ms = int(msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec / 1000000)
        req_msg = {
            "Timestamp": datetime.datetime.fromtimestamp(timestamp_ms / 1000.0).isoformat(),
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
        if self.k % 5 == 0:
            self.buffer.append(req_msg)  # 将消息添加到缓冲区


        if len(self.buffer)%self.batch_size == 0:
            self.bulk_insert()  # 当达到批量大小时进行批量插入

    def bulk_insert(self):
        """将缓存的消息批量写入 Redis"""
        if not self.buffer:
            return

        try:
            # 使用 Redis 列表存储数据
            for msg in self.buffer:
                self.redis_client.lpush('sensor_data', json.dumps(msg))
                self.redis_client.publish('sensor_update', json.dumps(msg))# 发布消息

                # 确保列表只保留最新的 3000 条数据
                if self.redis_client.llen('sensor_data') > 5000:
                    self.redis_client.rpop('sensor_data')  # 弹出最旧的数据

            self.get_logger().info(f"Inserted {len(self.buffer)} messages to Redis")
            self.buffer.clear()  # 清空缓冲区
        except redis.RedisError as e:
            self.get_logger().error(f"Redis error during bulk insert: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

    def __del__(self):
        """在节点销毁时进行清理"""
        try:
            if self.buffer:
                self.bulk_insert()  # 清理时插入剩余的消息
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
        redis_writer.bulk_insert()  # 确保在关闭前写入剩余数据
        redis_writer.destroy_node()
        rclpy.shutdown()
