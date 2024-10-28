import rclpy
from rclpy.node import Node
from mc_core_interface.msg import ServoFeedback
from pymongo import MongoClient
import datetime


class MongoDBWriter(Node):
    def __init__(self):
        super().__init__('mongodb_writer')

        # 连接到MongoDB
        self.client = MongoClient('mongodb://192.168.10.45:27017/')  # 替换为您的MongoDB地址
        self.db = self.client['AiRobDb']  # 替换为您的数据库名称
        self.collection = self.db['Rosmsg']

        # 创建 TTL 索引以优化查询并自动删除过期数据
        self.collection.create_index("Timestamp", expireAfterSeconds=600)  # 设置为 600 秒

        self.subscription = self.create_subscription(
            msg_type=ServoFeedback,
            topic='servo_data_msg',
            callback=self.listener_callback,
            qos_profile=10
        )

        self.k = 0
        self.batch_size = 1000  # 批量插入的大小
        self.buffer = []  # 缓存消息

    def listener_callback(self, msg):
        """处理接收到的消息并写入MongoDB"""
        timestamp_ms = int(msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec / 1000000)
        req_msg = {
            "_id": timestamp_ms,  # 使用毫秒时间戳作为_id
            "Timestamp": datetime.datetime.fromtimestamp(timestamp_ms / 1000.0),
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
        if self.k % 5 != 0:
            return

        self.buffer.append(req_msg)

        # 当缓冲区达到一定大小时批量插入
        if len(self.buffer) >= self.batch_size:
            self.bulk_insert()

    def bulk_insert(self):
        """批量插入缓冲区中的消息"""
        if self.buffer:
            try:
                self.collection.insert_many(self.buffer, ordered=False)  # 批量插入
                self.get_logger().info(f"Inserted {len(self.buffer)} messages to MongoDB")
            except Exception as e:
                self.get_logger().error(f"Error inserting messages to MongoDB: {e}")
            finally:
                self.buffer.clear()  # 清空缓冲区

    def __del__(self):
        """在节点销毁时插入剩余的消息"""
        try:
            if hasattr(self, 'buffer') and self.buffer:
                self.bulk_insert()
        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {e}")


if __name__ == '__main__':
    rclpy.init(args=None)
    mongodb_writer = MongoDBWriter()

    try:
        rclpy.spin(mongodb_writer)
    except KeyboardInterrupt:
        mongodb_writer.get_logger().info("Node interrupted by user")
    finally:
        mongodb_writer.destroy_node()
        rclpy.shutdown()
