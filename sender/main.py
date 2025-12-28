import time
import json
import xxhash
import random
from udp_handler import UDPBufferedSender


def run_sender():
    # 初始化之前写的通用发送器
    sender = UDPBufferedSender()
    print("XXH3 校验发送端已启动...")

    try:
        for i in range(1, 101):
            # 1. 准备原始业务数据
            business_data = {
                "msg_id": i,
                "device": "Sensor_Alpha",
                "value": round(random.uniform(10.5, 99.9), 2),
                "timestamp": time.time()
            }

            # 2. 将数据转为 JSON 字符串，用于计算哈希
            # 注意：sort_keys=True 保证字段顺序一致，防止哈希值因顺序变动而改变
            json_str = json.dumps(business_data, sort_keys=True)

            # 3. 计算 XXH3 校验值 (使用 64 位哈希)
            checksum = xxhash.xxh3_64_hexdigest(json_str)

            # 4. 构建包含校验位的最终数据包
            final_packet = {
                "data": business_data,
                "hash": checksum
            }

            # 5. 压入缓冲队列
            # 注意：此时 put 接收一个字典，udp_handler 会自动处理序列化
            sender.put(**final_packet)

            # 模拟高速发送
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n发送端停止")


if __name__ == "__main__":
    run_sender()