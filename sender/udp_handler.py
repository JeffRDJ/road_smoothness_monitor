import socket
import threading
import time
import json
import os
from collections import deque


class UDPBufferedSender:
    def __init__(self, config_path="config.json"):
        # 1. 加载配置
        self.config = self._load_config(config_path)

        # 2. 网络参数初始化
        self.target_addr = (self.config['target_ip'], self.config['target_port'])
        self.local_addr = (self.config['local_ip'], self.config['local_port'])

        # 3. 核心缓冲区：maxlen 实现自动丢弃旧数据
        self.buffer = deque(maxlen=self.config['buffer_capacity'])
        self.lock = threading.Lock()

        # 4. 创建 UDP Socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind(self.local_addr)
            print(f"[Init] UDP发送端就绪，绑定本地地址: {self.sock.getsockname()}")
            print(f"[Init] 目标发送地址: {self.target_addr}")
        except Exception as e:
            print(f"[Error] 绑定失败: {e}")

        # 5. 启动后台消费者线程
        self.running = True
        self.worker_thread = threading.Thread(target=self._send_loop, daemon=True)
        self.worker_thread.start()

    def _load_config(self, path):
        if not os.path.exists(path):
            return {
                "target_ip": "127.0.0.1", "target_port": 8080,
                "local_ip": "0.0.0.0", "local_port": 0,
                "buffer_capacity": 100, "check_interval": 0.01
            }
        with open(path, 'r', encoding='utf-8') as f:
            return json.load(f)

    def _send_loop(self):
        """子线程逻辑：不断从队列中提取并发送"""
        while self.running:
            item = None
            with self.lock:
                if self.buffer:
                    item = self.buffer.popleft()  # 提取最早的数据

            if item:
                try:
                    # 序列化：将字典转为 JSON 字节流
                    packet = json.dumps(item).encode('utf-8')
                    self.sock.sendto(packet, self.target_addr)
                except Exception as e:
                    print(f"[Send Error] {e}")
            else:
                # 队列为空，短暂休眠释放 CPU
                time.sleep(self.config['check_interval'])

    def put(self, **kwargs):
        """
        供其他程序调用的核心接口。
        支持传入任意数量的键值对，例如 put(id=1, temp=20.5)
        """
        # 自动补全一个精确到毫秒的时间戳
        if "timestamp" not in kwargs:
            kwargs["timestamp"] = round(time.time(), 3)

        with self.lock:
            # deque 会自动处理 maxlen，如果满了，最左边的会被自动挤掉
            self.buffer.append(kwargs)

    def close(self):
        self.running = False
        self.sock.close()