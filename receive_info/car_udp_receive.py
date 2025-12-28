# -*- coding: utf-8 -*-
import socket
import json
import xxhash
import sys
import os


def start_receiver():
    # 确保端口与发送端 JSON 配置中的 target_port 一致
    port = 55555

    # 创建 UDP Socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Windows 下绑定 0.0.0.0 即可接收局域网广播
    try:
        sock.bind(("0.0.0.0", port))
    except Exception as e:
        print(f"无法绑定端口 {port}: {e}")
        print("请检查该端口是否被其他程序（如 NetAssist）占用。")
        return

    total_received = 0
    corrupted_count = 0

    # 清个屏，让界面好看点
    os.system('cls' if os.name == 'nt' else 'clear')
    print("=" * 50)
    print(f"Windows UDP 接收监控端已启动")
    print(f"监听端口: {port}  |  等待来自 Xavier (192.168.1.102) 的数据...")
    print("=" * 50)

    while True:
        try:
            # 1. 接收原始数据
            raw_data, addr = sock.recvfrom(4096)
            total_received += 1

            # 2. 解析 JSON
            packet = json.loads(raw_data.decode('utf-8'))
            received_data = packet.get("data", {})
            received_hash = packet.get("hash", "")

            # 3. 本地计算哈希进行比对 (注意：Python 3 默认字符处理与 Python 2 不同)
            # 使用与发送端完全一致的序列化参数
            local_json_str = json.dumps(received_data, sort_keys=True)
            local_hash = xxhash.xxh3_64_hexdigest(local_json_str)

            # 4. 校验逻辑
            if local_hash == received_hash:
                status = "√ [正常]"
                score = received_data.get('score', 0.0)
                road_status = received_data.get('status', 'Unknown')
                seq = received_data.get('seq', 'N/A')

                info = f"序号:{seq} | 平整度:{score:.4f} | 状态:{road_status}"
            else:
                status = "× [损坏]"
                corrupted_count += 1
                info = "HASH 校验不匹配，数据可能在传输中丢包或错乱"

            # 5. 打印输出
            Damage_rate = (corrupted_count / total_received) * 100
            print('Damage_rate:' , Damage_rate,'%',sep="")
            print(f"{status} {info} | 来自: {addr[0]}")
            print('received_data',received_data)

        except KeyboardInterrupt:
            print("\n用户终止程序。")
            break
        except Exception as e:
            print(f"解析错误: {e}")


if __name__ == "__main__":
    start_receiver()