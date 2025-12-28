import socket
import configparser
import os


def run_udp_sender():
    # 1. 初始化配置解析器
    config = configparser.ConfigParser()

    # 确保读取当前目录下的 config.ini
    config_path = os.path.join(os.path.dirname(__file__), 'config.ini')
    if not os.path.exists(config_path):
        print(f"错误: 找不到配置文件 {config_path}")
        return

    config.read(config_path, encoding='utf-8')

    # 2. 从配置中读取参数
    try:
        local_ip = config.get('Network', 'local_ip')
        local_port = config.getint('Network', 'local_port')
        target_ip = config.get('Network', 'target_ip')
        target_port = config.getint('Network', 'target_port')
        message = config.get('Data', 'message')
    except Exception as e:
        print(f"配置文件解析失败: {e}")
        return

    # 3. 创建 UDP 套接字 (AF_INET 为 IPv4, SOCK_DGRAM 为 UDP)
    # 使用 with 语句可以确保程序退出时自动关闭 socket
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            # 4. 绑定本机 IP 和端口 (这一步是你要求的“指定本机端口”)
            # 注意：如果 local_ip 写 0.0.0.0 则表示绑定所有网卡
            s.bind((local_ip, local_port))
            print(f"成功绑定本机地址: {local_ip}:{local_port}")

            # 5. 发送数据
            # UDP 是无连接的，直接向目标地址发送字节流
            print(f"正在发送数据到 {target_ip}:{target_port} ...")
            s.sendto(message.encode('utf-8'), (target_ip, target_port))

            print("发送完成！")

        except Exception as e:
            print(f"发送过程中出错: {e}")


if __name__ == "__main__":
    run_udp_sender()