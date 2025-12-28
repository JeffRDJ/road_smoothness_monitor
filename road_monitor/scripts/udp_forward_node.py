#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import json
import xxhash
import time
from road_monitor.msg import RoadQuality  # 确保包名正确
from udp_handler import UDPBufferedSender

class RoadQualityForwarder:
    def __init__(self):
        # 1. 初始化 ROS 节点
        rospy.init_node('road_quality_udp_forwarder', anonymous=True)

        # 2. 初始化你的 UDP 发送器
        self.sender = UDPBufferedSender()
        
        # 3. 订阅 /road_quality 话题
        rospy.Subscriber('/road_quality', RoadQuality, self.callback)
        
        rospy.loginfo("路面质量 UDP 转发器已启动 (带 XXH3 校验)")

    def callback(self, msg):
        try:
            # 1. 从 ROS 消息中提取业务数据
            business_data = {
                "score": round(float(msg.score), 4),
                "status": msg.status,
                "device": "Experimental_Car",
                # 使用消息 Header 中的时间戳（转为浮点秒）
                "timestamp": msg.header.stamp.to_sec(),
                "seq": msg.header.seq
            }

            # 2. 序列化并计算 XXH3 校验值
            # sort_keys=True 保证哈希一致性
            json_str = json.dumps(business_data, sort_keys=True)
            checksum = xxhash.xxh3_64_hexdigest(json_str)

            # 3. 构建最终发送包
            final_packet = {
                "data": business_data,
                "hash": checksum
            }

            # 4. 通过 UDP 发送器压入缓冲
            # 假设你的 sender.put 接受关键字参数
            self.sender.put(**final_packet)
            
            rospy.logdebug("已发送数据包 seq: %d, hash: %s", msg.header.seq, checksum)

        except Exception as e:
            rospy.logerr("数据转发失败: %s", str(e))

    def run(self):
        # 保持节点运行
        rospy.spin()

if __name__ == "__main__":
    try:
        forwarder = RoadQualityForwarder()
        forwarder.run()
    except rospy.ROSInterruptException:
        print("\n转发端停止")
