#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import socket
import json
import xxhash
import sys
from road_monitor_car_receive.msg import RoadQuality # 注意这里包名已改
from std_msgs.msg import Header

class UdpToRosBridge:
    def __init__(self):
        rospy.init_node('udp_bridge_node', anonymous=True)
        
        # 从参数服务器读取配置，如果没有则使用默认值
        self.port = rospy.get_param('~udp_port', 55555)
        self.ip = rospy.get_param('~target_ip', '0.0.0.0')
        self.target_frame = rospy.get_param('~frame_id', 'base_link')
        
        self.pub = rospy.Publisher('/road_quality', RoadQuality, queue_size=10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5) 
        
        try:
            self.sock.bind((self.ip, self.port))
            rospy.loginfo("UDP 桥接节点启动 | 监听: %s:%d", self.ip, self.port)
        except Exception as e:
            rospy.logerr("无法绑定端口: %s", e)
            sys.exit(1)

    def run(self):
        while not rospy.is_shutdown():
            try:
                raw_data, addr = self.sock.recvfrom(4096)
                packet = json.loads(raw_data.decode('utf-8'))
                
                received_data = packet.get("data", {})
                received_hash = packet.get("hash", "")

                # 校验哈希
                local_json_str = json.dumps(received_data, sort_keys=True)
                local_hash = xxhash.xxh3_64_hexdigest(local_json_str)
                rospy.loginfo("Local: %s | Received: %s", local_hash, received_hash)
                if local_hash == received_hash:
                    msg = RoadQuality()
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = self.target_frame
                    msg.score = float(received_data.get('score', 0.0))
                    msg.status = received_data.get('status', 'Unknown')
                    
                    self.pub.publish(msg)
                else:
                    rospy.logwarn("哈希校验不匹配，数据包损坏")

            except socket.timeout:
                continue
            except Exception as e:
                rospy.logerr("接收出错: %s", e)

if __name__ == '__main__':
    try:
        bridge = UdpToRosBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass
