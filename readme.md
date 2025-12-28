##   receive_info
- car_udp_receive.py 负责监听55555端口接受小车端广播的数据，
- onfig_recv.ini 配置监听地址和端口，以及每次接收的最大字节数
## road_monitor/scripts
- udp_forward_node.py 订阅发布imu话题/imu/data_raw获取imu数据 计算获得路面平整度信息
- road_monitor_node.py 以及udp_forward_node.py负责将udp_forward_node.py发布的路面平整度信息使用udp协议广播到其他设备的55555端口
- config\config.json 配置 发送端的和目标ip与端口
- msg\RoadQuality.msg 自定义消息类型
## images 一些实验的截图
## sender 没啥用，一些调试产物
