# 关于松林小车使用imu数据进行路面平整度计算并使用udp进行广播的代码实现
# !!!所有设备均需要在同一局域网相同网段下
# receive_info  电脑端   模拟的rsu设备
- car_udp_receive.py 负责监听55555端口接受小车端广播的数据，
- onfig_recv.ini 配置监听地址和端口，以及每次接收的最大字节数
# 小车A road_monitor/scripts 
- udp_forward_node.py 订阅发布imu话题/imu/data_raw获取imu数据 计算获得路面平整度信息
- road_monitor_node.py 以及udp_forward_node.py负责将udp_forward_node.py发布的路面平整度信息使用udp协议广播到其他设备的55555端口
- config\config.json 配置 发送端的和目标ip与端口
- msg\RoadQuality.msg 自定义消息类型
#  小车B road_monitor_car_receive  
## config/
- control_config.yaml    关于路面平整度对于控制小车的运动的阈值及相应推荐车速
- udp_config.yaml 小车端 UDP 接收的网路配置
## launch/
- receive.launch 启动udp接受服务并开始控制小车的运动
## msg/
- RoadQuality.msg  自定义ros话题数据结构
## scripts/
- udp_receiver_node.py  接受udp数据的程序
## src/
- src\road_adaptive_control.cpp  控制小车运动的c++代码
 
## images 一些实验的截图 和演示gif
## sender 没啥用，一些调试产物
# 项目部署教程
# 电脑端 （路测rsu）
连接到小车A所在的网络
直接运行 car_udp_receive.py脚本即可
# 小车A 部署road_monitor这个ros功能包
- 编译该功能包

        rosrun road_monitor road_monitor_node.py  # 计算路面平整度信息

- 新开终端

        roslaunch road_monitor udp_forward_node.py  向55555端口广播数据


# 小车B 部署road_monitor_car_receive这个ros功能包
- 将小车B连接到小车A同在的局域网
对于松林小车 需要

        sudo modprobe gs_usb

        sudo ip link set can0 up type can bitrate 500000
        # 启动小车底盘驱动
        roslaunch scout_bringup scout_minimal.launch
- 编译该功能包

        roslaunch road_monitor_car_receive receive.launch
- 正常情况下可以接收到数据并控制小车运动，若未收到消息，可检查：

        · 小车网络配置问题，
        ·值得提出的是如果小车A是使用的小车自带的路由器，小车B也连接了这个路由器提供的wifi，那么此时需要将小车B的eth0(连接了B小车自带的路由器的网卡)给关掉（因为默认情况下ip都是一样的会冲突）如下命令：
        # 关闭 （临时生效）
        sudo ip link set eth0 down
        # 开启(临时生效)
        sudo ip link set eth0 up
        是否是连接在小车A发送消息的那张网卡所代表的网络

