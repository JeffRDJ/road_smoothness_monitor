cmake_minimum_required(VERSION 3.0.2)
project(road_monitor_car_receive)

# 1. 必须包含 roscpp 和 geometry_msgs
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

# 2. 消息定义
add_message_files(
  FILES
  RoadQuality.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# 3. 运行依赖
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 4. 【核心修改】先定义可执行文件
add_executable(road_adaptive_control_node src/road_adaptive_control.cpp)

# 5. 【核心修改】再添加依赖（确保消息头文件先生成）
add_dependencies(road_adaptive_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 6. 【核心修改】必须链接库文件
target_link_libraries(road_adaptive_control_node
  ${catkin_LIBRARIES}
)
