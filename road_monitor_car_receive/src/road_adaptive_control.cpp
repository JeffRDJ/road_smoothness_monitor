#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <road_monitor_car_receive/RoadQuality.h>
#include <mutex>

// 全局变量
ros::Publisher g_vel_pub;
std::mutex g_mutex;

// 运动速度参数（全局变量，通过参数服务器更新）
double g_normal_speed = 0.25;
double g_moderate_speed = 0.15;
double g_bump_speed = 0.08;

// 判断阈值参数（新增）
double g_moderate_threshold = 0.3;
double g_bump_threshold = 0.8;

const double DEFAULT_ANGULAR = 0.0;
double g_current_target_speed = 0.25;
std::string g_current_status = "Smooth";

void publishCmd(double linear, double angular) {
    geometry_msgs::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;
    g_vel_pub.publish(cmd);
}

void roadQualityCB(const road_monitor_car_receive::RoadQuality::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(g_mutex);
    double score = msg->score;
    std::string status = msg->status;
    
    // 使用从参数服务器读取的阈值进行逻辑判断
    if (status == "Very Rough / Bump" || score > g_bump_threshold) {
        g_current_target_speed = g_bump_speed;
        g_current_status = "Very Rough";
    } 
    else if (status == "Moderate" || score > g_moderate_threshold) {
        g_current_target_speed = g_moderate_speed;
        g_current_status = "Moderate";
    } 
    else {
        g_current_target_speed = g_normal_speed;
        g_current_status = "Smooth";
    }

    ROS_INFO_THROTTLE(1.0, "[Control] Score: %.3f | Thresh: (M:%.1f, B:%.1f) | Speed: %.2f m/s", 
                      score, g_moderate_threshold, g_bump_threshold, g_current_target_speed);
}

void controlTimerCallback(const ros::TimerEvent&) {
    std::lock_guard<std::mutex> lock(g_mutex);
    publishCmd(g_current_target_speed, DEFAULT_ANGULAR);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "road_adaptive_control_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~"); // 私有句柄

    // 读取速度配置
    pnh.param("normal_speed", g_normal_speed, 0.25);
    pnh.param("moderate_speed", g_moderate_speed, 0.15);
    pnh.param("bump_speed", g_bump_speed, 0.08);

    // 读取阈值配置 (新增)
    pnh.param("moderate_threshold", g_moderate_threshold, 0.3);
    pnh.param("bump_threshold", g_bump_threshold, 0.8);

    g_current_target_speed = g_normal_speed; 

    g_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber sub = nh.subscribe("/road_quality", 10, roadQualityCB);
    ros::Timer timer = nh.createTimer(ros::Duration(0.05), controlTimerCallback);

    ROS_INFO("Road Control Node Ready.");
    ROS_INFO("Thresholds -> Moderate: %.2f, Bump: %.2f", g_moderate_threshold, g_bump_threshold);

    ros::spin();
    return 0;
}
