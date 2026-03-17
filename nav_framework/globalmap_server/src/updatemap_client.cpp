#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "globalmap_server/update_map.h"
#include "std_msgs/String.h"

//////////////////////////////////////////////////////////////////////
// 参数配置：
std::string mode2_map_topic;
std::string mode3_map_topic;
std::string navigation_mode_topic;
//////////////////////////////////////////////////////////////////////

// 消息模式设置，同时充当进程锁
// mode：0-空闲中，1-正在处理service，2-模式2，3-模式3
uint8_t mode = 0;
// 初始化地图消息
nav_msgs::OccupancyGrid map;
std::string nav_mode = "invalid";

void Mode2MapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    // 软进程锁：如果mode=1代表正在进行service处理，不接受新的地图
    if (mode == 0)
    {

        if (nav_mode == "Mode 2")
        {
            ROS_DEBUG("Mode2 Map received");
            mode = 2;
            map = *msg;
        }
        else
        {
            ROS_WARN("Mode2 Map received, but not in mode2");
        }
    }
    else
    {
        ROS_WARN("Mode2 Map received, but service is busy");
    }
}

void Mode3MapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    // 软进程锁：如果mode=1代表正在进行service处理，不接受新的地图
    if (mode == 0)
    {
        if (nav_mode == "Mode 3")
        {
            ROS_DEBUG("Mode3 Map received");
            mode = 3;
            map = *msg;
        }
        else
        {
            ROS_WARN("Mode3 Map received, but not in mode3");
        }
    }
    else
    {
        ROS_WARN("Mode3 Map received, but service is busy");
    }
}

void NavigationModeCallback(const std_msgs::String::ConstPtr &msg)
{
    //ROS_INFO("Navigation Mode: %s", msg->data.c_str());
    nav_mode = msg->data;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "updatemap_client");
    ros::NodeHandle nh;
    // 创建客户端
    ros::ServiceClient client = nh.serviceClient<globalmap_server::update_map>("Update_Map");
    // 创建消息类型
    globalmap_server::update_map srv;
    // 读取参数
    nh.param<std::string>("mode2_map_topic", mode2_map_topic, "/mode2/occupancy_grid");
    nh.param<std::string>("mode3_map_topic", mode3_map_topic, "/mode3/occupancy_grid");
    nh.param<std::string>("navigation_mode_topic", navigation_mode_topic, "/mode");
    ROS_WARN("Mode2 Map topic: %s", mode2_map_topic.c_str());
    ROS_WARN("Mode3 Map topic: %s", mode3_map_topic.c_str());
    ROS_WARN("Navigation Mode topic: %s", navigation_mode_topic.c_str());
    // 初始化subscriber
    ros::Subscriber mode2_map_sub = nh.subscribe(mode2_map_topic, 1, &Mode2MapCallback);
    ros::Subscriber mode3_map_sub = nh.subscribe(mode3_map_topic, 1, &Mode3MapCallback);
    ros::Subscriber navigation_mode_sub = nh.subscribe(navigation_mode_topic, 1, &NavigationModeCallback);
    // 消息存储
    // srv.request.global_map = map
    ros::AsyncSpinner spinner(3);
    ros::Duration(1).sleep();
    spinner.start();
    ROS_DEBUG("[ SERVICE ] update map client started");
    // verbose_logging 可通过外部 rosparam 开启 debug 级日志
    // 如果没有查询到nav_mode，则等待
    while(nav_mode == "invalid" && ros::ok())
    {
        //ROS_INFO("Waiting for navigation mode");
        ros::Duration(1).sleep();
    }
    while (ros::ok())
    {
        // 如果mode进程空闲，则spin
        ros::spinOnce();
        // 如果mode不为0，则进入service处理模式
        if (mode != 0)
        {
            srv.request.mode = mode; // set mode
            mode = 1;                // set service busy
            // 地图消息存储
            // 清空之前的数据
            srv.request.update_map.data.clear();
            srv.request.update_map.data.resize(map.info.height * map.info.width);
            srv.request.update_map = map;
            bool status = client.call(srv);
            // 调用服务
            if (status)
            {
                // ROS_INFO("Update_Map service called");
                if (srv.response.updated)
                {
                    ROS_DEBUG(" [ OK ] Map Updated");
                }
                else
                {
                    ROS_WARN(" [ FAILED ] Call Service ok,but update failed.");
                }
            }
            else
            {
                ROS_ERROR("[ ERROR ] Failed to call service Update_Map");
            }
            // 服务调用结束，进程空闲
            mode = 0;
        }
    }

    return 0;
}