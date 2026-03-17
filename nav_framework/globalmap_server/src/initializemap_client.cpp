#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/OccupancyGrid.h"
#include "globalmap_server/initialize_map.h"

//////////////////////////////////////////////////////////////////////
// 参数：
std::string map_topic;
std_msgs::Bool msg;
//
//////////////////////////////////////////////////////////////////////
// 初始化地图消息,后续替换成callback的地图
nav_msgs::OccupancyGrid map;

void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    ROS_DEBUG("Map received");
    map = *msg;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "initializemap_client");
    ros::NodeHandle nh;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    // 创建客户端
    ros::ServiceClient client = nh.serviceClient<globalmap_server::initialize_map>("Initialize_Map");
    // 创建消息类型
    globalmap_server::initialize_map srv;
    // 读取参数
    nh.param<std::string>("mode1_map_topic", map_topic, "/mode1/occupancy_grid");
    ROS_WARN("Map topic: %s", map_topic.c_str());
    // 初始化subscriber
    ros::Subscriber map_sub = nh.subscribe(map_topic, 1, &MapCallback);
    ros::Publisher global_status = nh.advertise<std_msgs::Bool>("/mode1/shutdown_request", 1);
    // 消息存储
    // srv.request.global_map = map;
    ROS_DEBUG("[ SERVICE ] initialize map client started");

    while (map.data.size() == 0 && ros::ok())
    {
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    ros::Duration(1).sleep();
    srv.request.global_map = map;
    bool status;
    while (ros::ok())
    {
        status = client.call(srv);
        // 调用服务
        if (status)
        {
            ROS_DEBUG("Initialize_Map service called");
            if (srv.response.initialized)
            {
                ROS_DEBUG(" [ OK ] Map initialized");
                msg.data = true;
                global_status.publish(msg);
                break;
            }
            else
            {
                ROS_WARN(" [ FAILED ] Map already initialized or resolution mismatch");
                msg.data = false;
                global_status.publish(msg);
            }
        }
        else
        {
            ROS_ERROR("[ ERROR ] Failed to call service Initialize_Map");
            msg.data = false;
            global_status.publish(msg);
        }
        ros::Duration(0.5).sleep();
    }
    ros::Duration(1).sleep();
    return 0;
}