#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include "globalmap_server/initialize_path.h"

//////////////////////////////////////////////////////////////////////

// 服务端作用：启动后接收对应规划器的路径，并以service的形式发送给map_server
// server返回路径是否成功接收，并在client中进行显示
std::string global_path_topic;
nav_msgs::Path global_path;
// bool mode_received = false;

//////////////////////////////////////////////////////////////////////

void PathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    global_path = *msg;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "initialize_globalpath_client");
    ros::NodeHandle nh;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    // 创建客户端
    ros::ServiceClient client = nh.serviceClient<globalmap_server::initialize_path>("Initialize_Path");
    // 创建消息类型
    globalmap_server::initialize_path srv;
    // 读取参数
    nh.param<std::string>("/path_subscribe_topic/global_path", global_path_topic, "/global_path");
    ROS_WARN("Global path topic: %s", global_path_topic.c_str());
    // 初始化subscriber
    ros::Subscriber globalpath_sub = nh.subscribe<nav_msgs::Path>(global_path_topic, 1, &PathCallback);
    ROS_DEBUG("[ SERVICE ] Initialize_Path client started");

    while(global_path.poses.size() == 0 && ros::ok())
    {
        // ROS_INFO("Waitinsg for global path");
        ros::Duration(1).sleep();
        ros::spinOnce();
    }

    // srv的赋值
    srv.request.global = true;
    srv.request.global_path = global_path;
    // 调用服务
    bool status = client.call(srv);

    if (status)
    {
        ROS_DEBUG("Initialize_Path service called");
        if (srv.response.success)
        {
            ROS_DEBUG(" [ OK ] Path initialized");
        }
        else
        {
            ROS_WARN(" [ FAILED ] Path already initialized");
        }
    }
    else
    {
        ROS_ERROR("[ ERROR ] Failed to call service Initialize_Path");
    }
    ros::Duration(1).sleep();
    return 0;
}