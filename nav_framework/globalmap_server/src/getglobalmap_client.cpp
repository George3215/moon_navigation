#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "globalmap_server/get_globalmap.h"

//////////////////////////////////////////////////////////////////////

bool verbose_logging = false;

// void ModeCallback(const std_msgs::String::ConstPtr &msg)
// {
//     if (msg->data == "Mode 1")
//     {
//         ROS_INFO("Mode 1");
//         mode_received = true;
//     }
//     else if (msg->data == "Mode 2")
//     {
//         ROS_INFO("Mode 2");
//         mode_received = true;
//     }
//     else if (msg->data == "Mode 3")
//     {
//         ROS_INFO("Mode 3");
//         mode_received = true;
//     }
//     else
//     {
//         ROS_ERROR("Mode Error");
//     }
// }

int main(int argc, char **argv)
{

    ros::init(argc, argv, "getglobalmap_client");
    ros::NodeHandle nh;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    // 创建客户端
    ros::ServiceClient client = nh.serviceClient<globalmap_server::get_globalmap>("Get_Globalmap");
    // 创建消息类型
    globalmap_server::get_globalmap srv;
    // 读取参数
    nh.param<bool>("verbose_logging", verbose_logging, false);
    ROS_WARN("Verbose logging: %s", verbose_logging ? "enabled" : "disabled");
    ROS_DEBUG("[ SERVICE ] Get_globalmap client started");
    ros::Duration(1).sleep();
    // srv的赋值
    srv.request.global = true;
    // 调用服务
    while (ros::ok())
    {
        bool status = client.call(srv);
        if (status)
        {
            if (verbose_logging)
                ROS_INFO("Get Globalmap service called");
            if (srv.response.success)
            {
                ROS_DEBUG(" [ OK ] Globalmap and Goal Published");
                break;
            }
            else
            {
                ROS_WARN(" [ FAILED ] Call Service ok,BUT Globalmap and Goal not Published");
                ROS_WARN(" [ FAILED ] Check if Globalmap or Goal not Initialize?");
                ros::Duration(1).sleep();
            }
        }
        else
        {
            ROS_ERROR("[ ERROR ] Failed to call service Get_globalmap");
        }
        ros::Duration(0.5).sleep();
    }
    ros::Duration(1).sleep();
    return 0;
}