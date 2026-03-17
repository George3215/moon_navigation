#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "globalmap_server/get_globalmap.h"

//////////////////////////////////////////////////////////////////////

std::string mode_topic;
bool mode_received = false;
std::string mode;
double local_publish_interval;
ros::Time last_local_publish_time;
bool verbose_logging = false;

//////////////////////////////////////////////////////////////////////

void ModeCallback(const std_msgs::String::ConstPtr &msg)
{
    if (msg->data == "Mode 1")
    {
        //ROS_INFO("Mode 1");
        mode_received = true;
        mode = "Mode 1";
    }
    else if (msg->data == "Mode 2")
    {
        //ROS_INFO("Mode 2");
        mode_received = true;
        mode = "Mode 2";
    }
    else if (msg->data == "Mode 3")
    {
        //ROS_INFO("Mode 3");
        mode_received = true;
        mode = "Mode 3";
    }
    else
    {
        ROS_ERROR("Mode Error");
        mode_received = true;
        mode = "Mode Error";
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "getlocalmap_client");
    ros::NodeHandle nh;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    // 创建客户端
    ros::ServiceClient client = nh.serviceClient<globalmap_server::get_globalmap>("Get_Localmap");
    // 创建消息类型
    globalmap_server::get_globalmap srv;
    // 读取参数
    nh.param<std::string>("navigation_mode_topic", mode_topic, "/mode_topic");
    ROS_WARN("Mode topic: %s", mode_topic.c_str());
    nh.param<double>("publish_topic/local_publish_interval", local_publish_interval, 1.0);
    ROS_WARN("Local publish interval: %f", local_publish_interval);
    nh.param<bool>("verbose_logging", verbose_logging, false);
    ROS_WARN("Verbose logging: %s", verbose_logging ? "enabled" : "disabled");
    // 初始化subscriber
    ros::Subscriber mode_sub = nh.subscribe<std_msgs::String>(mode_topic, 1, &ModeCallback);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ROS_DEBUG("[ SERVICE ] Get_Localmap client started");
    ros::Duration(1).sleep();
    // srv的赋值
    srv.request.global = false;
    bool status;
    last_local_publish_time = ros::Time::now();
    while (ros::ok())
    {
        // 如果时间间隔未到，则等待
        if ((ros::Time::now() - last_local_publish_time).toSec() < local_publish_interval)
        {
            if (verbose_logging)
                ROS_INFO("Localmap publish interval not reached");
            ros::Duration(1).sleep();
            continue;
        }
        // 如果接收到mode消息，则进行服务调用
        if (mode_received)
        {
            // 检查模式类别
            if (mode == "Mode 1")
            {
                if (verbose_logging)
                    ROS_INFO("Mode 1, No need to get localmap");
                ros::Duration(1).sleep();
                continue;
            }
            else if (mode == "Mode 2" || mode == "Mode 3")
            {
                if (verbose_logging)
                    ROS_INFO("Get localmap service called");
                // 调用服务
                status = client.call(srv);
                if (status)
                {
                    if (verbose_logging)
                        ROS_INFO("Get Localmap service called");
                    if (srv.response.success)
                    {
                        if (verbose_logging)
                            ROS_DEBUG(" [ OK ] Localmap Published");
                        last_local_publish_time = ros::Time::now();
                    }
                    else
                    {
                        ROS_WARN(" [ FAILED ] Call Service ok,BUT Localmap not Published");
                    }
                }
                else
                {
                    ROS_ERROR("[ ERROR ] Failed to call service Get_Localmap");
                }
                ros::Duration(1).sleep();
                continue;
            }
            else
            {
                ROS_ERROR("Mode Error");
                ros::Duration(1).sleep();
                continue;
            }
        }
        else
        {
            //ROS_INFO("Waiting for mode message");
            ros::Duration(1).sleep();
        }
    }
    return 0;
}