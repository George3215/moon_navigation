#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/OccupancyGrid.h>

// 功能：将traversability map转换为occupancy grid
// 输入：traversability map或者其他形式的gridmap
// 输出：occupancy grid

class OccupancyGridConverter
{
public:
    OccupancyGridConverter(ros::NodeHandle &nodehandle);

private:
    ros::NodeHandle nh;
    ros::Publisher occupancyGridPub;
    ros::Subscriber gridMapSub;
    ros::Subscriber shutdownSub;
    int threshold;
    double factor;
    bool get_01_value;

    double side_invalid_border,back_invalid_border,front_all_valid_border;

    // std::string GRIDMAP_TOPIC = "/traversability_estimation/traversability_map";
    // std::string OCCUPANCYGRID_TOPIC = "/occupancy_grid";
    std::string GRIDMAP_TOPIC, OCCUPANCYGRID_TOPIC, transform_layer, map_frame;

    void gridMapCallback(const grid_map_msgs::GridMap::ConstPtr &gridMapMsg);
    void shutdownCallback(const std_msgs::Bool &message);
};

// 构造函数
OccupancyGridConverter::OccupancyGridConverter(ros::NodeHandle &nodehandle)
{
    nh = nodehandle;
    nh.param<std::string>("trav2occ/grid_map_topic", GRIDMAP_TOPIC, "/grid_map_topic");
    nh.param<std::string>("trav2occ/occupancy_grid_topic", OCCUPANCYGRID_TOPIC, "/occupancygrid");
    nh.param<std::string>("trav2occ/transform_layer", transform_layer, "layer");
    nh.param<bool>("trav2occ/get_01_value", get_01_value, false);
    nh.param<int>("trav2occ/threshold", threshold, 50);
    nh.param<double>("trav2occ/downscale_cost_factor", factor, 1.0);
    nh.param<std::string>("trav2occ/map_frame", map_frame, "map");
    nh.param<double>("trav2occ/side_invalid_border", side_invalid_border, 1.0);
    nh.param<double>("trav2occ/back_invalid_border", back_invalid_border, 1.0);
    nh.param<double>("trav2occ/front_all_valid_border", front_all_valid_border, 1.0);
    ROS_WARN("grid_map_topic: %s", GRIDMAP_TOPIC.c_str());
    ROS_WARN("occupancy_grid_topic: %s", OCCUPANCYGRID_TOPIC.c_str());
    ROS_WARN("transform_layer: %s", transform_layer.c_str());
    ROS_WARN("get_01_value: %d", get_01_value);
    ROS_WARN("threshold: %d", threshold);
    ROS_WARN("downscale_cost_factor: %f", factor);
    ROS_WARN("side_invalid_border: %f", side_invalid_border);
    ROS_WARN("back_invalid_border: %f", back_invalid_border);
    ROS_WARN("front_all_valid_border: %f", front_all_valid_border);

    gridMapSub = nh.subscribe<grid_map_msgs::GridMap>(GRIDMAP_TOPIC, 1, &OccupancyGridConverter::gridMapCallback, this);
    shutdownSub = nh.subscribe("/mode3/shutdown_request", 1, &OccupancyGridConverter::shutdownCallback, this);
    occupancyGridPub = nh.advertise<nav_msgs::OccupancyGrid>(OCCUPANCYGRID_TOPIC, 1);
}

void OccupancyGridConverter::shutdownCallback(const std_msgs::Bool &message)
{
    if (message.data)
    {
        ROS_WARN("Shutting down the node");
        ros::shutdown();
    }
    else
    {
        ROS_WARN("Received false, not shutting down the node");
    }
}

// 回调函数
void OccupancyGridConverter::gridMapCallback(const grid_map_msgs::GridMap::ConstPtr &gridMapMsg)
{

    nav_msgs::OccupancyGrid occupancyGridMsg;
    // ROS_WARN("Convert grid map message to occupancy grid message");
    grid_map::GridMap gridMap;
    grid_map::GridMapRosConverter::fromMessage(*gridMapMsg, gridMap);
    grid_map::GridMapRosConverter::toOccupancyGrid(gridMap, transform_layer, 0, 1, occupancyGridMsg);
    occupancyGridMsg.header.frame_id = map_frame;

    // 若不需要二值化则直接发布
    if (!get_01_value)
    {
        for (int i = 0; i < occupancyGridMsg.data.size(); i++)
        {
            // 检查是否为无效值，直接跳过后续处理
            if (occupancyGridMsg.data[i] == -1)
            {
                continue;
            }

            // 其他有效值的处理
            occupancyGridMsg.data[i] = 100 - occupancyGridMsg.data[i];
            if (occupancyGridMsg.data[i] < 0)
            {
                occupancyGridMsg.data[i] = 0;
            }
            occupancyGridMsg.data[i] = int(occupancyGridMsg.data[i] / factor);
        }
    }

    // 若需要二值化则进行二值化处理
    else
    {
        for (int i = 0; i < occupancyGridMsg.data.size(); i++)
        {
            occupancyGridMsg.data[i] = 100 - occupancyGridMsg.data[i];
            if (occupancyGridMsg.data[i] == 101)
            {
                occupancyGridMsg.data[i] = -1;
            }
            else if (occupancyGridMsg.data[i] < threshold)
            {
                occupancyGridMsg.data[i] = 0;
            }
            else if (occupancyGridMsg.data[i] >= threshold)
            {
                occupancyGridMsg.data[i] = 100;
            }
        }
    }
    // occupancyGridMsg.info.origin.position.x = 0;
    // occupancyGridMsg.info.origin.position.y = 0;
    // occupancyGridMsg.data[0] = 100;
    // occupancyGridMsg.data[1] = 0;
    // occupancyGridMsg.data[2] = -1;
    // occupancyGridMsg.data[2] = 100;

    occupancyGridPub.publish(occupancyGridMsg);
}

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "trav2occ_global");
    ros::NodeHandle nh;

    OccupancyGridConverter converter(nh);

    // Spin ROS node
    ros::spin();

    return 0;
}