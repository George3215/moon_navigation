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
    nh.param<std::string>("trav2occ_roughness/grid_map_topic", GRIDMAP_TOPIC, "/grid_map_topic");
    nh.param<std::string>("trav2occ_roughness/occupancy_grid_topic", OCCUPANCYGRID_TOPIC, "/occupancygrid");
    nh.param<std::string>("trav2occ_roughness/transform_layer", transform_layer, "layer");
    nh.param<bool>("trav2occ_roughness/get_01_value", get_01_value, false);
    nh.param<int>("trav2occ_roughness/threshold", threshold, 50);
    nh.param<double>("trav2occ_roughness/downscale_cost_factor", factor, 1.0);
    nh.param<std::string>("trav2occ_roughness/map_frame", map_frame, "map");
    ROS_WARN("grid_map_topic: %s", GRIDMAP_TOPIC.c_str());
    ROS_WARN("occupancy_grid_topic: %s", OCCUPANCYGRID_TOPIC.c_str());
    ROS_WARN("transform_layer: %s", transform_layer.c_str());
    ROS_WARN("get_01_value: %d", get_01_value);
    ROS_WARN("threshold: %d", threshold);
    ROS_WARN("downscale_cost_factor: %f", factor);

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
    grid_map::GridMapRosConverter::toOccupancyGrid(gridMap, transform_layer, 0, threshold, occupancyGridMsg);
    occupancyGridMsg.header.frame_id = map_frame;

    for (int i = 0; i < occupancyGridMsg.data.size(); i++)
    {
        if (occupancyGridMsg.data[i] == 0)
        {
            occupancyGridMsg.data[i] = -1;
        }
        // else if(oc)
        // {
        //     occupancyGridMsg.data[i] = 0;
        // }
    }
    // }
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
    ros::init(argc, argv, "trav2occ_roughness");
    ros::NodeHandle nh;

    OccupancyGridConverter converter(nh);

    // Spin ROS node
    ros::spin();

    return 0;
}