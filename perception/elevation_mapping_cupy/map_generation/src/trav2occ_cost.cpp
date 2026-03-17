#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <cmath>

// 功能：将 custom_traversability_cost 图层 [0,1] 转换为 OccupancyGrid [0,100]
// 配套 custom_traversability_cost.py 使用

class TraversabilityCostConverter
{
public:
    TraversabilityCostConverter(ros::NodeHandle &nh);

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber grid_map_sub_;
    ros::Subscriber shutdown_sub_;

    std::string grid_map_topic_, occupancy_topic_, layer_name_, map_frame_;
    double car_radius_;
    int min_cost_threshold_;

    void gridMapCallback(const grid_map_msgs::GridMap::ConstPtr &msg);
    void shutdownCallback(const std_msgs::Bool &msg);
};

TraversabilityCostConverter::TraversabilityCostConverter(ros::NodeHandle &nh) : nh_(nh)
{
    nh_.param<std::string>("trav2occ_cost/grid_map_topic", grid_map_topic_, "/elevation_mapping/traversability");
    nh_.param<std::string>("trav2occ_cost/occupancy_grid_topic", occupancy_topic_, "/mode3/occupancy_grid");
    nh_.param<std::string>("trav2occ_cost/transform_layer", layer_name_, "custom_traversability_cost");
    nh_.param<std::string>("trav2occ_cost/map_frame", map_frame_, "odom");
    nh_.param<double>("trav2occ_cost/car_radius", car_radius_, 0.55);
    nh_.param<int>("trav2occ_cost/min_cost_threshold", min_cost_threshold_, 5);

    ROS_WARN("[trav2occ_cost] grid_map_topic: %s", grid_map_topic_.c_str());
    ROS_WARN("[trav2occ_cost] occupancy_topic: %s", occupancy_topic_.c_str());
    ROS_WARN("[trav2occ_cost] layer: %s", layer_name_.c_str());
    ROS_WARN("[trav2occ_cost] car_radius: %.2f", car_radius_);

    grid_map_sub_ = nh_.subscribe<grid_map_msgs::GridMap>(
        grid_map_topic_, 1, &TraversabilityCostConverter::gridMapCallback, this);
    shutdown_sub_ = nh_.subscribe(
        "/mode3/shutdown_request", 1, &TraversabilityCostConverter::shutdownCallback, this);
    pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(occupancy_topic_, 1);
}

void TraversabilityCostConverter::shutdownCallback(const std_msgs::Bool &msg)
{
    if (msg.data)
        ros::shutdown();
}

void TraversabilityCostConverter::gridMapCallback(const grid_map_msgs::GridMap::ConstPtr &msg)
{
    grid_map::GridMap gridMap;
    grid_map::GridMapRosConverter::fromMessage(*msg, gridMap);

    nav_msgs::OccupancyGrid occ;
    // 插件输出 [0, 1]，直接映射到占据栅格 [0, 100]
    grid_map::GridMapRosConverter::toOccupancyGrid(gridMap, layer_name_, 0.0, 1.0, occ);
    occ.header.frame_id = map_frame_;

    const int width = static_cast<int>(occ.info.width);
    const int height = static_cast<int>(occ.info.height);
    const double resolution = occ.info.resolution;

    // 1. 噪声抑制：低于阈值的代价归零
    for (auto &v : occ.data)
    {
        if (v >= 0 && v < min_cost_threshold_)
            v = 0;
    }

    // 2. 车体足迹区域标记为 unknown
    const int cx = width / 2;
    const int cy = height / 2;
    const int r_cells = std::max(1, static_cast<int>(std::round(car_radius_ / std::max(1e-6, resolution))));

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            const int dx = x - cx;
            const int dy = y - cy;
            if (dx * dx + dy * dy <= r_cells * r_cells)
                occ.data[y * width + x] = -1;
        }
    }

    pub_.publish(occ);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trav2occ_cost");
    ros::NodeHandle nh;
    TraversabilityCostConverter converter(nh);
    ros::spin();
    return 0;
}
