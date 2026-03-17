#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <cmath>

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
    double side_invalid_border, back_invalid_border, front_all_valid_border;
    double car_width, car_length;
    double car_radius;


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
    nh.param<std::string>("trav2occ_slope/grid_map_topic", GRIDMAP_TOPIC, "/grid_map_topic");
    nh.param<std::string>("trav2occ_slope/occupancy_grid_topic", OCCUPANCYGRID_TOPIC, "/occupancygrid");
    nh.param<std::string>("trav2occ_slope/transform_layer", transform_layer, "layer");
    nh.param<bool>("trav2occ_slope/get_01_value", get_01_value, false);
    nh.param<int>("trav2occ_slope/threshold", threshold, 50);
    nh.param<double>("trav2occ_slope/downscale_cost_factor", factor, 1.0);
    nh.param<std::string>("trav2occ_slope/map_frame", map_frame, "map");
    // nh.param<double>("trav2occ_slope/side_invalid_border", side_invalid_border, 1.0);
    // nh.param<double>("trav2occ_slope/back_invalid_border", back_invalid_border, 1.0);
    // nh.param<double>("trav2occ_slope/front_all_valid_border", front_all_valid_border, 1.0);
    // nh.param<double>("trav2occ_slope/car_width", car_width, 1.0);
    // nh.param<double>("trav2occ_slope/car_length", car_length, 1.0);
    nh.param<double>("trav2occ_slope/car_radius", car_radius, 1.0);
    ROS_WARN("grid_map_topic: %s", GRIDMAP_TOPIC.c_str());
    ROS_WARN("occupancy_grid_topic: %s", OCCUPANCYGRID_TOPIC.c_str());
    ROS_WARN("transform_layer: %s", transform_layer.c_str());
    ROS_WARN("get_01_value: %d", get_01_value);
    ROS_WARN("threshold: %d", threshold);
    ROS_WARN("downscale_cost_factor: %f", factor);
    // ROS_WARN("side_invalid_border: %f", side_invalid_border);
    // ROS_WARN("back_invalid_border: %f", back_invalid_border);
    // ROS_WARN("front_all_valid_border: %f", front_all_valid_border);
    // ROS_WARN("car_width: %f", car_width);
    // ROS_WARN("car_length: %f", car_length);
    ROS_WARN("car_radius: %f", car_radius);

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
    grid_map::GridMapRosConverter::toOccupancyGrid(gridMap, transform_layer, 0, 100, occupancyGridMsg);
    occupancyGridMsg.header.frame_id = map_frame;

    // Apply thresholding and scaling to suppress low-confidence noise.
    for (int i = 0; i < static_cast<int>(occupancyGridMsg.data.size()); ++i)
    {
        int v = occupancyGridMsg.data[i];
        if (v < 0)
        {
            continue;
        }

        if (get_01_value)
        {
            occupancyGridMsg.data[i] = (v >= threshold) ? 100 : 0;
            continue;
        }

        if (v < threshold)
        {
            occupancyGridMsg.data[i] = 0;
        }
        else
        {
            int scaled = static_cast<int>((v - threshold) * factor);
            if (scaled > 100)
                scaled = 100;
            if (scaled < 0)
                scaled = 0;
            occupancyGridMsg.data[i] = scaled;
        }
    }

    // Mask rover body footprint as unknown around map center.
    const int width = static_cast<int>(occupancyGridMsg.info.width);
    const int height = static_cast<int>(occupancyGridMsg.info.height);
    const double resolution = occupancyGridMsg.info.resolution;
    const int cx = width / 2;
    const int cy = height / 2;
    const int r_cells = std::max(1, static_cast<int>(std::round(car_radius / std::max(1e-6, resolution))));

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            const int dx = x - cx;
            const int dy = y - cy;
            if (dx * dx + dy * dy <= r_cells * r_cells)
            {
                occupancyGridMsg.data[y * width + x] = -1;
            }
        }
    }

    // for (int i = 0; i < occupancyGridMsg.data.size(); i++)
    // {
    //     if (occupancyGridMsg.data[i] == 0)
    //     {
    //         occupancyGridMsg.data[i] = -1;
    //     }
    // }

    // // 去掉无效区域
    // double resolution = occupancyGridMsg.info.resolution; // 每格的分辨率
    // double width = occupancyGridMsg.info.width * resolution;
    // double height = occupancyGridMsg.info.height * resolution;

    // // 地图中心（车辆所在位置）
    // double origin_x = occupancyGridMsg.info.origin.position.x;
    // double origin_y = occupancyGridMsg.info.origin.position.y;

    // // 车辆位置对应的索引
    // int vehicle_x_idx = width / 2 / resolution;
    // int vehicle_y_idx = height / 2 / resolution;

    // // 根据车辆的宽度和长度计算无效区域
    // int car_half_width_cells = car_width / resolution /2;
    // int car_half_length_cells = car_length / resolution /2;

    // // 计算边界的格子数
    // int side_invalid_border_cells = side_invalid_border / resolution;
    // int back_invalid_border_cells = back_invalid_border / resolution;
    // int front_all_valid_border_cells = front_all_valid_border / resolution;

    // // 遍历地图的每个格子
    // for (int y = 0; y < occupancyGridMsg.info.height; ++y)
    // {
    //     for (int x = 0; x < occupancyGridMsg.info.width; ++x)
    //     {
    //         int index = y * occupancyGridMsg.info.width + x;

    //         // 计算当前格子相对车辆的位置
    //         int dx = x - vehicle_x_idx;
    //         int dy = y - vehicle_y_idx;

    //         // // 判断前方全有效区域
    //         // if (dx >= front_all_valid_border_cells)
    //         // {
    //         //     continue; // 前方区域不做修改
    //         // }

    //         // // 判断左右和后方无效区域
    //         // if (abs(dy) > side_invalid_border_cells || dx < -back_invalid_border_cells)
    //         // {
    //         //     occupancyGridMsg.data[index] = -1; // 无效
    //         // }

    //         // 判断车辆位置周围无效区域
    //         if (abs(dx) <= car_half_length_cells && abs(dy) <= car_half_width_cells)
    //         {
    //             occupancyGridMsg.data[index] = -1; // 无效
    //         }
            

    //     }
    // }




    occupancyGridPub.publish(occupancyGridMsg);
}

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "trav2occ_slope");
    ros::NodeHandle nh;

    OccupancyGridConverter converter(nh);

    // Spin ROS node
    ros::spin();

    return 0;
}