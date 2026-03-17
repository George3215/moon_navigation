#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_listener.h"
#include "nav_msgs/OccupancyGrid.h"
#include "pcl_ros/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include <algorithm>
#include <cmath>
#include <vector>

// 订阅和发布
ros::Subscriber ptcloud_sub;
ros::Publisher fused_map_pub;
// 参数定义
std::string input_topic, output_topic;
std::string output_map_frame_id, input_ptcloud_frame_id;
int fused_map_size;
double fused_map_resolution;
int fuse_map_number, fuse_map_threshold;
u_int8_t currentmap_number;
bool fuse_map_01value;
bool map_initialized = false;
bool first_initialize = true;
// 地图定义
nav_msgs::OccupancyGrid hazard_map; // 收到的占据栅格地图
nav_msgs::OccupancyGrid fused_map;  // 融合后的占据栅格地图
std::vector<bool> grid_visited;
std::vector<uint8_t> hit_count_map;
pcl::PointCloud<pcl::PointXYZ> ptcloud;
// 模式
// mode
std::string mode_topic_;
ros::Subscriber mode_subscriber_;
bool mode_activate_;
void ModeCallback(const std_msgs::String::ConstPtr &msg);
std::string mode_;
std::string set_mode_name_;

void ModeCallback(const std_msgs::String::ConstPtr &msg)
{
    mode_ = msg->data;
    // ROS_INFO("Mode: %s", mode_.c_str());
}

void Initialize_fused_map(const sensor_msgs::PointCloud2::ConstPtr &msg, tf::TransformListener &listener, tf::StampedTransform &transform)
{
    if (first_initialize)
    {
        input_ptcloud_frame_id = msg->header.frame_id;
        // 地图参数初始化
        fused_map.header.frame_id = output_map_frame_id;
        fused_map.info.resolution = fused_map_resolution;
        fused_map.info.width = int(fused_map_size / fused_map_resolution);
        fused_map.info.height = int(fused_map_size / fused_map_resolution);
        // 在地图坐标系初始化一张以车体为中心的占据栅格地图
        fused_map.data.resize(fused_map.info.width * fused_map.info.height, -1);
        // 初始化向量长度
        grid_visited.resize(fused_map.info.width * fused_map.info.height, false);
        hit_count_map.resize(fused_map.info.width * fused_map.info.height, 0);
        first_initialize = false;
        ROS_INFO("Fused map initialized");
    }
    else
    {
        fused_map.data.assign(fused_map.info.width * fused_map.info.height, -1);
        hit_count_map.assign(fused_map.info.width * fused_map.info.height, 0);
    }
    // 根据车体到地图的坐标变换，计算地图的原点
    fused_map.info.origin.position.x = transform.getOrigin().x() - fused_map_size / 2;
    fused_map.info.origin.position.y = transform.getOrigin().y() - fused_map_size / 2;
    fused_map.info.origin.position.z = 0;
    fused_map.info.origin.orientation.x = 0;
    fused_map.info.origin.orientation.y = 0;
    fused_map.info.origin.orientation.z = 0;
    fused_map.info.origin.orientation.w = 1;
    map_initialized = true;
}

void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg, tf::TransformListener &listener, tf::StampedTransform &transform)
{
    // ROS_INFO("Received pointcloud");
    // 模式判断：如果模式不正确了，将map_initialized置为false，并返回
    if (mode_activate_)
    {
        if (mode_ != set_mode_name_)
        {
            map_initialized = false;
            // ROS_WARN("Mode is not correct, map_initialized is set to false");
            ros::Duration(1.0).sleep();
            return;
        }
        // else
        // {
        //     ROS_INFO("Mode is correct");
        // }
    }

    // 如果地图未初始化，则初始化地图
    if (!map_initialized)
    {
        Initialize_fused_map(msg, listener, transform);
        currentmap_number = 0;
    }
    currentmap_number++;
    // 将向量全部赋值为false
    grid_visited.assign(fused_map.info.width * fused_map.info.height, false);
    // 将点云转化到地图坐标系
    sensor_msgs::PointCloud2 original_pointcloud, transformed_ptcloud;
    original_pointcloud = *msg;
    original_pointcloud.header.stamp = ros::Time(0);
    try
    {
        pcl_ros::transformPointCloud(output_map_frame_id, original_pointcloud, transformed_ptcloud, listener);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }
    // 将点云转化为占据栅格地图
    pcl::fromROSMsg(transformed_ptcloud, ptcloud);
    for (int i = 0; i < ptcloud.points.size(); i++)
    {
        int x = int((ptcloud.points[i].x - fused_map.info.origin.position.x) / fused_map_resolution);
        int y = int((ptcloud.points[i].y - fused_map.info.origin.position.y) / fused_map_resolution);
        // 如果点云在地图范围内且没有被赋值过，则将点云对应的栅格赋值
        if (x >= 0 && x < fused_map.info.width && y >= 0 && y < fused_map.info.height && !grid_visited[y * fused_map.info.width + x])
        {
            const int idx = y * fused_map.info.width + x;
            grid_visited[idx] = true;
            if (hit_count_map[idx] < static_cast<uint8_t>(fuse_map_number))
            {
                ++hit_count_map[idx];
            }
            fused_map.data[idx] = static_cast<int8_t>(std::round(100.0 * hit_count_map[idx] / std::max(1, fuse_map_number)));
        }
    }
    // 地图二值化策略-遍历地图的每个格子
    if (currentmap_number >= fuse_map_number && fuse_map_01value)
    {
        for (int i = 0; i < fused_map.data.size(); i++)
        {
            if (hit_count_map[i] >= static_cast<uint8_t>(fuse_map_threshold))
            {
                fused_map.data[i] = 100;
            }
            else
            {
                fused_map.data[i] = -1;
            }
        }
    }

    // 如果收到了足够多的地图，则发布融合后的地图
    if (currentmap_number >= fuse_map_number)
    {
        fused_map.header.stamp = ros::Time::now();
        fused_map_pub.publish(fused_map);
        map_initialized = false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fuse_hazardmap");
    ros::NodeHandle nh;
    // 参数读取
    nh.param<std::string>("fuse_hazardmap/ptcloud_input_topic", input_topic, "/hazard_detection/pointcloud");
    nh.param<std::string>("fuse_hazardmap/map_output_topic", output_topic, "/hazard_detection/fused_map");
    nh.param<std::string>("fuse_hazardmap/output_map_frame_id", output_map_frame_id, "map");
    nh.param<std::string>("fuse_hazardmap/input_ptcloud_frame_id", input_ptcloud_frame_id, "base_link");
    nh.param<int>("fuse_hazardmap/fused_map_size", fused_map_size, 10.0);
    nh.param<double>("fuse_hazardmap/fused_map_resolution", fused_map_resolution, 0.1);
    nh.param<int>("fuse_hazardmap/fuse_map_number", fuse_map_number, 5);
    nh.param<bool>("fuse_hazardmap/fuse_map_01value", fuse_map_01value, false);
    nh.param<int>("fuse_hazardmap/fuse_map_threshold", fuse_map_threshold, 5);
    ROS_DEBUG("fuse_hazardmap/ptcloud_input_topic: %s", input_topic.c_str());
    ROS_DEBUG("fuse_hazardmap/map_output_topic: %s", output_topic.c_str());
    ROS_DEBUG("fuse_hazardmap/output_map_frame_id: %s", output_map_frame_id.c_str());
    ROS_DEBUG("fuse_hazardmap/input_ptcloud_frame_id: %s", input_ptcloud_frame_id.c_str());
    ROS_DEBUG("fuse_hazardmap/fused_map_size: %d", fused_map_size);
    ROS_DEBUG("fuse_hazardmap/fused_map_resolution: %f", fused_map_resolution);
    ROS_DEBUG("fuse_hazardmap/fuse_map_number: %d", fuse_map_number);
    ROS_DEBUG("fuse_hazardmap/fuse_map_01value: %d", fuse_map_01value);
    ROS_DEBUG("fuse_hazardmap/fuse_map_threshold: %d", fuse_map_threshold);
    // 模式参数
    nh.param<bool>("/fuse_hazardmap/mode_activate", mode_activate_, false);
    nh.param<std::string>("/fuse_hazardmap/mode_topic", mode_topic_, "/mode");
    nh.param<std::string>("/fuse_hazardmap/set_mode_name", set_mode_name_, "mode_name");
    ROS_DEBUG("---------------[ mode ]---------------");
    ROS_DEBUG("mode_activate: %d", mode_activate_);
    ROS_DEBUG("mode_topic: %s", mode_topic_.c_str());
    ROS_DEBUG("set_mode_name: %s", set_mode_name_.c_str());
    // 模式订阅
    if (mode_activate_)
    {
        mode_subscriber_ = nh.subscribe(mode_topic_, 1, &ModeCallback);
    }

    // 坐标变换
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try
    {
        listener.waitForTransform(output_map_frame_id, input_ptcloud_frame_id, ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform(output_map_frame_id, input_ptcloud_frame_id, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    // 订阅和发布
    ptcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(input_topic, 1, boost::bind(PointCloudCallback, _1, boost::ref(listener), boost::ref(transform)));
    fused_map_pub = nh.advertise<nav_msgs::OccupancyGrid>(output_topic, 1);
    while (ros::ok())
    {
        try
        {
            listener.waitForTransform(output_map_frame_id, input_ptcloud_frame_id, ros::Time(0), ros::Duration(1.0));
            listener.lookupTransform(output_map_frame_id, input_ptcloud_frame_id, ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
        }
        ros::spinOnce();
    }
}