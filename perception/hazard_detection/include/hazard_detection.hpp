#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Header.h"
#include "std_msgs/Time.h"
#include "std_msgs/String.h"

#include "tf/transform_listener.h"
#include "tf_conversions/tf_eigen.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"

#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "Eigen/Geometry"
#include "nav_msgs/OccupancyGrid.h"

class Hazard_Detection
{
public:
    Hazard_Detection(ros::NodeHandle nh);

private:
    void ReadParam();
    void InitializeLocalMap();
    void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void PoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
    void GetTransformListener(tf::TransformListener &tf_listener, std::string current_frame_id, std::string target_frame_id);
    void PtcloudTransform(tf::TransformListener &tf_listener, pcl::PointCloud<pcl::PointXYZ> &cloud_orig, pcl::PointCloud<pcl::PointXYZ> &cloud_trans, std::string current_frame_id, std::string target_frame_id);
    void GetTransformXYZ(tf::TransformListener &tf_listener, std::string current_frame_id, std::string target_frame_id, double &x, double &y, double &z);
    void PCL_RANSAC(pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out);
    void PCL_DownSample(pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out);
    void PCL_PassThrough_1(pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out);
    void PCL_PassThrough_2(pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out);
    void PCLOutlierRemoval(pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out);
    void GetHazardPosition(pcl::PointCloud<pcl::PointXYZ> &ptcloud_3d, pcl::PointCloud<pcl::PointXYZ> &ptcloud2d);
    void GenerateLocalMap(pcl::PointCloud<pcl::PointXYZ> ptcloud_raw, pcl::PointCloud<pcl::PointXYZ> ptcloud_2d);

    //  ros相关
    ros::NodeHandle nodehandle_;
    ros::Subscriber image_sub, pointcloud_sub, pose_sub;
    ros::Publisher ptcloud_raw_pub, ptcloud_converted_pub, ptdebug_pub, ptcloud2d_pub;
    ros::Publisher localmap_pub;
    //  传入参数
    std::string image_topic_name, pointcloud_topic_name, pose_topic_name;
    std::string camera_frame_id, vehicle_frame_id, map_frame_id;
    // 采样滤波器参数
    double downsample_resolution;
    double ransac_threshold;
    double passthrough_filter_xmin, passthrough_filter_xmax;
    double passthrough_filter2_xmin, passthrough_filter2_xmax;
    double passthrough_filter_ymin, passthrough_filter_ymax;
    double passthrough_filter2_ymin, passthrough_filter2_ymax;
    double passthrough_filter_zmin, passthrough_filter_zmax;
    double outlier_k, outliner_threshold;
    // 障碍物参数
    double obstacle_resolution, obstacle_front_threshold, obstacle_back_threshold;
    int safety_dis;
    // 地图参数
    double map_resolution, map_front_x, map_back_x, map_y;
    bool visual_local_map;

    // 收发消息内容

    pcl::PointCloud<pcl::PointXYZ> ptcloud_sensor1;

    pcl::PointCloud<pcl::PointXYZ> ptcloud_temp1;
    pcl::PointCloud<pcl::PointXYZ> ptcloud_temp2;

    pcl::PointCloud<pcl::PointXYZ> ptcloud_raw;       // 原始pcl三维点云，每个点包含xyz三个坐标
    pcl::PointCloud<pcl::PointXYZ> ptcloud_converted; // 转换后的pcl三维点云，每个点包含xyz三个坐标
    pcl::PointCloud<pcl::PointXYZ> ptcloud_2d;        // 转换后的pcl三维点云投影变换到2d平面
    geometry_msgs::PoseWithCovarianceStamped pose;    // 位姿
    tf::TransformListener car2cam_listener, cam2car_listener;
    tf::TransformListener car2map_listener, map2car_listener; // tf监听器
    double car2cam_tf_x, car2cam_tf_y, car2cam_tf_z;          // 车体坐标系到相机坐标系的变换
    nav_msgs::OccupancyGrid local_map;                        // 局部地图

    // mode
    std::string mode_topic_;
    ros::Subscriber mode_subscriber_;
    bool mode_activate_;
    void ModeCallback(const std_msgs::String::ConstPtr &msg);
    std::string mode_;
    std::string set_mode_name_;


    ros::Publisher debug_sensor2_pub;
};