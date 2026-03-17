#include "hazard_detection.hpp"

//  @@@
//  函数：构造函数
//  功能：读取参数并初始化话题订阅发布
//  参数：nodehandle
Hazard_Detection::Hazard_Detection(ros::NodeHandle nh)
{
    nodehandle_ = nh;
    ReadParam();
    if (visual_local_map)
    {
        InitializeLocalMap();
    }
    // 话题订阅
    pointcloud_sub = nodehandle_.subscribe(pointcloud_topic_name, 1, &Hazard_Detection::PointCloudCallback, this);
    pose_sub = nodehandle_.subscribe(pose_topic_name, 1, &Hazard_Detection::PoseCallback, this);
    //  话题发布
    ptcloud_raw_pub = nodehandle_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/hazard_detection/ptcloud_filtered", 1);
    ptcloud_converted_pub = nodehandle_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/hazard_detection/ptcloud_converted", 1);
    ptdebug_pub = nodehandle_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/hazard_detection/ptdebug", 1);
    ptcloud2d_pub = nodehandle_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/hazard_detection/ptcloud2d", 1);
    if (visual_local_map)
    {
        localmap_pub = nodehandle_.advertise<nav_msgs::OccupancyGrid>("/hazard_detection/local_map", 1);
    }
    // 模式订阅
    if (mode_activate_)
    {
        mode_subscriber_ = nodehandle_.subscribe(mode_topic_, 1, &Hazard_Detection::ModeCallback, this);
    }

}

//  @@@
//  函数：读取参数
//  功能：从ros参数服务器中读取参数
//  参数：
void Hazard_Detection::ReadParam()
{
    ROS_WARN("Reading Parameters From ROS Param Server...");
    ROS_DEBUG("-----------------------------------------");

    // 话题名称
    nodehandle_.param<std::string>("/hazard_detection/pointcloud_topic_name", pointcloud_topic_name, "/pointcloud_topic");
    nodehandle_.param<std::string>("/hazard_detection/pose_topic_name", pose_topic_name, "/pose_topic");
    ROS_DEBUG("pointcloud_topic_name: %s", pointcloud_topic_name.c_str());
    ROS_DEBUG("pose_topic_name: %s", pose_topic_name.c_str());

    //  坐标系名称
    ROS_DEBUG("---------------[ frames ]---------------");
    nodehandle_.param<std::string>("/hazard_detection/camera_frame_id", camera_frame_id, "camera_frame");
    nodehandle_.param<std::string>("/hazard_detection/vehicle_frame_id", vehicle_frame_id, "vehicle_frame");
    nodehandle_.param<std::string>("/hazard_detection/map_frame_id", map_frame_id, "map_frame");
    ROS_DEBUG("camera_frame_id: %s", camera_frame_id.c_str());
    ROS_DEBUG("vehicle_frame_id: %s", vehicle_frame_id.c_str());
    ROS_DEBUG("map_frame_id: %s", map_frame_id.c_str());

    // 滤波器参数
    nodehandle_.param<double>("/hazard_detection/downsample_filter/downsample_resolution", downsample_resolution, 0.1);
    nodehandle_.param<double>("/hazard_detection/ransac_filter/dis_threshold", ransac_threshold, 0.1);
    nodehandle_.param<double>("/hazard_detection/passthrough_filter/x_axis/x_min", passthrough_filter_xmin, 0.0);
    nodehandle_.param<double>("/hazard_detection/passthrough_filter/x_axis/x_max", passthrough_filter_xmax, 10.0);
    nodehandle_.param<double>("/hazard_detection/passthrough_filter/x_axis_2/x_min", passthrough_filter2_xmin, 1.0);
    nodehandle_.param<double>("/hazard_detection/passthrough_filter/x_axis_2/x_max", passthrough_filter2_xmax, 9.0);
    nodehandle_.param<double>("/hazard_detection/passthrough_filter/y_axis/y_min", passthrough_filter_ymin, -10.0);
    nodehandle_.param<double>("/hazard_detection/passthrough_filter/y_axis/y_max", passthrough_filter_ymax, 10.0);
    nodehandle_.param<double>("/hazard_detection/passthrough_filter/y_axis_2/y_min", passthrough_filter2_ymin, -8.0);
    nodehandle_.param<double>("/hazard_detection/passthrough_filter/y_axis_2/y_max", passthrough_filter2_ymax, 8.0);
    nodehandle_.param<double>("/hazard_detection/passthrough_filter/z_axis/height_min", passthrough_filter_zmin, -2.0);
    nodehandle_.param<double>("/hazard_detection/passthrough_filter/z_axis/height_max", passthrough_filter_zmax, 0.0);
    nodehandle_.param<double>("/hazard_detection/outlier_filter/outlier_k", outlier_k, 50);
    nodehandle_.param<double>("/hazard_detection/outlier_filter/outliehreshold", outliner_threshold, 1.0);
    ROS_DEBUG("---------------[ filters ]---------------");
    ROS_DEBUG("downsample_resolution: %f", downsample_resolution);
    ROS_DEBUG("ransac_threshold: %f", ransac_threshold);
    ROS_DEBUG("passthrough_filter_xmin: %f", passthrough_filter_xmin);
    ROS_DEBUG("passthrough_filter_xmax: %f", passthrough_filter_xmax);
    ROS_DEBUG("passthrough_filter2_xmin: %f", passthrough_filter2_xmin);
    ROS_DEBUG("passthrough_filter2_xmax: %f", passthrough_filter2_xmax);
    ROS_DEBUG("passthrough_filter_ymin: %f", passthrough_filter_ymin);
    ROS_DEBUG("passthrough_filter_ymax: %f", passthrough_filter_ymax);
    ROS_DEBUG("passthrough_filter2_ymin: %f", passthrough_filter2_ymin);
    ROS_DEBUG("passthrough_filter2_ymax: %f", passthrough_filter2_ymax);
    ROS_DEBUG("passthrough_filter_zmin: %f", passthrough_filter_zmin);
    ROS_DEBUG("passthrough_filter_zmax: %f", passthrough_filter_zmax);
    ROS_DEBUG("outlier_k: %f", outlier_k);
    ROS_DEBUG("outliner_threshold: %f", outliner_threshold);

    // 障碍物参数
    nodehandle_.param<double>("/hazard_detection/obstacle_threshold/obstacle_resolution", obstacle_resolution, 0.2);
    nodehandle_.param<double>("/hazard_detection/obstacle_threshold/obstacle_front_threshold", obstacle_front_threshold, 10);
    nodehandle_.param<double>("/hazard_detection/obstacle_threshold/obstacle_back_threshold", obstacle_back_threshold, 10);
    ROS_DEBUG("---------------[ obstacles ]---------------");
    ROS_DEBUG("obstacle_resolution: %f", obstacle_resolution);
    ROS_DEBUG("obstacle_front_threshold: %f", obstacle_front_threshold);
    ROS_DEBUG("obstacle_back_threshold: %f", obstacle_back_threshold);
    nodehandle_.param<int>("/hazard_detection/grid_map/safety_dis", safety_dis, 5);
    ROS_DEBUG("safety_dis: %d", safety_dis);

    // 是否可视化地图
    nodehandle_.param<bool>("/hazard_detection/visual_local_map", visual_local_map, true);
    nodehandle_.param<double>("/hazard_detection/grid_map/resolution", map_resolution, 0.1);
    ROS_DEBUG("---------------[ map ]---------------");
    ROS_DEBUG("map_resolution: %f", map_resolution);
    ROS_DEBUG("visual_local_map: %d", visual_local_map);

    // 模式参数
    nodehandle_.param<bool>("/hazard_detection/mode_activate", mode_activate_, false);
    nodehandle_.param<std::string>("/hazard_detection/mode_topic", mode_topic_, "/mode");
    nodehandle_.param<std::string>("/hazard_detection/set_mode_name", set_mode_name_, "mode_name");
    ROS_DEBUG("---------------[ mode ]---------------");
    ROS_DEBUG("mode_activate: %d", mode_activate_);
    ROS_DEBUG("mode_topic: %s", mode_topic_.c_str());
    ROS_DEBUG("set_mode_name: %s", set_mode_name_.c_str());
    ROS_DEBUG("-----------------------------------------");
    ROS_WARN("Read Parameters From ROS Param Server Finished!");
}

void Hazard_Detection::ModeCallback(const std_msgs::String::ConstPtr &msg)
{
    mode_ = msg->data;
}


//  @@@
//  函数：点云回调函数
//  功能：接收点云话题
//  参数：点云话题指针
void Hazard_Detection::PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    // ROS_WARN("CALLBACK START.:{%f}", ros::Time::now().toSec());
    // 模式检测：如果现在处于模式触发，则检测对应模式是否正确，不正确就不处理
    if (mode_activate_)
    {
        if (mode_ != set_mode_name_)
        {
            // ROS_WARN("Mode is not correct, do not process!");
            // ROS_WARN("Waiting for Correct Mode... Current Mode: %s", mode_.c_str());
            ros::Duration(1.0).sleep();
            return;
        }
        // else
        // {
        //     // ROS_INFO("Mode is correct, start process!");
        // }
    }

    // 转换为pcl格式
    ptcloud_temp1.clear();
    ptcloud_temp2.clear();
    pcl::fromROSMsg(*msg, ptcloud_sensor1);

    
    // ###################前处理#######################
    // PCL_DownSample(ptcloud_sensor1, ptcloud_temp2);
    
    // // debug
    ptcloud_temp1 = ptcloud_sensor1;
    ptcloud_temp1.header = ptcloud_temp2.header;
    ptcloud_temp1.header.frame_id = ptcloud_sensor1.header.frame_id;

    // 转换到车体坐标系
    if (ptcloud_temp1.header.frame_id != vehicle_frame_id)
    {
        // ROS_WARN("Transforming Point Cloud to Vehicle Frame...");
        
        PtcloudTransform(cam2car_listener, ptcloud_temp1, ptcloud_raw, ptcloud_temp1.header.frame_id, vehicle_frame_id);
    }
    else
    {
        ROS_WARN("Point Cloud is Already in Vehicle Frame!");
        ptcloud_raw = ptcloud_temp1;
    }

    ptcloud_temp1.clear();
    ptcloud_temp2.clear();

    // xy直通滤波
    PCL_PassThrough_1(ptcloud_raw, ptcloud_temp1);
    ptcloud_raw_pub.publish(ptcloud_temp1);

    // PCL_RANSAC
    PCL_RANSAC(ptcloud_temp1, ptcloud_temp2);
    ptdebug_pub.publish(ptcloud_temp2);
    ptcloud_temp1.clear();

    // 二次直通滤波
    PCL_PassThrough_2(ptcloud_temp2, ptcloud_temp1);
    ptcloud_temp2.clear();

    // 离群点滤波
    try
    {
        PCLOutlierRemoval(ptcloud_temp1, ptcloud_temp2);
    }
    catch (const std::exception &e)
    {

        std::cerr << e.what() << '\n';
        ptcloud_temp2 = ptcloud_temp1;
    }
    
    ptcloud_converted_pub.publish(ptcloud_temp2);
    ptcloud_temp1.clear();

    //  点云投影到2d
    GetHazardPosition(ptcloud_temp2, ptcloud_temp1);
    ptcloud_temp2.clear();

    //  再做一次下采样
    PCL_DownSample(ptcloud_temp1, ptcloud_2d);

    //  消息发布
    ptcloud2d_pub.publish(ptcloud_2d);
    // ##############地图生成####################
    if (visual_local_map)
    {
        // 生成局部地图
        GenerateLocalMap(ptcloud_raw, ptcloud_2d);
    }
}

//  @@@
//  函数：位姿回调函数
//  功能：接收位姿话题
//  参数：位姿话题指针
void Hazard_Detection::PoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{

    // ROS_INFO("Pose Received!");
    // pose = *msg;
    //  获取坐标变换
    GetTransformListener(cam2car_listener, camera_frame_id, vehicle_frame_id);
    GetTransformListener(car2cam_listener, vehicle_frame_id, camera_frame_id);
    GetTransformListener(car2map_listener, vehicle_frame_id, map_frame_id);
    GetTransformListener(map2car_listener, map_frame_id, vehicle_frame_id);
    // if (multi_sensor)
    // {
    //     GetTransformListener(sensor2_to_car_listener, sensor2_frame_id, vehicle_frame_id);
    // }
    GetTransformXYZ(car2cam_listener, vehicle_frame_id, camera_frame_id, car2cam_tf_x, car2cam_tf_y, car2cam_tf_z);
}

//  @@@
//  函数：监听tf变换
//  功能：得到tf变换
//  参数：tf listener
void Hazard_Detection::GetTransformListener(tf::TransformListener &tf_listener, std::string current_frame_id, std::string target_frame_id)
{

    tf_listener.waitForTransform(target_frame_id, current_frame_id, ros::Time(0), ros::Duration(1.0));
}


//  @@@
//  函数：点云tf变换
//  功能：将点云变换到坐标系
//  参数：转换前后点云
void Hazard_Detection::PtcloudTransform(tf::TransformListener &tf_listener, pcl::PointCloud<pcl::PointXYZ> &cloud_orig, pcl::PointCloud<pcl::PointXYZ> &cloud_trans, std::string current_frame_id, std::string target_frame_id)
{
    // 遇到Lookup would require extrapolation into the past，一般是时间戳的问题
    // 由于处理需要时间，所以时间会不同步，赋值时不要把header直接赋值，而是赋值header.frameid
    // 如果还不行，可以清空header，然后赋值header.frameid
    // tf::TransformListener listener;
    // listener.waitForTransform(target_frame_id, current_frame_id, ros::Time(0), ros::Duration(1));
    // tf转换
    try
    {
        pcl_ros::transformPointCloud(target_frame_id, cloud_orig, cloud_trans, tf_listener);
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("Transform Point Cloud Failed!");
        ROS_ERROR("%s", ex.what());
    }
}

//  @@@
//  函数：tf变换坐标
//  功能：得到当前坐标系到目标坐标系的坐标变换
//  参数：坐标系名称
void Hazard_Detection::GetTransformXYZ(tf::TransformListener &tf_listener, std::string current_frame_id, std::string target_frame_id, double &x, double &y, double &z)
{

    //  tf转换
    try
    {
        tf::StampedTransform transform;
        tf_listener.lookupTransform(target_frame_id, current_frame_id, ros::Time(0), transform);
        x = transform.getOrigin().x();
        y = transform.getOrigin().y();
        z = transform.getOrigin().z();
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("Get Transform XYZ Failed!");
        ROS_ERROR("%s", ex.what());
    }

}

//  @@@
//  函数：PCL-RANSAC
//  功能：去掉地面点云
//  参数：输入输出点云
void Hazard_Detection::PCL_RANSAC(pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out)
{
    // ROS_INFO("PCL RANSAC!");
    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud = cloud_in.makeShared();
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
    pcl::PointCloud<pcl::PointXYZ> ground_cloud;
    pcl::PointCloud<pcl::PointXYZ> obj_cloud;

    // 创建分割时所需要的模型系数对象，coefficients及存储内点的点索引集合对象inliers
    // 创建分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // 可选择配置，设置模型系数需要优化
    seg.setOptimizeCoefficients(true);
    // 必要的配置，设置分割的模型类型，所用的随机参数估计方法，距离阀值，输入点云
    seg.setModelType(pcl::SACMODEL_PLANE); // 设置模型类型
    //                SACMODEL_PLANE, 三维平面
    //                SACMODEL_LINE,    三维直线
    //                SACMODEL_CIRCLE2D, 二维圆
    //                SACMODEL_CIRCLE3D,  三维圆
    //                SACMODEL_SPHERE,      球
    //                SACMODEL_CYLINDER,    柱
    //                SACMODEL_CONE,        锥
    //                SACMODEL_TORUS,       环面
    //                SACMODEL_PARALLEL_LINE,   平行线
    //                SACMODEL_PERPENDICULAR_PLANE, 垂直平面
    //                SACMODEL_PARALLEL_LINES,  平行线
    //                SACMODEL_NORMAL_PLANE,    法向平面
    //                SACMODEL_NORMAL_SPHERE,   法向球
    //                SACMODEL_REGISTRATION,
    //                SACMODEL_REGISTRATION_2D,
    //                SACMODEL_PARALLEL_PLANE,  平行平面
    //                SACMODEL_NORMAL_PARALLEL_PLANE,   法向平行平面
    //                SACMODEL_STICK
    seg.setMethodType(pcl::SAC_PROSAC); // 设置随机采样一致性方法类型
    // you can modify the parameter below
    seg.setMaxIterations(10000);                // 设置最大迭代次数
    seg.setDistanceThreshold(ransac_threshold); // 设定距离阀值，距离阀值决定了点被认为是局内点是必须满足的条件
    seg.setInputCloud(cloud);
    // 引发分割实现，存储分割结果到点几何inliers及存储平面模型的系数coefficients
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
        std::cout << "error! Could not found any inliers!" << std::endl;
    }
    try
    {
        // 从点云中抽取分割的处在平面上的点集
        pcl::ExtractIndices<pcl::PointXYZ> extractor; // 点提取对象
        extractor.setInputCloud(cloud);
        extractor.setIndices(inliers);
        // true表示的是输入点集以外的点
        extractor.setNegative(true);
        extractor.filter(obj_cloud);
        // // 从点云中抽取分割的处在平面上的点集
        // pcl::ExtractIndices<pcl::PointXYZ> extractor1; // 点提取对象
        // extractor1.setInputCloud(cloud);
        // extractor1.setIndices(inliers);
        // // false表示的是输入点集的点
        // extractor1.setNegative(false);
        // extractor1.filter(ground_cloud);
        // std::cout << "filter done." << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    // 输出
    cloud_out.resize(obj_cloud.points.size());
    cloud_out = obj_cloud;
    // std::cout << "cloud_in size: " << cloud_in.points.size() << std::endl;
    // std::cout << "cloud_out size: " << cloud_out.points.size() << std::endl;
    // std::cout << "ground_cloud size: " << ground_cloud.points.size() << std::endl;
}

// @@@
// 函数：PCL下采样滤波器
// 功能：将点云下采样，减少后续计算量
// 参数：输入输出点云
void Hazard_Detection::PCL_DownSample(pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out)
{
    // ROS_INFO("PCL Downsample Filter!");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud = cloud_in.makeShared();
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
    // 体素采样滤波
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    sor.filter(cloud_filtered);
    cloud_out.resize(cloud_filtered.points.size());
    cloud_out = cloud_filtered;
}

//  @@@
//  函数：PCL直通滤波器
//  功能：将一定范围内的点云滤除，其它点云保留
//  参数：输入输出点云
void Hazard_Detection::PCL_PassThrough_1(pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out)
{
    // ROS_INFO("PCL Pass Through Filter!");
    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud = cloud_in.makeShared();
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;

    // x轴滤波
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(passthrough_filter_xmin, passthrough_filter_xmax);
    pass.setNegative(false);
    pass.filter(cloud_filtered);
    // y轴滤波
    cloud = cloud_filtered.makeShared();
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(passthrough_filter_ymin, passthrough_filter_ymax);
    pass.setNegative(false);
    pass.filter(cloud_filtered);
    // // z轴滤波
    // cloud = cloud_filtered.makeShared();
    // pass.setInputCloud(cloud);
    // pass.setFilterFieldName("z");
    // pass.setFilterLimits(passthrough_filter_zmin, passthrough_filter_zmax);
    // pass.setNegative(true);
    // pass.filter(cloud_filtered);

    // 输出
    cloud_out.resize(cloud_filtered.points.size());
    cloud_out = cloud_filtered;

}

//  @@@
//  函数：PCL直通滤波器2
//  功能：将一定范围内的点云滤除，其它点云保留
//  参数：输入输出点云
void Hazard_Detection::PCL_PassThrough_2(pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out)
{
    // ROS_INFO("PCL Pass Through Filter!");
    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud = cloud_in.makeShared();
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;

    // x轴滤波
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(passthrough_filter2_xmin, passthrough_filter2_xmax);
    pass.setNegative(false);
    pass.filter(cloud_filtered);

    // y轴滤波
    cloud = cloud_filtered.makeShared();
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(passthrough_filter2_ymin, passthrough_filter2_ymax);
    pass.setNegative(false);
    pass.filter(cloud_filtered);

    // z轴滤波
    cloud = cloud_filtered.makeShared();
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(passthrough_filter_zmin, passthrough_filter_zmax);
    pass.setNegative(true);
    pass.filter(cloud_filtered);

    // 输出
    cloud_out.resize(cloud_filtered.points.size());
    cloud_out = cloud_filtered;
}

//  @@@
//  函数：PCL离群点去除滤波器
//  功能：移除点云中的噪声或离群点
//  参数：输入输出点云
void Hazard_Detection::PCLOutlierRemoval(pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out)
{
    // ROS_INFO("PCL Outlier Removal Filter!");
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud = cloud_in.makeShared();
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
    if (cloud->points.size() < outlier_k * 10)
    {
        // ROS_WARN("Point Cloud Size is Too Small.Skip Outlier Removal Filter");
        cloud_out.resize(cloud->points.size());
        cloud_out = *cloud;
        return;
    }

    sor.setInputCloud(cloud);                   // 设置待滤波的点云
    sor.setMeanK(outlier_k);                    // 设置在进行统计时考虑查询点邻居点数
    sor.setStddevMulThresh(outliner_threshold); // 设置判断是否为离群点的阈值
    sor.filter(cloud_filtered);                 // 将滤波结果保存在cloud_filtered中

    // 输出
    cloud_out.resize(cloud_filtered.points.size());
    cloud_out = cloud_filtered;
}

// @@@
// 函数：障碍物位置确定
// 功能：较高的障碍物会视角遮挡后方，因此默认被遮挡的区域都是不可达的
// 参数：处理后的点云
void Hazard_Detection::GetHazardPosition(pcl::PointCloud<pcl::PointXYZ> &ptcloud_3d, pcl::PointCloud<pcl::PointXYZ> &ptcloud2d)
{
    // 直接根据几何关系点云遮挡是不可行的，因为遮挡的区域过大，会导致很多可达区域被误判为不可达
    // 因此直接将点云后面的一部分区域都判定为不可达
    // ROS_INFO("Get Hazard Position!");
    ptcloud2d.points.clear();
    ptcloud2d.header.frame_id = ptcloud_3d.header.frame_id;
    // double x, y, z;
    // GetTransformXYZ(vehicle_frame_id, camera_frame_id, x, y, z);
    // x = 0, y = 0;
    double x = car2cam_tf_x;
    double y = car2cam_tf_y;
    pcl::PointXYZ point;
    point.z = -0.3;
    // Dense simulated clouds can easily exceed 65535 points, so use size_t here.
    for (size_t p = 0; p < ptcloud_3d.points.size(); ++p)
    {
        // 将点云的每一个点后面的一部分区域都判定为不可达
        // ptcloud2d.points.push_back(ptcloud_3d.points[p]);
        // 前方区域视作不可达
        for (float i = 0; i < obstacle_front_threshold; i = i + obstacle_resolution)
        {
            point.x = (ptcloud_3d.points[p].x - x) * (1 - i / 100.0) + x;
            point.y = (ptcloud_3d.points[p].y - y) * (1 - i / 100.0) + y;
            point.z = ptcloud_3d.points[p].z;
            ptcloud2d.points.push_back(point);
        }
        // 后方区域视作不可达
        for (float i = 0; i < obstacle_back_threshold; i = i + obstacle_resolution)
        {
            point.x = (ptcloud_3d.points[p].x - x) * (1 + i / 100.0) + x;
            point.y = (ptcloud_3d.points[p].y - y) * (1 + i / 100.0) + y;
            point.z = ptcloud_3d.points[p].z;
            ptcloud2d.points.push_back(point);
        }
        // 左右侧部分区域视作不可达
        for (float i = 0; i < 5 * obstacle_resolution; i = i + obstacle_resolution)
        {
            point.x = (ptcloud_3d.points[p].x - x) * (1 - i / 100.0) + x;
            point.y = ptcloud_3d.points[p].y;
            point.z = ptcloud_3d.points[p].z;
            ptcloud2d.points.push_back(point);

            point.x = (ptcloud_3d.points[p].x - x) * (1 + i / 100.0) + x;
            point.y = ptcloud_3d.points[p].y;
            point.z = ptcloud_3d.points[p].z;
            ptcloud2d.points.push_back(point);

            point.x = ptcloud_3d.points[p].x;
            point.y = (ptcloud_3d.points[p].y - y) * (1 - i / 100.0) + y;
            point.z = ptcloud_3d.points[p].z;
            ptcloud2d.points.push_back(point);

            point.x = ptcloud_3d.points[p].x;
            point.y = (ptcloud_3d.points[p].y - y) * (1 + i / 100.0) + y;
            point.z = ptcloud_3d.points[p].z;
            ptcloud2d.points.push_back(point);
        }
    }
}

void Hazard_Detection::InitializeLocalMap()
{
    ROS_DEBUG("Initialize Local Map!");
    local_map.header.frame_id = vehicle_frame_id;
    local_map.info.resolution = map_resolution;
    // 根据点云滤波范围自动设置map大小
    map_front_x = passthrough_filter2_xmax;
    map_back_x = -passthrough_filter2_xmin;
    map_y = passthrough_filter2_ymax - passthrough_filter2_ymin;
    local_map.info.width = int((map_front_x + map_back_x) / map_resolution);
    local_map.info.height = (map_y / map_resolution);
    // 根据map_front_x,map_back_x,map_y求出地图的origin坐标
    local_map.info.origin.position.x = -map_back_x;
    local_map.info.origin.position.y = -map_y / 2;
    local_map.info.origin.position.z = 0;
    local_map.info.origin.orientation.x = 0;
    local_map.info.origin.orientation.y = 0;
    local_map.info.origin.orientation.z = 0;
    local_map.info.origin.orientation.w = 1;
    // 全部初始化为-1
    local_map.data.resize(local_map.info.width * local_map.info.height, -1);
}

// @@@
// 函数：生成局部地图
// 功能：利用2d点云生成局部地图
// 参数：ptcloud2d
void Hazard_Detection::GenerateLocalMap(pcl::PointCloud<pcl::PointXYZ> ptcloud_raw, pcl::PointCloud<pcl::PointXYZ> ptcloud_2d)
{
    // ROS_INFO("Generate Local Map!");
    local_map.header.stamp = ros::Time::now();
    // 清空地图数据
    local_map.data.clear();
    local_map.data.resize(local_map.info.width * local_map.info.height, -1);
    // ROS_WARN("local_map resolution: %f", local_map.info.resolution);
    // 将ptcloud_raw中的点云投影到地图上,对应格子置为0
    for (size_t p = 0; p < ptcloud_raw.points.size(); ++p)
    {
        int x = int((ptcloud_raw.points[p].x - local_map.info.origin.position.x) / local_map.info.resolution);
        int y = int((ptcloud_raw.points[p].y - local_map.info.origin.position.y) / local_map.info.resolution);
        if (x >= 0 && x < local_map.info.width && y >= 0 && y < local_map.info.height)
        {
            local_map.data[y * local_map.info.width + x] = 0;
        }
    }
    // 将ptcloud_2d中的点云投影到地图上,对应格子置为100
    for (size_t p = 0; p < ptcloud_2d.points.size(); ++p)
    {
        int x = int((ptcloud_2d.points[p].x - local_map.info.origin.position.x) / local_map.info.resolution);
        int y = int((ptcloud_2d.points[p].y - local_map.info.origin.position.y) / local_map.info.resolution);
        if (x >= 0 && x < local_map.info.width && y >= 0 && y < local_map.info.height)
        {
            local_map.data[y * local_map.info.width + x] = 100;
        }
    }
    //  发布局部地图
    localmap_pub.publish(local_map);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hazard_detection");
    ros::NodeHandle nh;
    Hazard_Detection hazard_detection(nh);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}