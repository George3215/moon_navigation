#include "map_server_service.cpp"
//  @@@
//  函数：构造函数
MapServer::MapServer(ros::NodeHandle nh)
{
    nh_ = nh;
    // // 读取参数
    nh_.param<double>("/globalmap_resolution", globalmap_resolution_, 1.0);
    nh_.param<double>("/localmap_resolution", localmap_resolution_, 0.25);
    ROS_WARN("globalmap_resolution: %f, localmap_resolution: %f", globalmap_resolution_, localmap_resolution_);
    nh_.param<std::string>("/navigation_pose_topic", navigation_pose_topic_, "/initialpose");
    nh_.param<std::string>("/navigation_goal_topic", navigation_goal_topic_, "/move_base_simple/goal");
    nh_.param<std::string>("/navigation_mode_topic", navigation_mode_topic_, "/mode");
    ROS_WARN("navigation_pose_topic: %s", navigation_pose_topic_.c_str());
    ROS_WARN("navigation_goal_topic: %s", navigation_goal_topic_.c_str());
    ROS_WARN("navigation_mode_topic: %s", navigation_mode_topic_.c_str());
    nh.param<bool>("/visualize_map", visulize_global_map_, false);
    ROS_WARN("visualize_map: %d", visulize_global_map_);
    // planner:发布全局地图和全局终点
    nh_.param<std::string>("/publish_topic/global_goal", planner_global_goal_topic_, "/planner/global_goal");
    nh_.param<std::string>("/publish_topic/global_map", planner_global_map_topic_, "/planner/global_map");
    ROS_WARN("planner_global_goal_topic: %s", planner_global_goal_topic_.c_str());
    ROS_WARN("planner_global_map_topic: %s", planner_global_map_topic_.c_str());
    // planner:发布局部地图和局部终点
    nh_.param<std::string>("/publish_topic/local_goal", planner_local_goal_topic_, "/planner/local_goal");
    nh_.param<std::string>("/publish_topic/local_map", planner_local_map_topic_, "/planner/local_map");
    ROS_WARN("planner_local_goal_topic: %s", planner_local_goal_topic_.c_str());
    ROS_WARN("planner_local_map_topic: %s", planner_local_map_topic_.c_str());
    // planner:局部地图的大小
    nh_.param<double>("/publish_topic/local_map_size", local_map_size_, 10.0);
    ROS_WARN("local_map_size: %f", local_map_size_);
    // planner:发布全局路径和更新的时间间隔
    nh_.param<std::string>("/global_path_topic", global_path_topic_, "/planner/global_path");
    nh_.param<double>("/global_path_update_interval", global_path_update_interval_, 0.5);
    ROS_WARN("global_path_topic: %s", global_path_topic_.c_str());
    ROS_WARN("global_path_update_interval: %f", global_path_update_interval_);
    // collision_check:订阅全局和局部路径
    nh_.param<std::string>("/collision_check/global_path_updated_topic", global_path_updated_topic_, "/global_path_updated");
    nh_.param<std::string>("/collision_check/local_path_updated_topic", local_path_updated_topic_, "/local_path_updated");
    ROS_WARN("global_path_updated_topic: %s", global_path_updated_topic_.c_str());
    ROS_WARN("local_path_updated_topic: %s", local_path_updated_topic_.c_str());
    // 重规划话题名
    nh_.param<std::string>("/collision_check/replan_signal_topic", replan_signal_topic_, "/replan_signal");
    ROS_WARN("replan_signal_topic: %s", replan_signal_topic_.c_str());
    // 被视为障碍的阈值
    nh_.param<int>("/collision_check/obstacle_cost_threshold", obstacle_threshold_, 50);
    ROS_WARN("obstacle_cost_threshold: %d", obstacle_threshold_);
    // 障碍物膨胀半径
    nh_.param<double>("/collision_check/inflation_radius", inflation_radius_, 0.5);
    ROS_WARN("inflation_radius: %f", inflation_radius_);
    nh_.param<bool>("/verbose_logging", verbose_logging_, false);
    ROS_WARN("verbose_logging: %s", verbose_logging_ ? "enabled" : "disabled");
    nh_.param<double>("/local_goal_lookahead", local_goal_lookahead_, 5.0);
    ROS_WARN("local_goal_lookahead: %f", local_goal_lookahead_);

    // 服务初始化
    Initialize_Map_srv_ = nh_.advertiseService("Initialize_Map", &MapServer::Initialize_Map, this);
    Update_Map_srv_ = nh_.advertiseService("Update_Map", &MapServer::Update_Map, this);
    Get_Globalmap_srv_ = nh_.advertiseService("Get_Globalmap", &MapServer::Get_Globalmap, this);
    Initialize_Path_srv_ = nh_.advertiseService("Initialize_Path", &MapServer::Initialize_Path, this);
    Get_Localmap_srv_ = nh_.advertiseService("Get_Localmap", &MapServer::Get_Localmap, this);

    // // 订阅起点和终点话题，模式话题
    navigation_pose_sub_ = nh_.subscribe(navigation_pose_topic_, 1, &MapServer::PoseCallback, this);
    navigation_goal_sub_ = nh_.subscribe(navigation_goal_topic_, 1, &MapServer::GoalCallback, this);
    navigation_mode_sub_ = nh_.subscribe(navigation_mode_topic_, 1, &MapServer::ModeCallback, this);

    // 规划层
    // // 发布终点和地图：全局
    planner_global_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(planner_global_goal_topic_, 0, true);
    planner_global_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(planner_global_map_topic_, 0, true);
    // // 发布终点和地图：局部
    planner_local_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(planner_local_goal_topic_, 0, true);
    planner_local_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(planner_local_map_topic_, 0, true);
    // // 发布全局路径
    global_path_pub_ = nh_.advertise<nav_msgs::Path>(global_path_topic_, 1, true);

    // 碰撞检测
    // // 订阅全局和局部路径
    global_path_updated_sub_ = nh_.subscribe(global_path_updated_topic_, 1, &MapServer::GlobalPathCallback, this);
    local_path_updated_sub_ = nh_.subscribe(local_path_updated_topic_, 1, &MapServer::LocalPathCallback, this);
    // // 发布重规划信号
    replan_signal_pub_ = nh_.advertise<std_msgs::String>(replan_signal_topic_, 1, true);

    if (visulize_global_map_)
    {
        // 监视@ 发布地图topic：全局地图和模式地图，便于监视
        global_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map_server/global_map", 1, true);
        mode_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map_server/mode_map", 1, true);
    }
    // 启动障碍检测循环在独立线程中
    collision_detection_thread_ = std::thread(&MapServer::CollisionDetectionLoop, this, std::ref(replan_signal_pub_));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_server");

    ros::NodeHandle nh;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    MapServer map_server(nh);
    ros::AsyncSpinner spinner(8);
    spinner.start();
    ROS_INFO("[ SERVICE ] Map server started");

    ros::waitForShutdown();
    return 0;
}