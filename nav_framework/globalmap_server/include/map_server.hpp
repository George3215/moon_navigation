#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
// 感知层初始化地图和更新地图的服务
#include "globalmap_server/initialize_map.h"
#include "globalmap_server/update_map.h"
// 规划层获取全局地图消息发布的服务
#include "globalmap_server/get_globalmap.h"
// 规划层储存全局路径的服务
#include "globalmap_server/initialize_path.h"
#include "tf/transform_datatypes.h"
// 多线程
#include <thread>
// // 全局路径使用的数据结构
// #include <vector>
// #include <array>

class MapServer
{
    public:
    MapServer(ros::NodeHandle nh);

    private:
    bool Initialize_Map(globalmap_server::initialize_map::Request &req, globalmap_server::initialize_map::Response &res);
    bool Update_Map(globalmap_server::update_map::Request &req, globalmap_server::update_map::Response &res);
    bool Get_Globalmap(globalmap_server::get_globalmap::Request &req, globalmap_server::get_globalmap::Response &res);
    bool Initialize_Path(globalmap_server::initialize_path::Request &req, globalmap_server::initialize_path::Response &res);
    bool Get_Localmap(globalmap_server::get_globalmap::Request &req, globalmap_server::get_globalmap::Response &res);

    void Update_GlobalPath();
    geometry_msgs::PoseStamped Get_LocalGoal();
    bool IsPointInObstacle(double world_x, double world_y, int threshold);

    void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void ModeCallback(const std_msgs::String::ConstPtr &msg);

    void GlobalPathCallback(const nav_msgs::Path::ConstPtr &msg);
    void LocalPathCallback(const nav_msgs::Path::ConstPtr &msg);
    bool CollisionCheck(std::string mode, double inflation_radius_grid, int obstacle_threshold_);
    void CollisionDetectionLoop(ros::Publisher& replan_signal_pub_);
    
    ros::NodeHandle nh_;
    // ros服务：服务类型
    ros::ServiceServer Initialize_Map_srv_;
    ros::ServiceServer Update_Map_srv_;
    ros::ServiceServer Get_Globalmap_srv_;
    ros::ServiceServer Initialize_Path_srv_;
    ros::ServiceServer Get_Localmap_srv_;

    // 订阅和发布话题:起终点
    ros::Subscriber navigation_pose_sub_,navigation_goal_sub_;
    ros::Subscriber navigation_mode_sub_;
    ros::Publisher pose_pub,local_goal_pub,global_goal_pub;
    // 全局路径发布
    ros::Publisher global_path_pub_;

    // 全局地图
    nav_msgs::OccupancyGrid global_map_;
    nav_msgs::OccupancyGrid global_map_raw_;
    // 模式地图：存储每个栅格的数据来源
    nav_msgs::OccupancyGrid mode_map_;
    ros::Publisher global_map_pub_,mode_map_pub_;
    bool visulize_global_map_=false;

    // 地图参数
    double globalmap_resolution_,localmap_resolution_;
    double local_map_size_;

    // 起点终点订阅话题
    std::string navigation_pose_topic_,navigation_goal_topic_,navigation_mode_topic_;
    // 起点终点发布话题
    std::string planner_global_goal_topic_,planner_global_map_topic_;
    std::string planner_local_goal_topic_,planner_local_map_topic_;

    // 路径检测模块：订阅发布的局部路径话题
    std::string global_path_updated_topic_,local_path_updated_topic_;
    //// 订阅全局和局部路径话题
    ros::Subscriber global_path_updated_sub_,local_path_updated_sub_;
    //// 发布重规划的信号话题
    std::string replan_signal_topic_;
    ros::Publisher replan_signal_pub_;
    std_msgs::String replan_signal;
    //// 被视为障碍的阈值
    int obstacle_threshold_;
    double inflation_radius_;
    //// 碰撞检测线程
    std::thread collision_detection_thread_;

    // 存储全局和局部路径
    nav_msgs::Path global_path_updated_,local_path_updated_;


    // 发布话题：全局地图，全局终点，局部地图，局部终点
    ros::Publisher planner_global_goal_pub_,planner_global_map_pub_;
    ros::Publisher planner_local_goal_pub_,planner_local_map_pub_;

    // 起点终点
    double start_x_,start_y_,globalgoal_x_,globalgoal_y_;
    bool pose_initialized_=false,goal_initialized_=false;
    geometry_msgs::PoseStamped local_goal_;
    // 当前模式
    std::string mode_;

    // 全局路径
    std::string global_path_topic_;
    nav_msgs::Path global_path_;
    nav_msgs::Path global_path_updated;
    // 全局路径的更新时间间隔
    double global_path_update_interval_;
    ros::Time last_global_path_update_time_;
    bool verbose_logging_ = false;

    // 前进式路径跟踪：记录上一次匹配到的全局路径索引，防止回退
    int last_tracked_index_ = 0;
    // 局部目标前瞻距离(m)：在全局路径上向前寻找局部目标的最小距离
    double local_goal_lookahead_ = 5.0;
    
};