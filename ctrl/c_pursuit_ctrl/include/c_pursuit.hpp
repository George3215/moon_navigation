#include "ros/ros.h"
#include "ros/time.h"
// #include "std_msgs/Float64.h"
#include "std_msgs/String.h"

#include "tf/transform_listener.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"
#include <algorithm>

class C_Pursuit
{
    public:
    C_Pursuit(ros::NodeHandle nh);

    private:
    void ReadParam();
    void PathCallback(const nav_msgs::Path::ConstPtr& msg);

    void GlobalPathCallback(const nav_msgs::Path::ConstPtr& msg);
    void LocalPathCallback(const nav_msgs::Path::ConstPtr& msg);

    void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void ModeCallback(const std_msgs::String::ConstPtr& msg);
    void TransformPath(const geometry_msgs::PoseWithCovarianceStamped &pose, nav_msgs::Path &path, nav_msgs::Path &path_converted);
    void UpdateIndex(int &path_index,int &final_index,int path_size);
    void CC_Pursuit(nav_msgs::Path &path_converted,int &path_index,int &final_index,double &set_speed,double &set_steering);
    void Pure_Pursuit(nav_msgs::Path &path_converted,int &path_index,int &final_index,double &set_speed,double &set_steering);
    void PublishCmd(double &set_speed,double &set_steering);
    //  ros相关
    ros::NodeHandle nodehandle_;
    ros::Subscriber path_sub,pose_sub,goal_sub,mode_sub;
    ros::Subscriber global_path_sub,local_path_sub;
    ros::Publisher cmd_pub;
    ros::Publisher path_converted_pub;
    ros::Publisher look_forward_pt_pub;
    //  传入参数
    std::string path_topic_name,pose_topic_name,goal_topic_name,cmd_topic_name,mode_topic_name;
    std::string chassis_frame_id, map_frame_id;
    std::string global_path_topic_name,local_path_topic_name;
    double vehicle_length;
    double dth,safety_corridor,k3;
    double default_speed;
    double mode1_speed,mode2_speed,mode3_speed;
    double time_duration;
    bool use_closest_index;
    bool use_pure_pursuit;
    bool verbose_logging_ = false;
    bool pursuit_finished = true;
    double max_yaw_rate_ = 1.2;
    double min_tracking_speed_ = 0.2;
    double turn_speed_gain_ = 0.35;
    double rotate_in_place_yaw_rate_ = 1.2;
    double heading_error_kp_ = 1.2;
    double rotate_in_place_heading_thresh_ = 0.9;
    //  收发消息内容
    nav_msgs::Path path;
    geometry_msgs::PoseWithCovarianceStamped pose;
    geometry_msgs::PoseStamped goal;
    geometry_msgs::Twist cmd_vel;
    geometry_msgs::PointStamped look_forward_point;
    // 路径状态指示
    bool path_valid,stop_when_newpath;
    // pure pursuit 相关
    nav_msgs::Path path_converted;
    int path_index,final_index;
    // cmd相关
    double set_speed,set_steering;
    // mode相关
    std::string mode;


    double path_rear_x,path_rear_y,path_front_x,path_front_y;
    double path_goal_x,path_goal_y;
    double A, B, C,dis;
    double x_foot,y_foot,dis_front2foot;
    double x_unit, y_unit;
    double dth1;
    double dth_point_x,dth_point_y;
    double L,alpha,R;
    
};