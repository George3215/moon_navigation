#include "c_pursuit.hpp"
#include <cmath>

//  @@@
//  函数：构造函数
//  功能：读取参数并初始化话题订阅发布
//  参数：nodehandle
C_Pursuit::C_Pursuit(ros::NodeHandle nh)
    : nodehandle_(nh)
{
    ReadParam();
    // path_sub = nodehandle_.subscribe(path_topic_name, 1, &C_Pursuit::PathCallback, this);
    pose_sub = nodehandle_.subscribe(pose_topic_name, 1, &C_Pursuit::PoseCallback, this);
    goal_sub = nodehandle_.subscribe(goal_topic_name, 1, &C_Pursuit::GoalCallback, this);
    mode_sub = nodehandle_.subscribe(mode_topic_name, 1, &C_Pursuit::ModeCallback, this);
    cmd_pub = nodehandle_.advertise<geometry_msgs::Twist>(cmd_topic_name, 1);
    path_converted_pub = nodehandle_.advertise<nav_msgs::Path>("/c_Pursuit/path_converted", 1);
    look_forward_pt_pub = nodehandle_.advertise<geometry_msgs::PointStamped>("/c_Pursuit/look_forward_point", 1);

    global_path_sub = nodehandle_.subscribe(global_path_topic_name, 1, &C_Pursuit::GlobalPathCallback, this);
    local_path_sub = nodehandle_.subscribe(local_path_topic_name, 1, &C_Pursuit::LocalPathCallback, this);
}
//  @@@
//  函数：读取参数
//  功能：从ros参数服务器中读取参数
//  参数：
void C_Pursuit::ReadParam()
{

    ROS_INFO("Reading Parameters From ROS Param Server...");
    // 话题名称
    nodehandle_.param<std::string>("/c_pursuit/path_topic_name", path_topic_name, "/c_pursuit/path");
    nodehandle_.param<std::string>("/c_pursuit/pose_topic_name", pose_topic_name, "/c_pursuit/pose");
    nodehandle_.param<std::string>("/c_pursuit/goal_topic_name", goal_topic_name, "/c_pursuit/goal");
    nodehandle_.param<std::string>("/c_pursuit/cmd_topic_name", cmd_topic_name, "/cmd_vel");
    nodehandle_.param<std::string>("/c_pursuit/mode_topic_name", mode_topic_name, "/c_pursuit/mode");
    ROS_DEBUG("path_topic_name: %s", path_topic_name.c_str());
    ROS_DEBUG("pose_topic_name: %s", pose_topic_name.c_str());
    ROS_DEBUG("goal_topic_name: %s", goal_topic_name.c_str());
    ROS_DEBUG("cmd_topic_name: %s", cmd_topic_name.c_str());
    ROS_DEBUG("mode_topic_name: %s", mode_topic_name.c_str());
    // 车辆参数
    nodehandle_.param<std::string>("/c_pursuit/chassis_frame_id", chassis_frame_id, "base_link");
    nodehandle_.param<std::string>("/c_pursuit/map_frame_id", map_frame_id, "map");
    nodehandle_.param<double>("/c_pursuit/vehicle_length", vehicle_length, 0.305);
    ROS_DEBUG("chassis_frame_id: %s", chassis_frame_id.c_str());
    ROS_DEBUG("map_frame_id: %s", map_frame_id.c_str());
    ROS_DEBUG("vehicle_length: %f", vehicle_length);
    // c-pursuit参数
    nodehandle_.param<double>("/c_pursuit/dth", dth, 1.0);
    nodehandle_.param<double>("/c_pursuit/safety_corridor", safety_corridor, 1.0);
    nodehandle_.param<double>("/c_pursuit/k3", k3, 1.0);
    ROS_DEBUG("look_ahead_distance: %f", dth);
    ROS_DEBUG("safety_corridor: %f", safety_corridor);
    ROS_DEBUG("k3: %f", k3);
    nodehandle_.param<bool>("/c_pursuit/stop_when_newpath", stop_when_newpath, true);
    ROS_DEBUG("stop_when_newpath: %d", stop_when_newpath);
    // 速度参数
    nodehandle_.param<double>("/c_pursuit/default_speed", default_speed, 1.0);
    nodehandle_.param<double>("/c_pursuit/mode1_speed", mode1_speed, 1.0);
    nodehandle_.param<double>("/c_pursuit/mode2_speed", mode2_speed, 0.8);
    nodehandle_.param<double>("/c_pursuit/mode3_speed", mode3_speed, 0.5);
    ROS_DEBUG("default_speed: %f", default_speed);
    ROS_DEBUG("mode1_speed: %f", mode1_speed);
    ROS_DEBUG("mode2_speed: %f", mode2_speed);
    ROS_DEBUG("mode3_speed: %f", mode3_speed);
    // 是否采用最近点索引
    nodehandle_.param<bool>("/c_pursuit/use_closest_index", use_closest_index, false);
    ROS_DEBUG("use_closest_index: %d", use_closest_index);
    // pure_pursuit
    nodehandle_.param<bool>("/c_pursuit/use_pure_pursuit", use_pure_pursuit, false);
    // verbose logging
    nodehandle_.param<bool>("/verbose_logging", verbose_logging_, false);
    nodehandle_.param<double>("/c_pursuit/max_yaw_rate", max_yaw_rate_, 1.6);
    nodehandle_.param<double>("/c_pursuit/min_tracking_speed", min_tracking_speed_, 0.08);
    nodehandle_.param<double>("/c_pursuit/turn_speed_gain", turn_speed_gain_, 0.18);
    nodehandle_.param<double>("/c_pursuit/rotate_in_place_yaw_rate", rotate_in_place_yaw_rate_, max_yaw_rate_);
    nodehandle_.param<double>("/c_pursuit/heading_error_kp", heading_error_kp_, 1.2);
    nodehandle_.param<double>("/c_pursuit/rotate_in_place_heading_thresh", rotate_in_place_heading_thresh_, 0.9);
    // follow path topic
    nodehandle_.param<std::string>("/c_pursuit/global_path_topic_name", global_path_topic_name, "/global_path");
    nodehandle_.param<std::string>("/c_pursuit/local_path_topic_name", local_path_topic_name, "/local_path");
    ROS_DEBUG("global_path_topic_name: %s", global_path_topic_name.c_str());
    ROS_DEBUG("local_path_topic_name: %s", local_path_topic_name.c_str());

    ROS_INFO("Read Parameters From ROS Param Server Finished!");
    path_valid = 0;
}

void C_Pursuit::ModeCallback(const std_msgs::String::ConstPtr &msg)
{
    mode = msg->data;
    if (mode == "Mode 1")
    {
        ROS_DEBUG("Mode 1");
        default_speed = mode1_speed;
        // use_pure_pursuit = true;
        stop_when_newpath = false;
    }
    else if (mode == "Mode 2")
    {
        ROS_DEBUG("Mode 2");
        default_speed = mode2_speed;
        // use_pure_pursuit = false;
        stop_when_newpath = true;
    }
    else if (mode == "Mode 3")
    {
        ROS_DEBUG("Mode 3");
        default_speed = mode3_speed;
        // use_pure_pursuit = false;
        stop_when_newpath = true;
    }
    else
    {
        ROS_ERROR("Invalid Mode!");
        default_speed = 0;
    }
}

void C_Pursuit::GlobalPathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    // path_valid = 0;
    //  ROS_INFO("Path Callback");
    //  path为空
    if (mode != "Mode 1")
    {
        return;
    }
    ROS_DEBUG("Mode 1, Follow Global Path.");
    if (msg->poses.size() == 0)
    {
        ROS_ERROR("[Path Callback]Path is empty!");
        set_speed = 0;
        set_steering = 0;
        path_valid = 0;
        return;
    }
    // frame_id不正确
    else if (msg->header.frame_id != map_frame_id)
    {
        ROS_ERROR("[Path Callback]Path frame_id is not map!");
        path_valid = 0;
        return;
    }
    // 更新path

    path = *msg;
    int path_size = (int)msg->poses.size();
    ROS_DEBUG("Global Path size = [%d]", path_size);
    //  更新索引，从第0个点开始跟
    UpdateIndex(path_index, final_index, path_size);
    if (stop_when_newpath)
    {
        ROS_INFO("Stop when new path received.");
        set_speed = 0;
        set_steering = 0;
        PublishCmd(set_speed, set_steering);
        ros::Duration(1.0).sleep();
    }
    path_valid = 1;
}

void C_Pursuit::LocalPathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    if (!(mode == "Mode 2" || mode == "Mode 3"))
    {
        return;
    }
    ROS_DEBUG("Mode 2 or Mode 3, Follow Local Path.");
    // path_valid = 0;
    //  ROS_INFO("Path Callback");
    //  path为空
    if (msg->poses.size() == 0)
    {
        ROS_ERROR("[Path Callback]Path is empty!");
        set_speed = 0;
        set_steering = 0;
        path_valid = 0;
        return;
    }
    // frame_id不正确
    else if (msg->header.frame_id != map_frame_id)
    {
        ROS_ERROR("[Path Callback]Path frame_id is not map!");
        path_valid = 0;
        return;
    }
    // 更新path

    path = *msg;
    int path_size = (int)msg->poses.size();
    ROS_DEBUG("Local Path size = [%d]", path_size);
    //  更新索引，从第0个点开始跟
    UpdateIndex(path_index, final_index, path_size);
    if (stop_when_newpath)
    {
        ROS_INFO("Stop when new path received.");
        set_speed = 0;
        set_steering = 0;
        PublishCmd(set_speed, set_steering);
        ros::Duration(1.0).sleep();
    }
    path_valid = 1;
}

//  @@@
//  函数：path的回调函数
//  功能：判断消息是否有效，有效则储存在path中，并且进行path的坐标转换
//  参数：path指针
void C_Pursuit::PathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    // path_valid = 0;
    //  ROS_INFO("Path Callback");
    //  path为空
    if (msg->poses.size() == 0)
    {
        ROS_ERROR("[Path Callback]Path is empty!");
        set_speed = 0;
        set_steering = 0;
        path_valid = 0;
        return;
    }
    // frame_id不正确
    else if (msg->header.frame_id != map_frame_id)
    {
        ROS_ERROR("[Path Callback]Path frame_id is not map!");
        path_valid = 0;
        return;
    }
    // 更新path

    path = *msg;
    int path_size = (int)msg->poses.size();
    ROS_DEBUG("Path size = [%d]", path_size);
    TransformPath(pose, path, path_converted);
    //  更新索引，从第0个点开始跟
    UpdateIndex(path_index, final_index, path_size);
    if (stop_when_newpath)
    {
        ROS_INFO("Stop when new path received.");
        set_speed = 0;
        set_steering = 0;
        PublishCmd(set_speed, set_steering);
        ros::Duration(1.0).sleep();
    }
    path_valid = 1;
}

//  @@@
//  函数：pose的回调函数
//  功能：判断消息是否有效并储存，而后进行横纵向控制和消息发布
//  参数：pose指针
void C_Pursuit::PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    // ROS_INFO("Pose Callback");
    // frame_id不正确
    // if (msg->header.frame_id != chassis_frame_id)
    // {
    //     ROS_ERROR("[Pose Callback]Pose frame_id is not chassis_frame_id!");
    //     return;
    // }
    // 更新pose
    pose = *msg;
    if (!path_valid)
    {
        // ROS_WARN("Path is not valid!");
        set_speed = 0;
        set_steering = 0;
        PublishCmd(set_speed, set_steering);
        return;
    }
    // 转换path
    TransformPath(pose, path, path_converted);

    // pure_pursuit
    if (use_pure_pursuit)
    {
        Pure_Pursuit(path_converted, path_index, final_index, set_speed, set_steering);
    }
    else
    {
        CC_Pursuit(path_converted, path_index, final_index, set_speed, set_steering);
    }

    // 发布控制指令
    PublishCmd(set_speed, set_steering);
}
//  @@@
//  函数：goal的回调函数
//  功能：判断消息是否有效，有效则进行储存
//  参数：goal指针
void C_Pursuit::GoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{

    ROS_DEBUG("Goal Callback");
    // frame_id不正确
    if (msg->header.frame_id != map_frame_id)
    {
        ROS_ERROR("[Goal Callback]Goal frame_id is not map!");
        return;
    }
    // 更新goal
    goal = *msg;
}
//  @@@
//  函数：path的tf变换
//  功能：将原地图坐标下的path消息通过tf变换，转换成车体坐标系下的path消息
//  参数：当前pose，原path消息，转换后的path消息
void C_Pursuit::TransformPath(const geometry_msgs::PoseWithCovarianceStamped &pose, nav_msgs::Path &path, nav_msgs::Path &path_converted)
{
    // 定义转换变量
    geometry_msgs::PoseStamped path_input[path.poses.size()];
    geometry_msgs::PoseStamped path_output[path.poses.size()];
    // path路径点读入并暂存
    for (int i = 0; i < (int)path.poses.size(); i++)
    {
        path_input[i].header.frame_id = path.header.frame_id;
        path_input[i].pose = path.poses[i].pose;
    }
    // 读取tf变换关系
    tf::TransformListener listener;
    listener.waitForTransform(chassis_frame_id, map_frame_id, ros::Time(0), ros::Duration(1));
    // tf转换
    try
    {
        for (int i = 0; i < (int)path.poses.size(); i++)
        {
            listener.transformPose(chassis_frame_id, path_input[i], path_output[i]);
        }
        // 转换后的路径点写入path_converted
        path_converted.header.frame_id = chassis_frame_id;
        path_converted.poses.resize(path.poses.size());
        for (int i = 0; i < (int)path.poses.size(); i++)
        {
            path_converted.poses[i] = path_output[i];
            // ROS_INFO("[Path callback]Point%d,pos[%f , %f]",i,path_converted.poses[i].pose.position.x,path_converted.poses[i].pose.position.y);
        }
        // 发布转换的路径
        path_converted_pub.publish(path_converted);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        // ros::Duration(1.0).sleep();
    }
}
//  @@@
//  函数：更新path索引
//  功能：path更新后，更新索引，使其从第0个点开始重新跟随
//  参数：path索引
void C_Pursuit::UpdateIndex(int &path_index, int &final_index, int path_size)
{
    ROS_DEBUG("New path received, index updated.");
    final_index = path_size - 1;
    if (pursuit_finished != false)
    {
        time_duration = ros::Time::now().toSec();
    }
    pursuit_finished = false;

    if (use_closest_index)
    {
        // 初始化变量
        double pre_look_ahead_distance = dth; // 预瞄距离
        // ROS_INFO("Pre Look-ahead Distance: [%f]", pre_look_ahead_distance);
        bool found_valid_point = false; // 标志是否找到满足条件的点

        // 遍历路径点，找到预瞄距离以外的最近点
        for (int i = 0; i < path_converted.poses.size(); ++i)
        {
            // 路径点在车体坐标系中的位置
            double path_x = path_converted.poses[i].pose.position.x;
            double path_y = path_converted.poses[i].pose.position.y;

            // 计算路径点到车体的距离
            double distance = sqrt(path_x * path_x + path_y * path_y);

            // 如果路径点在预瞄距离范围以外且是前方点
            if (distance >= pre_look_ahead_distance && path_x > 0)
            {
                path_index = i;           // 记录路径点索引
                found_valid_point = true; // 找到有效点
                break;                    // 找到最近的合适点后退出
            }
        }

        // 如果未找到满足条件的点，选择路径终点作为目标点
        if (!found_valid_point)
        {
            path_index = final_index; // 更新索引为终点索引
            ROS_DEBUG("[Pure Pursuit] No valid look-ahead point found. Using final point.");
        }

        // 输出当前选定点的信息
        if (verbose_logging_)
        {
            ROS_INFO("[Pure Pursuit] Selected Look-ahead Index: [%d], Position: [x=%.3f, y=%.3f]",
                     path_index,
                     path_converted.poses[path_index].pose.position.x,
                     path_converted.poses[path_index].pose.position.y);
        }
    }

    else
    // 使用第0个点作为起点
    {
        path_index = 0;
        final_index = path_size - 1;
    }
}

//  @@@
//  函数：pure-pursuit算法
//  功能：Pure-pursuit
//  参数：
void C_Pursuit::Pure_Pursuit(nav_msgs::Path &path_converted, int &path_index, int &final_index, double &set_speed, double &set_steering)
{
    if (path_converted.poses.empty())
    {
        ROS_WARN("[Pure Pursuit] Path is empty!");
        return;
    }

    // 初始化变量
    double pre_look_ahead_distance = dth; // 预瞄距离
    // ROS_INFO("Pre Look-ahead Distance: [%f]", pre_look_ahead_distance);
    if (verbose_logging_)
    {
        ROS_INFO("[Pure Pursuit] Current path index start from: [%d]", path_index);
    }
    int look_ahead_index = path_index; // 初始化预瞄点索引
    bool found_valid_point = false;        // 标志是否找到合适的预瞄点

    // 遍历路径点，找到满足条件的预瞄点
    for (int i = path_index; i < (int)path_converted.poses.size(); i++)
    {
        double path_x = path_converted.poses[i].pose.position.x;
        double path_y = path_converted.poses[i].pose.position.y;

        // 计算路径点距离
        double distance = sqrt(path_x * path_x + path_y * path_y);

        // 判断是否为满足预瞄条件的前方点
        if (distance >= pre_look_ahead_distance && path_x > 0)
        {
            look_ahead_index = i;  // 更新预瞄点索引
            found_valid_point = true; // 找到合适的点
            break; // 找到最近的满足条件的点后退出
        }
    }

    // 如果没有找到合适的点，选择路径终点作为预瞄点
    if (!found_valid_point)
    {
        look_ahead_index = final_index; // 更新索引为终点
        ROS_DEBUG("[Pure Pursuit] No valid look-ahead point found. Using final point.");
    }

    // 更新路径索引为当前预瞄点索引，确保索引单调递增
    path_index = std::max(path_index, look_ahead_index);
    if (verbose_logging_)
    {
        ROS_INFO("[Pure Pursuit] Selected Look-ahead Index: [%d]", look_ahead_index);
    }

    // 获取选中的预瞄点坐标
    double target_x = path_converted.poses[look_ahead_index].pose.position.x;
    double target_y = path_converted.poses[look_ahead_index].pose.position.y;

    // 发布预瞄点
    look_forward_point.header.frame_id = chassis_frame_id;
    look_forward_point.header.stamp = ros::Time::now();
    look_forward_point.point.x = target_x;
    look_forward_point.point.y = target_y;
    look_forward_point.point.z = 0;
    look_forward_pt_pub.publish(look_forward_point);
    if (verbose_logging_)
    {
        ROS_INFO("Look-ahead Point: [%f, %f]", target_x, target_y);
    }

    // 判断预瞄点是否在车体后方
    if (target_x <= 0)
    {
        ROS_DEBUG("[Pure Pursuit] Look-ahead point is behind the vehicle.");

        // 根据预瞄点位置决定旋转方向
        if (target_y > 0)
        {
            set_speed = 0;
            set_steering = std::min(max_yaw_rate_, std::fabs(rotate_in_place_yaw_rate_)); // 逆时针自转
            ROS_DEBUG("[Pure Pursuit] Rotate Left.");
        }
        else
        {
            set_speed = 0;
            set_steering = -std::min(max_yaw_rate_, std::fabs(rotate_in_place_yaw_rate_)); // 顺时针自转
            ROS_DEBUG("[Pure Pursuit] Rotate Right.");
        }
        return;
    }

    // 如果预瞄点在车体前方，执行纯追踪逻辑
    double L = sqrt(target_x * target_x + target_y * target_y); // 预瞄点距离
    double alpha = atan2(target_y, target_x);                   // 航向误差

    // 航向误差很大时，差速车优先原地转向，避免大弯半径拉偏。
    if (std::fabs(alpha) > rotate_in_place_heading_thresh_)
    {
        set_speed = 0.0;
        set_steering = (alpha > 0.0 ? 1.0 : -1.0) *
                       std::min(max_yaw_rate_, std::fabs(rotate_in_place_yaw_rate_));
        return;
    }

    // 曲率项 + 航向P项；其中航向P项就是可调KP。
    double steering_gain = 1.2;
    double curvature_omega = steering_gain * (2.0 * default_speed * sin(alpha)) / std::max(0.20, L);
    double heading_omega = heading_error_kp_ * alpha;
    set_steering = curvature_omega + heading_omega;
    set_steering = std::max(-max_yaw_rate_, std::min(max_yaw_rate_, set_steering));

    // 转向越大速度越低，保证弯道可跟随且不冲出路径。
    double normalized_turn = std::fabs(set_steering) / std::max(0.05, max_yaw_rate_);
    double speed_scale = std::max(0.15, 1.0 - turn_speed_gain_ * normalized_turn);
    set_speed = std::max(min_tracking_speed_, default_speed * speed_scale);

    // 终点附近减速停车
    double goal_distance = sqrt(path_converted.poses[final_index].pose.position.x * path_converted.poses[final_index].pose.position.x +
                                path_converted.poses[final_index].pose.position.y * path_converted.poses[final_index].pose.position.y);
    if (goal_distance < safety_corridor)
    {
        set_speed = 0.0;
        set_steering = 0.0;
        pursuit_finished = true;
    }

    // 输出调试信息
    if (verbose_logging_)
    {
        ROS_INFO("Pure Pursuit: target_x = [%f], target_y = [%f], R = [%f], steering = [%f]", target_x, target_y, R, set_steering);
    }
}


//  @@@
//  函数：c-pursuit算法
//  功能：c-pursuit
//  参数：
void C_Pursuit::CC_Pursuit(nav_msgs::Path &path_converted, int &path_index, int &final_index, double &set_speed, double &set_steering)
{
    if (path_converted.poses.size() == 0)
    {
        ROS_WARN("[Pure Pursuit]path is empty!");
        return;
    }

    // ###根据index从path取出坐标点并进行预处理### //
    //  取出当前坐标点和front坐标点
    path_rear_x = path_converted.poses[path_index].pose.position.x;
    path_rear_y = path_converted.poses[path_index].pose.position.y;
    path_front_x = path_converted.poses[path_index + 1].pose.position.x;
    path_front_y = path_converted.poses[path_index + 1].pose.position.y;
    // 计算直线方程：Ax+By+C = 0
    A = path_front_y - path_rear_y;
    B = path_rear_x - path_front_x;
    C = path_front_x * path_rear_y - path_rear_x * path_front_y;
    // ROS_INFO("FRONT_X = [%f],FRONT_Y = [%f]", path_front_x, path_front_y);
    // ROS_INFO("REAR_X = [%f],REAR_Y = [%f]", path_rear_x, path_rear_y);
    // ROS_INFO("CAR X = [%f],CAR Y = [%f]", pose.pose.pose.position.x, pose.pose.pose.position.y);
    // ROS_INFO("A = [%f],B = [%f],C = [%f]", A, B, C);
    // 求点到直线的距离
    dis = abs(C) / sqrt(A * A + B * B);
    // ROS_INFO("Dis = [%f]", dis);
    // 求点到直线的垂足
    // @x = (B*B*x0-A*B*y0-A*C)/(A*A+B*B);
    // @y = (A*A*y0-A*B*x0-B*C)/(A*A+B*B);
    x_foot = (-A * C) / (A * A + B * B);
    y_foot = (-B * C) / (A * A + B * B);
    // 求front路径点到垂足的距离
    dis_front2foot = sqrt((path_front_x - x_foot) * (path_front_x - x_foot) + (path_front_y - y_foot) * (path_front_y - y_foot));
    // 求直线单位向量
    x_unit = (path_front_x - path_rear_x) / sqrt((path_front_x - path_rear_x) * (path_front_x - path_rear_x) + (path_front_y - path_rear_y) * (path_front_y - path_rear_y));
    y_unit = (path_front_y - path_rear_y) / sqrt((path_front_x - path_rear_x) * (path_front_x - path_rear_x) + (path_front_y - path_rear_y) * (path_front_y - path_rear_y));
    // ROS_INFO("x_unit = [%f],y_unit = [%f]", x_unit, y_unit);
    //  求预瞄距离dth1
    dth1 = dth - k3 * dis;
    //  pure_pursuit
    // if (use_pure_pursuit)
    // {
    //     dth1 = dth;
    // }
    // ROS_INFO("dth1 = [%f]", dth1);
    //  沿单位向量预瞄距离dth1，得到预瞄点
    dth_point_x = x_foot + dth1 * x_unit;
    dth_point_y = y_foot + dth1 * y_unit;
    // 若预瞄点位置超过front点，则选取front点作为预瞄点
    if (dis_front2foot < dth1)
    {
        dth_point_x = path_front_x;
        dth_point_y = path_front_y;
    }
    //  ###纯追踪算法计算部分###  //
    L = sqrt(dth_point_y * dth_point_y + dth1 * dth1);
    alpha = atan2(dth_point_y, dth1);
    // ROS_INFO("L = [%f],alpha = [%f]",L,alpha);
    //  转向半径
    R = L / (2 * sin(alpha));
    // ROS_INFO("R = [%f]",R);
    //  判断方向：如果直线单位向量与车体x轴夹角为0-pi，则为左转，-pi到0为右转
    //  if(y_unit < 0)
    //  {
    //      R = -R;
    //  }
    //   ###判断是否需要更新index### //
    // ###判断是否达到最终目标点### //
    // 根据index判断
    if (path_index == final_index)
    {
        ROS_INFO("pursuit finished!");
        if (pursuit_finished == false)
        {
            time_duration = ros::Time::now().toSec() - time_duration;
            pursuit_finished = true;
        }
        ROS_DEBUG("Time Duration = [%f]", time_duration);
        set_speed = 0;
        set_steering = 0;
        return;
    }
    // 根据距离判断
    double goal_distance = sqrt(path_converted.poses[path_converted.poses.size() - 1].pose.position.x * path_converted.poses[path_converted.poses.size() - 1].pose.position.x + path_converted.poses[path_converted.poses.size() - 1].pose.position.y * path_converted.poses[path_converted.poses.size() - 1].pose.position.y);
    if (goal_distance < safety_corridor)
    {
        ROS_INFO("pursuit finished!");
        if (pursuit_finished == false)
        {
            time_duration = ros::Time::now().toSec() - time_duration;
            pursuit_finished = true;
        }
        ROS_DEBUG("Time Duration = [%f]", time_duration);
        set_speed = 0;
        set_steering = 0;
        return;
    }
    //  如果已经到达目标点附近，则index+1
    if (sqrt(path_front_x * path_front_x + path_front_y * path_front_y) < safety_corridor)
    {
        path_index += 1;
        ROS_DEBUG("pursuit index = [%d]", path_index);
        // set_speed = 0;
        // return;
    }

    //  ###若距离大于安全距离，则进行自转以确保安全，否则进行前进### //
    if (!(dis > safety_corridor))
    {
        // 速度：单位m/s
        set_speed = default_speed;
        // 角速度：单位omega
        set_steering = default_speed / R;
        return;
    }

    else
    {
        ROS_DEBUG("[Pure Pursuit]Dis = [%f],Rover position unsafe.", dis);
        // 策略：使车快速回到目标path上-使垂足在车体坐标系下接近(0,0)
        // 先自转使垂足y坐标为0
        if (y_foot > 0.5) // 垂足在二三象限
        {
            // 逆时针自转
            set_steering = std::min(max_yaw_rate_, std::fabs(rotate_in_place_yaw_rate_));
            set_speed = 0;
            ROS_DEBUG("[Pure Pursuit]Rotate Left");
            return;
        }
        else if (y_foot < -0.5) // 垂足在一四象限
        {
            // 顺时针自转
            set_steering = -std::min(max_yaw_rate_, std::fabs(rotate_in_place_yaw_rate_));
            set_speed = 0;
            ROS_DEBUG("[Pure Pursuit]Rotate Right");
            return;
        }
        else if (x_foot > 0.5) // 垂足在x轴正半轴
        {
            // 前进使x坐标接近0
            set_steering = 0;
            set_speed = 0.5;
            ROS_DEBUG("[Pure Pursuit]Forward");
            return;
        }
        else if (x_foot < -0.5) // 垂足在x轴上
        {
            // 后退使x坐标接近0
            set_steering = 0;
            set_speed = -0.5;
            ROS_DEBUG("[Pure Pursuit]Backward");
            return;
        }
        else
        {
            set_steering = 0;
            set_speed = 0;
            ROS_DEBUG("[Pure Pursuit]Wait for replan.");
            return;
        }
    }
}

//  @@@
//  函数：发布控制指令
//  功能：将速度和角速度发布到话题
//  参数：速度和角速度
void C_Pursuit::PublishCmd(double &set_speed, double &set_steering)
{
    cmd_vel.linear.x = set_speed;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = std::max(-max_yaw_rate_, std::min(max_yaw_rate_, set_steering));
    cmd_pub.publish(cmd_vel);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "c_pursuit");
    ros::NodeHandle nh;
    C_Pursuit c_pursuit(nh);
    ros::AsyncSpinner spinner(5);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
