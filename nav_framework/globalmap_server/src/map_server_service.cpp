#include "map_server.hpp"
#include <cmath>
#include <cstdint>

//  @@@ SERVICE
//  Service: 初始化地图
bool MapServer::Initialize_Map(globalmap_server::initialize_map::Request &req, globalmap_server::initialize_map::Response &res)
{
    ROS_WARN("Initialize_Map service called");
    if (global_map_.data.size() == 0)
    {
        if (std::fabs(req.global_map.info.resolution - globalmap_resolution_) > 1e-5)
        {
            ROS_ERROR("Resolution mismatch, global map resolution: %f, expected resolution: %f", req.global_map.info.resolution, globalmap_resolution_);
            res.initialized = false;
            return true;
        }
        // 存储全局地图
        if (globalmap_resolution_ != localmap_resolution_)
        {
            // 如果分辨率不一致，需要对地图进行相应放大
            ROS_DEBUG("Size mismatch,Zooming global map");
            // 计算放大比例
            const double scale = globalmap_resolution_ / localmap_resolution_;
            // 初始化全局地图
            global_map_.header = req.global_map.header;
            global_map_.info.width = std::max(1, static_cast<int>(std::round(req.global_map.info.width * scale)));
            global_map_.info.height = std::max(1, static_cast<int>(std::round(req.global_map.info.height * scale)));
            global_map_.info.resolution = localmap_resolution_;
            global_map_.info.origin = req.global_map.info.origin;
            global_map_.data.resize(global_map_.info.height * global_map_.info.width);
            // 进行地图放大：双线性插值，避免最近邻复制产生条纹块状伪影
            const int src_w = req.global_map.info.width;
            const int src_h = req.global_map.info.height;
            auto src_value = [&](int x, int y) -> int
            {
                const int cx = std::max(0, std::min(src_w - 1, x));
                const int cy = std::max(0, std::min(src_h - 1, y));
                return req.global_map.data[cy * src_w + cx];
            };

            for (int y = 0; y < static_cast<int>(global_map_.info.height); ++y)
            {
                const double src_y = ((y + 0.5) / scale) - 0.5;
                const int y0 = static_cast<int>(std::floor(src_y));
                const int y1 = y0 + 1;
                const double wy = src_y - y0;

                for (int x = 0; x < static_cast<int>(global_map_.info.width); ++x)
                {
                    const double src_x = ((x + 0.5) / scale) - 0.5;
                    const int x0 = static_cast<int>(std::floor(src_x));
                    const int x1 = x0 + 1;
                    const double wx = src_x - x0;

                    const int v00 = src_value(x0, y0);
                    const int v10 = src_value(x1, y0);
                    const int v01 = src_value(x0, y1);
                    const int v11 = src_value(x1, y1);

                    // Unknown cells (-1) are ignored in interpolation if neighbors are available.
                    double accum = 0.0;
                    double weight = 0.0;
                    auto add_sample = [&](int v, double w)
                    {
                        if (v >= 0)
                        {
                            accum += static_cast<double>(v) * w;
                            weight += w;
                        }
                    };

                    add_sample(v00, (1.0 - wx) * (1.0 - wy));
                    add_sample(v10, wx * (1.0 - wy));
                    add_sample(v01, (1.0 - wx) * wy);
                    add_sample(v11, wx * wy);

                    const int dst_idx = y * global_map_.info.width + x;
                    if (weight <= 1e-6)
                    {
                        global_map_.data[dst_idx] = -1;
                    }
                    else
                    {
                        global_map_.data[dst_idx] = static_cast<int8_t>(std::round(accum / weight));
                    }
                }
            }
            // 存储原始地图
            global_map_raw_ = req.global_map;
        }
        else
        {
            // 直接存储
            ROS_DEBUG("Globalmap size match, no need to zoom");
            global_map_ = req.global_map;
            global_map_raw_ = req.global_map;
        }
        ROS_DEBUG("map grid: %d x %d,resolution = %f", global_map_.info.height, global_map_.info.width, global_map_.info.resolution);
        // 将模式地图初始化为模式一
        mode_map_.header = global_map_.header;
        mode_map_.info = global_map_.info;
        mode_map_.data.resize(global_map_.info.height * global_map_.info.width);
        mode_map_.data.assign(global_map_.info.height * global_map_.info.width, 1);
        res.initialized = true;
        // 发布全局地图和模式地图
        if (visulize_global_map_)
        {
            global_map_pub_.publish(global_map_);
            mode_map_pub_.publish(mode_map_);
        }
        ROS_DEBUG(" [ OK ] Map initialized");
        ROS_INFO(" [ INFO ] Map size: %d x %d ,Map frame: %s ", global_map_.info.width, global_map_.info.height, global_map_.header.frame_id.c_str());
        return true;
    }
    else
    {
        ROS_WARN("Map already initialized");
        res.initialized = false;
        return true;
    }
    if (visulize_global_map_)
    {
        global_map_pub_.publish(global_map_);
        mode_map_pub_.publish(mode_map_);
    }
}

//  @@@ SERVICE
//  Service: 更新地图，将局部地图更新到全局
//  策略：对每个格子判断模式，高模式的地图覆盖低模式的地图
bool MapServer::Update_Map(globalmap_server::update_map::Request &req, globalmap_server::update_map::Response &res)
{
    // ROS_DEBUG("Update_Map service called");
    nav_msgs::OccupancyGrid update_map = req.update_map;

    // 检查目前是否已经初始化全局地图
    if (global_map_.data.size() == 0)
    {
        ROS_ERROR("Received Update Map,BUT Globalmap not Initialized yet");
        res.updated = false;
        return true;
    }
    // 检查地图分辨率是否一致
    if (std::fabs(update_map.info.resolution - localmap_resolution_) > 1e-5)
    {
        ROS_ERROR("Resolution mismatch, update map resolution: %f, global map resolution: %f", update_map.info.resolution, localmap_resolution_);
        res.updated = false;
        return true;
    }
    // 检查frameid是否一致
    if (update_map.header.frame_id != global_map_.header.frame_id)
    {
        ROS_ERROR("Frame id mismatch, update map frame id: %s, global map frame id: %s", update_map.header.frame_id.c_str(), global_map_.header.frame_id.c_str());
        res.updated = false;
        return true;
    }

    // 代码块：将局部地图更新到全局地图
    {
        // 根据全局和局部地图的origin，计算局部地图的在全局地图中的相对位置，并进行存储
        // 计算局部地图在全局地图中的相对位置
        int dx = round((update_map.info.origin.position.x - global_map_.info.origin.position.x) / global_map_.info.resolution);
        int dy = round((update_map.info.origin.position.y - global_map_.info.origin.position.y) / global_map_.info.resolution);
        // 遍历局部地图的每一个单元格
        for (int i = 0; i < update_map.info.width; i++)
        {
            for (int j = 0; j < update_map.info.height; j++)
            {

                // 计算单元格在全局地图中的对应位置
                int x = i + dx;
                int y = j + dy;
                // 检查对应位置是否在全局地图的范围内
                if (x >= 0 && x < global_map_.info.width && y >= 0 && y < global_map_.info.height)
                {
                    if (update_map.data[j * update_map.info.width + i] == -1)
                    {
                        // -1代表未知，因此不进行更新
                        continue;
                    }
                    // 更新局部地图策略
                    else
                    {
                        // 如果原格子地图来源为模式一
                        if (mode_map_.data[y * global_map_.info.width + x] == 1)
                        {
                            // 策略：直接覆盖
                            global_map_.data[y * global_map_.info.width + x] = update_map.data[j * update_map.info.width + i];
                        }
                        // 如果原格子地图来源为模式2
                        else if (mode_map_.data[y * global_map_.info.width + x] == 2)
                        {
                            // 1-模式二更新模式二地图
                            if (req.mode == 2)
                            {
                                // 相同模式更新
                                // 模式二地图包含-1，0，100三种灰度
                                // 如果为-1则保持不变，如果是0或100则都更新
                                if (update_map.data[j * update_map.info.width + i] == 0 || update_map.data[j * update_map.info.width + i] == 100)
                                {
                                    global_map_.data[y * global_map_.info.width + x] = update_map.data[j * update_map.info.width + i];
                                }
                            }
                            // 1-模式三更新模式二地图
                            if (req.mode == 3)
                            {
                                // 直接覆盖
                                global_map_.data[y * global_map_.info.width + x] = update_map.data[j * update_map.info.width + i];
                            }
                            
                        }
                        else if (mode_map_.data[y * global_map_.info.width + x] == 3)
                        {

                            // 2-模式三更新模式三地图
                            if (req.mode == 3)
                            {
                                // 模式三地图灰度从-1到100
                                // 如果为-1则保持不变，如果是0到100则都更新
                                if (update_map.data[j * update_map.info.width + i] >= 0)
                                {
                                    global_map_.data[y * global_map_.info.width + x] = update_map.data[j * update_map.info.width + i];
                                }
                            }
                            // 2-模式二更新模式三地图：不更新
                        }
                    }
                    // 将模式地图的对应位置更新:取已被更新模式的最大值
                    if (mode_map_.data[y * global_map_.info.width + x] < req.mode)
                    {
                        mode_map_.data[y * global_map_.info.width + x] = req.mode;
                    }
                }
            }
        }
    }

    // 代码块：发布全局地图和模式地图
    if (visulize_global_map_)
    {
        global_map_pub_.publish(global_map_);
        mode_map_pub_.publish(mode_map_);
    }
    res.updated = true;
    return true;
}

//  @@@ SERVICE
//  Service: 获取全局地图
//  策略：接收指令后，发布低精度全局地图，全局起点，全局终点
bool MapServer::Get_Globalmap(globalmap_server::get_globalmap::Request &req, globalmap_server::get_globalmap::Response &res)
{
    ROS_DEBUG("Get_Globalmap service called");
    res.success = true;
    if (req.global == false)
    {
        ROS_WARN("Received Get Globalmap Service,BUT service is set False");
        res.success = false;
        return true;
    }
    // 检查是否已经初始化全局地图
    if (global_map_raw_.data.size() == 0)
    {
        ROS_WARN("Received Get Globalmap Service,BUT Globalmap not Initialized");
        res.success = false;
        // res.map_ok = false;
    }
    // 检查全局起点与终点是否初始化
    if (pose_initialized_ == false)
    {
        ROS_WARN("Received Get Globalmap Service,BUT Pose not initialized");
        res.success = false;
        // res.pose_ok = false;
    }
    if (goal_initialized_ == false)
    {
        ROS_WARN("Received Get Globalmap Service,BUT Goal not initialized");
        res.success = false;
        // res.goal_ok = false;
    }
    if (res.success == true)
    {
        // 发布全局地图和目标点
        planner_global_map_pub_.publish(global_map_raw_);
        geometry_msgs::PoseStamped global_goal;
        global_goal.header.stamp = ros::Time::now();
        global_goal.header.frame_id = global_map_.header.frame_id;
        global_goal.pose.position.x = globalgoal_x_;
        global_goal.pose.position.y = globalgoal_y_;
        global_goal.pose.orientation.w = 1.0;
        planner_global_goal_pub_.publish(global_goal);
        ROS_DEBUG(" [ OK ] Globalmap Published");
        // 不知道为什么规划器接收不到，就再发一遍吧
        planner_global_map_pub_.publish(global_map_raw_);
        planner_global_goal_pub_.publish(global_goal);
    }
    return true;
}

//  @@@ SERVICE
//  Service: 接收全局路径
//  策略：接收全局路径并存储
bool MapServer::Initialize_Path(globalmap_server::initialize_path::Request &req, globalmap_server::initialize_path::Response &res)
{
    ROS_INFO("Initialize_Path service called");
    if (global_path_.poses.size() != 0)
    {
        ROS_WARN("Path already initialized");
        res.success = false;
        return true;
    }
    if (req.global == false)
    {
        ROS_WARN("Received Initialize Path Service,BUT service is set False");
        res.success = false;
        return true;
    }
    // 将路径储存到全局路径中
    global_path_ = req.global_path;
    ROS_DEBUG(" [ OK ] Path initialized");
    res.success = true;
    return true;
}

// @@@ SERVICE
// Service: 获取局部地图
// 策略：接收指令后，发布局部地图，局部起点，局部终点
bool MapServer::Get_Localmap(globalmap_server::get_globalmap::Request &req, globalmap_server::get_globalmap::Response &res)
{
    if (verbose_logging_)
        ROS_INFO("Get_Localmap service called");
    res.success = true;
    if (req.global == true)
    {
        ROS_WARN("Received Get Localmap Service,BUT global is set True");
        res.success = false;
        return true;
    }
    // 检查是否已经初始化全局地图
    if (global_map_raw_.data.size() == 0)
    {
        ROS_WARN("Received Get Localmap Service,BUT Globalmap not Initialized");
        res.success = false;
        // res.map_ok = false;
    }
    // 检查全局起点与终点是否初始化
    if (pose_initialized_ == false)
    {
        ROS_WARN("Received Get Localmap Service,BUT Pose not initialized");
        res.success = false;
        // res.pose_ok = false;
    }
    if (goal_initialized_ == false)
    {
        ROS_WARN("Received Get Localmap Service,BUT Goal not initialized");
        res.success = false;
        // res.goal_ok = false;
    }
    // 检查全局路径是否已经初始化
    if (global_path_.poses.size() == 0)
    {
        ROS_WARN("Received Get Localmap Service,BUT Global Path not initialized");
        res.success = false;
        // res.path_ok = false;
    }
    if (res.success == true)
    {
        // 发布局部地图和目标点
        // 初始化局部地图
        nav_msgs::OccupancyGrid local_map;
        local_map.header.stamp = ros::Time::now();
        local_map.header.frame_id = global_map_.header.frame_id;
        // 根据范围取出局部地图：以车辆为中心，取local_map_size_范围内的地图
        // 计算局部地图的范围
        int local_map_size = round(local_map_size_ / localmap_resolution_);
        int local_map_width = local_map_size * 2;
        int local_map_height = local_map_size * 2;

        // 计算局部地图的origin
        int local_map_origin_x = round(start_x_ / localmap_resolution_) - local_map_size;
        int local_map_origin_y = round(start_y_ / localmap_resolution_) - local_map_size;
        // 初始化局部地图
        local_map.info.width = local_map_width;
        local_map.info.height = local_map_height;
        local_map.info.resolution = localmap_resolution_;
        local_map.info.origin.position.x = local_map_origin_x * localmap_resolution_;
        local_map.info.origin.position.y = local_map_origin_y * localmap_resolution_;
        local_map.info.origin.orientation.w = 1.0;
        local_map.data.resize(local_map_width * local_map_height);
        local_map.data.assign(local_map_width * local_map_height, -1);

        // 根据全局和局部地图的origin，计算局部地图的在全局地图中的相对位置，并进行存储
        // 计算局部地图在全局地图中的相对位置
        int dx = round((local_map.info.origin.position.x - global_map_.info.origin.position.x) / global_map_.info.resolution);
        int dy = round((local_map.info.origin.position.y - global_map_.info.origin.position.y) / global_map_.info.resolution);

        // 遍历局部地图的每一个单元格
        for (int i = 0; i < local_map.info.width; i++)
        {
            for (int j = 0; j < local_map.info.height; j++)
            {
                // 计算单元格在全局地图中的对应位置
                int x = i + dx;
                int y = j + dy;
                // 检查对应位置是否在全局地图的范围内
                if (x >= 0 && x < global_map_.info.width && y >= 0 && y < global_map_.info.height)
                {
                    // 将全局地图中的局部地图部分复制到局部地图中
                    local_map.data[j * local_map_width + i] = global_map_.data[y * global_map_.info.width + x];
                }
                else
                {
                    // ROS_INFO("Out of range");
                }
            }
        }
        // 发布局部地图
        planner_local_map_pub_.publish(local_map);

        local_goal_ = Get_LocalGoal();
        // 发布局部目标点
        planner_local_goal_pub_.publish(local_goal_);
        if (verbose_logging_)
            ROS_DEBUG(" [ OK ] Localmap and Goal Published");
        // // 不知道为什么规划器接收不到，就再发一遍吧
        // planner_local_map_pub.publish(global_map_raw_);
        // planner_local_goal_pub_.publish(local_goal);
    }
    return true;
}

// @@@ FUNCTION
// Function: 更新全局路径
// 策略：前进式跟踪 — 只从上次匹配的索引开始向前搜索，防止回退到已通过的路径点
void MapServer::Update_GlobalPath()
{
    if (global_path_.poses.empty()) return;

    // 只允许沿路径向前搜索，防止索引回退到已通过路径点。
    int search_start = std::max(last_tracked_index_, 0);
    double min_distance = 1e9;
    int min_index = search_start;

    for (int i = search_start; i < (int)global_path_.poses.size(); i++)
    {
        double distance = sqrt(pow(global_path_.poses[i].pose.position.x - start_x_, 2) +
                               pow(global_path_.poses[i].pose.position.y - start_y_, 2));
        if (distance < min_distance)
        {
            min_distance = distance;
            min_index = i;
        }
    }

    // 强制索引单调前进，避免局部抖动导致回退。
    last_tracked_index_ = std::max(last_tracked_index_, min_index);

    // 更新全局路径
    global_path_updated.header.stamp = ros::Time::now();
    global_path_updated.header.frame_id = global_path_.header.frame_id;
    global_path_updated.poses.clear();
    // 将车辆当前位置作为全局路径第一个点
    geometry_msgs::PoseStamped start_pose;
    start_pose.header.stamp = ros::Time::now();
    start_pose.header.frame_id = global_path_.header.frame_id;
    start_pose.pose.position.x = start_x_;
    start_pose.pose.position.y = start_y_;
    start_pose.pose.orientation.w = 1.0;
    global_path_updated.poses.push_back(start_pose);
    // 将之后的点添加到更新后的全局路径中
    if (min_index + 1 >= (int)global_path_.poses.size())
    {
        global_path_updated.poses.push_back(global_path_.poses[min_index]);
    }
    else
    {
        for (int i = min_index + 1; i < (int)global_path_.poses.size(); i++)
        {
            global_path_updated.poses.push_back(global_path_.poses[i]);
        }
    }
    // 发布更新后的全局路径
    global_path_pub_.publish(global_path_updated);
}

// @@@ FUNCTION
// Function: 在路径中找到局部目标点
// 策略：在更新后的全局路径上，找到一个满足以下条件的前瞻目标点：
//  1. 距离车辆至少 local_goal_lookahead_ 米
//  2. 不在障碍物中
//  3. 在局部地图范围内
geometry_msgs::PoseStamped MapServer::Get_LocalGoal()
{
    geometry_msgs::PoseStamped local_goal;
    local_goal.header.stamp = ros::Time::now();
    local_goal.header.frame_id = global_path_updated.header.frame_id;

    if (global_path_updated.poses.size() <= 1)
    {
        // 路径只有一个点（车辆自身），直接返回
        local_goal.pose.position = global_path_updated.poses[0].pose.position;
        local_goal.pose.orientation.w = 1.0;
        return local_goal;
    }

    // 在全局路径上从前往后搜索，找到满足前瞻距离且不在障碍中的目标点
    int selected_index = -1;
    double half_local_map = local_map_size_; // 局部地图半径(m)

    const double min_goal_distance = std::max(0.8, local_goal_lookahead_ * 0.3);
    for (int i = 1; i < (int)global_path_updated.poses.size(); i++)
    {
        double px = global_path_updated.poses[i].pose.position.x;
        double py = global_path_updated.poses[i].pose.position.y;
        double dist = sqrt(pow(px - start_x_, 2) + pow(py - start_y_, 2));

        // 条件1：距离车辆至少 local_goal_lookahead_ 米
        if (dist < local_goal_lookahead_)
            continue;

        // 条件2：在局部地图范围内
        if (fabs(px - start_x_) > half_local_map || fabs(py - start_y_) > half_local_map)
            break; // 超出局部地图范围，取之前找到的点

        // 条件3：不在障碍物中
        if (IsPointInObstacle(px, py, obstacle_threshold_))
            continue; // 跳过障碍中的路径点

        selected_index = i;
        break;
    }

    // 如果没有找到满足所有条件的点，退化为选择路径上第一个不在障碍中的点
    if (selected_index < 0)
    {
        for (int i = 1; i < (int)global_path_updated.poses.size(); i++)
        {
            double px = global_path_updated.poses[i].pose.position.x;
            double py = global_path_updated.poses[i].pose.position.y;
            double dist = sqrt(pow(px - start_x_, 2) + pow(py - start_y_, 2));
            if (dist >= min_goal_distance && !IsPointInObstacle(px, py, obstacle_threshold_))
            {
                selected_index = i;
                break;
            }
        }
    }

    // 如果所有路径点均在障碍中，使用最后一个点
    if (selected_index < 0)
    {
        selected_index = (int)global_path_updated.poses.size() - 1;
        ROS_WARN("All global path waypoints are in obstacles, using last point as local goal");
    }

    local_goal.pose.position = global_path_updated.poses[selected_index].pose.position;

    // 计算orientation
    if (selected_index + 1 < (int)global_path_updated.poses.size())
    {
        double dx = global_path_updated.poses[selected_index + 1].pose.position.x -
                     global_path_updated.poses[selected_index].pose.position.x;
        double dy = global_path_updated.poses[selected_index + 1].pose.position.y -
                     global_path_updated.poses[selected_index].pose.position.y;
        local_goal.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(dy, dx));
    }
    else if (selected_index > 0)
    {
        double dx = global_path_updated.poses[selected_index].pose.position.x -
                     global_path_updated.poses[selected_index - 1].pose.position.x;
        double dy = global_path_updated.poses[selected_index].pose.position.y -
                     global_path_updated.poses[selected_index - 1].pose.position.y;
        local_goal.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(dy, dx));
    }
    else
    {
        local_goal.pose.orientation.w = 1.0;
    }

    return local_goal;
}

// @@@ FUNCTION
// Function: 检查世界坐标点是否在障碍物中
bool MapServer::IsPointInObstacle(double world_x, double world_y, int threshold)
{
    if (global_map_.data.empty()) return false;

    int gx = round((world_x - global_map_.info.origin.position.x) / global_map_.info.resolution);
    int gy = round((world_y - global_map_.info.origin.position.y) / global_map_.info.resolution);

    if (gx < 0 || gx >= (int)global_map_.info.width || gy < 0 || gy >= (int)global_map_.info.height)
        return true; // 地图外视为障碍

    int val = global_map_.data[gy * global_map_.info.width + gx];
    return val > threshold || val < 0;
}

//  @@@ CALLBACK
//  Callback: 订阅起点
void MapServer::PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    // ROS_INFO("Pose Callback");
    start_x_ = msg->pose.pose.position.x;
    start_y_ = msg->pose.pose.position.y;
    pose_initialized_ = true;
    // ROS_INFO("Start point: %f, %f", start_x_, start_y_);
    // 检查全局路径是否已经被初始化，若已初始化则更新全局路径
    if (global_path_.poses.size() != 0)
    {
        // 查看间隔时间是否大于设定的时间间隔，如是就更新
        if ((ros::Time::now() - last_global_path_update_time_).toSec() > global_path_update_interval_)
        {
            last_global_path_update_time_ = ros::Time::now();
            Update_GlobalPath();
        }
    }
}

//  @@@ CALLBACK
//  Callback: 订阅终点
void MapServer::GoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // ROS_INFO("Goal Callback");
    globalgoal_x_ = msg->pose.position.x;
    globalgoal_y_ = msg->pose.position.y;
    goal_initialized_ = true;
    ROS_INFO("Global goal: %f, %f", globalgoal_x_, globalgoal_y_);
    // 接收到新的goal后，需要清空全局路径
    if (global_path_.poses.size() != 0)
    {
        ROS_WARN("[NOTICE] New Goal received, global path cleared");
        global_path_.poses.clear();
        global_path_updated.poses.clear();
        last_tracked_index_ = 0;
    }
}

//  @@@ CALLBACK
//  Callback: 订阅模式
void MapServer::ModeCallback(const std_msgs::String::ConstPtr &msg)
{
    if (msg->data == "Mode 1")
    {
        ROS_DEBUG("Mode 1");
        mode_ = "Mode 1";
    }
    else if (msg->data == "Mode 2")
    {
        ROS_DEBUG("Mode 2");
        mode_ = "Mode 2";
    }
    else if (msg->data == "Mode 3")
    {
        ROS_DEBUG("Mode 3");
        mode_ = "Mode 3";
    }
    else
    {
        ROS_ERROR("Mode Error");
        mode_ = "Mode Error";
    }
}

//  Callback: 订阅全局路径
void MapServer::GlobalPathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    // ROS_INFO("Global Path Callback");
    // 只有在模式为1时才接收全局路径
    if (mode_ == "Mode 1")
    {
        global_path_updated_ = *msg;
    }
}

//  @@@ CALLBACK
//  Callback: 订阅局部路径
void MapServer::LocalPathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    // ROS_INFO("Local Path Callback");
    // 只有在模式为2或3时才接收局部路径
    if (mode_ == "Mode 2" || mode_ == "Mode 3")
    {
        local_path_updated_ = *msg;
    }
}

/// @@@ FUNCTION
//  Function: 碰撞检测
//  策略：检查全局路径和局部路径与地图是否有碰撞，如果有碰撞就发布重规划信号
bool MapServer::CollisionCheck(std::string mode, double inflation_radius_grid, int obstacle_threshold_)
{
    const int inflation_radius = std::max(0, (int)round(inflation_radius_grid));

    // 点级碰撞检查：包含障碍阈值、未知区域和膨胀半径。
    auto point_collides = [&](double world_x, double world_y) -> bool
    {
        int x_center = round((world_x - global_map_.info.origin.position.x) / global_map_.info.resolution);
        int y_center = round((world_y - global_map_.info.origin.position.y) / global_map_.info.resolution);

        for (int dx = -inflation_radius; dx <= inflation_radius; dx++)
        {
            for (int dy = -inflation_radius; dy <= inflation_radius; dy++)
            {
                if (dx * dx + dy * dy > inflation_radius * inflation_radius)
                    continue;

                int x = x_center + dx;
                int y = y_center + dy;
                if (x < 0 || x >= (int)global_map_.info.width || y < 0 || y >= (int)global_map_.info.height)
                    continue;

                int val = global_map_.data[y * global_map_.info.width + x];
                if (val > obstacle_threshold_ || val < 0)
                    return true;
            }
        }
        return false;
    };

    // 线段碰撞检查：检查路径点之间是否穿越障碍。
    auto segment_collides = [&](const geometry_msgs::PoseStamped &p0,
                                const geometry_msgs::PoseStamped &p1) -> bool
    {
        const double x0 = p0.pose.position.x;
        const double y0 = p0.pose.position.y;
        const double x1 = p1.pose.position.x;
        const double y1 = p1.pose.position.y;
        const double seg_len = hypot(x1 - x0, y1 - y0);
        const double step = std::max(0.05, global_map_.info.resolution * 0.5);
        const int samples = std::max(1, (int)ceil(seg_len / step));

        for (int s = 0; s <= samples; s++)
        {
            const double t = (double)s / (double)samples;
            const double x = x0 + (x1 - x0) * t;
            const double y = y0 + (y1 - y0) * t;
            if (point_collides(x, y))
                return true;
        }
        return false;
    };

    if (mode == "Mode 1")
    {
        // 检查全局路径是否与地图有碰撞
        if (global_path_updated_.poses.size() != 0)
        {
            for (int i = 0; i < (int)global_path_updated_.poses.size(); i++)
            {
                if (point_collides(global_path_updated_.poses[i].pose.position.x,
                                   global_path_updated_.poses[i].pose.position.y))
                {
                    replan_signal.data = "True";
                    replan_signal_pub_.publish(replan_signal);
                    ROS_WARN("Collision detected in global path waypoint, Replan");
                    return true;
                }

                if (i + 1 < (int)global_path_updated_.poses.size() &&
                    segment_collides(global_path_updated_.poses[i], global_path_updated_.poses[i + 1]))
                {
                    replan_signal.data = "True";
                    replan_signal_pub_.publish(replan_signal);
                    ROS_WARN("Collision detected in global path segment, Replan");
                    return true;
                }
            }
            // 如果无碰撞，就发布False
            replan_signal.data = "False";
            replan_signal_pub_.publish(replan_signal);
            // ROS_INFO("No Collision");
        }
        else
        {
            ROS_WARN("Global path is empty");
        }
    }
    else if (mode == "Mode 2" || mode == "Mode 3")
    {
        // 检查局部路径是否与地图有碰撞
        if (local_path_updated_.poses.size() != 0)
        {
            for (int i = 0; i < (int)local_path_updated_.poses.size(); i++)
            {
                if (point_collides(local_path_updated_.poses[i].pose.position.x,
                                   local_path_updated_.poses[i].pose.position.y))
                {
                    replan_signal.data = "True";
                    replan_signal_pub_.publish(replan_signal);
                    ROS_WARN("Collision detected in local path waypoint, Replan");
                    return true;
                }

                if (i + 1 < (int)local_path_updated_.poses.size() &&
                    segment_collides(local_path_updated_.poses[i], local_path_updated_.poses[i + 1]))
                {
                    replan_signal.data = "True";
                    replan_signal_pub_.publish(replan_signal);
                    ROS_WARN("Collision detected in local path segment, Replan");
                    return true;
                }
            }
            // 如果无碰撞，就发布False
            replan_signal.data = "False";
            replan_signal_pub_.publish(replan_signal);
            // ROS_INFO("No Collision");
        }
    }
    return false;
}

// ///  @@@ FUNCTION
// //  Function: 碰撞检测
// //  策略：检查全局路径和局部路径与地图是否有碰撞，如果有碰撞就发布重规划信号
// bool MapServer::CollisionCheck(std::string mode, double inflation_radius_grid, int obstacle_threshold_)
// {
//     if (mode == "Mode 1")
//     {
//         // 检查全局路径是否与地图有碰撞
//         if (global_path_updated_.poses.size() != 0)
//         {
//             for (int i = 0; i < global_path_updated_.poses.size(); i++)
//             {
//                 int x = round((global_path_updated_.poses[i].pose.position.x - global_map_.info.origin.position.x) / global_map_.info.resolution);
//                 int y = round((global_path_updated_.poses[i].pose.position.y - global_map_.info.origin.position.y) / global_map_.info.resolution);
//                 if (global_map_.data[y * global_map_.info.width + x] > obstacle_threshold_)
//                 {
//                     // 发布重规划信号
//                     replan_signal.data = "True";
//                     replan_signal_pub_.publish(replan_signal);
//                     ROS_WARN("Collision detected in global path, Replan");
//                     return true;
//                 }
//             }
//             // 如果无碰撞，就发布False
//             replan_signal.data = "False";
//             replan_signal_pub_.publish(replan_signal);
//             // ROS_INFO("No Collision");
//         }
//         else
//         {
//             ROS_WARN("Global path is empty");
//         }
//     }
//     else if (mode == "Mode 2" || mode == "Mode 3")
//     {
//         // 检查局部路径是否与地图有碰撞
//         if (local_path_updated_.poses.size() != 0)
//         {
//             for (int i = 0; i < local_path_updated_.poses.size(); i++)
//             {
//                 int x = round((local_path_updated_.poses[i].pose.position.x - global_map_.info.origin.position.x) / global_map_.info.resolution);
//                 int y = round((local_path_updated_.poses[i].pose.position.y - global_map_.info.origin.position.y) / global_map_.info.resolution);
//                 if (global_map_.data[y * global_map_.info.width + x] > obstacle_threshold_)
//                 {
//                     // 发布重规划信号
//                     replan_signal.data = "True";
//                     replan_signal_pub_.publish(replan_signal);
//                     ROS_WARN("Collision detected in local path, Replan");
//                     return true;
//                 }
//             }
//             // 如果无碰撞，就发布False
//             replan_signal.data = "False";
//             replan_signal_pub_.publish(replan_signal);
//             // ROS_INFO("No Collision");
//         }
//     }
//     return false;
// }

void MapServer::CollisionDetectionLoop(ros::Publisher &replan_signal_pub_)
{
    ros::Time last_replan_time = ros::Time::now();
    double replan_cooldown = 3.5; // 重规划冷却时间(秒)，防止频繁重规划
    int consecutive_collision_count = 0;

    while (ros::ok())
    {
        // 检查模式是否被初始化
        if (mode_ != "Mode 1" && mode_ != "Mode 2" && mode_ != "Mode 3")
        {
            ros::Duration(1.0).sleep();
            continue;
        }

        // 冷却期内不检测
        if ((ros::Time::now() - last_replan_time).toSec() < replan_cooldown)
        {
            ros::Duration(1.0).sleep();
            continue;
        }

        // 根据膨胀半径和地图分辨率计算需要膨胀的栅格数
        double inflation_radius_grid = round(inflation_radius_ / global_map_.info.resolution);
        // 障碍检测
        bool collision = CollisionCheck(mode_, inflation_radius_grid, obstacle_threshold_);
        std_msgs::String replan_signal;
        if (collision)
        {
            ROS_WARN("Collision detected, Replan");
            replan_signal.data = "True";
            last_replan_time = ros::Time::now();
            consecutive_collision_count++;
            // 增加冷却时间：每连续碰撞增加0.5s，上限设为7.5s
            replan_cooldown = std::min(7.5, 3.5 + 0.5 * consecutive_collision_count);
        }
        else
        {
            replan_signal.data = "False";
            consecutive_collision_count = 0;
            replan_cooldown = 3.5;
        }

        // 发布重规划信号
        replan_signal_pub_.publish(replan_signal);
        ros::Duration(1.0).sleep();

        // 让ROS能够处理其他回调函数
        ros::spinOnce();
    }
}