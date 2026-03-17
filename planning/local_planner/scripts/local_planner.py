import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from astar_cost import AStar
import matplotlib.pyplot as plt
from nav_msgs.msg import Path
from std_msgs.msg import String

map_ok = False
start_ok = False
goal_ok = False
map_msg = None
last_replan_time = None
# start_pt_x, start_pt_y = None
# goal_pt_x, goal_pt_y = None
start_pose = PoseWithCovarianceStamped()
goal_pose = PoseStamped()

def manual_test_mode():

    start_pose = PoseWithCovarianceStamped()
    goal_pose = PoseStamped()
    start_pose.pose.pose.position.x = -30
    start_pose.pose.pose.position.y = -40
    goal_pose.pose.position.x = 10
    goal_pose.pose.position.y = -8

    return start_pose, goal_pose


def visualize_path(path, start, goal, title="Path Visualization"):
    """
    使用 matplotlib 可视化路径和地图
    :param path: 路径点列表 [(x, y), ...]，网格坐标系
    :param start: 起点 (网格坐标)
    :param goal: 终点 (网格坐标)
    :param title: 可视化标题
    """
    # 创建画布
    plt.figure(figsize=(8, 8))
    # 可视化起点和终点 
    plt.scatter(start[0], start[1], color="green", label="Start", s=100, marker="o")  # 起点
    plt.scatter(goal[0], goal[1], color="red", label="Goal", s=100, marker="x")  # 终点

    # 可视化路径
    if path:
        # print("path:", path)
        path_x, path_y = zip(*path)  # 解压路径点
        plt.plot(path_x, path_y, color="blue", label="Path", linewidth=2)  # 注意行列顺序

    plt.title(title)
    plt.xlabel("Grid Index (X - Column)")
    plt.ylabel("Grid Index (Y - Row)")
    plt.legend()
    plt.grid(True)
    plt.show()

def map_callback(msg):
    global map_msg,map_ok
    # rospy.loginfo("Map received!")
    map_msg = msg
    map_ok = True
    # print("map_ok",map_ok)
    
    

def start_callback(msg):
    global start_pose,start_ok
    start_pose = msg
    start_ok = True


def goal_callback(msg):
    global goal_pose, goal_ok, replan_flag, last_replan_time
    # 如果是循环规划，检查这次的goal是否和上次一样
    # 增加最小距离阈值，避免目标微小抖动造成频繁重规划
    if msg:
        dx = goal_pose.pose.position.x - msg.pose.position.x
        dy = goal_pose.pose.position.y - msg.pose.position.y
        dist = (dx*dx + dy*dy) ** 0.5
        if dist > goal_change_trigger_distance:
            now = rospy.Time.now()
            if last_replan_time is None or (now - last_replan_time).to_sec() >= replan_min_interval_sec:
                replan_flag = True
                last_replan_time = now
                rospy.loginfo("Goal changed (dist=%.2f m), replan triggered!", dist)
    goal_pose = msg
    goal_ok = True


def replan_callback(msg):
    global replan_flag, last_replan_time
    if msg.data == 'True':
        now = rospy.Time.now()
        if last_replan_time is None or (now - last_replan_time).to_sec() >= replan_min_interval_sec:
            replan_flag = True
            last_replan_time = now
            rospy.logdebug("Replan flag set!")
    # else:
    #     replan_flag = False

def main():
    rospy.init_node('local_planner')
    global map_msg, start_pose, goal_pose, map_ok, start_ok, goal_ok, path_pub

    # 获取参数
    manual_mode = rospy.get_param('~manual_mode', False)  # 是否手动模式
    algorithm_type = rospy.get_param('~algorithm_type', 'A*')  # 默认选择A*
    resolution = rospy.get_param('~resolution', 0.5)  # 搜索分辨率
    visualization = rospy.get_param('~visualization', False)  # 是否可视化
    path_frame_id = rospy.get_param('~path_frame_id', 'odom')  # 路径消息的frame_id
    loop_planning = rospy.get_param('~loop_planning', True)  # 是否循环规划
    trigger_replan_topic = rospy.get_param('~trigger_replan_topic', '/map_server/replan_signal')  # 触发重新规划的话题

    astar_params = {
        "diagonal_movement": rospy.get_param('~astar_diagonal_movement', True),
        "heuristic_weight": rospy.get_param('~astar_heuristic_weight', 0.85),
        "dilation_radius_meters": rospy.get_param('~astar_dilation_radius_meters', 0.5),
        "traverse_threshold": rospy.get_param('~astar_traverse_threshold', 68),
        "hard_obstacle_threshold": rospy.get_param('~astar_hard_obstacle_threshold', 88),
        "dilation_cost_threshold": rospy.get_param('~astar_dilation_cost_threshold', 75),
        "unknown_cost": rospy.get_param('~astar_unknown_cost', 88),
        "high_cost_penalty_gain": rospy.get_param('~astar_high_cost_penalty_gain', 2.5),
        "terrain_cost_weight": rospy.get_param('~astar_terrain_cost_weight', 3.0),
        "turn_penalty_gain": rospy.get_param('~astar_turn_penalty_gain', 0.35),
        "risk_threshold": rospy.get_param('~astar_risk_threshold', 50),
        "risk_penalty_gain": rospy.get_param('~astar_risk_penalty_gain', 1.5),
    }

    global replan_min_interval_sec, goal_change_trigger_distance
    replan_min_interval_sec = rospy.get_param('~replan_min_interval_sec', 3.5)
    goal_change_trigger_distance = rospy.get_param('~goal_change_trigger_distance', 1.5)
    # 消息发布
    rospy.Subscriber('/local_planner/local_map', OccupancyGrid, map_callback)
    path_pub = rospy.Publisher('/local_planner/local_path', Path, queue_size=1)
    
    if manual_mode:
        # 手动给定起点
        start_pose, goal_pose = manual_test_mode()
        start_ok, goal_ok = True
        rospy.loginfo("Manual mode: using manual poses for testing.")
    else:
        # 正常模式
        rospy.Subscriber('/pose_with_covariance', PoseWithCovarianceStamped, start_callback)
        rospy.Subscriber('/local_planner/local_goal', PoseStamped, goal_callback)
    
    # 对于循环规划，指定replan_flag，并订阅话题
    global replan_flag
    replan_flag = False
    if loop_planning:
        rospy.Subscriber(trigger_replan_topic, String, replan_callback)
   
    # 第一次规划
    first_plan = True
    planning(manual_mode, algorithm_type, resolution, visualization, path_frame_id, astar_params, first_plan)
    
    # 重置标志位
    start_ok,goal_ok,map_ok = False,False,False
    first_plan = False
    
    # 如果循环规划，就循环等待replan_flag
    if loop_planning:
        while not rospy.is_shutdown():
            if replan_flag:
                rospy.loginfo("Replanning...")
                planning(manual_mode, algorithm_type, resolution, visualization, path_frame_id, astar_params, first_plan)
                # 重置标志位
                start_ok,goal_ok,map_ok = False,False,False
                replan_flag = False
                rospy.sleep(1.0)
            else:
                rospy.logdebug("Waiting for replan signal...")
                rospy.sleep(1.0)



def planning(manual_mode, algorithm_type, resolution, visualization, path_frame_id, astar_params, first_plan):
    
    global map_msg, start_pose, goal_pose, map_ok, start_ok, goal_ok, path_pub
    while not rospy.is_shutdown():
        if(start_ok and goal_ok and map_ok):
            rospy.logdebug("All data received!")
            break
        else:
            rospy.logdebug("map_ok %s start_ok %s goal_ok %s", map_ok, start_ok, goal_ok)
            if not map_ok:
                rospy.loginfo_throttle(5, "Waiting for map...")
            if not start_ok:
                rospy.loginfo_throttle(5, "Waiting for start pose...")
            if not goal_ok:
                rospy.loginfo_throttle(5, "Waiting for goal pose...")
            rospy.sleep(1.5)

    start_pt = (start_pose.pose.pose.position.x, start_pose.pose.pose.position.y)
    goal_pt = (goal_pose.pose.position.x, goal_pose.pose.position.y)
    # 规划器放在全局变量中
    global planner
    if(first_plan):
        # 根据算法类型选择规划器
        if algorithm_type == 'A*':
            planner = AStar(map_msg, start_pt, goal_pt, resolution, **astar_params)
        # elif algorithm_type == 'Hybrid A*':
        #     planner = HybridAStar(map_msg, start_pose.pose, goal_pose.pose, resolution, **hybrid_params)
        else:
            rospy.logerr("Unknown algorithm type!")
            return
    else:
        if algorithm_type == 'A*':
            planner.re_init_(map_msg, start_pt, goal_pt, resolution, **astar_params)
        # elif algorithm_type == 'Hybrid A*':
        #     planner.re_init_(map_msg, start_pose.pose, goal_pose.pose, resolution, **hybrid_params)
        else:
            rospy.logerr("Unknown algorithm type!")
            return

    # 规划路径
    path = planner.a_star_search()

    # 可视化路径
    if path:
        if visualization:
            if manual_mode:
                visualize_path(
                    path,
                    start=(start_pose.pose.position.x, start_pose.pose.position.y),
                    goal=(goal_pose.pose.position.x, goal_pose.pose.position.y),
                    title=f"{algorithm_type} Path Visualization"
                )
            else:
                visualize_path(
                    path,
                    start=(start_pose.pose.pose.position.x, start_pose.pose.pose.position.y),
                    goal=(goal_pose.pose.position.x, goal_pose.pose.position.y),
                    title=f"{algorithm_type} Path Visualization"
                )
            
            rospy.logdebug("Path visualized!")
        # 发布路径消息
        local_path = Path()
        local_path.header.frame_id = path_frame_id
        local_path.header.stamp = rospy.Time.now()
        for pt in path:
            pose = PoseStamped()
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.orientation.w = 1.0
            local_path.poses.append(pose)
        # 发布路径
        path_pub.publish(local_path)
        rospy.sleep(0.1)
        path_pub.publish(local_path)
        rospy.logdebug("Path published!")
    else:
        rospy.logwarn("No path found!")


if __name__ == "__main__":
    main()
