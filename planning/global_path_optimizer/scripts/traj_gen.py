######
import B_spline
import Hermite
import config
import numpy as np
import matplotlib.pyplot as plt
import rospy
######
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from scipy.spatial.transform import Rotation as R

######
pose_valid = False
last_goal_pt = np.array([0, 0])
mode = "invalid"
######
def pose_callback(msg):
    global start_vec,goal_vec,start_pt,goal_pt,pose_valid
    start_pt = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
    # # 将pose的orientation转为rpy
    # q = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    # 创建一个Rotation对象
    r = R.from_quat(q)
    # 取出车体在xy平面的方向向量
    # x轴方向,取出yaw角分量
    dx = np.cos(r.as_euler('zyx')[0])
    dy = np.sin(r.as_euler('zyx')[0])

    # 归一化
    length = np.sqrt(dx**2 + dy**2)
    dx = dx / length * config.start_times
    dy = dy / length * config.start_times
    # print('dx:', dx, 'dy:', dy)
    ## 保存
    start_vec = np.array([dx, dy])
    pose_valid = True
    # pose = msg

def global_path_callback(msg):
    global start_vec,goal_vec,start_pt,goal_pt,pose_valid
    global update_path,last_goal_pt
    global mode_switch
    global_path = msg
    # print('global_path:', global_path)
    if pose_valid == False:
        return
    if len(global_path.poses) < 2:
        return
    # 从path中得到goal位置和方向
    goal_pt = np.array([global_path.poses[1].pose.position.x, global_path.poses[1].pose.position.y])
    
    if(goal_pt[0] == last_goal_pt[0] and goal_pt[1] == last_goal_pt[1]):
        # print('False')
        update_path = False
    else:
        update_path = True
        last_goal_pt = goal_pt


    global mode
    if mode != "Mode 1":
        return
    if mode_switch == False and update_path == False:
        return
    


    if len(global_path.poses) > 2:
        # 下个点到localgoal的方向作为goal的方向
        dy = global_path.poses[2].pose.position.y - global_path.poses[1].pose.position.y
        dx = global_path.poses[2].pose.position.x - global_path.poses[1].pose.position.x
        # 单位化
        length = np.sqrt(dx**2 + dy**2)
        dx = dx / length * config.end_times
        dy = dy / length * config.end_times
        # 保存
        goal_vec = np.array([dx, dy])
    else:
        # pose到goal的连线作为goal的方向
        dy = goal_pt[1] - start_pt[1]
        dx = goal_pt[0] - start_pt[0]
        # 单位化
        length = np.sqrt(dx**2 + dy**2)
        dx = dx / length * config.end_times
        dy = dy / length * config.end_times
        # 保存
        goal_vec = np.array([dx, dy])
    # 对globalpath的第一段进行Hermite插值
    global_path_out = Path()
    global_path_out.header = global_path.header
    global_path_out.poses = []
    x= Hermite.cubic_hermite_evaluate(np.array([start_pt, start_vec,goal_pt,goal_vec]), config.global_sample_num)
    
    if(False):
        plt.plot(x[:, 0], x[:, 1], 'b-', label='Hermite')
        # 绘制起点和终点
        plt.plot(start_pt[0], start_pt[1], 'ro', label='Start point')
        plt.plot(goal_pt[0], goal_pt[1], 'ro', label='Goal point')
        # 绘制起点和终点向量
        plt.quiver(start_pt[0], start_pt[1], start_vec[0], start_vec[1], angles='xy', scale_units='xy', scale=10)
        plt.quiver(goal_pt[0], goal_pt[1], goal_vec[0], goal_vec[1], angles='xy', scale_units='xy', scale=10)
        plt.legend(loc='best')
        plt.show()
        pass

    global_path_out = Path()
    global_path_out.header = global_path.header
    for i in range(x.shape[0]):
        pose = PoseStamped()
        pose.header.frame_id = global_path.header.frame_id
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x[i, 0]
        pose.pose.position.y = x[i, 1]
        pose.pose.orientation.w = 1
        global_path_out.poses.append(pose)
    global_trajectory_pub.publish(global_path_out)
    pose_valid = False
    mode_switch = False

def mode_callback(msg):
    global mode,mode_switch
    # 如果当前是模式一，检查模式是否第一次变化
    if msg.data == "Mode 1" and mode != "Mode 1":
        mode_switch = True
    mode = msg.data

if __name__ == "__main__":
    # 初始化节点
    rospy.init_node('global_trajectory_optimizer', anonymous=True)
    # 发布者和订阅者
    ## 全局
    global_path_sub = rospy.Subscriber(config.global_path_input_topic, Path, global_path_callback)
    global_trajectory_pub = rospy.Publisher(config.global_trajectory_output_topic, Path, queue_size=1)
    ## 车辆位姿
    pose_sub = rospy.Subscriber(config.pose_topic, PoseWithCovarianceStamped, pose_callback)
    # ## debug
    # pose_pub = rospy.Publisher("/pose_out/debug", PoseStamped, queue_size=1)
    goal_pub = rospy.Publisher("/goal_out/debug", PoseStamped, queue_size=1)

    ## 模式切换
    if(config.mode_activate == True):
        mode_sub = rospy.Subscriber(config.mode_topic, String, mode_callback)
    # 循环
    while(not rospy.is_shutdown()):
        rospy.spin()













if __name__ == '__main__':
    # 初始化节点
    rospy.init_node('trajectory_optimizer', anonymous=True)
    # 发布者和订阅者
    ## 全局
    global_path_sub = rospy.Subscriber(config.global_path_input_topic, Path, global_path_callback)
    global_trajectory_pub = rospy.Publisher(config.global_trajectory_output_topic, Path, queue_size=10)
    ## 局部
    local_path_sub = rospy.Subscriber(config.local_path_input_topic, Path, local_path_callback)
    local_trajectory_pub = rospy.Publisher(config.local_trajectory_output_topic, Path, queue_size=10)
    ## 车辆位姿
    pose_sub = rospy.Subscriber(config.pose_topic, PoseWithCovarianceStamped, pose_callback)
    ## 模式切换
    if(config.mode_activate == true):
        mode_sub = rospy.Subscriber(config.mode_topic, String, mode_callback)
    # 循环
    rospy.spin()


