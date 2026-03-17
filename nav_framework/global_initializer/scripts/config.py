
# image config
# 图像路径
image_directory = "./images/marsyard2022_terrain_hm.jpg"
# 图像对应仿真环境的实际尺寸/边长(m)
image_real_meter = 54
# 图像每个灰度对应的实际高度(m) 必须在0-1之间
image_height_per_gray = 4.820803273566/255
# 能通过的最大坡度，以初始化可通行性地图(度)
max_slope_angle = 2

# a* search
# manual_start_goal = True
manual_start_goal = False
# if manual is set to True
start = (40,50)
goal = (95,95)
# if manual is set to False
# 请输入地图坐标系下的起点和终点
pose_topic = "/pose_with_covariance" #geometry_msgs/PoseWithCovarianceStamped
goal_topic = "/move_base_simple/goal" #geometry_msgs/PoseStamped
# 如果需要将目标点根据map_origin进行坐标变换，设置为True
transform_origin = True


a_star_search_resolution = 1 #m
# 全局路径步长
global_path_step_size = 5 #m
visual = False

# output
#resolution tested with 1.0 0.5 and 2.0
output_topic = "/mode1/occupancy_grid"
output_resolution_meter = 0.2 #m  # 改为0.2m，得到270x270的地图尺寸，保留足够的细节


output_frame_id = "odom"
output_map_origin = (-27,-27,0)
obstacle_threshold = 50 # 灰度值大于该值的像素被认为是障碍物

# 全局地图话题（由 map_server 发布的 OccupancyGrid）
globalmap_topic = "/map_server/global_map"
path_topic = "/global_planner/global_path"
