# 全局路径
global_path_input_topic = '/map_server/global_path_updated'
global_trajectory_output_topic = '/trajectory_ctrl/global_path_updated'
global_sample_num = 14
# 车辆位置
pose_topic = '/pose_with_covariance'
# 模式切换设定
mode_activate = True
mode_topic = '/navigation_mode'
pub_duration = 0.5

#Global Hermite:
start_times = 6
end_times = 4