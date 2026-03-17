from __future__ import division
# Qt
from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtWidgets import QLabel, QComboBox
from PyQt5.QtGui import QPixmap, QFont
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot
from python_qt_binding.QtGui import QImage, QPixmap
from python_qt_binding.QtWidgets import QHeaderView, QMenu, QTreeWidgetItem, QWidget
from PyQt5.QtWidgets import QWidget, QToolTip
# ROS
import roslib
import roslib.message
import roslib.names
import rospkg
import rospy
import roslaunch
import rviz
from nav_msgs.msg import OccupancyGrid
# Others
import os
from datetime import datetime
import subprocess
import sys
from colorama import Fore, Style, init

 
class MyWidget(QWidget):
    def __init__(self):
        super(MyWidget, self).__init__()
        # read UI file
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('navigation_gui'), 'resource',
                               'hunter_combo.ui')
        loadUi(ui_file, self)
        self.setObjectName('HUNTER_UI')
        self.tracking_launches = {}  # 存储多个启动的launch对象

        current_time = datetime.now()
        current_time_str = current_time.strftime('%H_%M_%S')
        current_file_path = __file__
        split_string = current_file_path.split('src')
        self.bag_path = split_string[0] + "bag/" + current_time_str + ".bag"
        self.launch_file_ = None
        
        self.pushButton_1.clicked.connect(self.open_button_slot) 
        self.pushButton_2.clicked.connect(self.close_button_slot)

        # self.client_button_1.clicked.connect(self.gui_launch_button)


        # self.pushButton_3.clicked.connect(self.start_bag)
        # # self.pushButton_3.setToolTip("Bag path: ../ws/bag/xxx.bag")
        # self.pushButton_4.clicked.connect(self.stop_bag)

        # self.pushButton_5.clicked.connect(self.map_server_launch_button)


        # perception
        self.perception_button_01.clicked.connect(self.launch_2d_perception)
        self.perception_button_02.clicked.connect(self.stop_2d_perception)
        self.perception_button_03.clicked.connect(self.launch_2_5d_perception)
        self.perception_button_04.clicked.connect(self.stop_2_5d_perception)


        # server
        self.server_button_01.clicked.connect(self.map_server_launch_button)
        self.server_button_02.clicked.connect(self.stop_map_server)

        # clients
        self.client_button_01.clicked.connect(self.initiallize_global_map_launch)
        self.client_button_02.clicked.connect(self.initiallize_global_path_launch)
        self.client_button_03.clicked.connect(self.get_global_map_launch)
        self.client_button_04.clicked.connect(self.get_local_map_launch)
        self.client_button_05.clicked.connect(self.update_map_launch)
        self.client_button_06.clicked.connect(self.stop_initiallize_global_map)
        self.client_button_07.clicked.connect(self.stop_initiallize_global_path)
        self.client_button_08.clicked.connect(self.stop_get_global_map)
        self.client_button_09.clicked.connect(self.stop_get_local_map)
        self.client_button_10.clicked.connect(self.stop_update_map)

        # bash
        self.vlm_button_01.clicked.connect(self.VLM_perception_bash)
        self.vlm_button_02.clicked.connect(self.stop_VLM_perception_bash)
        self.manual_button_01.clicked.connect(self.manual_mode_bash)
        self.manual_button_02.clicked.connect(self.stop_manual_mode_bash)

        self.script_button_01.clicked.connect(self.initialize_map_bash)
        self.script_button_02.clicked.connect(self.stop_initialize_map_bash)
        self.script_button_03.clicked.connect(self.initialize_path_bash)
        self.script_button_04.clicked.connect(self.stop_initialize_path_bash)
        self.script_button_05.clicked.connect(self.local_planning_bash)
        self.script_button_06.clicked.connect(self.stop_local_planning_bash)
        self.script_button_07.clicked.connect(self.trajecotry_ctrl_launch)
        self.script_button_08.clicked.connect(self.stop_trajecotry_ctrl_launch)
        self.script_button_09.clicked.connect(self.global_path_optimize_bash)
        self.script_button_10.clicked.connect(self.stop_global_path_optimize_bash)



        # #QLabel连接到Gif
        # self.gif_label = QtWidgets.QLabel(self)
        # # 设置 QLabel 的位置和大小
        # self.gif_label.setGeometry(0, 0, 180, 180)
        # gif_path = os.path.join(rp.get_path('navigation_gui'), 'resource',
        #                          'klee.gif')
        # self.movie = QtGui.QMovie(gif_path)
        # self.gif_label.setMovie(self.movie)

        # # 设置 GIF 的缩放大小
        # self.movie.setScaledSize(QtCore.QSize(180, 180))

        # self.movie.start()

        # self.Combo.currentIndexChanged.connect(self.on_combo_changed)



    @Slot()
    def gui_launch_button(self):
        print(f"{Fore.GREEN}==> Launching gui.launch...")
        
        rp = rospkg.RosPack()
        pkg_path = rp.get_path("navigation_gui")
        full_launch_path = pkg_path + "/launch/gui_launch.launch"
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        tracking_launch = roslaunch.parent.ROSLaunchParent(
            uuid, [full_launch_path])
        tracking_launch.start()

    @Slot()
    def open_button_slot(self):
        combo_text = self.Combo.currentText()
        if combo_text == "simulation":
            self.launch_simulation_workflow()
            return
        if combo_text == "initialization":
            self.launch_initialization_workflow()
            return
        if combo_text == "navigation":
            self.launch_navigation_workflow()
            return

        if self.launch_file_ == "None":
            print(f"{Fore.RED}==> No Launch File Selected Yet...")
        else:
            self.select_launch_file(combo_text)

        split_launch = self.launch_file_.split("/")
        rp = rospkg.RosPack()
        pkg_path = rp.get_path(split_launch[0])
        full_launch_path = pkg_path + "/" + "/".join(split_launch[1:])
        print(f"{Fore.BLUE}==> Launching {full_launch_path}....{Style.RESET_ALL}")
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        tracking_launch = roslaunch.parent.ROSLaunchParent(
            uuid, [full_launch_path])
        tracking_launch.start()

    def _start_named_launch(self, launch_key, pkg_name, relative_launch_path):
        if launch_key in self.tracking_launches:
            print(f"{Fore.YELLOW}==> {launch_key} is already running...{Style.RESET_ALL}")
            return
        rp = rospkg.RosPack()
        pkg_path = rp.get_path(pkg_name)
        full_launch_path = pkg_path + relative_launch_path
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        tracking_launch = roslaunch.parent.ROSLaunchParent(uuid, [full_launch_path])
        self.tracking_launches[launch_key] = tracking_launch
        tracking_launch.start()

    def _wait_for_global_map_initialized(self, timeout_sec=60.0):
        start_t = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            elapsed = rospy.Time.now().to_sec() - start_t
            if elapsed > timeout_sec:
                return False
            try:
                msg = rospy.wait_for_message("/map_server/global_map", OccupancyGrid, timeout=1.0)
                if len(msg.data) > 0:
                    return True
            except Exception:
                pass
        return False

    @Slot()
    def launch_simulation_workflow(self):
        print(f"{Fore.GREEN}==> LaunchBox workflow: simulation{Style.RESET_ALL}")
        self._start_named_launch("sim_marsyard", "leo_erc_gazebo", "/launch/leo_marsyard.launch")
        rospy.sleep(2.0)
        self._start_named_launch("sim_rviz", "leo_erc_viz", "/launch/rviz.launch")

    @Slot()
    def launch_initialization_workflow(self):
        print(f"{Fore.GREEN}==> LaunchBox workflow: initialization{Style.RESET_ALL}")
        self.map_server_launch_button()
        rospy.sleep(2.0)
        self.initiallize_global_map_launch()
        rospy.sleep(1.0)
        self.initialize_map_bash()
        print(f"{Fore.BLUE}==> Waiting for /map_server/global_map initialization...{Style.RESET_ALL}")
        if not self._wait_for_global_map_initialized(timeout_sec=90.0):
            print(f"{Fore.RED}==> Global map init timeout. Please check map server and initialize map script logs.{Style.RESET_ALL}")
            return
        print(f"{Fore.GREEN}==> Global map initialized. Continue path initialization...{Style.RESET_ALL}")
        self.initiallize_global_path_launch()
        rospy.sleep(1.0)
        self.initialize_path_bash()
        print(f"{Fore.CYAN}==> Please use RViz '2D Nav Goal' to set navigation goal.{Style.RESET_ALL}")

    @Slot()
    def launch_navigation_workflow(self):
        print(f"{Fore.GREEN}==> LaunchBox workflow: navigation{Style.RESET_ALL}")
        self.launch_2d_perception()
        rospy.sleep(1.0)
        self.launch_2_5d_perception()
        rospy.sleep(1.0)
        self.get_local_map_launch()
        rospy.sleep(1.0)
        self.update_map_launch()
        rospy.sleep(1.0)
        self.global_path_optimize_bash()
        rospy.sleep(1.0)
        self.local_planning_bash()
        rospy.sleep(1.0)
        self.trajecotry_ctrl_launch()

    @Slot()
    def close_button_slot(self):
        cmds = ["killall -9 rosmaster",
                "killall -9 roscore",
                "killall gzserver",
                "killall gzclient"]
        for cmd in cmds:
            print(f"{Fore.RED}==> Waiting for {cmd}....{Style.RESET_ALL}")
            subprocess.Popen(cmd, shell=True)





    # perception
    @Slot()
    def launch_2d_perception(self):
        print(f"{Fore.GREEN}==> Running 2D perception...")

        rp = rospkg.RosPack()
        pkg_path = rp.get_path("hazard_detection")
        full_launch_path = pkg_path + "/launch/detect_and_mapping.launch"
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # 存储对应的launch实例
        if '2d_perception' not in self.tracking_launches:
            tracking_launch = roslaunch.parent.ROSLaunchParent(uuid, [full_launch_path])
            self.tracking_launches['2d_perception'] = tracking_launch
            tracking_launch.start()
        else:
            print(f"{Fore.YELLOW}==> 2D perception is already running...{Style.RESET_ALL}")

    @Slot()
    def stop_2d_perception(self):
        if '2d_perception' in self.tracking_launches:
            print(f"{Fore.GREEN}==> Stopping 2D perception...")
            self.tracking_launches['2d_perception'].shutdown()
            del self.tracking_launches['2d_perception']
        else:
            print(f"{Fore.GREEN}==> 2D perception is not running.")

    @Slot()
    def launch_2_5d_perception(self):
        print(f"{Fore.GREEN}==> Running 2.5D perception...")

        rp = rospkg.RosPack()
        pkg_path = rp.get_path("elevation_mapping_cupy")
        full_launch_path = pkg_path + "/launch/mapping_leo.launch"
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # 存储对应的launch实例
        if '2_5d_perception' not in self.tracking_launches:
            tracking_launch = roslaunch.parent.ROSLaunchParent(uuid, [full_launch_path])
            self.tracking_launches['2_5d_perception'] = tracking_launch
            tracking_launch.start()
        else:
            print(f"{Fore.YELLOW}==> 2.5D perception is already running...{Style.RESET_ALL}")
    
    @Slot()
    def stop_2_5d_perception(self):
        if '2_5d_perception' in self.tracking_launches:
            print(f"{Fore.GREEN}==> Stopping 2.5D perception...")
            self.tracking_launches['2_5d_perception'].shutdown()
            del self.tracking_launches['2_5d_perception']
        else:
            print(f"{Fore.GREEN}==> 2.5D perception is not running.")
    
        
    @Slot()
    def trajecotry_ctrl_launch(self):
        print(f"{Fore.GREEN}==> Running trajecotry ctrl...")

        rp = rospkg.RosPack()
        pkg_path = rp.get_path("c_pursuit_ctrl")
        full_launch_path = pkg_path + "/launch/c_pursuit.launch"
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # 存储对应的launch实例
        if 'c_pursuit' not in self.tracking_launches:
            tracking_launch = roslaunch.parent.ROSLaunchParent(uuid, [full_launch_path])
            self.tracking_launches['c_pursuit'] = tracking_launch
            tracking_launch.start()
        else:
            print(f"{Fore.YELLOW}==> c_pursuit is already running...{Style.RESET_ALL}")
    
    @Slot()
    def stop_trajecotry_ctrl_launch(self):
        if 'c_pursuit' in self.tracking_launches:
            print(f"{Fore.GREEN}==> Stopping c_pursuit...")
            self.tracking_launches['c_pursuit'].shutdown()
            del self.tracking_launches['c_pursuit']
        else:
            print(f"{Fore.GREEN}==> c_pursuit is not running.")

    # server
    @Slot()
    def map_server_launch_button(self):
        print(f"{Fore.GREEN}==> Launching map_server.launch...")
        
        rp = rospkg.RosPack()
        pkg_path = rp.get_path("globalmap_server")
        full_launch_path = pkg_path + "/launch/map_server.launch"
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        tracking_launch = roslaunch.parent.ROSLaunchParent(
            uuid, [full_launch_path])
        tracking_launch.start()


    # clients
    @Slot()        

    def initiallize_global_map_launch(self):
        print(f"{Fore.GREEN}==> Initialize global map client.launch...")
        
        rp = rospkg.RosPack()
        pkg_path = rp.get_path("globalmap_server")
        full_launch_path = pkg_path + "/launch/initializemap_client.launch"
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        tracking_launch = roslaunch.parent.ROSLaunchParent(
            uuid, [full_launch_path])
        tracking_launch.start()
    
    @Slot()
    def initiallize_global_path_launch(self):
        print(f"{Fore.GREEN}==> Initialize global path client.launch...")
        
        rp = rospkg.RosPack()
        pkg_path = rp.get_path("globalmap_server")
        full_launch_path = pkg_path + "/launch/initialize_globalpath_client.launch"
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        tracking_launch = roslaunch.parent.ROSLaunchParent(
            uuid, [full_launch_path])
        tracking_launch.start()

    @Slot()
    def get_global_map_launch(self):
        print(f"{Fore.GREEN}==> Get global map client.launch...")
        
        rp = rospkg.RosPack()
        pkg_path = rp.get_path("globalmap_server")
        full_launch_path = pkg_path + "/launch/getglobalmap_client.launch"
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        tracking_launch = roslaunch.parent.ROSLaunchParent(
            uuid, [full_launch_path])
        tracking_launch.start()

    @Slot()
    def get_local_map_launch(self):
        print(f"{Fore.GREEN}==> Get local map client.launch...")
        
        rp = rospkg.RosPack()
        pkg_path = rp.get_path("globalmap_server")
        full_launch_path = pkg_path + "/launch/getlocalmap_client.launch"
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        tracking_launch = roslaunch.parent.ROSLaunchParent(
            uuid, [full_launch_path])
        tracking_launch.start()

    @Slot()
    def update_map_launch(self):
        print(f"{Fore.GREEN}==> Update map client.launch...")

        rp = rospkg.RosPack()
        pkg_path = rp.get_path("globalmap_server")
        full_launch_path = pkg_path + "/launch/updatemap_client.launch"
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        tracking_launch = roslaunch.parent.ROSLaunchParent(
            uuid, [full_launch_path])
        tracking_launch.start()
    
    @Slot()
    def stop_map_server(self):
        process = subprocess.Popen(['ps', 'aux'], stdout=subprocess.PIPE)
        grep_process = subprocess.Popen(['grep', 'map_server'], stdin=process.stdout, stdout=subprocess.PIPE)
        process.stdout.close()

        output, _ = grep_process.communicate()
        lines = output.decode('utf-8').split('\n')

        pid_list = []
        for line in lines:
            if 'map_server' in line:
                pid_list.append(int(line.split()[1]))
            
        for item in pid_list:
            rospy.logwarn(f"Map Server PID: {item}")
            stop_map_server_command = f"kill -2 {int(item)}"
            subprocess.Popen(stop_map_server_command, shell=True)

        rospy.logwarn("Map Server process has been killed.")

        pass

    @Slot()
    def stop_initiallize_global_map(self):
        process = subprocess.Popen(['ps', 'aux'], stdout=subprocess.PIPE)
        grep_process = subprocess.Popen(['grep', 'initializemap_client'], stdin=process.stdout, stdout=subprocess.PIPE)
        process.stdout.close()

        output, _ = grep_process.communicate()
        lines = output.decode('utf-8').split('\n')

        pid_list = []
        for line in lines:
            if 'initializemap_client' in line:
                pid_list.append(int(line.split()[1]))
            
        for item in pid_list:
            rospy.logwarn(f"Initiallize Global Map PID: {item}")
            stop_initiallize_global_map_command = f"kill -2 {int(item)}"
            subprocess.Popen(stop_initiallize_global_map_command, shell=True)

        rospy.logwarn("Initiallize Global Map process has been killed.")

        pass
    
    @Slot()
    def stop_initiallize_global_path(self):
        process = subprocess.Popen(['ps', 'aux'], stdout=subprocess.PIPE)
        grep_process = subprocess.Popen(['grep', 'initialize_globalpath_client'], stdin=process.stdout, stdout=subprocess.PIPE)
        process.stdout.close()

        output, _ = grep_process.communicate()
        lines = output.decode('utf-8').split('\n')

        pid_list = []
        for line in lines:
            if 'initialize_globalpath_client' in line:
                pid_list.append(int(line.split()[1]))
            
        for item in pid_list:
            rospy.logwarn(f"Initiallize Global Path PID: {item}")
            stop_initiallize_global_path_command = f"kill -2 {int(item)}"
            subprocess.Popen(stop_initiallize_global_path_command, shell=True)

        rospy.logwarn("Initiallize Global Path process has been killed.")

        pass
    
    @Slot()
    def stop_get_global_map(self):
        process = subprocess.Popen(['ps', 'aux'], stdout=subprocess.PIPE)
        grep_process = subprocess.Popen(['grep', 'getglobalmap_client'], stdin=process.stdout, stdout=subprocess.PIPE)
        process.stdout.close()

        output, _ = grep_process.communicate()
        lines = output.decode('utf-8').split('\n')

        pid_list = []
        for line in lines:
            if 'getglobalmap_client' in line:
                pid_list.append(int(line.split()[1]))
            
        for item in pid_list:
            rospy.logwarn(f"Get Global Map PID: {item}")
            stop_get_global_map_command = f"kill -2 {int(item)}"
            subprocess.Popen(stop_get_global_map_command, shell=True)

        rospy.logwarn("Get Global Map process has been killed.")

        pass

    @Slot()
    def stop_get_local_map(self):
        process = subprocess.Popen(['ps', 'aux'], stdout=subprocess.PIPE)
        grep_process = subprocess.Popen(['grep', 'getlocalmap_client'], stdin=process.stdout, stdout=subprocess.PIPE)
        process.stdout.close()

        output, _ = grep_process.communicate()
        lines = output.decode('utf-8').split('\n')

        pid_list = []
        for line in lines:
            if 'getlocalmap_client' in line:
                pid_list.append(int(line.split()[1]))
            
        for item in pid_list:
            rospy.logwarn(f"Get Local Map PID: {item}")
            stop_get_local_map_command = f"kill -2 {int(item)}"
            subprocess.Popen(stop_get_local_map_command, shell=True)

        rospy.logwarn("Get Local Map process has been killed.")

        pass

    @Slot()
    def stop_update_map(self):
        process = subprocess.Popen(['ps', 'aux'], stdout=subprocess.PIPE)
        grep_process = subprocess.Popen(['grep', 'updatemap_client'], stdin=process.stdout, stdout=subprocess.PIPE)
        process.stdout.close()

        output, _ = grep_process.communicate()
        lines = output.decode('utf-8').split('\n')

        pid_list = []
        for line in lines:
            if 'updatemap_client' in line:
                pid_list.append(int(line.split()[1]))
            
        for item in pid_list:
            rospy.logwarn(f"Update Map PID: {item}")
            stop_update_map_command = f"kill -2 {int(item)}"
            subprocess.Popen(stop_update_map_command, shell=True)

        rospy.logwarn("Update Map process has been killed.")

        pass

    # bash
    @Slot()
    def VLM_perception_bash(self):
        print(f"{Fore.GREEN}==> Running VLM perception...")
        rp = rospkg.RosPack()
        pkg_path = rp.get_path("terrain_classification")
        full_launch_path = pkg_path + "/scripts/VLM_depth_launch.sh"
        subprocess.Popen(full_launch_path, shell=True)

    @Slot()
    def stop_VLM_perception_bash(self):
        print("==> Stopping processes related to VLM_ros_depth_topic.py...")
        # 查找运行中的进程
        process = subprocess.Popen(['ps', 'aux'], stdout=subprocess.PIPE)
        grep_process = subprocess.Popen(['grep', 'VLM_ros_depth_topic.py'], stdin=process.stdout, stdout=subprocess.PIPE)
        process.stdout.close()
        output, _ = grep_process.communicate()
        lines = output.decode('utf-8').split('\n')
        pid_list = []
        for line in lines:
            if 'VLM_ros_depth_topic.py' in line and 'grep' not in line:
                try:
                    # 提取 PID (第二列为 PID)
                    pid_list.append(int(line.split()[1]))
                except (IndexError, ValueError):
                    rospy.logwarn(f"无法解析进程行: {line}")

        # 终止所有找到的进程
        for pid in pid_list:
            rospy.logwarn(f"正在终止进程 PID: {pid}")
            try:
                # 使用 SIGTERM (-15) 信号优雅终止进程，或 SIGKILL (-9) 强制终止
                stop_command = f"kill -15 {pid}"
                subprocess.Popen(stop_command, shell=True)
            except Exception as e:
                rospy.logerr(f"终止进程 {pid} 时出错: {e}")

        rospy.logwarn("所有 VLM_ros_depth_topic.py 相关进程已被终止。")

    @Slot()
    def manual_mode_bash(self):
        print(f"{Fore.GREEN}==> Running manual mode...")
        rp = rospkg.RosPack()
        pkg_path = rp.get_path("terrain_classification")
        full_launch_path = pkg_path + "/scripts/manual_mode.sh"
        subprocess.Popen(full_launch_path, shell=True)

    @Slot()
    def stop_manual_mode_bash(self):
        print("==> Stopping processes related to mode_handler.py...")
        # 查找运行中的进程
        process = subprocess.Popen(['ps', 'aux'], stdout=subprocess.PIPE)
        grep_process = subprocess.Popen(['grep', 'mode_handler.py'], stdin=process.stdout, stdout=subprocess.PIPE)
        process.stdout.close()
        output, _ = grep_process.communicate()
        lines = output.decode('utf-8').split('\n')
        pid_list = []
        for line in lines:
            if 'mode_handler.py' in line and 'grep' not in line:
                try:
                    # 提取 PID (第二列为 PID)
                    pid_list.append(int(line.split()[1]))
                except (IndexError, ValueError):
                    rospy.logwarn(f"无法解析进程行: {line}")

        # 终止所有找到的进程
        for pid in pid_list:
            rospy.logwarn(f"正在终止进程 PID: {pid}")
            try:
                # 使用 SIGTERM (-15) 信号优雅终止进程，或 SIGKILL (-9) 强制终止
                stop_command = f"kill -15 {pid}"
                subprocess.Popen(stop_command, shell=True)
            except Exception as e:
                rospy.logerr(f"终止进程 {pid} 时出错: {e}")

        rospy.logwarn("所有 manual_mode.py 相关进程已被终止。")


    @Slot()
    def initialize_map_bash(self):
        print(f"{Fore.GREEN}==> Running initialize_map...")
        rp = rospkg.RosPack()
        pkg_path = rp.get_path("global_initializer")
        full_launch_path = pkg_path + "/scripts/global_map_initialize.sh"
        subprocess.Popen(full_launch_path, shell=True)


    @Slot()
    def stop_initialize_map_bash(self):

        print("==> Stopping processes related to image2map_initialize.py...")
        # 查找运行中的进程
        process = subprocess.Popen(['ps', 'aux'], stdout=subprocess.PIPE)
        grep_process = subprocess.Popen(['grep', 'image2map_initialize.py'], stdin=process.stdout, stdout=subprocess.PIPE)
        process.stdout.close()
        output, _ = grep_process.communicate()
        lines = output.decode('utf-8').split('\n')
        pid_list = []
        for line in lines:
            if 'image2map_initialize.py' in line and 'grep' not in line:
                try:
                    # 提取 PID (第二列为 PID)
                    pid_list.append(int(line.split()[1]))
                except (IndexError, ValueError):
                    rospy.logwarn(f"无法解析进程行: {line}")

        # 终止所有找到的进程
        for pid in pid_list:
            rospy.logwarn(f"正在终止进程 PID: {pid}")
            try:
                # 使用 SIGTERM (-15) 信号优雅终止进程，或 SIGKILL (-9) 强制终止
                stop_command = f"kill -15 {pid}"
                subprocess.Popen(stop_command, shell=True)
            except Exception as e:
                rospy.logerr(f"终止进程 {pid} 时出错: {e}")

        rospy.logwarn("所有 image2map_initialize.py 相关进程已被终止。")

    @Slot()
    def initialize_path_bash(self):
        print(f"{Fore.GREEN}==> Running initialize_path...")
        rp = rospkg.RosPack()
        pkg_path = rp.get_path("global_initializer")
        full_launch_path = pkg_path + "/scripts/global_planning.sh"
        subprocess.Popen(full_launch_path, shell=True)
    
    @Slot()
    def stop_initialize_path_bash(self):

        print("==> Stopping processes related to image_planning_server.py...")
        # 查找运行中的进程
        process = subprocess.Popen(['ps', 'aux'], stdout=subprocess.PIPE)
        grep_process = subprocess.Popen(['grep', 'image_planning_server.py'], stdin=process.stdout, stdout=subprocess.PIPE)
        process.stdout.close()
        output, _ = grep_process.communicate()
        lines = output.decode('utf-8').split('\n')
        pid_list = []
        for line in lines:
            if 'image_planning_server.py' in line and 'grep' not in line:
                try:
                    # 提取 PID (第二列为 PID)
                    pid_list.append(int(line.split()[1]))
                except (IndexError, ValueError):
                    rospy.logwarn(f"无法解析进程行: {line}")

        # 终止所有找到的进程
        for pid in pid_list:
            rospy.logwarn(f"正在终止进程 PID: {pid}")
            try:
                # 使用 SIGTERM (-15) 信号优雅终止进程，或 SIGKILL (-9) 强制终止
                stop_command = f"kill -15 {pid}"
                subprocess.Popen(stop_command, shell=True)
            except Exception as e:
                rospy.logerr(f"终止进程 {pid} 时出错: {e}")

        rospy.logwarn("所有 image_planning_server.py 相关进程已被终止。")    

    @Slot()
    def local_planning_bash(self):
        print(f"{Fore.GREEN}==> Running local_planning...")
        rp = rospkg.RosPack()
        pkg_path = rp.get_path("local_planner")
        full_launch_path = pkg_path + "/scripts/local_planner.sh"
        subprocess.Popen(full_launch_path, shell=True)

    @Slot()
    def stop_local_planning_bash(self):

        print("==> Stopping processes related to local_planner.py...")
        # 查找运行中的进程
        process = subprocess.Popen(['ps', 'aux'], stdout=subprocess.PIPE)
        grep_process = subprocess.Popen(['grep', 'local_planner.py'], stdin=process.stdout, stdout=subprocess.PIPE)
        process.stdout.close()
        output, _ = grep_process.communicate()
        lines = output.decode('utf-8').split('\n')
        pid_list = []
        for line in lines:
            if 'local_planner.py' in line and 'grep' not in line:
                try:
                    # 提取 PID (第二列为 PID)
                    pid_list.append(int(line.split()[1]))
                except (IndexError, ValueError):
                    rospy.logwarn(f"无法解析进程行: {line}")

        # 终止所有找到的进程
        for pid in pid_list:
            rospy.logwarn(f"正在终止进程 PID: {pid}")
            try:
                # 使用 SIGTERM (-15) 信号优雅终止进程，或 SIGKILL (-9) 强制终止
                stop_command = f"kill -15 {pid}"
                subprocess.Popen(stop_command, shell=True)
            except Exception as e:
                rospy.logerr(f"终止进程 {pid} 时出错: {e}")

        rospy.logwarn("所有 local_planner.py 相关进程已被终止。")

    @Slot()
    def global_path_optimize_bash(self):
        print(f"{Fore.GREEN}==> Running global_path_optimizer...")
        rp = rospkg.RosPack()
        pkg_path = rp.get_path("global_path_optimizer")
        full_launch_path = pkg_path + "/scripts/global_path_optimize.sh"
        subprocess.Popen(full_launch_path, shell=True)

    @Slot()
    def stop_global_path_optimize_bash(self):
        print("==> Stopping processes related to local_planner.py...")
        # 查找运行中的进程
        process = subprocess.Popen(['ps', 'aux'], stdout=subprocess.PIPE)
        grep_process = subprocess.Popen(['grep', 'traj_gen.py'], stdin=process.stdout, stdout=subprocess.PIPE)
        process.stdout.close()
        output, _ = grep_process.communicate()
        lines = output.decode('utf-8').split('\n')
        pid_list = []
        for line in lines:
            if 'traj_gen.py' in line and 'grep' not in line:
                try:
                    # 提取 PID (第二列为 PID)
                    pid_list.append(int(line.split()[1]))
                except (IndexError, ValueError):
                    rospy.logwarn(f"无法解析进程行: {line}")

        # 终止所有找到的进程
        for pid in pid_list:
            rospy.logwarn(f"正在终止进程 PID: {pid}")
            try:
                # 使用 SIGTERM (-15) 信号优雅终止进程，或 SIGKILL (-9) 强制终止
                stop_command = f"kill -15 {pid}"
                subprocess.Popen(stop_command, shell=True)
            except Exception as e:
                rospy.logerr(f"终止进程 {pid} 时出错: {e}")

        rospy.logwarn("所有 traj_gen.py 相关进程已被终止。")








    @Slot()
    def start_bag(self):
        process = subprocess.Popen(['ps', 'aux'], stdout=subprocess.PIPE)
        grep_process = subprocess.Popen(['grep', 'rosbag'], stdin=process.stdout, stdout=subprocess.PIPE)
        process.stdout.close()

        output, _ = grep_process.communicate()
        lines = output.decode('utf-8').split('\n')

        pid_list = []
        for line in lines:
            if 'rosbag' in line:
                pid_list.append(int(line.split()[1]))
            
        for item in pid_list:
            rospy.logwarn(f"Rosbag PID: {item}")
            stop_bag_command = f"kill -2 {int(item)}"
            subprocess.Popen(stop_bag_command, shell=True)

        rospy.logwarn("Rosbag process has been killed.")

        pass

    @Slot()
    def stop_bag(self):
        topics = rospy.get_param("/topics_to_bag")
        topics_str = ' '.join(topics)
        print(topics_str)
        print(f"{Fore.GREEN}==> Bagging {topics_str}....{Style.RESET_ALL}")
        print(f"{Fore.GREEN}==> Saving to {self.bag_path}....{Style.RESET_ALL}")
    
        start_bag_command = f"rosbag record -O {self.bag_path} {topics_str}"
        subprocess.Popen(start_bag_command, shell=True)



    @Slot()
    def on_combo_changed(self):
        selected_launch_file = self.Combo.currentText()
        print(f"{Fore.GREEN}==> Selected {selected_launch_file}....{Style.RESET_ALL}")

        pass
    
    def select_launch_file(self, combo_text):
        launch_file_ = rospy.get_param(combo_text)[0]
        self.launch_file_ = launch_file_

    def close_plugin(self):
        try:
            self.mysub.unregister()

        except AttributeError as e:
            rospy.logerr("Subscriber doesn't open.")
