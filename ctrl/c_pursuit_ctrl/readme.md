# c-pursuit
## introduction
保守纯追踪算法
能够按顺序跟随路径点，同时保持车辆在安全距离内
基本解决了原始纯追踪算法对不平滑路径跟踪效果不好的问题
## usage
c-pursuit节点
```bash
roslaunch c_pursuit_ctrl c_pursuit.launch
```
发送路径节点,可以使用该节点自定义路径，也可以接收规划器发送的路径
```bash
roslaunch c_pursuit_ctrl path_publisher.launch
```
## parameters
参数文件位于`config/c_pursuit.yaml`
## reference
Efficient autonomous navigation for planetary rovers with limited resources
Levin Gerdes, Martin Azkarate, José Ricardo Sánchez-Ibáñez, Luc Joudrier, Carlos Jesús Perez-del-Pulgar
