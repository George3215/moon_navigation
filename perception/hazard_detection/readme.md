# hazard_detection
## introduction
危险检测算法
通过接收深度相机的点云并进行处理得到可通过性的2D地图
实现一种较简单的避障算法
## usage
hazard_detection节点
```bash
roslaunch hazard_detection detect_and_mapping.launch
```
依赖融合的点云，见
## parameters
参数文件位于`config/config.yaml`
## reference
Efficient autonomous navigation for planetary rovers with limited resources
Levin Gerdes, Martin Azkarate, José Ricardo Sánchez-Ibáñez, Luc Joudrier, Carlos Jesús Perez-del-Pulgar
