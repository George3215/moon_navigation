# map_generation
## introduction
基于png低精度图像生成全局gridmap

## usage
将png文件裁剪成地图实际大小
```bash
python resize_png.py
```
将png低精地图转换为gridmap消息并发布，同时初始化service给traversability estimation
```bash
roslaunch map_generation global_gridmap.launch
```
将traversability estimation得到的traversability map转换为occupancy grid消息并发布
```bash
roslaunch map_generation traversability_to_occgrid.launch
```

## parameters
参数文件位于`config`文件夹中

## reference
ETH gridmap https://github.com/ANYbotics/grid_map