#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import config
import cv2
import numpy as np
import sys
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid




# 将图像压缩到栅格地图相同的尺寸大小,每单位像素代表1m2
def image_resize(image,map_size):
    # 获取图像的尺寸
    image_size = image.shape
    print(image_size[0], image_size[1])
    # 获取栅格地图的尺寸
    map_height, map_width = map_size
    # 计算图像和栅格地图的尺寸比例
    scale = (map_size[0]/float(image_size[0]), map_size[1]/float(image_size[1]))
    print("scale image to", int(scale[0]*100), "% * ", int(scale[1]*100), "%")
    # 将图像按照比例缩放,指定缩放方法为
    image_resized = cv2.resize(image, (int(map_width),int(map_height)),interpolation=cv2.INTER_AREA)
    # 保存图像
    #cv2.imwrite("image_resized.png", image_resized)
    return image_resized

# 将图像根据灰度值转换为栅格地图的高度，重新进行灰度值着色
def image_reheight(image, image_height_per_gray):
    # 获取图像的灰度值
    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # cv2.imshow("image_gray", image_gray)

    # 将灰度值 (0-255) 转换为高度值
    max_gray_value = 255
    image_height_map = image_gray.astype(float) * float(image_height_per_gray)
    print("Max height value:", max_gray_value * image_height_per_gray)
    return image_height_map


def calculate_accessibility_map(image, map_resolution_meter, max_slope_angle):
    """
    计算每个像素到周围8个像素的坡度，并标记是否可通行。

    :param image: 输入的灰度高度图（单位：米）
    :param map_resolution_meter: 每个像素代表的地图实际面积，单位为米/像素
    :param max_slope_angle: 最大允许的坡度，单位为角度
    :return: 可通行地图（0表示不可通行，255表示可通行）
    """

    # 如果输入不是灰度图，转换为灰度图
    if len(image.shape) > 2:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # 获取图像的尺寸
    height, width = image.shape[:2]

    # 初始化可通行性地图为全白
    accessibility_map = np.array([[255] * width for _ in range(height)], dtype=np.uint8)

    # 定义8个邻居的偏移量
    neighbors = [
        (-1, -1), (-1, 0), (-1, 1), 
        (0, -1),         (0, 1), 
        (1, -1), (1, 0), (1, 1)
    ]

    # 转换最大坡度角度为对应的斜率
    max_slope_ratio = np.tan(np.radians(max_slope_angle))

    # 遍历每个像素
    for i in range(height):
        for j in range(width):
            current_height = image[i, j]

            # 遍历8个邻居
            for dx, dy in neighbors:
                ni, nj = i + dx, j + dy
                # 确保邻居在图像范围内
                if 0 <= ni < height and 0 <= nj < width:
                    neighbor_height = image[ni, nj]

                    # 计算水平距离（假设对角线距离为 \sqrt{2}）
                    horizontal_distance = map_resolution_meter * (1 if dx == 0 or dy == 0 else np.sqrt(2))

                    # 计算坡度
                    height_difference = abs(current_height - neighbor_height)
                    slope = height_difference / horizontal_distance

                    # 如果坡度在允许范围内，标记为可通行
                    if slope <= max_slope_ratio:
                        accessibility_map[i, j] = 0

    # 显示可通行性地图
    #cv2.imshow("accessibility_map", accessibility_map)
    return accessibility_map

def apply_gaussian_blur(accessibility_map, kernel_size=5):
    """
    对可通行性地图应用高斯模糊以平滑结果。

    :param accessibility_map: 输入的可通行性地图
    :param kernel_size: 高斯模糊核的大小（必须为奇数）
    :return: 平滑后的可通行性地图
    """
    blurred_map = cv2.GaussianBlur(accessibility_map, (kernel_size, kernel_size), 0)
    return blurred_map

def apply_morphological_operations(accessibility_map, kernel_size=2):
    """
    对可通行性地图应用形态学操作以优化连续性。

    :param accessibility_map: 输入的可通行性地图
    :param kernel_size: 内核大小（必须为奇数）
    :return: 优化后的可通行性地图
    """
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))
    # 膨胀操作
    dilated_map = cv2.dilate(accessibility_map, kernel, iterations=1)
    # # 腐蚀操作
    cleaned_map = cv2.erode(dilated_map, kernel, iterations=1)
    return cleaned_map

def filter_small_connected_regions(accessibility_map, min_size=2):
    """
    移除可通行性地图中小的连通区域。

    :param accessibility_map: 输入的可通行性地图
    :param min_size: 最小连通区域的像素数量
    :return: 优化后的可通行性地图
    """
    # 获取连通分量
    num_labels, labels = cv2.connectedComponents(accessibility_map)
    optimized_map = np.zeros_like(accessibility_map)
    for label in range(1, num_labels):  # 从1开始忽略背景
        region = (labels == label)
        if np.sum(region) >= min_size:
            optimized_map[region] = 255
    return optimized_map

def connect_regions_via_distance(accessibility_map, distance_threshold=1):
    """
    使用距离变换将分散区域连成一片。

    :param accessibility_map: 输入的二值可通行性地图
    :param distance_threshold: 距离阈值，控制连接的范围
    :return: 连续区域的地图
    """
    # 计算距离变换
    dist_transform = cv2.distanceTransform(255 - accessibility_map, cv2.DIST_L2, 5)

    # 基于阈值生成新区域
    connected_map = (dist_transform < distance_threshold).astype(np.uint8) * 255

    return connected_map




# 将图像转换为栅格地图
def image_to_map(image_height_map, map_size, output_resolution_meter):
    # filp image_height_map
    image_height_map = np.flip(image_height_map, 0)
    # 创建地图
    map = OccupancyGrid()
    # 设置地图
    map_height, map_width = int(map_size[0]), int(map_size[1])
    print("map size:", map_height, map_width)
    map.header.frame_id = config.output_frame_id
    map.header.stamp = rospy.Time.now()
    map.info.width = map_width
    map.info.height = map_height
    map.info.resolution = output_resolution_meter
    map.info.origin.position.x = config.output_map_origin[0]
    map.info.origin.position.y = config.output_map_origin[1]
    map.info.origin.position.z = config.output_map_origin[2]
    # 将图像高度值转换为栅格地图的数据
    map.data = []
    for i in range(map_height):
        for j in range(map_width):
            map.data.append(int(image_height_map[i, j]))
            
    return map


# main
def main():
    rospy.init_node("image_to_map")


    # 从路径里读取图像
    sys.path.append(config.image_directory)
    image = cv2.imread(config.image_directory)
    # 发布地图相关
    map_topic = rospy.get_param("~map_topic", config.output_topic)
    map_publisher = rospy.Publisher(map_topic, OccupancyGrid, queue_size=0)
    map_size = (config.image_real_meter/config.output_resolution_meter,    
        config.image_real_meter/config.output_resolution_meter)
    # 将图像压缩到栅格地图相同的尺寸大小
    image_resized = image_resize(image, map_size)
    # cv2.imshow("image_resized", image_resized)
    # 将图像根据灰度值转换为栅格地图的高度，重新着色
    image_height_map = image_reheight(image_resized, config.image_height_per_gray)
    image_accessibility_map = calculate_accessibility_map(image_height_map, config.output_resolution_meter, config.max_slope_angle)
    image_cleaned = filter_small_connected_regions(image_accessibility_map)
    #cv2.imshow("image_cleaned", image_cleaned)
    image_connected = connect_regions_via_distance(image_cleaned)
    #cv2.imshow("image_connected", image_connected)
    image_morphological = apply_morphological_operations(image_connected)
    #cv2.imshow("image_morphological", image_morphological)
    # cv2.waitKey(0)
    # 将图像转换为栅格地图
    map = OccupancyGrid()
    # 图像归一化.waitKey(0)
    image_input = cv2.normalize(image_morphological, None, 0, 100, cv2.NORM_MINMAX, cv2.CV_8U)
    # check if image input data is int8
    print("image_input data type:", image_input.dtype)

    map = image_to_map(image_input, map_size, config.output_resolution_meter)
    

    while not rospy.is_shutdown():
        map_publisher.publish(map)
        rospy.sleep(1.0)



if __name__ == "__main__":
    main()