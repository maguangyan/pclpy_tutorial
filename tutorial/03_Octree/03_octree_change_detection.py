# -*- coding: utf-8 -*-
# @Time : DATE:2021/9/20
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 03_octree_change_detection.py

import pclpy
from pclpy import pcl
import numpy as np

if __name__ == '__main__':
    # 八叉树分辨率——八叉树体素的边长
    resolution = 32.0
    # 实例化基于八叉树的点云变化检测类
    # octree = pcl.octree.OctreePointCloudChangeDetector    # pclpy未实现pcl::octree::OctreePointCloudChangeDetector类，暂无法实现
    # 生成点云数据
    cloud_size = 128
    a = np.random.ranf(cloud_size * 3).reshape(-1, 3) * 64.0
    cloud = pcl.PointCloud.PointXYZ.from_array(a)

