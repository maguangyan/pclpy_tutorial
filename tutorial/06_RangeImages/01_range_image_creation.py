# -*- coding: utf-8 -*-
# @Time : DATE:2021/9/23
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 01_range_image_creation.py


import pclpy
from pclpy import pcl
import numpy as np

if __name__ == '__main__':
    # 生成点云数据
    cloud_size = 5
    a = np.random.ranf(cloud_size * 3).reshape(-1, 3) * 1024
    cloud = pcl.PointCloud.PointXYZ.from_array(a)
