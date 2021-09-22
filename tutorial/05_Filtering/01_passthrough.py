# -*- coding: utf-8 -*-
# @Time : DATE:2021/9/22
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 01_passthrough.py

import pclpy
from pclpy import pcl
import numpy as np

if __name__ == '__main__':
    # 生成点云数据
    cloud_size = 5
    a = np.random.ranf(cloud_size * 3).reshape(-1, 3) * 1024
    cloud = pcl.PointCloud.PointXYZ.from_array(a)
    cloud_filtered = pcl.PointCloud.PointXYZ()

    print('Cloud before filtering:')
    for point in cloud.points:
        print(point.x, ' ',
              point.y, ' ',
              point.z, ' ')

    # 滤波
    pa = pcl.filters.PassThrough.PointXYZ()
    pa.setInputCloud(cloud)
    pa.setFilterFieldName('z')
    pa.setFilterLimits(0.0, 512.0)
    # pa.setFilterLimitsNegative(True) # 控制过滤掉范围内还是范围内的点，默认为Flase，即0-512内的点被保留
    pa.filter(cloud_filtered)

    print('Cloud after filtering: ')
    for point in cloud_filtered.points:
        print(point.x, ' ',
              point.y, ' ',
              point.z, ' ')
