# -*- coding: utf-8 -*-
# @Time : DATE:2021/10/4
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 01_ICP.py

import pclpy
from pclpy import pcl
import numpy as np

if __name__ == '__main__':
    # 生成点云
    cloud_size = 5
    a = np.random.ranf(cloud_size * 3).reshape(-1, 3) * 1024
    cloud_in = pcl.PointCloud.PointXYZ().from_array(a)

    cloud_out = pcl.PointCloud.PointXYZ()

    print('Saved', cloud_in.size(), 'data points to input:')
    for point in cloud_in.xyz:
        print(point)

    cloud_out = cloud_in
    print('size:', cloud_out.size())
    for point in cloud_out.points:
        point.x += 0.7
    print('Transformed', cloud_in.size(), 'data points:')
    for point in cloud_out.xyz:
        print(point)

