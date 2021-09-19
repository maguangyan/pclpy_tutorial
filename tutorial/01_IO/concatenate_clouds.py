# -*- coding: utf-8 -*-
# @Time : DATE:2021/9/19
# @Author : yan
# @Email : 1792659158@qq.com
# @File : concate_two_pc.py

import pclpy
from pclpy import pcl
import numpy as np

if __name__ == '__main__':
    # 加载点云
    size_a, size_b = 100, 70
    a = np.random.ranf(size_a*3).reshape(-1, 3)
    cloud1 = pcl.PointCloud.PointXYZ.from_array(a)

    b = np.random.ranf(size_b*3).reshape(-1, 3)
    cloud2 = pcl.PointCloud.PointXYZ.from_array(b + 1)

    # 连接点 这里使用Numpy进行拼接
    c = np.vstack((cloud1.xyz, cloud2.xyz))
    cloud = pcl.PointCloud.PointXYZ.from_array(c)
    print(cloud.size())

    # 连接字段 pclpy似乎没有实现pcl::concatenateFields，这里使用Numpy进行拼接
    a_n = np.random.ranf(size_a * 4).reshape(-1, 4)  # 随机生成一点法线
    d = np.hstack((a, a_n))  # 拼接字段
    cloud_dn = pcl.PointCloud.PointNormal.from_array(d)

    # 上面的拼接法线效果不是很明显，试试拼接颜色
    a_c = np.random.ranf(size_a*3).reshape(-1, 3)*255    # 随机生成颜色
    cloud_dc = pcl.PointCloud.PointXYZRGB.from_array(a, a_c)

    # 可视化
    viewer = pcl.visualization.PCLVisualizer("viewer")
    viewer.setBackgroundColor(0.0, 0.0, 0.0)
    viewer.addPointCloud(cloud_dc, "sample cloud")  # 显示点云
    while not viewer.wasStopped():
        viewer.spinOnce(10)
