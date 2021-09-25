# -*- coding: utf-8 -*-
# @Time : DATE:2021/9/25
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 06_FPFH_descriptors.py


import pclpy
from pclpy import pcl
import numpy as np
import sys
import time
import matplotlib.pyplot as plt


if __name__ == '__main__':
    # 生成点云数据
    # 加载点云
    cloud = pcl.PointCloud.PointXYZ()
    reader = pcl.io.PCDReader()
    reader.read("../../data/bunny.pcd", cloud)
    print(cloud.size())
    # 构造法线估计类
    ne = pcl.features.NormalEstimation.PointXYZ_Normal()
    ne.setInputCloud(cloud)

    tree = pcl.search.KdTree.PointXYZ()
    ne.setSearchMethod(tree)

    normals = pcl.PointCloud.Normal()
    ne.setRadiusSearch(0.03)
    # 计算法线
    ne.compute(normals)
    print(normals.size())
    cloud_normals = pcl.PointCloud.PointNormal().from_array(
        np.hstack((cloud.xyz, normals.normals, normals.curvature.reshape(-1, 1))))
    for i in range(cloud_normals.size()):
        if not pcl.common.isFinite(cloud_normals.at(i)):
            print('cloud_normals[%d] is not finite\n', i)

    # 构造FPFH estimation类，把cloud和normals传递进去
    start = time.time()
    # fpfh = pcl.features.FPFHEstimation.PointXYZ_Normal_FPFHSignature33()
    fpfh = pcl.features.FPFHEstimationOMP.PointXYZ_Normal_FPFHSignature33(4)  # 使用OMP加速
    fpfh.setInputCloud(cloud)
    fpfh.setInputNormals(normals)
    # 或者，如果cloud是PointNormal类型，执行fpfh.setInputNormals(cloud);

    # 构造一个kd树
    # 它的内容将根据给定的输入点云填充到对象内部(因为没有给出其他搜索面)。
    tree = pcl.search.KdTree.PointXYZ()
    fpfh.setSearchMethod(tree)

    # 输出
    pfhs = pcl.PointCloud.FPFHSignature33()

    # 使用5cmm球形范围内的邻居点
    # 注意：在这里使用的半径必须大于用来估计表面法线的半径!!
    fpfh.setRadiusSearch(0.05)

    # 计算特征
    fpfh.compute(pfhs)
    end = time.time()
    print('cost time is:', end-start)
    print(pfhs.size())  # pfhs与cloud size应该相同

    # # 定义一个plotter 注意这里输入的数据是原始数据，而不是计算出来的直方图 暂时无法把原始数据提取出来
    # plotter = pcl.visualization.PCLPlotter()
    # for i in range(len(pfhs.histogram)):
    #     data = pfhs.histogram[i, :]
    #     plotter.addHistogramData(data, 125, "Histogram")
    # plotter.setBackgroundColor(1, 1, 1)
    # # 显示结果
    # plotter.plot()

    # 可以使用Matplotlib绘制直方图
    plt.plot(pfhs.histogram[1])
    plt.show()