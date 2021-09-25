# -*- coding: utf-8 -*-
# @Time : DATE:2021/9/25
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 07_VFH_descriptors.py

import pclpy
from pclpy import pcl
import numpy as np
import sys
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

    # 构造VFH estimation类，把cloud和normals传递进去
    vfh = pcl.features.VFHEstimation.PointXYZ_Normal_VFHSignature308()
    vfh.setInputCloud(cloud)
    vfh.setInputNormals(normals)
    # 或者，如果cloud是PointNormal类型，执行vfh.setInputNormals(cloud);

    # 构造一个kd树
    # 它的内容将根据给定的输入点云填充到对象内部(因为没有给出其他搜索面)。
    tree = pcl.search.KdTree.PointXYZ()
    vfh.setSearchMethod(tree)

    # 输出
    vfhs = pcl.PointCloud.VFHSignature308()

    # 计算特征
    vfh.compute(vfhs)

    print(vfhs.size())  # pfhs的size应该为1

    # # 使用plotter显示直方图 注意这里输入的数据是原始数据，而不是计算出来的直方图
    # plotter = pcl.visualization.PCLPlotter()
    #
    # data = [i for i in vfhs.points]
    # plotter.addHistogramData(data, 125, "Histogram")
    # plotter.setBackgroundColor(1, 1, 1)
    # # 显示结果
    # plotter.plot()

    # 使用matplotlib绘制直方图
    plt.plot(vfhs.histogram[0])
    plt.show()

