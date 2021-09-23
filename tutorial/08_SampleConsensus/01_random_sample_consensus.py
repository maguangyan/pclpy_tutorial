# -*- coding: utf-8 -*-
# @Time : DATE:2021/9/23
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 01_random_sample_consensus.py

import pclpy
from pclpy import pcl
import numpy as np
import sys


def simpleVis(cloud):
    # Open 3D viewer and add point cloud
    viewer = pcl.visualization.PCLVisualizer()
    viewer.setBackgroundColor(0, 0, 0)
    viewer.addPointCloud(cloud, 'sample cloud')
    viewer.setPointCloudRenderingProperties(0, 3, 'sample cloud')
    # viewer.addCoordinateSystem(1.0, 'global')
    viewer.initCameraParameters()
    return viewer


def printUsage(program):
    print('Usage:', program, ' [options]\n',
          "Options:\n",
          "-------------------------------------------\n",
          "-p          显示带异常值的平面点云\n",
          "-s          显示带异常值的球形点云\n",
          "-pf         显示根据平面模型的随机采样一致性得到的平面点云\n",
          "-sf         显示根据球形模型的随机采样一致性得到的球形点云\n",
          )


if __name__ == '__main__':
    # 生成点云数据
    cloud = pcl.PointCloud.PointXYZ()
    final = pcl.PointCloud.PointXYZ()
    size = 500

    point = pcl.point_types.PointXYZ()

    if len(sys.argv) == 1:
        printUsage(sys.argv[0])
        exit()
    elif sys.argv[1] == '-p' or sys.argv[1] == '-pf':
        for i in range(size):
            point.x = np.random.rand() * 1024
            point.y = np.random.rand() * 1024
            if i % 5 == 0:
                point.z = np.random.rand() * 1024  # 1/5的外点
            else:
                point.z = -(point.x + point.y)  # 剩下的是平面z=-(x+y)上的点
            cloud.push_back(point)
    elif sys.argv[1] == '-s' or sys.argv[1] == '-sf':
        for i in range(size):
            point.x = np.random.rand() * 1024
            point.y = np.random.rand() * 1024
            if i % 5 == 0:
                point.z = np.random.rand() * 1024  # 1/5的外点
            elif i % 2 == 0:
                point.z = np.sqrt(np.fabs(1024 ** 2 - point.x ** 2 - point.y ** 2))  # 剩下的是圆x^2+y^2+z^2=1024^2上的点
            else:
                point.z = -np.sqrt(np.fabs(1024 ** 2 - point.x ** 2 - point.y ** 2))
            cloud.push_back(point)
    else:
        printUsage(sys.argv[0])
        exit()
    inliers = pcl.vectors.Int()

    # 创建随机采样一致性实例并计算估计模型
    model_s = pcl.sample_consensus.SampleConsensusModelSphere.PointXYZ(cloud)
    model_p = pcl.sample_consensus.SampleConsensusModelPlane.PointXYZ(cloud)

    if sys.argv[1] == '-pf':
        ransac = pcl.sample_consensus.RandomSampleConsensus.PointXYZ(model_p)
        ransac.setDistanceThreshold(0.01)
        ransac.computeModel()
        ransac.getInliers(inliers)
    elif sys.argv[1] == '-sf':
        ransac = pcl.sample_consensus.RandomSampleConsensus.PointXYZ(model_s)
        ransac.setDistanceThreshold(0.01)
        ransac.computeModel()
        ransac.getInliers(inliers)

    # 将所有通过模型计算的内点复制到另外一个点云 pclpy中未完成copyPointCloud函数
    extract = pcl.filters.ExtractIndices.PointXYZ()
    extract.setInputCloud(cloud)
    extract.setIndices(inliers)
    extract.setNegative(False)
    extract.filter(final)
    viewer = None
    if sys.argv[1] == '-p' or sys.argv[1] == '-s':
        viewer = simpleVis(cloud)
    elif sys.argv[1] == '-pf' or sys.argv[1] == '-sf':
        viewer = simpleVis(final)
    while not viewer.wasStopped():
        viewer.spinOnce(10)
