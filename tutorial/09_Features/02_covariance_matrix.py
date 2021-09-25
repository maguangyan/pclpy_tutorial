# -*- coding: utf-8 -*-
# @Time : DATE:2021/9/24
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 02_covariance_matrix.py

import pclpy
from pclpy import pcl
import numpy as np
import sys

if __name__ == '__main__':
    # 生成点云数据
    cloud = pcl.PointCloud.PointXYZ.from_array([[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 1]])
    # covariance_matrix = np.zeros((3, 3), dtype=np.float64)
    # xyz_centorid = np.zeros((4, 1), dtype=np.float32)
    # pcl.common.compute3DCentroid(cloud, xyz_centorid)
    # pcl.common.computeCovarianceMatrix(cloud, xyz_centorid, covariance_matrix)

    # compute3DCentroid和computeCovarianceMatrix函数有些问题，下面使用numpy计算质心和协方差
    xyz_centorid = np.mean(cloud.xyz, axis=0)
    xyz_centorid = np.hstack((xyz_centorid, 1)).reshape(4, 1)
    print(xyz_centorid)
    covariance_matrix = np.cov(cloud.xyz.T)
    print(covariance_matrix)