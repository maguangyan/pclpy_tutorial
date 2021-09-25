# -*- coding: utf-8 -*-
# @Time : DATE:2021/9/25
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 05_PHF_descriptor.py


import pclpy
from pclpy import pcl
import numpy as np
import sys

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
    cloud_normals = pcl.PointCloud.PointNormal().from_array(np.hstack((cloud.xyz, normals.normals, normals.curvature.reshape(-1, 1))))
    for i in range(cloud_normals.size()):
        if not pcl.common.isFinite(cloud_normals.at(i)):
            print('cloud_normals[%d] is not finite\n', i)

    # 构造PFH estimation类，把cloud和normals传递进去
    pfh = pcl.features.PFHEstimation.PointXYZ_Normal_PFHSignature125()
    pfh.setInputCloud(cloud)
    pfh.setInputNormals(normals)
    # 或者，如果cloud是PointNormal类型，执行pfh.setInputNormals(cloud);

    # 构造一个kd树
    # 它的内容将根据给定的输入点云填充到对象内部(因为没有给出其他搜索面)。
    tree = pcl.search.KdTree.PointXYZ()
    pfh.setSearchMethod(tree)

    # 输出
    pfhs = pcl.PointCloud.PFHSignature125()

    # 使用5cmm球形范围内的邻居点
    # 注意：在这里使用的半径必须大于用来估计表面法线的半径!!
    pfh.setRadiusSearch(0.05)

    # 计算特征
    pfh.compute(pfhs)

    print(pfhs.size())  # pfhs与cloud size应该相同



