# -*- coding: utf-8 -*-
# @Time : DATE:2021/9/25
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 10_GRSD_descriptors.py

import pclpy
from pclpy import pcl
import numpy as np
import sys
import matplotlib.pyplot as plt
import pyvista as pv


if __name__ == '__main__':
    # 加载点云
    cloud = pcl.PointCloud.PointXYZ()
    reader = pcl.io.PCDReader()
    reader.read("../../data/min_cut_segmentation_tutorial.pcd", cloud)

    # 构造法线估计类
    ne = pcl.features.NormalEstimation.PointXYZ_Normal()
    ne.setInputCloud(cloud)

    tree = pcl.search.KdTree.PointXYZ()
    ne.setSearchMethod(tree)

    cloud_normals = pcl.PointCloud.Normal()
    ne.setRadiusSearch(0.3)

    # 计算法线
    ne.compute(cloud_normals)

    # Create the GASD estimation class, and pass the input dataset to it
    grsd = pcl.features.GRSDEstimation.PointXYZ_Normal_GRSDSignature21()

    grsd.setInputCloud(cloud)
    grsd.setRadiusSearch(0.1)
    tree = pcl.search.KdTree.PointXYZ()
    grsd.setSearchMethod(tree)
    grsd.setInputNormals(cloud_normals)

    descriptors = pcl.PointCloud.GRSDSignature21()
    grsd.compute(descriptors)

    plt.plot(descriptors.histogram[0])
    plt.show()

    viewer = pcl.visualization.PCLVisualizer("3D viewer")
    viewer.setBackgroundColor(0, 0, 0)
    rgb = pcl.visualization.PointCloudColorHandlerCustom.PointXYZ(cloud, 0.0, 255.0, 0.0)

    viewer.addPointCloud(cloud, rgb, "sample cloud")
    viewer.setPointCloudRenderingProperties(0, 1, "sample cloud")
    viewer.addCoordinateSystem(1)
    viewer.initCameraParameters()

    while not viewer.wasStopped():
        viewer.spinOnce(10)