# -*- coding: utf-8 -*-
# @Time : DATE:2021/10/7
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 08_bare_earth.py

import pclpy
from pclpy import pcl
import numpy as np

if __name__ == '__main__':
    cloud = pcl.PointCloud.PointXYZ()
    cloud_filtered = pcl.PointCloud.PointXYZ()
    ground = pcl.PointIndices()

    # 加载点云数据
    reader = pcl.io.PCDReader()
    reader.read('../../data/samp11-utm.pcd', cloud)
    print('Cloud before filtering:')
    print(cloud.size())

    # 开始分割
    pmf = pcl.segmentation.ProgressiveMorphologicalFilter.PointXYZ()
    pmf.setInputCloud(cloud)
    pmf.setMaxWindowSize(20)
    pmf.setSlope(1.0)
    pmf.setInitialDistance(0.5)
    pmf.setMaxDistance(3.0)
    pmf.extract(ground.indices)

    extract = pcl.filters.ExtractIndices.PointXYZ()
    extract.setInputCloud(cloud)
    extract.setIndices(ground)
    extract.filter(cloud_filtered)

    print('Object cloud after filtering:')
    print(cloud_filtered.size())

    # 保存地面点云
    writer = pcl.io.PCDWriter()
    writer.write('samp11-utm_ground.pcd', cloud_filtered)

    # 保存非地面点云
    cloud_filtered2= pcl.PointCloud.PointXYZ()
    extract.setNegative(True)
    extract.filter(cloud_filtered2)
    writer.write('samp11-utm_object.pcd', cloud_filtered2)

    # 可视化
    viewer = pcl.visualization.PCLVisualizer("viewer")
    viewer.setBackgroundColor(0.0, 0.0, 0.0)
    green = pcl.visualization.PointCloudColorHandlerCustom.PointXYZ(cloud_filtered, 0.0, 255.0, 0.0)
    viewer.addPointCloud(cloud_filtered, green, "ground")

    blue = pcl.visualization.PointCloudColorHandlerCustom.PointXYZ(cloud_filtered2, 0.0, 0.0, 255.0)
    viewer.addPointCloud(cloud_filtered2, blue, "object")

    viewer.setPointCloudRenderingProperties(0, 1, "ground")
    viewer.setPointCloudRenderingProperties(0, 1, "object")
    while not viewer.wasStopped():
        viewer.spinOnce(10)