# -*- coding: utf-8 -*-
# @Time : DATE:2021/9/22
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 06_remove_outliers.py

import pclpy
from pclpy import pcl
import numpy as np


def compareCloudShow(cloud, cloud_filtered):
    # Open 3D viewer and add point cloud and normals
    viewer = pcl.visualization.PCLVisualizer("viewer")
    v0 = 1
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v0)
    viewer.setBackgroundColor(0.0, 0.0, 0.0, v0)
    single_color = pcl.visualization.PointCloudColorHandlerCustom.PointXYZ(cloud, 0.0, 255.0, 0.0)
    viewer.addPointCloud(cloud, single_color, "sample cloud1", v0)

    v1 = 2
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v1)
    viewer.setBackgroundColor(0.0, 0.0, 0.0, v1)
    single_color = pcl.visualization.PointCloudColorHandlerCustom.PointXYZ(cloud_filtered, 0.0, 255.0, 0.0)
    viewer.addPointCloud(cloud_filtered, single_color, "sample cloud2", v1)

    viewer.setPointCloudRenderingProperties(0, 1, "sample cloud1", v0)
    viewer.setPointCloudRenderingProperties(0, 1, "sample cloud2", v1)
    viewer.addCoordinateSystem(1.0)
    while not viewer.wasStopped():
        viewer.spinOnce(10)


if __name__ == '__main__':
    # 读取点云数据
    cloud = pcl.PointCloud.PointXYZ()
    cloud_filtered = pcl.PointCloud.PointXYZ()
    cloud_p = pcl.PointCloud.PointXYZ()
    cloud_f = pcl.PointCloud.PointXYZ()
    reader = pcl.io.PCDReader()
    reader.read('../../data/table_scene_lms400.pcd', cloud)
    print("PointCloud before filtering: ",
          cloud.width * cloud.height, " data points ")

    # 创建ror滤波器
    ror = pcl.filters.RadiusOutlierRemoval.PointXYZ()
    ror.setInputCloud(cloud)
    ror.setRadiusSearch(0.01)
    ror.setMinNeighborsInRadius(10)
    ror.setKeepOrganized(True)
    ror.filter(cloud_filtered)

    # 可视化滤波效果
    compareCloudShow(cloud, cloud_filtered)




