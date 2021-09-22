# -*- coding: utf-8 -*-
# @Time : DATE:2021/9/22
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 02_voxel_grid.py

import pclpy
from pclpy import pcl
import numpy as np

if __name__ == '__main__':
    # 读取点云数据
    cloud = pcl.PointCloud.PointXYZ()
    cloud_filtered = pcl.PointCloud.PointXYZ()
    reader = pcl.io.PCDReader()
    reader.read('../../data/table_scene_lms400.pcd', cloud)
    print("PointCloud before filtering: ",
          cloud.width * cloud.height, " data points ")

    # 创建滤波器
    vox = pcl.filters.VoxelGrid.PointXYZ()
    vox.setInputCloud(cloud)
    vox.setLeafSize(0.01, 0.01, 0.01)
    vox.filter(cloud_filtered)
    print("PointCloud before filtering: ",
          cloud_filtered.width * cloud_filtered.height, " data points ")

    writer = pcl.io.PCDWriter()
    writer.write("table_scene_lms400_downsampled.pcd", cloud_filtered)
