# -*- coding: utf-8 -*-
# @Time : DATE:2021/10/7
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 05_region_growing_rgb_segmentation.py


import pclpy
from pclpy import pcl
import numpy as np

if __name__ == '__main__':
    cloud = pcl.PointCloud.PointXYZRGB()
    if pcl.io.loadPCDFile('../../data/region_growing_rgb_tutorial.pcd', cloud) == -1:
        print('Cloud reading failed.')
        exit(-1)

    indices = pcl.vectors.Int()
    ps = pcl.filters.PassThrough.PointXYZRGB()
    ps.setInputCloud(cloud)
    ps.setFilterFieldName('z')
    ps.setFilterLimits(0.0, 1.0)
    ps.filter(indices)

    # reg = pcl.segmentation.RegionGrowingRGB()
    