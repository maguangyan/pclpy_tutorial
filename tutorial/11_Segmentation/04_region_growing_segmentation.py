# -*- coding: utf-8 -*-
# @Time : DATE:2021/10/6
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 04_region_growing_segmentation.py

import pclpy
from pclpy import pcl
import numpy as np

if __name__ == '__main__':
    cloud = pcl.PointCloud.PointXYZ()
    if pcl.io.loadPCDFile('../../data/region_growing_tutorial.pcd', cloud) == -1:
        print('Cloud reading failed.')
        exit(-1)

    tree = pcl.search.KdTree.PointXYZ()
    normals = pcl.PointCloud.Normal()
    ne = pcl.features.NormalEstimation.PointXYZ_Normal()
    ne.setInputCloud(cloud)
    ne.setSearchMethod(tree)
    ne.setKSearch(50)
    ne.compute(normals)

    indices = pcl.vectors.Int()
    ps = pcl.filters.PassThrough.PointXYZ()
    ps.setInputCloud(cloud)
    ps.setFilterFieldName('z')
    ps.setFilterLimits(13000.0, 15000.0)
    ps.filter(indices)

    reg = pcl.segmentation.RegionGrowing.PointXYZ_Normal()
    reg.setInputCloud(cloud)
    reg.setMinClusterSize(50)
    reg.setMaxClusterSize(1000000)
    reg.setSearchMethod(tree)
    reg.setNumberOfNeighbours(30)
    # reg.setIndices(indices)
    reg.setInputNormals(normals)
    reg.setSmoothnessThreshold(3.0 / 180.0 * np.pi)
    reg.setCurvatureThreshold(1.0)

    clusters = pcl.vectors.PointIndices()
    reg.extract(clusters)

    print('Number of clusters is equal to ', len(clusters))
    print('First cluster has ', len(clusters[0].indices), ' points.')
    print('These are the indices of the points of the initial')
    print('cloud that belong to the first cluster:')
    counter = 0
    while counter < len(clusters[0].indices):
        print(clusters[0].indices[counter], ' ', end='')
        counter = counter + 1
        if counter % 10 == 0:
            print('')
    colored_cloud = reg.getColoredCloud()
    viewer = pcl.visualization.CloudViewer('Cluster viewer')
    viewer.showCloud(colored_cloud)
    while not viewer.wasStopped(10):
        pass
