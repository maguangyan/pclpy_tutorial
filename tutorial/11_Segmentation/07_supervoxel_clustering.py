# -*- coding: utf-8 -*-
# @Time : DATE:2021/10/7
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 07_supervoxel_clustering.py

import pclpy
from pclpy import pcl
import numpy as np
import sys

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Syntax is: %s <pcd-file> \n" % sys.argv[0],
              "--NT Dsables the single cloud transform \n"
              "-v <voxel resolution>\n-s <seed resolution>\n"
              "-c <color weight> \n-z <spatial weight> \n"
              "-n <normal_weight>\n")
        exit()

    cloud = pcl.PointCloud.PointXYZRGBA()
    print('Loading point cloud...')
    if pcl.io.loadPCDFile(sys.argv[1], cloud) == -1:
        print('"Error loading cloud file!')
        exit(-1)

    disable_transform = '--NT' in sys.argv
    voxel_resolution = 0.008
    voxel_res_specified = '-v' in sys.argv
    if voxel_res_specified:
        index = sys.argv.index('-v') + 1
        voxel_resolution = float(sys.argv[index])

    seed_resolution = 0.1
    seed_res_specified = '-s' in sys.argv
    if seed_res_specified:
        index = sys.argv.index('-s') + 1
        seed_resolution = float(sys.argv[index])

    color_importance = 0.2
    if '-c' in sys.argv:
        index = sys.argv.index('-c') + 1
        color_importance = float(sys.argv[index])

    spatial_importance = 0.4
    if '-z' in sys.argv:
        index = sys.argv.index('-z') + 1
        spatial_importance = float(sys.argv[index])

    normal_importance = 1.0
    if '-n' in sys.argv:
        index = sys.argv.index('-n') + 1
        spatial_importance = float(sys.argv[index])

    # 使用supervoxels
    sv = pcl.segmentation.SupervoxelClustering.PointXYZRGBA(voxel_resolution, seed_resolution)
    if disable_transform:
        sv.setUseSingleCameraTransform(False)

    sv.setInputCloud(cloud)
    sv.setColorImportance(color_importance)
    sv.setSpatialImportance(spatial_importance)
    sv.setNormalImportance(normal_importance)

    supervoxel_clusters = pcl.vectors.map_uint32t_PointXYZRGBA()
    print('Extracting supervoxels!')
    sv.extract(supervoxel_clusters)
    print("Found %d supervoxels" % len(supervoxel_clusters))

    viewer = pcl.visualization.PCLVisualizer('3D Viewer')
    viewer.setBackgroundColor(0, 0, 0)
    voxel_centroid_cloud = sv.getVoxelCentroidCloud()
    viewer.addPointCloud(voxel_centroid_cloud, 'voxel centroids')
    viewer.setPointCloudRenderingProperties(0, 2.0, 'voxel centroids')
    viewer.setPointCloudRenderingProperties(1, 0.95, 'voxel centroids')

    labeled_voxel_cloud = sv.getLabeledVoxelCloud()
    viewer.addPointCloud(labeled_voxel_cloud, 'labeled voxels')
    viewer.setPointCloudRenderingProperties(1, 0.5, 'labeled voxels')

    sv_normal_cloud = sv.makeSupervoxelNormalCloud(supervoxel_clusters)
    # 我们注释了法线可视化，这样图形很容易看到，取消注释可以看到超体素法线
    # 事实上，pclpy可视化法线存在bug，我们先注释掉它。
    # viewer.addPointCloudNormals(sv_normal_cloud, 1, 0.05, "supervoxel_normals")

    print('Getting supervoxel adjacency')
    # supervoxel_adjacency = 0    # std::multimap<std::uint32_t, std::uint32_t>未绑定
    # sv.getSupervoxelAdjacency(supervoxel_adjacency)
    # 为了绘制超体素邻接图，我们需要遍历超体素邻接multimap

    while not viewer.wasStopped():
        viewer.spinOnce(10)
