# 点云压缩

> pclpy未绑定pcl/compression/octree_pointcloud_compression类 ，点云压缩这一块儿，pclpy的支持不是很好，而且需要 openNI 设备，暂无法实现。感兴趣的同学可参考[官方文档](https://pcl.readthedocs.io/projects/tutorials/en/latest/compression.html#octree-compression) 



# 八叉树的空间分区和搜索操作

八叉树是一种基于树的数据结构，用于管理稀疏 3-D 数据。每个内部节点正好有八个子节点。在本教程中，我们将学习如何使用八叉树在点云数据中进行空间分区和邻居搜索。特别地，我们解释了如何执行“体素内的邻居搜索”、“K 最近邻搜索”和“半径内的邻居搜索”。

## 代码

见02_octree_search.py

```python
import pclpy
from pclpy import pcl
import numpy as np

if __name__ == '__main__':
    # 生成点云数据
    cloud_size = 1000
    a = np.random.ranf(cloud_size * 3).reshape(-1, 3) * 1024
    cloud = pcl.PointCloud.PointXYZ.from_array(a)

    resolution = 128.0

    octree = pcl.octree.OctreePointCloudSearch.PointXYZ(resolution)
    octree.setInputCloud(cloud)
    octree.addPointsFromInputCloud()

    searchPoint = pcl.point_types.PointXYZ()
    searchPoint.x = np.random.ranf(1) * 1024
    searchPoint.y = np.random.ranf(1) * 1024
    searchPoint.z = np.random.ranf(1) * 1024

    # 体素最近邻搜索
    pointIdxVec = pclpy.pcl.vectors.Int()

    if octree.voxelSearch(searchPoint, pointIdxVec) > 0:
        print('Neighbors within voxel search at (', searchPoint.x,
              '', searchPoint.y,
              '', searchPoint.z, ')\n')
        for i in range(len(pointIdxVec)):
            print("  ", cloud.x[pointIdxVec[i]],
                  " ", cloud.y[pointIdxVec[i]],
                  " ", cloud.z[pointIdxVec[i]], "\n")

    # k最近邻搜索
    k = 10
    pointIdxNKNSearch = pclpy.pcl.vectors.Int()
    pointNKNSquaredDistance = pclpy.pcl.vectors.Float()

    print('K nearest neighbor search at (', searchPoint.x,
          '', searchPoint.y,
          '', searchPoint.z,
          ') with k =', k, '\n')
    if octree.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0:
        for i in range(len(pointIdxNKNSearch)):
            print("  ", cloud.x[pointIdxNKNSearch[i]],
                  " ", cloud.y[pointIdxNKNSearch[i]],
                  " ", cloud.z[pointIdxNKNSearch[i]],
                  " (squared distance: ", pointNKNSquaredDistance[i], ")", "\n")

    # 半径最近邻搜索
    pointIdxRadiusSearch = pclpy.pcl.vectors.Int()
    pointRadiusSquaredDistance = pclpy.pcl.vectors.Float()

    radius = np.random.ranf(1) * 256.0
    print("Neighbors within radius search at (", searchPoint.x,
          " ", searchPoint.y, " ", searchPoint.z, ") with radius=",
          radius, '\n')
    if octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0:
        for i in range(len(pointIdxRadiusSearch)):
            print("  ", cloud.x[pointIdxRadiusSearch[i]],
                  " ", cloud.y[pointIdxRadiusSearch[i]],
                  " ", cloud.z[pointIdxRadiusSearch[i]],
                  " (squared distance: ", pointRadiusSquaredDistance[i], ")", "\n")
```

## 说明

首先定义并实例化一个共享的 PointCloud 结构，并用随机点填充它。

```python
# 生成点云数据
cloud_size = 1000
a = np.random.ranf(cloud_size * 3).reshape(-1, 3) * 1024
cloud = pcl.PointCloud.PointXYZ.from_array(a)
```

然后我们创建一个八叉树实例，用它的分辨率初始化。该八叉树在其叶节点内保留了一个点索引向量。分辨率参数描述了最低八叉树级别的最小体素的长度。因此，八叉树的深度是点云的分辨率和空间维度的函数。如果知道点云的边界框，则应使用defineBoundingBox 方法将其分配给八叉树。然后我们分配一个指向 PointCloud 的指针并将所有点添加到八叉树。

```python
resolution = 128.0

octree = pcl.octree.OctreePointCloudSearch.PointXYZ(resolution)
octree.setInputCloud(cloud)
octree.addPointsFromInputCloud()
```

一旦 PointCloud 与八叉树相关联，我们就可以执行搜索操作。这里使用的第一种搜索方法是“体素内的邻居搜索”。它将搜索点分配给相应的叶节点体素并返回点索引向量。这些指数与落在同一体素内的点有关。因此，搜索点和搜索结果之间的距离取决于八叉树的分辨率参数。

```python
# 体素最近邻搜索
pointIdxVec = pclpy.pcl.vectors.Int()

if octree.voxelSearch(searchPoint, pointIdxVec) > 0:
    print('Neighbors within voxel search at (', searchPoint.x,
          '', searchPoint.y,
          '', searchPoint.z, ')\n')
    for i in range(len(pointIdxVec)):
        print("  ", cloud.x[pointIdxVec[i]],
              " ", cloud.y[pointIdxVec[i]],
              " ", cloud.z[pointIdxVec[i]], "\n")
```

接下来，演示 K 最近邻搜索。在此示例中，K 设置为 10。“K 最近邻搜索”方法将搜索结果写入两个单独的向量。第一个 pointIdxNKNSearch 将包含搜索结果（索引引用关联的 PointCloud 数据集）。第二个向量保存搜索点和最近邻居之间的相应平方距离。

```python
# k最近邻搜索
k = 10
pointIdxNKNSearch = pclpy.pcl.vectors.Int()
pointNKNSquaredDistance = pclpy.pcl.vectors.Float()

print('K nearest neighbor search at (', searchPoint.x,
      '', searchPoint.y,
      '', searchPoint.z,
      ') with k =', k, '\n')
if octree.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0:
    for i in range(len(pointIdxNKNSearch)):
        print("  ", cloud.x[pointIdxNKNSearch[i]],
              " ", cloud.y[pointIdxNKNSearch[i]],
              " ", cloud.z[pointIdxNKNSearch[i]],
              " (squared distance: ", pointNKNSquaredDistance[i], ")", "\n")
```

“半径搜索中的邻居”的工作方式与“K 最近邻搜索”非常相似。它的搜索结果被写入两个单独的向量，描述点索引和平方搜索点距离。

```python
# 半径最近邻搜索
pointIdxRadiusSearch = pclpy.pcl.vectors.Int()
pointRadiusSquaredDistance = pclpy.pcl.vectors.Float()

radius = np.random.ranf(1) * 256.0
print("Neighbors within radius search at (", searchPoint.x,
      " ", searchPoint.y, " ", searchPoint.z, ") with radius=",
      radius, '\n')
if octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0:
    for i in range(len(pointIdxRadiusSearch)):
        print("  ", cloud.x[pointIdxRadiusSearch[i]],
              " ", cloud.y[pointIdxRadiusSearch[i]],
              " ", cloud.z[pointIdxRadiusSearch[i]],
              " (squared distance: ", pointRadiusSquaredDistance[i], ")", "\n")
```

## 运行

运行02_octree_search.py

运行结果：

> Neighbors within voxel search at ( 235.9908905029297  550.4008178710938  457.9418029785156 )
>
>    284.7338   645.86816   439.52136 
>
> K nearest neighbor search at ( 235.9908905029297  550.4008178710938  457.9418029785156 ) with k = 10 
>
>    270.4592   528.7933   455.80542  (squared distance:  1659.5142822265625 ) 
>
>    179.18915   504.20984   424.5981  (squared distance:  6471.84619140625 ) 
>
>    219.09846   518.7366   371.57288  (squared distance:  8747.5703125 ) 
>
>    282.65546   499.14508   375.4459  (squared distance:  11610.3076171875 ) 
>
>    284.7338   645.86816   439.52136  (squared distance:  11829.1982421875 ) 
>
>    152.6541   517.1704   523.69183  (squared distance:  12372.34765625 ) 
>
>    222.60712   437.74167   441.51077  (squared distance:  13141.1875 ) 
>
>    283.5196   504.1384   557.749  (squared distance:  14360.6708984375 ) 
>
>    236.29802   493.2118   336.7504  (squared distance:  17958.03515625 ) 
>
>    213.21176   569.0312   589.7377  (squared distance:  18236.12890625 ) 
>
> Neighbors within radius search at ( 235.9908905029297   550.4008178710938   457.9418029785156 ) with radius= [108.47992978] 
>
>    179.18915   504.20984   424.5981  (squared distance:  6471.84619140625 ) 
>
>    219.09846   518.7366   371.57288  (squared distance:  8747.5703125 ) 
>
>    282.65546   499.14508   375.4459  (squared distance:  11610.3076171875 ) 
>
>    270.4592   528.7933   455.80542  (squared distance:  1659.5142822265625 ) 

注意：由于我们的数据是随生成的，每次结果都不一样，甚至有时候可能KdTree 返回 0 最近邻，这时候就没有输出了。可以适当增大cloud_size，效果会更明显。

## 其他

PCL 八叉树组件提供了几种八叉树类型。它们的基本区别在于它们各自的叶节点特征。

- OctreePointCloudPointVector（等于 OctreePointCloud）：这个八叉树可以在每个叶节点上保存一个点索引列表。
- OctreePointCloudSinglePoint：这个八叉树类在每个叶节点上只保存一个点索引。仅存储分配给叶节点的最新点索引。
- OctreePointCloudOccupancy：这个八叉树在其叶节点不存储任何点信息。它可用于空间占用检查。
- OctreePointCloudDensity：这个八叉树计算每个叶节点体素内的点数。它允许空间密度查询。

如果需要高速创建八叉树，请查看八叉树双缓冲实现（Octree2BufBase 类）。这个类同时在内存中保持两个并行的八叉树结构。除了搜索操作外，这还可以实现空间变化检测。此外，先进的内存管理减少了八叉树构建过程中的内存分配和释放操作。双缓冲八叉树实现可以通过模板参数“OctreeT”分配给所有 OctreePointCloud 类。

所有八叉树都支持八叉树结构和八叉树数据内容的序列化和反序列化。

## 总结

PCL 八叉树实现是空间分区和搜索操作的强大工具。

# 无组织点云数据的空间变化检测

八叉树是一种用于组织稀疏 3-D 数据的基于树的数据结构。在本教程中，我们将学习如何使用八叉树实现来检测多个无组织点云之间的空间变化，这些点云的大小、分辨率、密度和点排序可能会有所不同。通过递归比较八叉树的树结构，可以识别由体素配置差异表示的空间变化。此外，我们解释了如何使用 pcl 八叉树“双缓冲”技术使我们能够随着时间的推移有效地处理多个点云。

## 代码

> pclpy未实现pcl::octree::OctreePointCloudChangeDetector类，暂无法实现

