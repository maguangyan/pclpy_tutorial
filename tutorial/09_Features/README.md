## PCL中的3D特征
见[3DFeaturesWork.py](./3DFeaturesWork.py)

这个文档介绍了PCL中的3D特征估计方法，并作为对pcl::Feature类内部结构感兴趣的用户或开发人员的指南。

* 为输入数据集中的所有点估计一组表面法线
```python
# 估计法线
# 输入数据集中的所有点估计一组表面法线
ne = pcl.features.NormalEstimation.PointXYZ_Normal()
ne.setInputCloud(cloud)
tree = pcl.search.KdTree.PointXYZ()
ne.setSearchMethod(tree)

cloud_normals = pcl.PointCloud.Normal()
ne.setRadiusSearch(0.003)
ne.compute(cloud_normals)
print(cloud_normals.size())
```
* 为输入数据集中的点子集估计一组表面法线。
```python
ne = pcl.features.NormalEstimation.PointXYZ_Normal()
ne.setInputCloud(cloud)
tree = pcl.search.KdTree.PointXYZ()
ne.setSearchMethod(tree)
ind = pcl.PointIndices()
[ind.indices.append(i) for i in range(0, cloud.size() // 2)]
ne.setIndices(ind)
cloud_normals = pcl.PointCloud.Normal()
ne.setRadiusSearch(0.003)
ne.compute(cloud_normals)
print(cloud_normals.size())
```
* 为输入数据集中的所有点估计一组表面法线，但使用另一个数据集估计它们的最近邻

    在这里，我们使用点云的下采样点云作为输入点云，使用原点云作为SearchSurface。
```python
ne = pcl.features.NormalEstimation.PointXYZ_Normal()
cloud_downsampled = pcl.PointCloud.PointXYZ()  # 获取一个降采样的点云 方法比较多，这里使用voxelized方法
vox = pcl.filters.VoxelGrid.PointXYZ()
vox.setInputCloud(cloud)
vox.setLeafSize(0.005, 0.005, 0.005)
vox.filter(cloud_downsampled)
ne.setInputCloud(cloud_downsampled)
ne.setSearchSurface(cloud)
tree = pcl.search.KdTree.PointXYZ()
ne.setSearchMethod(tree)
cloud_normals = pcl.PointCloud.Normal()
ne.setRadiusSearch(0.003)
ne.compute(cloud_normals)
print(cloud_normals.size())
```

下图分别为原始电云和下采样点云。

![image-20210919133518668](README.assets/image-20210919133518668.png)


> 详细内容可参考[how-3d-features-work](https://pcl.readthedocs.io/projects/tutorials/en/latest/how_features_work.html#how-3d-features-work)，本文档不再赘述。

## 使用积分图像的法线估计
见[normal_estimation_using_integral_images.py](./normal_estimation_using_integral_images.py)

* 使用积分图像的法线估计
```python
# 估计法线
normals = pcl.PointCloud.Normal()
ne = pcl.features.IntegralImageNormalEstimation.PointXYZ_Normal()
ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT)
ne.setMaxDepthChangeFactor(0.02)
ne.setNormalSmoothingSize(10.0)
ne.setInputCloud(cloud)
ne.compute(normals)
```

在可视化法线时，遇到了一下错误：
````
In..\Rendering\Core\vtkActor.cxx, line 43. Error: no override found for vtkactor
````
查看了pclpy的官方Issue，发现有人提过这个Issue，这个bug还未修复
> issue:https://github.com/davidcaron/pclpy/issues/58

感觉pclpy在可视化这一块儿bug比较多（目前已发现的有：无法可视化mesh、带法线的点云），在python中可视化功能可使用pyvista代替。事实上，作者可能也意识到了这个问题，在最新的pclpy v1.12.0移除了可视化模块。



