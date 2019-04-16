#pragma once

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/features/rops_estimation.h"
#include "pcl/features/feature.h"

void getRoPS(pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint,
	pcl::PointCloud<pcl::Normal>::Ptr normal = NULL,
	pcl::PointCloud<pcl::PointXYZ>::Ptr surface = NULL)
{
	/*
  template <typename PointInT, typename PointOutT>
  class PCL_EXPORTS ROPSEstimation : public pcl::Feature <PointInT, PointOutT>

  方法：
  void 	setNumberOfPartitionBins (unsigned int number_of_bins)
  Allows to set the number of partition bins that is used for distribution matrix calculation. More...

  void 	setNumberOfRotations (unsigned int number_of_rotations)
  This method sets the number of rotations. More...

  void 	setSupportRadius (float support_radius)
  Allows to set the support radius that is used to crop the local surface of the point. More...

  void 	setTriangles (const std::vector< pcl::Vertices > &triangles)
  This method sets the triangles of the mesh. More...

	*/

	pcl::ROPSEstimation<pcl::PointXYZ, pcl::Histogram<135>> rops;
	
	pcl::PointCloud<pcl::Histogram <135> >::Ptr histograms(new 
		pcl::PointCloud <pcl::Histogram <135> >());


	float support_radius = 0.0285f;				//局部表面裁剪支持的半径 (搜索半价)，
	unsigned int number_of_partition_bins = 5;	//以及用于组成分布矩阵的容器的数量
	unsigned int number_of_rotations = 3;		//和旋转的次数。最后的参数将影响描述器的长度。

												//搜索方法
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(keypoint);
	// rops 特征算法 对象 盒子数量 × 旋转次数×9 得到特征维度  3*5*9 =135
	rops.setSearchMethod(tree);//搜索算法
	rops.setSearchSurface(surface);//搜索平面
	rops.setInputCloud(keypoint);//输入点云
	rops.setRadiusSearch(support_radius);//搜索半径
	rops.setTriangles(triangles);//领域形状
	rops.setNumberOfPartitionBins(number_of_partition_bins);//盒子数量
	rops.setNumberOfRotations(number_of_rotations);//旋转次数
	rops.setSupportRadius(support_radius);// 局部表面裁剪支持的半径 

	rops.compute(*histograms);
}