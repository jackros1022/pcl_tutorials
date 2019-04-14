#pragma once
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/keypoints/uniform_sampling.h"

void getUniformSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint, double radius=0.05)
{
	
	/*
	ÌØÕ÷¼ì²âÆ÷
	template <typename PointT>
	class UniformSampling: public Filter<PointT>

	·½·¨
	virtual void 	setRadiusSearch (double radius)
			Set the 3D grid leaf size. More...

	*/

	pcl::UniformSampling<pcl::PointXYZ>::Ptr us(new pcl::UniformSampling<pcl::PointXYZ>);
	pcl::IndicesPtr indices;

	us->setInputCloud(cloud);
	us->setRadiusSearch(radius);
	us->filter(*keypoint);

}