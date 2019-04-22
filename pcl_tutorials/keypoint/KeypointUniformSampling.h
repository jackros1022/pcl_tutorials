#pragma once
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/keypoints/uniform_sampling.h"

void getUniformSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint, double radius=0.01)
{
	
	/*
	特征检测器
	template <typename PointT>
	class UniformSampling: public Filter<PointT>

	方法
	virtual void 	setRadiusSearch (double radius)
			Set the 3D grid leaf size. More...

	*/

	pcl::UniformSampling<pcl::PointXYZ>::Ptr us(new pcl::UniformSampling<pcl::PointXYZ>);
	pcl::IndicesPtr indices;

	us->setInputCloud(cloud);
	us->setRadiusSearch(radius);	//0.01 代表 1cm
	pcl::console::TicToc time; time.tic();
	us->filter(*keypoint);
	std::cout << " UniformSampling Function Time: " << time.toc() / 1000 << "s" << std::endl;
	std::cout << "Before UniformSampling size:" << cloud->size() << std::endl;
	std::cout << " --> After UniformSampling size:" << keypoint->size() << std::endl;

}