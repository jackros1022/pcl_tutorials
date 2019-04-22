#pragma once
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/segmentation/region_growing.h"

void getSAC_Segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointIndices::Ptr &inliers)
{
	/************************************************************************/
	/*  SACSegmentation                                                     */
	/************************************************************************/
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	//pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);
	seg.setInputCloud(cloud);
	pcl::console::TicToc time;
	time.tic();
	seg.segment(*inliers, *coefficients);

	cout << "SACSegmentation[MethodType:SAC_RANSAC] time: " << time.toc() / 1000 << "s" << endl;
}

void getSAC_SegmentationFromNormals() 
{
	/*
	template <typename PointT, typename PointNT>
	class SACSegmentationFromNormals: public SACSegmentation<PointT>
	
	*/
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;



}