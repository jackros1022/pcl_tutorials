#pragma once
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/extract_indices.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr getExtract_Indices(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
														pcl::PointIndices::Ptr& inliers,
														bool isNegative=true)
{

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_extract(new pcl::PointCloud<pcl::PointXYZ>);
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(isNegative);
	extract.filter(*cloud_extract);
	return cloud_extract;
}