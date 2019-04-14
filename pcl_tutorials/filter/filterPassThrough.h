#pragma once
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"

void getPassthroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtercloud)
{
	pcl::PassThrough<pcl::PointXYZ>::Ptr pt(new pcl::PassThrough<pcl::PointXYZ>);
	pt->setFilterFieldName("z");
	pt->setFilterLimits(0.6, 0.9);
	pt->setInputCloud(cloud);
	pt->setNegative(false);
	pt->filter(*filtercloud);

}