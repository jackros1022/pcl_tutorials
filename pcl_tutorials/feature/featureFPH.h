#pragma once
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>
#include "../keypoint/KeypointSIFT.h"

pcl::PointCloud<pcl::PFHSignature125>::Ptr getPFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	

}