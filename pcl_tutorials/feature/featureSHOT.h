#pragma once

/*
	描述能力很好
*/
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl\features\shot.h"
#include "pcl\features\shot_omp.h"
#include "pcl\features\shot_lrf.h"
#include "pcl\features\shot_lrf_omp.h"


void getSHOT()
{
	/*
	template <typename PointInT, typename PointNT, typename PointOutT = pcl::SHOT352, typename PointRFT = pcl::ReferenceFrame>
	class SHOTEstimation : public SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>
	*/
	pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal>::Ptr shot(new pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal>);


}