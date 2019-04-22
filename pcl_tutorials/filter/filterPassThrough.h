#pragma once
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/console/time.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/radius_outlier_removal.h"


/***********************双边滤波******************************************/
#include "pcl/filters/bilateral.h"
#include "pcl/filters/fast_bilateral.h"
#include "pcl/filters/fast_bilateral_omp.h"
#include "pcl/surface/bilateral_upsampling.h"
/************************************************************************/

#include "pcl/console/time.h"

void getPassthroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtercloud)
{
	pcl::PassThrough<pcl::PointXYZ>::Ptr pt(new pcl::PassThrough<pcl::PointXYZ>);
	pt->setFilterFieldName("z");
	pt->setFilterLimits(0.6, 0.9);
	pt->setInputCloud(cloud);
	pt->setNegative(false);
	pt->filter(*filtercloud);

}

void getVoxelGridFilter()
{


}

void getStatisticalOutlierRemovalFilter()
{


}

void getBilateralFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &filter)
{
	/*
	void 	applyFilter (PointCloud &output)
		Filter the input data and store the results into output. More...

	double 	computePointWeight (const int pid, const std::vector< int > &indices, const std::vector< float > &distances)
		Compute the intensity average for a single point. More...

	void 	setHalfSize (const double sigma_s)
		Set the half size of the Gaussian bilateral filter window. More...

	void 	setStdDev (const double sigma_r)
		Set the standard deviation parameter. More...

	void 	setSearchMethod (const KdTreePtr &tree)
		Provide a pointer to the search object. More...
	
	*/
//	pcl::BilateralFilter<pcl::PointXYZ>::Ptr bf(new pcl::BilateralFilter<pcl::PointXYZ>);

	/*
	void 	setSigmaS (float sigma_s)
		Set the standard deviation of the Gaussian used by the bilateral filter for the spatial neighborhood/window. More...

	void 	setSigmaR (float sigma_r)
		Set the standard deviation of the Gaussian used to control how much an adjacent pixel is downweighted because of the intensity difference (depth in our case). More...

	virtual void 	applyFilter (PointCloud &output)
		Filter the input data and store the results into output. More...
	
	*/
	pcl::FastBilateralFilter<pcl::PointXYZ>::Ptr fbf(new pcl::FastBilateralFilter<pcl::PointXYZ>);
	pcl::FastBilateralFilterOMP<pcl::PointXYZ>::Ptr fbfOmp(new pcl::FastBilateralFilterOMP<pcl::PointXYZ>);
	fbfOmp->setNumberOfThreads(10);
	fbfOmp->setSigmaR(0.1);
	fbfOmp->setSigmaS(0.5);		//参数多少合适？
	fbfOmp->setInputCloud(cloud);
	pcl::console::TicToc time; time.tic();
	fbfOmp->filter(*filter);
	std::cout << "getNormal Function Time: " << time.toc() / 1000 << "s" << std::endl;



}

// 双边滤波 下采样
void getBilateralUpsampling(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, 
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_upsampled)
{
	/*
	template <typename PointInT, typename PointOutT>
	class BilateralUpsampling: public CloudSurfaceProcessing<PointInT, PointOutT>
	*/
	pcl::BilateralUpsampling<pcl::PointXYZRGBA, pcl::PointXYZRGBA> bu;
	bu.setInputCloud(cloud);
	//bu.setWindowSize(window_size);
	//bu.setSigmaColor(sigma_color);
	//bu.setSigmaDepth(sigma_depth);

	// TODO need to fix this somehow
	bu.setProjectionMatrix(bu.KinectSXGAProjectionMatrix);

	bu.process(*cloud_upsampled);

}