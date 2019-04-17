#pragma once

/*
	描述能力很好，近邻搜索只能是半径

	Todo: 局部坐标系怎么使用？
*/
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl\features\shot.h"
#include "pcl\features\shot_omp.h"
#include "pcl\features\shot_lrf.h"
#include "pcl\features\shot_lrf_omp.h"

/************************************************************************/
//#include <Eigen/Core>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl/common/time.h>
//#include <pcl/common/common.h>
//#include <pcl/console/print.h>
//#include <pcl/features/shot_lrf.h>
//#include <pcl/filters/filter.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/io/pcd_io.h>
//
//#include <time.h>
//#include <fstream>
//
//// Types
//typedef pcl::PointXYZ PointT;
//typedef pcl::PointCloud<PointT> PointCloudT;
//typedef pcl::ReferenceFrame FeatureT;
//typedef pcl::SHOTLocalReferenceFrameEstimation<PointT, FeatureT> FeatureEstimationT;
//typedef pcl::PointCloud<FeatureT> FeatureCloudT;

// https://github.com/suneverust/TestLRF/blob/master/lrf.cpp
/************************************************************************/


pcl::PointCloud<pcl::SHOT352>::Ptr getSHOT(pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint,
											pcl::PointCloud<pcl::Normal>::Ptr normal = NULL,
											pcl::PointCloud<pcl::PointXYZ>::Ptr surface = NULL,
											double radius = 0.01)
{
	/*
	template <typename PointInT, typename PointNT, 
				typename PointOutT = pcl::SHOT352, typename PointRFT = pcl::ReferenceFrame>
	class SHOTEstimation : public SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>

	半径参数，很关键！
	*/
	pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal>::Ptr shot(new 
		pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal>);
	pcl::PointCloud<pcl::SHOT352>::Ptr shotDescriptor(new pcl::PointCloud<pcl::SHOT352>);
	shot->setRadiusSearch(radius);

	shot->setInputCloud(keypoint);		// 使用关键点计算特征描述符
	shot->setInputNormals(normal);		// 法线提前计算好
	shot->setSearchSurface(surface);				// 搜索的表面
	shot->compute(*shotDescriptor);			// 直接计算全局描述符,太耗时

	return shotDescriptor;
}

pcl::PointCloud<pcl::SHOT352>::Ptr getSHOTOMP(pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint,
												pcl::PointCloud<pcl::Normal>::Ptr normal = NULL,
												pcl::PointCloud<pcl::PointXYZ>::Ptr surface = NULL,
												double radius = 0.01) 
{
	/*
	template <typename PointInT, typename PointNT, typename PointOutT = pcl::SHOT352, typename PointRFT = pcl::ReferenceFrame>
	class SHOTEstimationOMP : public SHOTEstimation<PointInT, PointNT, PointOutT, PointRFT>
	
	*/
	pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal>::Ptr shot(new pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal>);
	pcl::PointCloud<pcl::SHOT352>::Ptr shotDescriptor(new pcl::PointCloud<pcl::SHOT352>);
	shot->setNumberOfThreads(10);
	shot->setRadiusSearch(radius);

	shot->setInputCloud(keypoint);
	shot->setInputNormals(normal);
	shot->setSearchSurface(surface);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	shot->setSearchMethod(tree);
	pcl::console::TicToc time; time.tic();
	shot->compute(*shotDescriptor);
	std::cout << " SHOTEstimationOMP Function Time: " << time.toc() / 1000 << "s" << std::endl;

	return shotDescriptor;

}



int getSHOT_lrf()
{
	/*

	// Initiate Point clouds
	PointCloudT::Ptr object(new PointCloudT);
	PointCloudT::Ptr object_downsample(new PointCloudT);
	FeatureCloudT::Ptr object_features(new FeatureCloudT);

	clock_t t_whole;

	// Get input data
	if (argc != 2)
	{
		pcl::console::print_error("Syntax is: %s object.pcd\n", argv[0]);
		return (1);
	}

	pcl::console::print_highlight("Loading point clouds...\n");

	if (pcl::io::loadPCDFile<PointT>(argv[1], *object) < 0)
	{
		pcl::console::print_error("Error loading object file!\n");
		return (1);
	}

	// Remove the nan points if any
	std::vector<int> indices_object_nan;
	pcl::removeNaNFromPointCloud(*object, *object, indices_object_nan);

	t_whole = clock();

	// Downsample
	pcl::console::print_highlight("Downsampling...\n");
	pcl::VoxelGrid<PointT> grid;
	grid.setLeafSize(0.2, 0.2, 0.2);
	grid.setInputCloud(object);
	grid.filter(*object_downsample);

	// Estimate features
	pcl::console::print_highlight("Estimating LRF...\n");
	FeatureEstimationT fest;
	pcl::search::KdTree<PointT>::Ptr tree_fest(new pcl::search::KdTree<PointT>());
	fest.setSearchMethod(tree_fest);
	fest.setRadiusSearch(1.0);
	fest.setInputCloud(object_downsample);
	fest.setSearchSurface(object);
	fest.compute(*object_features);
	pcl::io::savePCDFileASCII("test_object_lrf.pcd", *object_features);

	pcl::console::print_highlight("The Local Reference Frame on points [%f, %f, %f] is\n",
		(*object_downsample).points[100].x,
		(*object_downsample).points[100].y,
		(*object_downsample).points[100].z);
	pcl::console::print_highlight("x_axis    [%f, %f, %f] \n",
		(*object_features)[100].x_axis[0],
		(*object_features)[100].x_axis[1],
		(*object_features)[100].x_axis[2]
	);
	pcl::console::print_highlight("y_axis    [%f, %f, %f] \n",
		(*object_features)[100].y_axis[0],
		(*object_features)[100].y_axis[1],
		(*object_features)[100].y_axis[2]
	);
	pcl::console::print_highlight("z_axis    [%f, %f, %f] \n",
		(*object_features)[100].z_axis[0],
		(*object_features)[100].z_axis[1],
		(*object_features)[100].z_axis[2]
	);
	*/

	return 0;


}