#pragma once
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>
#include "pcl/features/fpfh_omp.h"
#include "../keypoint/KeypointSIFT.h"
#include "featureNormalEstimation.h"

pcl::PointCloud<pcl::PFHSignature125>::Ptr getPFH(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhFeature(new pcl::PointCloud<pcl::PFHSignature125>);
	pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
	getNormalEstimation(cloud, normal);


	/*

		template <typename PointInT, typename PointNT, typename PointOutT = pcl::PFHSignature125>
		class PFHEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>
	
	*/
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal>::Ptr pfh(new pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal>);

	pfh->setInputCloud(cloud);
	pfh->setInputNormals(normal);
	pfh->setRadiusSearch(0.03);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	pfh->setSearchMethod(tree);
	pcl::console::TicToc time; time.tic();
	pfh->compute(*pfhFeature);
	std::cout << " PFHSignature125 Function Time: " << time.toc() / 1000 << "s" << std::endl;
	std::cout << " --> pfhFeature->size: " << pfhFeature->size() << std::endl;

	return pfhFeature;

}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr getFPFH(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhFeature(new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
	getNormalEstimation(cloud, normal);

	/*

	  template <typename PointInT, typename PointNT, typename PointOutT = pcl::FPFHSignature33>
	  class FPFHEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>
	*/
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>::Ptr fpfh(new pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>);
	fpfh->setInputCloud(cloud);
	fpfh->setInputNormals(normal);
	fpfh->setRadiusSearch(0.05);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	fpfh->setSearchMethod(tree);
	pcl::console::TicToc time; time.tic();
	fpfh->compute(*fpfhFeature);
	std::cout << " FPFHSignature33 Function Time: " << time.toc() / 1000 << "s" << std::endl;
	std::cout << " --> fpfhFeature->size: " << fpfhFeature->size() << std::endl;
	return fpfhFeature;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr getFPFHOMP(	pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
														pcl::PointCloud<pcl::PointXYZ>::Ptr& keypoint, 
														pcl::PointCloud<pcl::Normal>::Ptr normal)
{
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhFeature(new pcl::PointCloud<pcl::FPFHSignature33>);
	//pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
	//getNormalEstimation(keypoint, normal);

	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>::Ptr fpfh(new 
		pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>);
	fpfh->setNumberOfThreads(10);

	fpfh->setInputCloud(keypoint);
	fpfh->setInputNormals(normal);
	//fpfh->setSearchSurface(cloud);

	fpfh->setRadiusSearch(0.05);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(keypoint);
	fpfh->setSearchMethod(tree);
	pcl::console::TicToc time; time.tic();
	fpfh->compute(*fpfhFeature);
	std::cout << " FPFHSignature33 Function Time: " << time.toc() / 1000 << "s" << std::endl;
	std::cout << " --> fpfhFeature->size: " << fpfhFeature->size() << std::endl;
	return fpfhFeature;

}