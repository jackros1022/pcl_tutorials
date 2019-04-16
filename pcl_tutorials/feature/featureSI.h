#pragma once
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/features/spin_image.h" //有GPU版本
#include "pcl/search/kdtree.h"
#include "pcl/console/time.h"



pcl::PointCloud<pcl::Histogram<153>>::Ptr getSpinImages(pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint,
														pcl::PointCloud<pcl::Normal>::Ptr normal = NULL,
														pcl::PointCloud<pcl::PointXYZ>::Ptr surface = NULL)
{
	/*
	template <typename PointInT, typename PointNT, typename PointOutT>
	class SpinImageEstimation : public Feature<PointInT, PointOutT>

	方法：
	void 	setImageWidth (unsigned int bin_count)
	Sets spin-image resolution. More...

	void 	setSupportAngle (double support_angle_cos)
	Sets the maximum angle for the point normal to get to support region. More...

	void 	setMinPointCountInNeighbourhood (unsigned int min_pts_neighb)
	Sets minimal points count for spin image computation. More...

	void 	setInputNormals (const PointCloudNConstPtr &normals)
	Provide a pointer to the input dataset that contains the point normals of the input XYZ dataset given by setInputCloud. More...

	void 	setRotationAxis (const PointNT &axis)
	Sets single vector a rotation axis for all input points. More...

	void 	setInputRotationAxes (const PointCloudNConstPtr &axes)
	Sets array of vectors as rotation axes for input points. More...

	void 	useNormalsAsRotationAxis ()
	Sets input normals as rotation axes (default setting). More...

	void 	setAngularDomain (bool is_angular=true)
	Sets/unsets flag for angular spin-image domain. More...

	void 	setRadialStructure (bool is_radial=true)
	Sets/unsets flag for radial spin-image structure. More...
	
	*/
	const unsigned int numberOfBins = 153;
	typedef pcl::PointCloud<pcl::Histogram<numberOfBins> > OutputCloud;

	// pcl::gpu::SpinImageEstimation GPU版本
	pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal,OutputCloud::PointType>::Ptr si(new
		pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, OutputCloud::PointType>(8, 0.5, 16));
	/* 参数：
		unsigned int image_width = 8,
		double support_angle_cos = 0.0,
		unsigned int min_pts_neighb = 0
	*/
	OutputCloud::Ptr spinImages(new OutputCloud);
	si->setInputCloud(keypoint);
	si->setInputNormals(normal);
	si->setSearchSurface(surface);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	si->setSearchMethod(tree);
	si->setRadiusSearch(20);
	std::cout << "Computing spin images..." << std::endl;
	pcl::console::TicToc time;time.tic();
	si->compute(*spinImages);
	std::cout << "SpinImageEstimation Function Time: " << time.toc() / 1000 << "s" << std::endl;
	std::cout << "SpinImageEstimation points.size (): " << spinImages->points.size() << std::endl;
	return spinImages;
}


void getSI_GPU(pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint,
	pcl::PointCloud<pcl::Normal>::Ptr normal = NULL,
	pcl::PointCloud<pcl::PointXYZ>::Ptr surface = NULL)
{


}