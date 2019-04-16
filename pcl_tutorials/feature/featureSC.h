#pragma once
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/features/3dsc.h"

//ShapeContext3DEstimation
void getSC3D(pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint,
			pcl::PointCloud<pcl::Normal>::Ptr normal = NULL,
			pcl::PointCloud<pcl::PointXYZ>::Ptr surface = NULL)
{
	/*
	template <typename PointInT, typename PointNT, typename PointOutT = pcl::ShapeContext1980>
	class ShapeContext3DEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>
	·½·¨£º

	void 	setMinimalRadius (double radius)
	The minimal radius value for the search sphere (rmin) in the original paper. More...

	void 	setPointDensityRadius (double radius)
	This radius is used to compute local point density density = number of points within this radius. More...
	*/
	pcl::ShapeContext3DEstimation<pcl::PointXYZ, pcl::Normal,pcl::ShapeContext1980>::Ptr sc3d(new
		pcl::ShapeContext3DEstimation<pcl::PointXYZ, pcl::Normal, pcl::ShapeContext1980>);

	pcl::ShapeContext3DEstimation<pcl::PointXYZ, pcl::Normal, pcl::UniqueShapeContext1960>::Ptr sc3d_Unique(new
		pcl::ShapeContext3DEstimation<pcl::PointXYZ, pcl::Normal, pcl::UniqueShapeContext1960>);
	//sc3d->set

	/*
	pcl::ShapeContext3DEstimation<pcl::PointXYZ, pcl::Normal, pcl::ShapeContext > shapeContext;
	shapeContext.setAzimuthBins(3);
	shapeContext.setElevationBins(3);
	shapeContext.setRadiusBins(3);
	// shapeContext.setPointDensityRadius (0.008);
	// Provide the original point cloud (without normals)
	shapeContext.setInputCloud(cloud_);
	// Provide the point cloud with normals
	shapeContext.setInputNormals(normals_); //TODO: out is passed in, but normals_ didn't, this is not consist
	// Use the same KdTree from the normal estimation
	shapeContext.setSearchMethod(tree_);
	// The search radius must be set to above the minimal search radius
	shapeContext.setMinimalRadius(radius_);
	shapeContext.setRadiusSearch(3 * radius_);
	// Actually compute the shape contexts
	shapeContext.compute(*shapeContextFeatures);
	

	*/
}