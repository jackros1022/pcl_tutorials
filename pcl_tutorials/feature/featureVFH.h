#pragma once
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "featureNormalEstimation.h"
#include "pcl/features/vfh.h"
#include "pcl/features/cvfh.h"
#include "pcl/features/our_cvfh.h"

typedef pcl::PointCloud<pcl::PointXYZ> InputCloud;
typedef pcl::PointCloud<pcl::VFHSignature308> OutputCloud;

void getVFH(pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint,
			pcl::PointCloud<pcl::Normal>::Ptr normal = NULL,
			pcl::PointCloud<pcl::PointXYZ>::Ptr surface = NULL)
{
	/*
	template<typename PointInT, typename PointNT, typename PointOutT = pcl::VFHSignature308>
	class VFHEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>
	方法：
	void 	computePointSPFHSignature (const Eigen::Vector4f &centroid_p, const Eigen::Vector4f &centroid_n, const pcl::PointCloud< PointInT > &cloud, const pcl::PointCloud< PointNT > &normals, const std::vector< int > &indices)
		Estimate the SPFH (Simple Point Feature Histograms) signatures of the angular (f1, f2, f3) and distance (f4) features for a given point from its neighborhood. More...

	void 	setViewPoint (float vpx, float vpy, float vpz)
		Set the viewpoint. More...

	void 	getViewPoint (float &vpx, float &vpy, float &vpz)
		Get the viewpoint. More...

	void 	setUseGivenNormal (bool use)
		Set use_given_normal_. More...

	void 	setNormalToUse (const Eigen::Vector3f &normal)
		Set the normal to use. More...

	void 	setUseGivenCentroid (bool use)
		Set use_given_centroid_. More...

	void 	setCentroidToUse (const Eigen::Vector3f &centroid)
		Set centroid_to_use_. More...

	void 	setNormalizeBins (bool normalize)
		set normalize_bins_ More...

	void 	setNormalizeDistance (bool normalize)
		set normalize_distances_ More...

	void 	setFillSizeComponent (bool fill_size)
		set size_component_ More...

	void 	compute (PointCloudOut &output)
		Overloaded computed method from pcl::Feature. More...
	*/
	pcl::VFHEstimation<InputCloud::PointType, pcl::Normal, OutputCloud::PointType>::Ptr vfh(new 
		pcl::VFHEstimation<InputCloud::PointType, pcl::Normal, OutputCloud::PointType>);

}

void getCVFH() 
{
	/*
	template<typename PointInT, typename PointNT, typename PointOutT = pcl::VFHSignature308>
	class CVFHEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>

	方法：
	void 	filterNormalsWithHighCurvature (const pcl::PointCloud< PointNT > &cloud, std::vector< int > &indices_to_use, std::vector< int > &indices_out, std::vector< int > &indices_in, float threshold)
		Removes normals with high curvature caused by real edges or noisy data. More...

	void 	setViewPoint (float vpx, float vpy, float vpz)
		Set the viewpoint. More...

	void 	setRadiusNormals (float radius_normals)
		Set the radius used to compute normals. More...

	void 	getViewPoint (float &vpx, float &vpy, float &vpz)
		Get the viewpoint. More...

	void 	getCentroidClusters (std::vector< Eigen::Vector3f, Eigen::aligned_allocator< Eigen::Vector3f > > &centroids)
		Get the centroids used to compute different CVFH descriptors. More...

	void 	getCentroidNormalClusters (std::vector< Eigen::Vector3f, Eigen::aligned_allocator< Eigen::Vector3f > > &centroids)
		Get the normal centroids used to compute different CVFH descriptors. More...

	void 	setClusterTolerance (float d)
		Sets max. More...

	void 	setEPSAngleThreshold (float d)
		Sets max. More...

	void 	setCurvatureThreshold (float d)
		Sets curvature threshold for removing normals. More...

	void 	setMinPoints (size_t min)
		Set minimum amount of points for a cluster to be considered. More...

	void 	setNormalizeBins (bool normalize)
		Sets wether if the CVFH signatures should be normalized or not. More...

	void 	compute (PointCloudOut &output)
		Overloaded computed method from pcl::Feature. More...
	
	*/
	pcl::CVFHEstimation<InputCloud, pcl::Normal, OutputCloud::PointType>::Ptr cvfh(new 
		pcl::CVFHEstimation<InputCloud, pcl::Normal, OutputCloud::PointType>);

}

void getOUR_CVFH()
{
	/*
	template<typename PointInT, typename PointNT, typename PointOutT = pcl::VFHSignature308>
	class OURCVFHEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>
	
	方法：
	Eigen::Matrix4f 	createTransFromAxes (Eigen::Vector3f &evx, Eigen::Vector3f &evy, Eigen::Vector3f &evz, Eigen::Affine3f &transformPC, Eigen::Matrix4f &center_mat)
		Creates an affine transformation from the RF axes. More...

	void 	computeRFAndShapeDistribution (PointInTPtr &processed, PointCloudOut &output, std::vector< pcl::PointIndices > &cluster_indices)
		Computes SGURF and the shape distribution based on the selected SGURF. More...

	bool 	sgurf (Eigen::Vector3f &centroid, Eigen::Vector3f &normal_centroid, PointInTPtr &processed, std::vector< Eigen::Matrix4f, Eigen::aligned_allocator< Eigen::Matrix4f > > &transformations, PointInTPtr &grid, pcl::PointIndices &indices)
		Computes SGURF. More...

	void 	filterNormalsWithHighCurvature (const pcl::PointCloud< PointNT > &cloud, std::vector< int > &indices_to_use, std::vector< int > &indices_out, std::vector< int > &indices_in, float threshold)
		Removes normals with high curvature caused by real edges or noisy data. More...

	void 	setViewPoint (float vpx, float vpy, float vpz)
		Set the viewpoint. More...

	void 	setRadiusNormals (float radius_normals)
		Set the radius used to compute normals. More...

	void 	setClusterTolerance (float d)
	Sets max. More...

	void 	setEPSAngleThreshold (float d)
		Sets max. More...

	void 	setCurvatureThreshold (float d)
		Sets curvature threshold for removing normals. More...

	void 	setMinPoints (size_t min)
		Set minimum amount of points for a cluster to be considered. More...

	void 	setNormalizeBins (bool normalize)
		Sets wether if the signatures should be normalized or not. More...

	void 	setRefineClusters (float rc)
		Sets the refinement factor for the clusters. More...

	void 	setAxisRatio (float f)
		Sets the min axis ratio between the SGURF axes to decide if disambiguition is feasible. More...

	void 	setMinAxisValue (float f)
		Sets the min disambiguition axis value to generate several SGURFs for the cluster when disambiguition is difficult. More...

	void 	compute (PointCloudOut &output)
		Overloaded computed method from pcl::Feature. More...
	*/
	pcl::OURCVFHEstimation<InputCloud::PointType, pcl::Normal, OutputCloud::PointType>::Ptr ourcvfh(new 
		pcl::OURCVFHEstimation<InputCloud::PointType, pcl::Normal, OutputCloud::PointType>);
}