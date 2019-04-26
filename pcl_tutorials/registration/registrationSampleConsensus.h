#pragma once
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh.h>
#include "pcl/features/fpfh_omp.h"
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
/***************************sample_consensus_prerejective****************/
#include <pcl/registration/sample_consensus_prerejective.h>
/**************************SampleConsensusInitialAlignment******************/
#include "pcl/registration/ia_ransac.h"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

/***********************sample_consensus*********************************/
#include "pcl/registration/correspondence_rejection_sample_consensus.h"
#include "pcl/registration/correspondence_rejection_sample_consensus_2d.h"
/************************************************************************/

#include "pcl/filters/extract_indices.h"



// Align a rigid object to a scene with clutter and occlusions
// 遮挡杂乱的配准
int computeSampleConsensusPrerejective()
{
	// Types
	typedef pcl::PointNormal PointNT;
	typedef pcl::PointCloud<PointNT> PointCloudT;
	typedef pcl::FPFHSignature33 FeatureT;
	typedef pcl::FPFHEstimation<PointNT, PointNT, FeatureT> FeatureEstimationT;
	typedef pcl::PointCloud<FeatureT> FeatureCloudT;
	typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;


	// Point clouds
	PointCloudT::Ptr object(new PointCloudT);
	PointCloudT::Ptr object_aligned(new PointCloudT);
	PointCloudT::Ptr scene(new PointCloudT);
	FeatureCloudT::Ptr object_features(new FeatureCloudT);
	FeatureCloudT::Ptr scene_features(new FeatureCloudT);

	//  加载目标物体和场景点云
	pcl::console::print_highlight("Loading point clouds...\n");
	if (pcl::io::loadPCDFile<PointNT>("chef.pcd", *object) < 0 ||
		pcl::io::loadPCDFile<PointNT>("rs1.pcd", *scene) < 0)
	{
		pcl::console::print_error("Error loading object/scene file!\n");
		return (1);
	}

	// 下采样
	pcl::console::print_highlight("Downsampling...\n");
	pcl::VoxelGrid<PointNT> grid;
	const float leaf = 0.005f;
	grid.setLeafSize(leaf, leaf, leaf);
	grid.setInputCloud(object);
	grid.filter(*object);
	grid.setInputCloud(scene);
	grid.filter(*scene);

	// 估计场景法线
	pcl::console::print_highlight("Estimating scene normals...\n");
	pcl::NormalEstimation<PointNT, PointNT> nest;
	nest.setRadiusSearch(0.01);
	nest.setInputCloud(scene);
	nest.compute(*scene);

	// 特征估计
	pcl::console::print_highlight("Estimating features...\n");
	FeatureEstimationT fest;
	fest.setRadiusSearch(0.025);
	fest.setInputCloud(object);
	fest.setInputNormals(object);
	fest.compute(*object_features);
	fest.setInputCloud(scene);
	fest.setInputNormals(scene);
	fest.compute(*scene_features);

	// 实施配准
	pcl::console::print_highlight("Starting alignment...\n");
	pcl::SampleConsensusPrerejective<PointNT, PointNT, FeatureT> align;
	align.setInputSource(object);
	align.setSourceFeatures(object_features);
	align.setInputTarget(scene);
	align.setTargetFeatures(scene_features);
	align.setMaximumIterations(50000); //  采样一致性迭代次数
	align.setNumberOfSamples(3); //  创建假设所需的样本数
	align.setCorrespondenceRandomness(5); //  使用的临近特征点的数目
	align.setSimilarityThreshold(0.9f); // 多边形边长度相似度阈值
	align.setMaxCorrespondenceDistance(2.5f * 0.005); //  判断是否为内点的距离阈值
	align.setInlierFraction(0.25f); //接受位姿假设所需的内点比例
	{
		pcl::ScopeTime t("Alignment");
		align.align(*object_aligned);
	}

	if (align.hasConverged())
	{
		// Print results
		printf("\n");
		Eigen::Matrix4f transformation = align.getFinalTransformation();
		pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(0, 0), transformation(0, 1), transformation(0, 2));
		pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0), transformation(1, 1), transformation(1, 2));
		pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(2, 0), transformation(2, 1), transformation(2, 2));
		pcl::console::print_info("\n");
		pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transformation(0, 3), transformation(1, 3), transformation(2, 3));
		pcl::console::print_info("\n");
		pcl::console::print_info("Inliers: %i/%i\n", align.getInliers().size(), object->size());

		// Show alignment
		pcl::visualization::PCLVisualizer visu("点云库PCL学习教程第二版-鲁棒位姿估计");
		int v1(0), v2(0);
		visu.createViewPort(0, 0, 0.5, 1, v1);
		visu.createViewPort(0.5, 0, 1, 1, v2);
		visu.setBackgroundColor(255, 255, 255, v1);
		visu.addPointCloud(scene, ColorHandlerT(scene, 0.0, 255.0, 0.0), "scene", v1);
		visu.addPointCloud(object_aligned, ColorHandlerT(object_aligned, 0.0, 0.0, 255.0), "object_aligned", v1);

		visu.addPointCloud(object, ColorHandlerT(object, 0.0, 255.0, 0.0), "object_before_aligned", v2);
		visu.addPointCloud(scene, ColorHandlerT(scene, 0.0, 0.0, 255.0), "scene_v2", v2);
		visu.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "scene");
		visu.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "object_aligned");
		visu.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "object_before_aligned");
		visu.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "scene_v2");
		visu.spin();
	}
	else
	{
		pcl::console::print_error("Alignment failed!\n");
		return (1);
	}

	return (0);
}

void computeSampleConsensus() 
{
	/*
	template <typename PointT>
	class CorrespondenceRejectorSampleConsensus: public CorrespondenceRejector

	方法：
	void 	getRemainingCorrespondences (const pcl::Correspondences &original_correspondences, pcl::Correspondences &remaining_correspondences)
	Get a list of valid correspondences after rejection from the original set of correspondences. More...

	virtual void 	setInputCloud (const PointCloudConstPtr &cloud)
	Provide a source point cloud dataset (must contain XYZ data!) More...

	PointCloudConstPtr const 	getInputCloud ()
	Get a pointer to the input point cloud dataset target. More...

	virtual void 	setInputSource (const PointCloudConstPtr &cloud)
	Provide a source point cloud dataset (must contain XYZ data!) More...

	PointCloudConstPtr const 	getInputSource ()
	Get a pointer to the input point cloud dataset target. More...

	virtual void 	setTargetCloud (const PointCloudConstPtr &cloud)
	Provide a target point cloud dataset (must contain XYZ data!) More...

	virtual void 	setInputTarget (const PointCloudConstPtr &cloud)
	Provide a target point cloud dataset (must contain XYZ data!) More...

	PointCloudConstPtr const 	getInputTarget ()
	Get a pointer to the input point cloud dataset target. More...

	bool 	requiresSourcePoints () const
	See if this rejector requires source points. More...

	void 	setSourcePoints (pcl::PCLPointCloud2::ConstPtr cloud2)
	Blob method for setting the source cloud. More...

	bool 	requiresTargetPoints () const
	See if this rejector requires a target cloud. More...

	void 	setTargetPoints (pcl::PCLPointCloud2::ConstPtr cloud2)
	Method for setting the target cloud. More...

	void 	setInlierThreshold (double threshold)
	Set the maximum distance between corresponding points. More...

	double 	getInlierThreshold ()
	Get the maximum distance between corresponding points. More...

	void 	setMaxIterations (int max_iterations)
	Set the maximum number of iterations. More...

	void 	setMaximumIterations (int max_iterations)
	Set the maximum number of iterations. More...

	int 	getMaxIterations ()
	Get the maximum number of iterations. More...

	int 	getMaximumIterations ()
	Get the maximum number of iterations. More...

	Eigen::Matrix4f 	getBestTransformation ()
	Get the best transformation after RANSAC rejection. More...

	void 	setRefineModel (const bool refine)
	Specify whether the model should be refined internally using the variance of the inliers. More...

	bool 	getRefineModel () const
	Get the internal refine parameter value as set by the user using setRefineModel. More...

	void 	getInliersIndices (std::vector< int > &inlier_indices)
	Get the inlier indices found by the correspondence rejector. More...

	void 	setSaveInliers (bool s)
	Set whether to save inliers or not. More...

	bool 	getSaveInliers ()
	Get whether the rejector is configured to save inliers. More...
	*/
	//pcl::registration::CorrespondenceRejectorSampleConsensus
	

	/*
	template <typename PointT>
	class CorrespondenceRejectorSampleConsensus2D: public CorrespondenceRejectorSampleConsensus<PointT>
	方法：
	void 	getRemainingCorrespondences (const pcl::Correspondences &original_correspondences, pcl::Correspondences &remaining_correspondences)
	Get a list of valid correspondences after rejection from the original set of correspondences. More...

	void 	setFocalLengths (const float fx, const float fy)
	Sets the focal length parameters of the target camera. More...

	void 	getFocalLengths (float &fx, float &fy) const
	Reads back the focal length parameters of the target camera. More...

	void 	setCameraCenters (const float cx, const float cy)
	Sets the camera center parameters of the target camera. More...

	void 	getCameraCenters (float &cx, float &cy) const
	Reads back the camera center parameters of the target camera. More...
	*/
	//pcl::registration::CorrespondenceRejectorSampleConsensus2D
}

void computeSampleConsensusInitialAlignment()
{
	/*
	section IV of "Fast Point Feature Histograms (FPFH) for 3D Registration,"

	template <typename PointSource, typename PointTarget, typename FeatureT>
	class SampleConsensusInitialAlignment : public Registration<PointSource, PointTarget>
	
	void 	setSourceFeatures (const FeatureCloudConstPtr &features)
	Provide a boost shared pointer to the source point cloud's feature descriptors. More...

	FeatureCloudConstPtr const 	getSourceFeatures ()
	Get a pointer to the source point cloud's features. More...

	void 	setTargetFeatures (const FeatureCloudConstPtr &features)
	Provide a boost shared pointer to the target point cloud's feature descriptors. More...

	FeatureCloudConstPtr const 	getTargetFeatures ()
	Get a pointer to the target point cloud's features. More...

	void 	setMinSampleDistance (float min_sample_distance)
	Set the minimum distances between samples. More...

	float 	getMinSampleDistance ()
	Get the minimum distances between samples, as set by the user. More...

	void 	setNumberOfSamples (int nr_samples)
	Set the number of samples to use during each iteration. More...

	int 	getNumberOfSamples ()
	Get the number of samples to use during each iteration, as set by the user. More...

	void 	setCorrespondenceRandomness (int k)
	Set the number of neighbors to use when selecting a random feature correspondence. More...

	int 	getCorrespondenceRandomness ()
	Get the number of neighbors used when selecting a random feature correspondence, as set by the user. More...

	void 	setErrorFunction (const boost::shared_ptr< ErrorFunctor > &error_functor)
	Specify the error function to minimize. More...

	boost::shared_ptr< ErrorFunctor > 	getErrorFunction ()
	Get a shared pointer to the ErrorFunctor that is to be minimized. More...
	*/

	//pcl::SampleConsensusInitialAlignment<>

}