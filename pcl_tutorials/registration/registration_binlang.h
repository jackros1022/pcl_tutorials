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

#include "pcl/features/moment_of_inertia_estimation.h"
#include "pcl/filters/extract_indices.h"
#include "../feature/featureMomentInvariants.h"
#include "pcl/segmentation/segment_differences.h"
#include <pcl/common/transforms.h>

using namespace std;

int registration_binlang()
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
	PointCloudT::Ptr object_aligned_re(new PointCloudT);
	PointCloudT::Ptr scene(new PointCloudT);
	FeatureCloudT::Ptr object_features(new FeatureCloudT);
	FeatureCloudT::Ptr scene_features(new FeatureCloudT);

	Eigen::Matrix4f guess;

	//  加载目标物体和场景点云
	pcl::console::print_highlight("Loading point clouds...\n");
	if (pcl::io::loadPCDFile<PointNT>("chef.pcd", *object) < 0 ||
		pcl::io::loadPCDFile<PointNT>("rs1.pcd", *scene) < 0)
	{
		pcl::console::print_error("Error loading object/scene file!\n");
		return (1);
	}

	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
	transform_2.translation() << 3, 0.0, 0.0;		//平移参数
	transform_2.rotate(Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitZ()));	//旋转参数
	pcl::PointCloud<PointNT>::Ptr transformed_object(new pcl::PointCloud<PointNT>());
	pcl::transformPointCloud(*object, *object, transform_2);


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
		align.align(*object_aligned, guess);
	}



	/****************************** show box **************************************/
	bool showbox = false;
	if (showbox)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr object_aligned_xyz(new pcl::PointCloud<pcl::PointXYZ>);
		object_aligned_xyz->resize(object_aligned->size());
		for (int i = 0; i < object_aligned->size(); i++)
		{
			object_aligned_xyz->points[i].x = object_aligned->points[i].x; 
			object_aligned_xyz->points[i].y = object_aligned->points[i].y; 
			object_aligned_xyz->points[i].z = object_aligned->points[i].z; 
		}

		//pcl::PCDWriter writer;
		//writer.write("object_aligned.pcd", *object_aligned_xyz);

		getMomentInvariants(object_aligned_xyz);
	}
	/************************************************************************/

	//pcl::search::KdTree<PointNT>::Ptr tree(new pcl::search::KdTree<PointNT>);
	//tree->setInputCloud(scene);

	//pcl::getPointCloudDifference<PointNT>(*scene, *object_aligned, 0.01, tree, *object_aligned_re);

	//cout <<"object_aligned_re->size: "<< object_aligned_re->size() << endl;

	if (align.hasConverged())	//
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
		pcl::visualization::PCLVisualizer visu("鲁棒位姿估计");
		
		int v1(0), v2(0);
		visu.createViewPort(0, 0, 0.5, 1, v1);
		visu.createViewPort(0.5, 0, 1, 1, v2);
		visu.setBackgroundColor(255, 255, 255, v1);
		visu.addCoordinateSystem(1, v2);
		visu.addPointCloud(scene, ColorHandlerT(scene, 0.0, 255.0, 0.0), "scene", v1);
		visu.addPointCloud(object_aligned, ColorHandlerT(object_aligned, 0.0, 0.0, 255.0), "object_aligned", v1);
		//visu.addPointCloud(object_aligned_re, ColorHandlerT(object_aligned_re, 0.0, 0.0, 255.0), "object_aligned_re", v1);


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
