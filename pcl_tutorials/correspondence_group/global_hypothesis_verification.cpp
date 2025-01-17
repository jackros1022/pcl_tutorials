#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
//#include <pcl/filters/uniform_sampling.h>       //pcl 1.8
#include <pcl/keypoints/uniform_sampling.h>       //pcl1.7
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/recognition/hv/hv_go.h>

#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include "pcl/console/time.h"

/************************************************************************/
// ransac 物体识别，俩头文件什么区别？
#include "pcl/recognition/obj_rec_ransac.h"
#include "pcl/recognition/ransac_based/obj_rec_ransac.h"
/************************************************************************/
typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

struct CloudStyle
{
	double r;
	double g;
	double b;
	double size;

	CloudStyle(double r,
		double g,
		double b,
		double size) :
		r(r),
		g(g),
		b(b),
		size(size)
	{
	}
};

CloudStyle style_white(255.0, 255.0, 255.0, 4.0);
CloudStyle style_red(255.0, 0.0, 0.0, 3.0);
CloudStyle style_green(0.0, 255.0, 0.0, 5.0);
CloudStyle style_cyan(93.0, 200.0, 217.0, 4.0);
CloudStyle style_violet(255.0, 0.0, 255.0, 8.0);

std::string model_filename_;
std::string scene_filename_;

//Algorithm params
bool show_keypoints_(true);

bool use_hough_(true);
bool use_GC_(false);
bool use_RANSAC_(false);

bool use_ICP_(true);
bool use_HV_(true);

float model_ss_(0.02f);
float scene_ss_(0.02f);
float rf_rad_(0.015f);
float descr_rad_(0.02f);
float cg_size_(0.01f);
float cg_thresh_(5.0f);
int   icp_max_iter_(50);
float icp_corr_distance_(0.005f);	//icp阈值
float hv_clutter_reg_(5.0f);
float hv_inlier_th_(0.005f);
float hv_occlusion_th_(0.01f);
float hv_rad_clutter_(0.03f);
float hv_regularizer_(3.0f);
float hv_rad_normals_(0.05);
bool hv_detect_clutter_(true);

/**
* Prints out Help message
* @param filename Runnable App Name
*/
void
showHelp(char *filename)
{
	std::cout << std::endl;
	std::cout << "***************************************************************************" << std::endl;
	std::cout << "*                                                                         *" << std::endl;
	std::cout << "*          Global Hypothese Verification Tutorial - Usage Guide          *" << std::endl;
	std::cout << "*                                                                         *" << std::endl;
	std::cout << "***************************************************************************" << std::endl << std::endl;
	std::cout << "Usage: " << filename << " model_filename.pcd scene_filename.pcd [Options]" << std::endl << std::endl;
	std::cout << "Options:" << std::endl;
	std::cout << "     -h:                          Show this help." << std::endl;
	std::cout << "     -k:                          Show keypoints." << std::endl;
	std::cout << "     --algorithm (Hough|GC):      Clustering algorithm used (default Hough)." << std::endl;
	std::cout << "     --model_ss val:              Model uniform sampling radius (default " << model_ss_ << ")" << std::endl;
	std::cout << "     --scene_ss val:              Scene uniform sampling radius (default " << scene_ss_ << ")" << std::endl;
	std::cout << "     --rf_rad val:                Reference frame radius (default " << rf_rad_ << ")" << std::endl;
	std::cout << "     --descr_rad val:             Descriptor radius (default " << descr_rad_ << ")" << std::endl;
	std::cout << "     --cg_size val:               Cluster size (default " << cg_size_ << ")" << std::endl;
	std::cout << "     --cg_thresh val:             Clustering threshold (default " << cg_thresh_ << ")" << std::endl << std::endl;
	std::cout << "     --icp_max_iter val:          ICP max iterations number (default " << icp_max_iter_ << ")" << std::endl;
	std::cout << "     --icp_corr_distance val:     ICP correspondence distance (default " << icp_corr_distance_ << ")" << std::endl << std::endl;
	std::cout << "     --hv_clutter_reg val:        Clutter Regularizer (default " << hv_clutter_reg_ << ")" << std::endl;
	std::cout << "     --hv_inlier_th val:          Inlier threshold (default " << hv_inlier_th_ << ")" << std::endl;
	std::cout << "     --hv_occlusion_th val:       Occlusion threshold (default " << hv_occlusion_th_ << ")" << std::endl;
	std::cout << "     --hv_rad_clutter val:        Clutter radius (default " << hv_rad_clutter_ << ")" << std::endl;
	std::cout << "     --hv_regularizer val:        Regularizer value (default " << hv_regularizer_ << ")" << std::endl;
	std::cout << "     --hv_rad_normals val:        Normals radius (default " << hv_rad_normals_ << ")" << std::endl;
	std::cout << "     --hv_detect_clutter val:     TRUE if clutter detect enabled (default " << hv_detect_clutter_ << ")" << std::endl << std::endl;
}

/**
* Parses Command Line Arguments (Argc,Argv)
* @param argc
* @param argv
*/
void
parseCommandLine(int argc,
	char *argv[])
{
	//Show help
	if (pcl::console::find_switch(argc, argv, "-h"))
	{
		showHelp(argv[0]);
		exit(0);
	}

	//Model & scene filenames
	std::vector<int> filenames;
	filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
	if (filenames.size() != 2)
	{
		std::cout << "Filenames missing.\n";
		showHelp(argv[0]);
		exit(-1);
	}

	model_filename_ = argv[filenames[0]];
	scene_filename_ = argv[filenames[1]];

	//Program behavior
	if (pcl::console::find_switch(argc, argv, "-k"))
	{
		show_keypoints_ = true;
	}

	std::string used_algorithm;
	if (pcl::console::parse_argument(argc, argv, "--algorithm", used_algorithm) != -1)
	{
		if (used_algorithm.compare("Hough") == 0)
		{
			use_hough_ = true;
		}
		else if (used_algorithm.compare("GC") == 0)
		{
			use_hough_ = false;
		}
		else
		{
			std::cout << "Wrong algorithm name.\n";
			showHelp(argv[0]);
			exit(-1);
		}
	}

	//General parameters
	pcl::console::parse_argument(argc, argv, "--model_ss", model_ss_);
	pcl::console::parse_argument(argc, argv, "--scene_ss", scene_ss_);
	pcl::console::parse_argument(argc, argv, "--rf_rad", rf_rad_);
	pcl::console::parse_argument(argc, argv, "--descr_rad", descr_rad_);
	pcl::console::parse_argument(argc, argv, "--cg_size", cg_size_);
	pcl::console::parse_argument(argc, argv, "--cg_thresh", cg_thresh_);
	pcl::console::parse_argument(argc, argv, "--icp_max_iter", icp_max_iter_);
	pcl::console::parse_argument(argc, argv, "--icp_corr_distance", icp_corr_distance_);
	pcl::console::parse_argument(argc, argv, "--hv_clutter_reg", hv_clutter_reg_);
	pcl::console::parse_argument(argc, argv, "--hv_inlier_th", hv_inlier_th_);
	pcl::console::parse_argument(argc, argv, "--hv_occlusion_th", hv_occlusion_th_);
	pcl::console::parse_argument(argc, argv, "--hv_rad_clutter", hv_rad_clutter_);
	pcl::console::parse_argument(argc, argv, "--hv_regularizer", hv_regularizer_);
	pcl::console::parse_argument(argc, argv, "--hv_rad_normals", hv_rad_normals_);
	pcl::console::parse_argument(argc, argv, "--hv_detect_clutter", hv_detect_clutter_);
}

int
main(int argc,
	char *argv[])
{
	parseCommandLine(argc, argv);

	pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr model_keypoints(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr scene_keypoints(new pcl::PointCloud<PointType>());
	pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>());
	pcl::PointCloud<NormalType>::Ptr scene_normals(new pcl::PointCloud<NormalType>());
	pcl::PointCloud<DescriptorType>::Ptr model_descriptors(new pcl::PointCloud<DescriptorType>());
	pcl::PointCloud<DescriptorType>::Ptr scene_descriptors(new pcl::PointCloud<DescriptorType>());
	pcl::console::TicToc time;

	/**
	* Load Clouds
	*/
	if (pcl::io::loadPCDFile(model_filename_, *model) < 0)
	{
		std::cout << "Error loading model cloud." << std::endl;
		showHelp(argv[0]);
		return (-1);
	}
	if (pcl::io::loadPCDFile(scene_filename_, *scene) < 0)
	{
		std::cout << "Error loading scene cloud." << std::endl;
		showHelp(argv[0]);
		return (-1);
	}

	/**
	* Compute Normals
	*/
	pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
	norm_est.setNumberOfThreads(10);
	norm_est.setKSearch(10);
	norm_est.setInputCloud(model);
	norm_est.compute(*model_normals);

	norm_est.setInputCloud(scene);
	norm_est.compute(*scene_normals);

	/**
	*  Downsample Clouds to Extract keypoints
	*/
	pcl::UniformSampling<PointType> uniform_sampling;
	uniform_sampling.setInputCloud(model);
	uniform_sampling.setRadiusSearch(model_ss_);
	uniform_sampling.filter(*model_keypoints);
	std::cout << "Model total points: " << model->size() << "; Selected Keypoints: " << model_keypoints->size() << std::endl;

	uniform_sampling.setInputCloud(scene);
	uniform_sampling.setRadiusSearch(scene_ss_);
	uniform_sampling.filter(*scene_keypoints);
	std::cout << "Scene total points: " << scene->size() << "; Selected Keypoints: " << scene_keypoints->size() << std::endl;

	/**
	*  Compute Descriptor for keypoints
	*/
	pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
	descr_est.setNumberOfThreads(10);
	descr_est.setRadiusSearch(descr_rad_);

	descr_est.setInputCloud(model_keypoints);
	descr_est.setInputNormals(model_normals);
	descr_est.setSearchSurface(model);
	descr_est.compute(*model_descriptors);

	descr_est.setInputCloud(scene_keypoints);
	descr_est.setInputNormals(scene_normals);
	descr_est.setSearchSurface(scene);
	descr_est.compute(*scene_descriptors);

	/**
	*  Find Model-Scene Correspondences with KdTree
	*	查找对应点对，其中既包含正确的对应点对，也包含部分错误的对应点对。 
	*/
	pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());
	pcl::KdTreeFLANN<DescriptorType> match_search;
	match_search.setInputCloud(model_descriptors);
	std::vector<int> model_good_keypoints_indices;
	std::vector<int> scene_good_keypoints_indices;

	for (size_t i = 0; i < scene_descriptors->size(); ++i)
	{
		std::vector<int> neigh_indices(1);
		std::vector<float> neigh_sqr_dists(1);
		if (!pcl_isfinite(scene_descriptors->at(i).descriptor[0]))  //skipping NaNs
		{
			continue;
		}
		int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
		if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f)
		{
			pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			model_scene_corrs->push_back(corr);
			model_good_keypoints_indices.push_back(corr.index_query);
			scene_good_keypoints_indices.push_back(corr.index_match);
		}
	}
	pcl::PointCloud<PointType>::Ptr model_good_kp(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr scene_good_kp(new pcl::PointCloud<PointType>());
	pcl::copyPointCloud(*model_keypoints, model_good_keypoints_indices, *model_good_kp);
	pcl::copyPointCloud(*scene_keypoints, scene_good_keypoints_indices, *scene_good_kp);

	std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;

	/**
	*  Clustering
	* 假设生成模块包含两个任务：一是获得场景中可能存在的候选模型，二是获 得每个候选模型的可能姿态（即变换假设）。
	* 变换假设计算：包括几何一致性、姿态聚类、约束解译树、RANSAC、博弈论以及扩展霍夫变换等
	*/
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	std::vector < pcl::Correspondences > clustered_corrs;

	if (use_hough_)
	{
		pcl::PointCloud<RFType>::Ptr model_rf(new pcl::PointCloud<RFType>());
		pcl::PointCloud<RFType>::Ptr scene_rf(new pcl::PointCloud<RFType>());

		pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
		time.toc();
		rf_est.setFindHoles(true);
		rf_est.setRadiusSearch(rf_rad_);

		rf_est.setInputCloud(model_keypoints);
		rf_est.setInputNormals(model_normals);
		rf_est.setSearchSurface(model);
		rf_est.compute(*model_rf);

		rf_est.setInputCloud(scene_keypoints);
		rf_est.setInputNormals(scene_normals);
		rf_est.setSearchSurface(scene);
		rf_est.compute(*scene_rf);

		//  Clustering
		pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
		clusterer.setHoughBinSize(cg_size_);
		clusterer.setHoughThreshold(cg_thresh_);
		clusterer.setUseInterpolation(true);
		clusterer.setUseDistanceWeight(false);

		clusterer.setInputCloud(model_keypoints);
		clusterer.setInputRf(model_rf);
		clusterer.setSceneCloud(scene_keypoints);
		clusterer.setSceneRf(scene_rf);
		clusterer.setModelSceneCorrespondences(model_scene_corrs);

		clusterer.recognize(rototranslations, clustered_corrs);
		std::cout << "\n Hough3DGrouping Time: " << time.toc() / 1000 << "s" << std::endl;

	}else if(use_GC_)
	{
		pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
		time.toc();
		gc_clusterer.setGCSize(cg_size_);
		gc_clusterer.setGCThreshold(cg_thresh_);

		gc_clusterer.setInputCloud(model_keypoints);
		gc_clusterer.setSceneCloud(scene_keypoints);
		gc_clusterer.setModelSceneCorrespondences(model_scene_corrs);

		gc_clusterer.recognize(rototranslations, clustered_corrs);
		std::cout << "GeometricConsistencyGrouping Time: " << time.toc() / 1000 << "s" << std::endl;

	}
	else {
		// 怎么使用RANSAC去除错误点对
		//pcl::recognition::ObjRecRANSAC(); 
	}

	/**
	* Stop if no instances
	*/
	if (rototranslations.size() <= 0)
	{
		cout << "*** No instances found! ***" << endl;
		return (0);
	}
	else
	{
		cout << "Recognized Instances: " << rototranslations.size() << endl << endl;
	}

	/**
	* Generates clouds for each instances found
	* 假设验证：目的在于将正确的假设从错误的假设中区分出来。
	* 现有方法包括：独立验证方法（ICP）和 全局验证方法
	*/
	std::vector<pcl::PointCloud<PointType>::ConstPtr> instances;

	for (size_t i = 0; i < rototranslations.size(); ++i)
	{
		pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType>());
		pcl::transformPointCloud(*model, *rotated_model, rototranslations[i]);
		instances.push_back(rotated_model);
	}

	/**
	* 独立验证方法（ICP）
	*/

	std::vector<pcl::PointCloud<PointType>::ConstPtr> registered_instances;
	if (use_ICP_)
	{
		cout << "--- ICP ---------" << endl;

		for (size_t i = 0; i < rototranslations.size(); ++i)
		{
			pcl::IterativeClosestPoint<PointType, PointType> icp;
			time.toc();
			icp.setMaximumIterations(icp_max_iter_);
			icp.setMaxCorrespondenceDistance(icp_corr_distance_);
			icp.setInputTarget(scene);
			icp.setInputSource(instances[i]);	//识别后，旋转的 rotated_model
			pcl::PointCloud<PointType>::Ptr registered(new pcl::PointCloud<PointType>);
			icp.align(*registered);
			registered_instances.push_back(registered);
			cout << "Instance Indices " << i << " ";
			std::cout << "\n IterativeClosestPoint(独立验证方法) Time: " << time.toc() / 1000 << "s" << std::endl;

			if (icp.hasConverged())
			{
				cout << "Aligned!" << endl;
			}
			else
			{
				cout << "Not Aligned!" << endl;
			}
		}
		cout << "-----------------" << endl << endl;
	}

	/**
	* 全局验证方法 (Hypothesis Verification)
	* 该方法：在不增加虚警率的同时 显著提高对被遮挡物体的检测识别率，其主要缺陷在于优化算法的运算量较大。
	*/
	std::vector<bool> hypotheses_mask;  // Mask Vector to identify positive hypotheses
	if (use_HV_ && !registered_instances.empty())
	{
		cout << "--- Hypotheses Verification ---" << endl;
	
		pcl::GlobalHypothesesVerification<PointType, PointType> GoHv;
		time.tic();
		GoHv.setSceneCloud(scene);  // Scene Cloud
		GoHv.addModels(registered_instances, true);  //Models to verify

		GoHv.setInlierThreshold(hv_inlier_th_);
		GoHv.setOcclusionThreshold(hv_occlusion_th_);
		GoHv.setRegularizer(hv_regularizer_);
		GoHv.setRadiusClutter(hv_rad_clutter_);
		GoHv.setClutterRegularizer(hv_clutter_reg_);
		GoHv.setDetectClutter(hv_detect_clutter_);
		GoHv.setRadiusNormals(hv_rad_normals_);

		GoHv.verify();
		GoHv.getMask(hypotheses_mask);  // i-element TRUE if hvModels[i] verifies hypotheses
		std::cout << "GlobalHypothesesVerification(全局验证方法) Time: " << time.toc() / 1000 << "s" << std::endl;

		for (int i = 0; i < hypotheses_mask.size(); i++)
		{
			if (hypotheses_mask[i])
			{
				cout << "Instance Indice " << i << " is GOOD! <---" << endl;
			}
			else
			{
				cout << "Instance Indice" << i << " is bad!" << endl;
			}
		}
		cout << "-------------------------------" << endl;
	}


	/**
	*  Visualization
	*/
	pcl::visualization::PCLVisualizer viewer("Hypotheses Verification");
	viewer.addPointCloud(scene, "scene_cloud");

	pcl::PointCloud<PointType>::Ptr off_scene_model(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints(new pcl::PointCloud<PointType>());

	pcl::PointCloud<PointType>::Ptr off_model_good_kp(new pcl::PointCloud<PointType>());
	pcl::transformPointCloud(*model, *off_scene_model, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
	pcl::transformPointCloud(*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
	pcl::transformPointCloud(*model_good_kp, *off_model_good_kp, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));

	if (show_keypoints_)
	{
		CloudStyle modelStyle = style_white;
		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler(off_scene_model, modelStyle.r, modelStyle.g, modelStyle.b);
		viewer.addPointCloud(off_scene_model, off_scene_model_color_handler, "off_scene_model");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, modelStyle.size, "off_scene_model");
	}

	if (show_keypoints_)
	{
		CloudStyle goodKeypointStyle = style_violet;
		pcl::visualization::PointCloudColorHandlerCustom<PointType> model_good_keypoints_color_handler(off_model_good_kp, goodKeypointStyle.r, goodKeypointStyle.g,
			goodKeypointStyle.b);
		viewer.addPointCloud(off_model_good_kp, model_good_keypoints_color_handler, "model_good_keypoints");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, goodKeypointStyle.size, "model_good_keypoints");

		pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_good_keypoints_color_handler(scene_good_kp, goodKeypointStyle.r, goodKeypointStyle.g,
			goodKeypointStyle.b);
		viewer.addPointCloud(scene_good_kp, scene_good_keypoints_color_handler, "scene_good_keypoints");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, goodKeypointStyle.size, "scene_good_keypoints");
	}

	for (size_t i = 0; i < instances.size(); ++i)
	{
		std::stringstream ss_instance;
		ss_instance << "instance_" << i;

		CloudStyle clusterStyle = style_red;
		pcl::visualization::PointCloudColorHandlerCustom<PointType> instance_color_handler(instances[i], clusterStyle.r, clusterStyle.g, clusterStyle.b);
		viewer.addPointCloud(instances[i], instance_color_handler, ss_instance.str());
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, clusterStyle.size, ss_instance.str());

		CloudStyle registeredStyles = hypotheses_mask[i] ? style_green : style_cyan;
		ss_instance << "_registered" << endl;
		pcl::visualization::PointCloudColorHandlerCustom<PointType> registered_instance_color_handler(
			registered_instances[i], registeredStyles.r,registeredStyles.g, registeredStyles.b);
		viewer.addPointCloud(registered_instances[i], registered_instance_color_handler, ss_instance.str());
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, registeredStyles.size, ss_instance.str());
	}

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	return (0);
}
