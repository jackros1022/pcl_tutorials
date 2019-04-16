#pragma once
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/console/time.h>
#include <pcl/filters/uniform_sampling.h>



/************************************************************************/
/*
法线作为Feature特征之一，虽然简单容易，但实际上计算量非常大的，不要直接多点云求取法线，否则非常消耗时间。
解决办法：
	1、利用indices的输入
*/
/************************************************************************/

void getNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normal)
{
	/************************************************************************/
	/*               
	template <typename PointInT, typename PointOutT>
	class NormalEstimation: public Feature<PointInT, PointOutT>


	bool 	computePointNormal (const pcl::PointCloud< PointInT > &cloud, const std::vector< int > &indices, Eigen::Vector4f &plane_parameters, float &curvature)
			Compute the Least-Squares plane fit for a given set of points, using their indices, and return the estimated plane parameters together with the surface curvature. More...

	bool 	computePointNormal (const pcl::PointCloud< PointInT > &cloud, const std::vector< int > &indices, float &nx, float &ny, float &nz, float &curvature)
			Compute the Least-Squares plane fit for a given set of points, using their indices, and return the estimated plane parameters together with the surface curvature. More...

	void 	setInputCloud (const PointCloudConstPtr &cloud) override
			Provide a pointer to the input dataset. More...

	void 	setViewPoint (float vpx, float vpy, float vpz)
			Set the viewpoint. More...

	void 	getViewPoint (float &vpx, float &vpy, float &vpz)
			Get the viewpoint. More...

	void 	useSensorOriginAsViewPoint ()
			sets whether the sensor origin or a user given viewpoint should be used. More...
	*/
	/************************************************************************/
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal>::Ptr ne(new
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal>);

	ne->setInputCloud(cloud);
	ne->setKSearch(5);

	if (true)	// 测试是否输入tree，计算视觉
	{
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		tree->setInputCloud(cloud);
		ne->setSearchMethod(tree);

	}
	pcl::console::TicToc time;time.tic();
	ne->compute(*normal);

	std::cout << "getNormal Function Time: " << time.toc() / 1000 << "s" << std::endl;
}

void getNormalOMP(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr normal) {

	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal>::Ptr ne(new pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal>);
	pcl::console::TicToc time;

	// 求取indices
	if (false)
	{
		pcl::UniformSampling<pcl::PointXYZ>::Ptr un(new pcl::UniformSampling<pcl::PointXYZ>);
		un->setRadiusSearch(0.01);
		un->setInputCloud(cloud);
		time.tic();
		un->filter(*cloud);
		std::cout << " UniformSampling Function Time: " << time.toc() / 1000 << "s" << std::endl;
		std::cout << " --> UniformSampling cloud->size: " << cloud->size() << std::endl;

	}



	// 计算法线
	ne->setNumberOfThreads(10);
	ne->setRadiusSearch(0.02);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	tree->setInputCloud(cloud);
	ne->setSearchMethod(tree);
	ne->setInputCloud(cloud);
	time.tic();
	ne->compute(*normal);
	std::cout << " getNormalOMP Function Time: " << time.toc() / 1000 << "s" << std::endl;

}

void getNormalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr normal) 
{
	/************************************************************************/
	/* 
	template <typename PointInT, typename PointOutT>
	class IntegralImageNormalEstimation: public Feature<PointInT, PointOutT>

void 	setRectSize (const int width, const int height)
	Set the regions size which is considered for normal estimation. More...

void 	setBorderPolicy (const BorderPolicy border_policy)
	Sets the policy for handling borders. More...

void 	computePointNormal (const int pos_x, const int pos_y, const unsigned point_index, PointOutT &normal)
	Computes the normal at the specified position. More...

void 	computePointNormalMirror (const int pos_x, const int pos_y, const unsigned point_index, PointOutT &normal)
	Computes the normal at the specified position with mirroring for border handling. More...

void 	setMaxDepthChangeFactor (float max_depth_change_factor)
	The depth change threshold for computing object borders. More...

void 	setNormalSmoothingSize (float normal_smoothing_size)
	Set the normal smoothing size. More...

void 	setNormalEstimationMethod (NormalEstimationMethod normal_estimation_method)
	Set the normal estimation method. More...

void 	setDepthDependentSmoothing (bool use_depth_dependent_smoothing)
	Set whether to use depth depending smoothing or not. More...

void 	setInputCloud (const typename PointCloudIn::ConstPtr &cloud) override
	Provide a pointer to the input dataset (overwrites the PCLBase::setInputCloud method) More...

float * 	getDistanceMap ()
	Returns a pointer to the distance map which was computed internally. More...

void 	setViewPoint (float vpx, float vpy, float vpz)
	Set the viewpoint. More...

void 	getViewPoint (float &vpx, float &vpy, float &vpz)
	Get the viewpoint. More...

void 	useSensorOriginAsViewPoint ()
	sets whether the sensor origin or a user given viewpoint should be used. More...
	*/
	/************************************************************************/
	pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal>::Ptr ne(new
		pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal>);

	
	ne->setNormalEstimationMethod(ne->AVERAGE_3D_GRADIENT);//法线计算方法
	ne->setMaxDepthChangeFactor(0.02f);	//最大深度变化系数
	ne->setNormalSmoothingSize(10.0f);	//优化法线时，考虑邻域大小
	if (cloud->isOrganized())
	{
		ne->setInputCloud(cloud);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		tree->setInputCloud(cloud);
		ne->setSearchMethod(tree);

		pcl::console::TicToc time;time.tic();
		ne->compute(*normal);
		std::cout << " IntegralImageNormalEstimation Function Time: " << time.toc() / 1000 << "s" << std::endl;

	}
	else
	{
		pcl::console::print_highlight(" cloud is not Organized \n");
		getNormalOMP(cloud, normal);
	}

	
}
