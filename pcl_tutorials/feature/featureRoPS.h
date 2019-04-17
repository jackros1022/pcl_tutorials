#pragma once

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/features/rops_estimation.h"
#include "pcl/features/feature.h"
#include "boost/shared_ptr.hpp"
#include <iostream>
#include <fstream>

#include <pcl/features/rops_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_plotter.h>// 直方图的可视化 方法2
#include <pcl/visualization/cloud_viewer.h>//可是化
#include <boost/thread/thread.hpp>//boost::this_thread::sleep 多进程

pcl::PointCloud<pcl::Histogram <135> >::Ptr getRoPS(std::vector <pcl::Vertices> triangles,
													pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint,
													pcl::PointCloud<pcl::Normal>::Ptr normal = NULL,
													pcl::PointCloud<pcl::PointXYZ>::Ptr surface = NULL)
{
	/*
  template <typename PointInT, typename PointOutT>
  class PCL_EXPORTS ROPSEstimation : public pcl::Feature <PointInT, PointOutT>

  方法：
  void 	setNumberOfPartitionBins (unsigned int number_of_bins)
  Allows to set the number of partition bins that is used for distribution matrix calculation. More...

  void 	setNumberOfRotations (unsigned int number_of_rotations)
  This method sets the number of rotations. More...

  void 	setSupportRadius (float support_radius)
  Allows to set the support radius that is used to crop the local surface of the point. More...

  void 	setTriangles (const std::vector< pcl::Vertices > &triangles)
  This method sets the triangles of the mesh. More...

	*/

	//boost::shared_ptr<pcl::ROPSEstimation<pcl::PointXYZ, pcl::Histogram<135>>> rop(new pcl::ROPSEstimation<pcl::PointXYZ, pcl::Histogram<135>>);

	pcl::ROPSEstimation<pcl::PointXYZ, pcl::Histogram<135>> rops;//pcl里面没有封装智能指针
	
	pcl::PointCloud<pcl::Histogram <135> >::Ptr histograms(new pcl::PointCloud <pcl::Histogram <135> >());


	float support_radius = 0.0285f;				//局部表面裁剪支持的半径 (搜索半价)，
	unsigned int number_of_partition_bins = 5;	//以及用于组成分布矩阵的容器的数量
	unsigned int number_of_rotations = 3;		//和旋转的次数。最后的参数将影响描述器的长度。

												//搜索方法
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(keypoint);
	// rops 特征算法 对象 盒子数量 × 旋转次数×9 得到特征维度  3*5*9 =135
	rops.setSearchMethod(tree);//搜索算法
	rops.setSearchSurface(surface);//搜索平面
	rops.setInputCloud(keypoint);//输入点云
	rops.setRadiusSearch(support_radius);//搜索半径
	rops.setTriangles(triangles);//领域形状
	rops.setNumberOfPartitionBins(number_of_partition_bins);//盒子数量
	rops.setNumberOfRotations(number_of_rotations);//旋转次数
	rops.setSupportRadius(support_radius);// 局部表面裁剪支持的半径 

	rops.compute(*histograms);
	return histograms;
}

int computeROPS() 
{
	//======= points 包含点云===========
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());

	//========关键点索引文件===============
	pcl::PointIndicesPtr indices = boost::shared_ptr <pcl::PointIndices>(new pcl::PointIndices());//关键点 索引
	std::ifstream indices_file;//文件输入流
	std::vector <pcl::Vertices> triangles;//=====triangles包含了多边形===领域形状
	std::ifstream triangles_file;

	if (pcl::io::loadPCDFile("./rops/points.pcd", *cloud_ptr) == -1)
		return (-1);
	indices_file.open("./rops/indices.txt", std::ifstream::in);
	if (!indices_file) {
		std::cout << "not found index.txt file" << std::endl;
		return (-1);
	}

	triangles_file.open("./rops/triangles.txt", std::ifstream::in);
	if (!triangles_file) {
		std::cout << "not found triangles.txt file" << std::endl;
		return (-1);
	}


	//========关键点索引文件===============
	for (std::string line; std::getline(indices_file, line);)//每一行为 一个点索引
	{
		std::istringstream in(line);
		unsigned int index = 0;
		in >> index;//索引
		indices->indices.push_back(index - 1);
	}
	indices_file.close();

	//=====triangles包含了多边形===领域形状
	for (std::string line; std::getline(triangles_file, line);)//每一行三个点索引
	{
		pcl::Vertices triangle;
		std::istringstream in(line);
		unsigned int vertex = 0;//索引
		in >> vertex;
		triangle.vertices.push_back(vertex - 1);
		in >> vertex;
		triangle.vertices.push_back(vertex - 1);
		in >> vertex;
		triangle.vertices.push_back(vertex - 1);
		triangles.push_back(triangle);
	}

	float support_radius = 0.0285f;//局部表面裁剪支持的半径 (搜索半价)，
	unsigned int number_of_partition_bins = 5;//以及用于组成分布矩阵的容器的数量
	unsigned int number_of_rotations = 3;//和旋转的次数。最后的参数将影响描述器的长度。
	pcl::search::KdTree<pcl::PointXYZ>::Ptr search_method(new pcl::search::KdTree<pcl::PointXYZ>);
	search_method->setInputCloud(cloud_ptr);

	// rops 特征算法 对象 盒子数量 × 旋转次数×9 得到特征维度  3*5*9 =135
	pcl::ROPSEstimation <pcl::PointXYZ, pcl::Histogram <135> > feature_estimator;
	feature_estimator.setSearchMethod(search_method);//搜索算法
	feature_estimator.setSearchSurface(cloud_ptr);//搜索平面
	feature_estimator.setInputCloud(cloud_ptr);//输入点云
	feature_estimator.setIndices(indices);//关键点索引
	feature_estimator.setTriangles(triangles);//领域形状,必需输入参数
	feature_estimator.setRadiusSearch(support_radius);//搜索半径
	feature_estimator.setNumberOfPartitionBins(number_of_partition_bins);//盒子数量
	feature_estimator.setNumberOfRotations(number_of_rotations);//旋转次数
	feature_estimator.setSupportRadius(support_radius);// 局部表面裁剪支持的半径 

	pcl::PointCloud<pcl::Histogram <135> >::Ptr histograms(new pcl::PointCloud <pcl::Histogram <135> >());
	feature_estimator.compute(*histograms);
	cout <<"histograms->size: "<< histograms->size() << endl;

	// 可视化
	pcl::visualization::PCLPlotter plotter;
	plotter.addFeatureHistogram(*histograms, 300); //设置的很坐标长度，该值越大，则显示的越细致
	plotter.plot();

	// 可视化点云
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	//viewer->setBackgroundColor(0, 0, 0);//背景黑色
	//viewer->addCoordinateSystem(1.0);//坐标系 尺寸
	//viewer->initCameraParameters();//初始化相机参数
	//viewer->addPointCloud<pcl::PointXYZ>(cloud_ptr, "sample cloud");//输入点云可视化
	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce(100);
	//	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	//}

	return (0);

}