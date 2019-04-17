#pragma once
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include "boost/shared_ptr.hpp"

#include <fstream>

#include "featureNormalEstimation.h"

/*
用贪婪投影三角化算法对有向点云进行三角化，具体方法是先将有向点云投影到某一局部二维坐标平面内，
再在坐标平面内进行平面内的三角化，再根据平面内三位点的拓扑连接关系获得一个三角网格曲面模型。

贪婪投影三角化算法原理是处理一系列可以使网格“生长扩大”的点（边缘点），
延伸这些点直到所有符合几何正确性和拓扑正确性的点都被连上。
该算法的优点是可以处理来自一个或者多个扫描仪扫描得到并且有多个连接处的散乱点云。
但该算法也有一定的局限性，它更适用于采样点云来自于表面连续光滑的曲面并且点云密度变化比较均匀的情况。

该算法的三角化过程是局部进行的，首先沿着一点的法线将该点投影到局部二维坐标平面内并连接其他悬空点，
然后在进行下一点。所以这里我们设置如下参数：

1）函数SetMaximumNearestNeighbors(unsigned)和SetMu(double)，
	这两个函数的作用是控制搜索邻域大小。前者定义了可搜索的邻域个数，
	后者规定了被样本点搜索其邻近点的最远距离，（是为了适应点云密度的变化），
	特征值一般是50-100和2.5-3（或者1.5每栅格）。

2）函数SetSearchRadius(double)，
	该函数设置了三角化后得到的每个三角形的最大可能边长。

3）函数SetMinimumAngle(double)和SetMaximumAngle(double)，
	这两个函数是三角化后每个三角形的最大角和最小角。两者至少要符合一个。典型值分别是10和120度（弧度）。

4）函数SetMaximumSurfaceAgle(double)和SetNormalConsistency(bool)，
	这两个函数是为了处理边缘或者角很尖锐以及一个表面的两边非常靠近的情况。
	为了处理这些特殊情况，函数SetMaximumSurfaceAgle(double)规定如果某点法线方向的偏离
	超过指定角度（注：大多数表面法线估计方法可以估计出连续变化的表面法线方向，即使在尖锐的边缘条件下），
	该点就不连接到样本点上。该角度是通过计算法向线段（忽略法线方向）之间的角度。
	函数SetNormalConsistency(bool)保证法线朝向，如果法线方向一致性标识没有设定，
	就不能保证估计出的法线都可以始终朝向一致。第一个函数特征值为45度（弧度）、第二个函数缺省值为false。
*/

void getTriangulation(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, 
							pcl::PointCloud<pcl::Normal>::Ptr &normals,
							std::vector <pcl::Vertices> &triangles) 
{
	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);


	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh polygonMesh;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.025);
	gp3.setMu(1.5);
	gp3.setMaximumNearestNeighbors(10);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
	tree->setInputCloud(cloud_with_normals);

	gp3.setSearchMethod(tree);
	gp3.reconstruct(polygonMesh);

	//保存网格图
	//pcl::io::savePLYFile("result.ply", polygonMesh);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr tcloud(new pcl::PointCloud<pcl::PointXYZ>);
	fromPCLPointCloud2(polygonMesh.cloud, *tcloud);

	// 把polygonMesh保存为txt文件
	//ofstream f1("result.txt");
	//for (int i = 0; i < tcloud->size(); i++)
	//{
	//	f1 << tcloud->points[i].x << " " << tcloud->points[i].y << " " << tcloud->points[i].z << endl;
	//}
	//f1.close();

	// 把polygonMesh保存为vector
	for (int i = 0; i < tcloud->size(); i++)
	{
		pcl::Vertices triangle;
		triangle.vertices.push_back(tcloud->points[i].x);
		triangle.vertices.push_back(tcloud->points[i].y);
		triangle.vertices.push_back(tcloud->points[i].z);
		triangles.push_back(triangle);
	}
	std::cout << "triangles.size: " << triangles.size() << std::endl;
}

void getGP3(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{

	// Normal estimation*
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setNumberOfThreads(10);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setRadiusSearch(0.02);
	n.compute(*normals);

	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.025);

	// Set typical values for the parameters
	gp3.setMu(1.5);
	gp3.setMaximumNearestNeighbors(10);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

	//保存网格图
	pcl::io::savePLYFile("result.ply", triangles);


	// 把triangles保存为txt文件
	pcl::PointCloud<pcl::PointXYZ>::Ptr tcloud(new pcl::PointCloud<pcl::PointXYZ>);
	fromPCLPointCloud2(triangles.cloud, *tcloud);
	ofstream f1("result.txt");
	for (int i=0;i< tcloud->size();i++)
	{
		f1 << tcloud->points[i].x << " " << tcloud->points[i].y << " " << tcloud->points[i].z << endl;
	}
	f1.close();

	//std::cout << triangles;
	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	// 显示
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPolygonMesh(triangles, "my");

	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	// 主循环
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}
