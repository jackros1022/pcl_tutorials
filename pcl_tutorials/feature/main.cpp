#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <pcl/io/pcd_io.h>
#include "pcl/common/io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>


using namespace std;

#include "featureNormalEstimation.h"


int main()
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
	pcl::console::TicToc time;

	time.tic();
	if (pcl::io::loadPCDFile("table_scene_lms400.pcd", *in_cloud) == -1){
		std::cout << "load pcd file failed!\n";getchar();return -1;}
	std::cout << "loadPCDFile Function Time: " << time.toc() / 1000 << "s" << std::endl;


	getNormalEstimation(in_cloud, normal);




	getchar();
	return 0;
}
