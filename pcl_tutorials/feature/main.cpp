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
#include "featureFPH.h"

#define PFHSignature
#define FPFHSignature

int main()
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
	
	pcl::console::TicToc time;time.tic();
	if (pcl::io::loadPCDFile("../data/milk.pcd", *in_cloud) == -1){
		std::cout << "load pcd file failed!\n";getchar();return -1;}
	std::cout << "loadPCDFile Function Time: " << time.toc() / 1000 << "s" << std::endl;


	//getNormalEstimation(in_cloud, normal);

#ifdef PFHSignature
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhSignature125(new pcl::PointCloud<pcl::PFHSignature125>);
	pfhSignature125 = getPFH(in_cloud);
#endif 

#ifdef FPFHSignature
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhSignature33(new pcl::PointCloud<pcl::FPFHSignature33>);
	fpfhSignature33 = getFPFH(in_cloud);
#endif 

	getchar();
	return 0;
}
