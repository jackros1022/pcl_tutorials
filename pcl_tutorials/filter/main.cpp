#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <pcl/io/pcd_io.h>
#include "pcl/common/io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <pcl/console/time.h>

#include "estimateRadius.h"

using namespace std;



int main()
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile("../data/pig.pcd", *in_cloud) == -1)
	{
		std::cout << "load pcd file failed!\n";
		getchar();
		return -1;
	}
	pcl::console::TicToc time;
	time.tic();
	float radius = 0.0;
	int idx = 1000;
	EstimateRadius(in_cloud, radius, idx);
	cout <<"EstimateRadius:  "<< radius << endl;

	cout << "»¨·ÑÁË: " << time.toc() / 1000 << "s" << endl;

	getchar();
	return 0;
}