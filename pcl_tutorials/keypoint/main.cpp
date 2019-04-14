#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <pcl/io/pcd_io.h>
#include "pcl/common/io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>

#include "KeypointSIFT.h"
#include "KeypointHarris.h"
#include "KeypointUniformSampling.h"

using namespace std;



int main()
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile("../data/pig.pcd", *in_cloud) == -1)
	{
		std::cout << "load pcd file failed!\n";
		getchar();
		return -1;
	}
	pcl::console::TicToc time; 
	time.tic();  

	//getKeypointSIFT(in_cloud, keypoint_cloud);
	//getHarris_3d(in_cloud, keypoint_cloud);
	getUniformSampling(in_cloud, keypoint_cloud);


	cout << "花费了: " << time.toc() / 1000 << "s" << endl;



	//可视化输入点云和关键点
	pcl::visualization::PCLVisualizer viewer("Keypoint");
	viewer.setBackgroundColor(255, 255, 255);
	viewer.addPointCloud(in_cloud, "cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, "cloud");
	viewer.addPointCloud(keypoint_cloud, "keypoints");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "keypoints");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 255, "keypoints");
	viewer.setShowFPS(true);
	viewer.spin();

	return 0;
}
