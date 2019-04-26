#include <iostream>
#include <pcl/io/pcd_io.h>
#include "pcl/common/io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
#include <pcl/segmentation/extract_clusters.h>

#include "../keypoint/KeypointUniformSampling.h"
#include "../filter/filterExtractIndices.h"
#include "../filter/filterPassThrough.h"

#include "../visualization/ViewCloud.h"
#include "segmentEuclidean.h"

#include "segmentSAC.h"

#include "segmentdifferences.h"
using namespace std;



int main()
{

	segment_differences();
/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr scene_pt(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	if (pcl::io::loadPCDFile("../data/milk_cartoon_all_small_clorox.pcd", *in_cloud) == -1)
	{
		std::cout << "load pcd file failed!\n";
		getchar();
		return -1;
	}
	pcl::console::TicToc time;time.tic();

	getPassthroughFilter(in_cloud, scene_pt);
	getSAC_Segmentation(scene_pt, inliers);

	pcl::PointCloud<pcl::PointXYZ>::Ptr no_plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	no_plane_cloud = getExtract_Indices(scene_pt, inliers);
	plane_cloud = getExtract_Indices(scene_pt, inliers, false);
	cout << "no_plane_cloud 花费了: " << time.toc() / 1000 << "s" << endl;


	// 分割 no_plane_cloud
	time.tic();
	boost::shared_ptr<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> cloudvector(new std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>);
	getUniformSampling(no_plane_cloud, no_plane_cloud,0.005);
	getEuclideanCluster(cloudvector, no_plane_cloud);
	cout << "花费了: " << time.toc() / 1000 << "s" << endl;

	//可视化输入点云和关键点
	boost::shared_ptr<ViewCloud> view(new ViewCloud);
	view->addPointCloud(plane_cloud);
	view->addPointCloud(cloudvector);
	view->showPointCloud(); //阻塞函数

	return 0;

*/
}