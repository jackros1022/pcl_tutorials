#include <iostream>
#include <pcl/io/pcd_io.h>
#include "pcl/common/io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
//#include "pcl/segmentation/euclidean_cluster_comparator.h"
#include <pcl/segmentation/extract_clusters.h>

#include "../keypoint/KeypointUniformSampling.h"
#include "../filter/filterExtractIndices.h"
#include "../filter/filterPassThrough.h"

#include "../visualization/ViewCloud.h"

#include "segmentSAC.h"
using namespace std;



int main()
{

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


	// 分割
	time.tic();
	boost::shared_ptr<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> cloudvector(new std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.02); // 2cm
	ec.setMinClusterSize(100);
	ec.setMaxClusterSize(25000);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(no_plane_cloud);
	ec.setSearchMethod(tree);
	ec.setInputCloud(no_plane_cloud);
	ec.extract(cluster_indices);
	cout << "EuclideanClusterExtraction 花费了: " << time.toc() / 1000 << "s" << endl;

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
			cloud_cluster->points.push_back(no_plane_cloud->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
		cloudvector->push_back(cloud_cluster);
	}


	cout << "花费了: " << time.toc() / 1000 << "s" << endl;

	//可视化输入点云和关键点
	boost::shared_ptr<ViewCloud> view(new ViewCloud);
	view->addPointCloud(in_cloud);
	view->addPointCloud(scene_pt);
	view->addPointCloud(no_plane_cloud);
	view->addPointCloud(plane_cloud);
	view->addPointCloud(cloudvector);
	view->showPointCloud(); //阻塞函数

	return 0;
}