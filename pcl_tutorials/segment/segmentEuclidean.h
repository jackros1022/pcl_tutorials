#pragma once

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/crf_normal_segmentation.h"
#include <pcl/console/time.h>


void getEuclideanCluster(boost::shared_ptr<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> cloudvector,
															pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.02); // 2cm
	ec.setMinClusterSize(200);
	//ec.setMaxClusterSize(25000);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	pcl::console::TicToc time; time.tic();
	ec.extract(cluster_indices);
	std::cout << "EuclideanClusterExtraction »¨·ÑÁË: " << time.toc() / 1000 << "s" << std::endl;

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
			cloud_cluster->points.push_back(cloud->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
		cloudvector->push_back(cloud_cluster);
	}

}