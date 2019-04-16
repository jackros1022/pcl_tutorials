#pragma once
#include <iostream>
#include "pcl/kdtree/kdtree_flann.h"

#define K 6 //TODO:increase K, normal estimation, estimate radius
#define NK 15 //K for normal


// 不知道干嘛用
void EstimateRadius(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_,		
													float radius_,	//估计的半径
													int idx)		//要估计点云中的某个点
{
	pcl::KdTreeFLANN<pcl::PointXYZ> flannkdtree;
	flannkdtree.setInputCloud(cloud_);
	pcl::PointXYZ searchPoint = cloud_->points[idx];
	// K nearest neighbor search
	std::vector<int> pointIdxNKNSearch(K + 1);
	std::vector<float> pointNKNSquaredDistance(K + 1);

	if (flannkdtree.nearestKSearch(searchPoint, K + 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
		for (size_t i = 0; i < pointNKNSquaredDistance.size(); ++i) {
			if (pointIdxNKNSearch[i] == 0) {
				continue;
			}

			std::cout << "Squared Distance is:" << pointNKNSquaredDistance[i] << std::endl;
			radius_ += sqrt(pointNKNSquaredDistance[i]);
		}

		std::cout << "radius is:" << radius_ << std::endl;
		radius_ /= K;
	}

	std::cout << "radius is:" << radius_ << std::endl;
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	if (flannkdtree.radiusSearch(searchPoint, radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
		std::cout << "search with radius " << radius_ << ": result contains " << pointIdxRadiusSearch.size() << " points" << std::endl;

		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			std::cout << "    " << cloud_->points[pointIdxRadiusSearch[i]].x
			<< " " << cloud_->points[pointIdxRadiusSearch[i]].y
			<< " " << cloud_->points[pointIdxRadiusSearch[i]].z
			<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
	}

	return;
}  
