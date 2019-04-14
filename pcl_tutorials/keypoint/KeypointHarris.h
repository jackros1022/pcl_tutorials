#pragma once
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/keypoints/harris_2d.h"
#include "pcl/keypoints/harris_3d.h"
#include "pcl/keypoints/harris_6d.h"
#include "pcl/keypoints/uniform_sampling.h"

void getHarris_2d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint)
{

}

void getHarris_3d(	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint)
{
	/*
	harris3D特征检测器
		template <typename PointInT, typename PointOutT, typename NormalT = pcl::Normal>
		class HarrisKeypoint3D : public Keypoint<PointInT, PointOutT>

	方法：
	void 	setInputCloud (const PointCloudInConstPtr &cloud) override
			Provide a pointer to the input dataset. More...

	void 	setMethod (ResponseMethod type)
			Set the method of the response to be calculated. More...

	void 	setRadius (float radius)
			Set the radius for normal estimation and non maxima supression. More...

	void 	setThreshold (float threshold)
			Set the threshold value for detecting corners. More...

	void 	setNonMaxSupression (bool=false)
			Whether non maxima suppression should be applied or the response for each point should be returned. More...

	void 	setRefine (bool do_refine)
			Whether the detected key points should be refined or not. More...

	void 	setNormals (const PointCloudNConstPtr &normals)
			输入法线信息
			Set normals if precalculated normals are available. More...

	void 	setSearchSurface (const PointCloudInConstPtr &cloud) override
			Provide a pointer to a dataset to add additional information to estimate the features for every point in the input dataset. More...

	void 	setNumberOfThreads (unsigned int nr_threads=0)
			Initialize the scheduler and set the number of threads to use. More...
	*/

	pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal>::Ptr harris3d(new pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal>);

	// Harris关键点类型 PointXYZI
	pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>);

	//Todo1：输入法线信息，直接计算会非常慢！

	harris3d->setInputCloud(cloud);
	harris3d->setRadius(0.2);
	harris3d->setRadiusSearch(0.2);
	harris3d->setNumberOfThreads(10);
	harris3d->compute(*result);
	// harris3d->getKeypointsIndices()
	std::cout << "Harris_keypoints的大小是" << result->size() << std::endl;
	copyPointCloud(*result, *keypoint);


}

void getHarris_6d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint)
{
	/*
	特征检测器
	template <typename PointInT, typename PointOutT, typename NormalT = pcl::Normal>
	class HarrisKeypoint6D : public Keypoint<PointInT, PointOutT>
	方法
	void 	setRadius (float radius)
			set the radius for normal estimation and non maxima supression. More...

	void 	setThreshold (float threshold)
			set the threshold value for detecting corners. More...

	void 	setNonMaxSupression (bool=false)
			whether non maxima suppression should be applied or the response for each point should be returned More...

	void 	setRefine (bool do_refine)
			whether the detected key points should be refined or not. More...

	virtual void 	setSearchSurface (const PointCloudInConstPtr &cloud)

	void 	setNumberOfThreads (unsigned int nr_threads=0)
			Initialize the scheduler and set the number of threads to use. More...
	*/
	//pcl::HarrisKeypoint6D < pcl::PointXYZ, pcl::PointXYZI, pcl::Normal>::Ptr harris6D(new pcl::HarrisKeypoint6D < pcl::PointXYZ, pcl::PointXYZI, pcl::Normal>);
	//pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>);

}