#pragma once

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <time.h>
#include <string>

struct color
{
	double r = 0.1;
	double g = 0.2;
	double b = 0.3;
};

class ViewCloud
{
public:

	ViewCloud():
		viewer(new pcl::visualization::PCLVisualizer("3D Viewer")),
		cloudvector(new std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>)
	{
		cloudvector->clear();
		srand(std::time(NULL));


	}

	~ViewCloud()
	{
	}

public:
	void addPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	void addPointCloud(boost::shared_ptr<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> cloudvector);
	void showPointCloud();
private:
	boost::shared_ptr<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> cloudvector;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	color cloudColor;

};

void ViewCloud::addPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	cloudvector->push_back(cloud);
}

void ViewCloud::addPointCloud(boost::shared_ptr<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> cloudvector)
{
	for each (auto cloud in *cloudvector)
	{
		this->cloudvector->push_back(cloud);
	}
}

void ViewCloud::showPointCloud()
{
	int number = 1;

	for each (auto cloud in *cloudvector)
	{
		std::string cloudName = boost::lexical_cast<std::string>(number);
		cloudColor.r = rand() / (RAND_MAX + 1.0);
		cloudColor.g = rand() / (RAND_MAX + 1.0);
		cloudColor.b = rand() / (RAND_MAX + 1.0);

		viewer->addPointCloud(cloud, cloudName);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, cloudName);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, cloudColor.r, cloudColor.g, cloudColor.b, cloudName);
		
		++number;
	}

	if (!viewer->wasStopped())
	{
		viewer->spin();
	}
}

