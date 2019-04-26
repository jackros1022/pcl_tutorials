#pragma once
#include <iostream>
#include <pcl/io/ply_io.h>

#include <pcl/point_types.h>

#include <pcl/segmentation/segment_differences.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/search/search.h>

int segment_differences()
{

	//Initialization of point clouds source, target and two outputs 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr src(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr out2(new pcl::PointCloud<pcl::PointXYZRGB>);

	//Reading two .ply files 
	pcl::io::loadPLYFile<pcl::PointXYZRGB>("./Neza_rot.ply", *src);
	pcl::io::loadPLYFile<pcl::PointXYZRGB>("./Neza_2.ply", *tgt);

	//Color handlers for red, green, blue and yellow color 
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red(src, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> blue(tgt, 0, 0, 255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> green(out, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> yellow(out2, 255, 255, 0);

	//Segment differences 

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	pcl::SegmentDifferences<pcl::PointXYZRGB> sdiff;
	sdiff.setInputCloud(src);
	sdiff.setTargetCloud(tgt);
	sdiff.setSearchMethod(tree);
	sdiff.setDistanceThreshold(0);
	sdiff.segment(*out);

	std::cout << *out;

	// Get point clod difference 
	pcl::getPointCloudDifference<pcl::PointXYZRGB>(*src, *tgt, 0, tree, *out2);

	std::cout << "\n\n" << *out2;

	//Visualiztion 
	pcl::visualization::PCLVisualizer vis("3D View");
	//vis.addPointCloud(src, red, "src", 0);
	//vis.addPointCloud(tgt, blue, "tgt", 0);
	vis.addPointCloud(out, green, "out", 0);
	vis.addPointCloud(out2, yellow, "out2", 0);
	vis.addCoordinateSystem(100);
	while (!vis.wasStopped())
	{
		vis.spinOnce();
	}
	return 0;
}
