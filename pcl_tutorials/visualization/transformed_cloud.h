#pragma once

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

// This function displays the help
void
showHelp(char * program_name)
{
	std::cout << std::endl;
	std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
	std::cout << "-h:  Show this help." << std::endl;
}
// This is the main function
int
transformed_cloud()
{
	// Load file | Works with PCD and PLY files
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());

	if (pcl::io::loadPCDFile("object_aligned.pcd", *source_cloud) < 0) {
		std::cout << "Error loading point cloud "<< std::endl;
		return -1;
	}

	//float theta = M_PI / 2; // 旋转角度
	float theta = 0;

	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
	transform_2.translation() << 0.5, 0.0, 0.0;		//平移参数
	transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));	//旋转参数

	printf("\nMethod #2: using an Affine3f\n");
	std::cout << transform_2.matrix() << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_2);

	/************************************************************************/
	/*       Visualization                                                  */
	/************************************************************************/
	
	printf("\nPoint cloud colors :  white  = original point cloud\n"
		"                             red  = transformed point cloud\n");
	pcl::visualization::PCLVisualizer viewer("Matrix transformation example");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(source_cloud, 255, 255, 255);
	viewer.addPointCloud(source_cloud, source_cloud_color_handler, "original_cloud");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(transformed_cloud, 230, 20, 20); // Red
	viewer.addPointCloud(transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

	viewer.addCoordinateSystem(1.0, "cloud", 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
	// viewer.setPosition(800, 400); // Setting visualiser window position

	while (!viewer.wasStopped()) { // Display the visualiser until 'q' key is pressed
		viewer.spinOnce();
	}

	return 0;
}