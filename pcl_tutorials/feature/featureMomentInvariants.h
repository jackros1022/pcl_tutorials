#pragma once
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include "pcl/features/moment_invariants.h"
#include "pcl/features/moment_of_inertia_estimation.h"
#include "pcl/visualization/pcl_visualizer.h"

void getMomentInvariants(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
	/*
	template <typename PointT>
	class PCL_EXPORTS MomentOfInertiaEstimation : public pcl::PCLBase <PointT>
	
	*/
	pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();		//无参输入

	std::vector <float> moment_of_inertia;
	std::vector <float> eccentricity;
	pcl::PointXYZ min_point_AABB;
	pcl::PointXYZ max_point_AABB;
	pcl::PointXYZ min_point_OBB;
	pcl::PointXYZ max_point_OBB;
	pcl::PointXYZ position_OBB;
	Eigen::Matrix3f rotational_matrix_OBB;
	float major_value, middle_value, minor_value;
	Eigen::Vector3f major_vector, middle_vector, minor_vector;
	Eigen::Vector3f mass_center;

	feature_extractor.getMomentOfInertia(moment_of_inertia);	//惯性矩特征
	feature_extractor.getEccentricity(eccentricity);			//偏心距	
	feature_extractor.getAABB(min_point_AABB, max_point_AABB);	//AABB对应的左右角坐标
	feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	feature_extractor.getEigenValues(major_value, middle_value, minor_value);		//特征值
	feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);	//特征向量（分解，求出坐标）
	feature_extractor.getMassCenter(mass_center);	//点云中心坐标

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("基于惯性矩与偏心率的描述子"));
	viewer->setBackgroundColor(1, 1, 1);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	viewer->addPointCloud<pcl::PointXYZ>(cloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud, 0, 0, 255), "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample cloud");
	viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 0.0, 0.0, "AABB");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, "AABB");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, "AABB");
	Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
	std::cout << "position_OBB: " << position_OBB << endl;	//物体位置
	std::cout << "mass_center: " << mass_center << endl;	//重心位置，不一样
	Eigen::Quaternionf quat(rotational_matrix_OBB);
	viewer->addCube(position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "OBB");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, "OBB");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, "OBB");
	viewer->setRepresentationToWireframeForAllActors();
	pcl::PointXYZ center(mass_center(0), mass_center(1), mass_center(2));
	pcl::PointXYZ x_axis(major_vector(0) + mass_center(0), major_vector(1) + mass_center(1), major_vector(2) + mass_center(2));
	pcl::PointXYZ y_axis(middle_vector(0) + mass_center(0), middle_vector(1) + mass_center(1), middle_vector(2) + mass_center(2));
	pcl::PointXYZ z_axis(minor_vector(0) + mass_center(0), minor_vector(1) + mass_center(1), minor_vector(2) + mass_center(2));
	viewer->addLine(center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
	viewer->addLine(center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
	viewer->addLine(center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");
	std::cout << "size of cloud :" << cloud->points.size() << endl;
	std::cout << "moment_of_inertia :" << moment_of_inertia.size() << endl;
	std::cout << "eccentricity :" << eccentricity.size() << endl;

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}


}