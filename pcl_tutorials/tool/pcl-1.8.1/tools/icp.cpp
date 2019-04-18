

#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/incremental_registration.h>

#include <string>
#include <iostream>
#include <fstream>
#include <vector>

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::ConstPtr CloudConstPtr;
typedef Cloud::Ptr CloudPtr;

int
main (int argc, char **argv)
{
  double dist = 0.05;
  pcl::console::parse_argument (argc, argv, "-d", dist);

  double rans = 0.05;
  pcl::console::parse_argument (argc, argv, "-r", rans);

  int iter = 50;
  pcl::console::parse_argument (argc, argv, "-i", iter);

  bool nonLinear = false;
  pcl::console::parse_argument (argc, argv, "-n", nonLinear);

  std::vector<int> pcd_indices;
  pcd_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

  pcl::IterativeClosestPoint<PointType, PointType>::Ptr icp;
  if (nonLinear)
  {
    std::cout << "Using IterativeClosestPointNonLinear" << std::endl;
    icp.reset (new pcl::IterativeClosestPointNonLinear<PointType, PointType> ());
  }
  else
  {
    std::cout << "Using IterativeClosestPoint" << std::endl;
    icp.reset (new pcl::IterativeClosestPoint<PointType, PointType> ());
  }
  icp->setMaximumIterations (iter);
  icp->setMaxCorrespondenceDistance (dist);
  icp->setRANSACOutlierRejectionThreshold (rans);

  pcl::registration::IncrementalRegistration<PointType> iicp;
  iicp.setRegistration (icp);

  for (size_t i = 0; i < pcd_indices.size (); i++)
  {
    CloudPtr data (new Cloud);
    if (pcl::io::loadPCDFile (argv[pcd_indices[i]], *data) == -1)
    {
      std::cout << "Could not read file" << std::endl;
      return -1;
    }

    if (!iicp.registerCloud (data))
    {
      std::cout << "Registration failed. Resetting transform" << std::endl;
      iicp.reset ();
      iicp.registerCloud (data);
    };

    CloudPtr tmp (new Cloud);
    pcl::transformPointCloud (*data, *tmp, iicp.getAbsoluteTransform ());

    std::cout << iicp.getAbsoluteTransform () << std::endl;

    std::string result_filename (argv[pcd_indices[i]]);
    result_filename = result_filename.substr (result_filename.rfind ("/") + 1);
    pcl::io::savePCDFileBinary (result_filename.c_str (), *tmp);
    std::cout << "saving result to " << result_filename << std::endl;
  }

  return 0;
}
