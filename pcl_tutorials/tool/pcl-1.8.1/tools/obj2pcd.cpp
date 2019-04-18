

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.obj output.pcd [options]\n", argv[0]);
  print_info ("where options are: \n");
  print_info ("     -copy_normals 0/1 : set to true (1) or false (0) if the output PointCloud should contain normals or not.\n");
}

template <typename T> void
saveCloud (const std::string &filename, const pcl::PointCloud<T> &cloud)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());
  
  PCDWriter w;
  w.writeBinaryCompressed (filename, cloud);
  
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Convert a OBJ file to PCD format. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .pcd and .obj files
  std::vector<int> pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  std::vector<int> obj_file_indices = parse_file_extension_argument (argc, argv, ".obj");
  if (pcd_file_indices.size () != 1 || obj_file_indices.size () != 1)
  {
    print_error ("Need one input OBJ file and one output PCD file.\n");
    return (-1);
  }

  // Load the OBJ file
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", argv[obj_file_indices[0]]);

  // Load the input file
  vtkSmartPointer<vtkPolyData> polydata;
  vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New ();
  reader->SetFileName (argv[obj_file_indices[0]]);
  reader->Update ();
  polydata = reader->GetOutput ();
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", polydata->GetNumberOfPoints ()); print_info (" points]\n");

  bool copy_normals = false;
  parse_argument (argc, argv, "-copy_normals", copy_normals);
  PCL_INFO ("Copy normals: %s.\n", copy_normals ? "true" : "false");

  if (copy_normals)
  {
    vtkSmartPointer<vtkPolyDataNormals> ng = vtkSmartPointer<vtkPolyDataNormals>::New ();
#if VTK_MAJOR_VERSION < 6
    ng->SetInput (polydata);
#else
    ng->SetInputData (polydata);
#endif
    ng->ComputePointNormalsOn ();
    ng->ComputeCellNormalsOff ();
    ng->Update ();
    polydata = ng->GetOutput ();

    pcl::PointCloud<pcl::PointNormal> cloud;
    vtkPolyDataToPointCloud (polydata, cloud);
    // Convert to pcd and save
    saveCloud (argv[pcd_file_indices[0]], cloud);
  }
  else
  { 
    pcl::PointCloud<pcl::PointXYZ> cloud;
    vtkPolyDataToPointCloud (polydata, cloud);
    // Convert to pcd and save
    saveCloud (argv[pcd_file_indices[0]], cloud);
  }

  return (0);
}

