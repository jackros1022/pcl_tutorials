#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/vtk_lib_io.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.obj output.vtk\n", argv[0]);
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Convert a OBJ file to VTK format. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .vtk and .obj files
  std::vector<int> vtk_file_indices = parse_file_extension_argument (argc, argv, ".vtk");
  std::vector<int> obj_file_indices = parse_file_extension_argument (argc, argv, ".obj");
  if (vtk_file_indices.size () != 1 || obj_file_indices.size () != 1)
  {
    print_error ("Need one input OBJ file and one output VTK file.\n");
    return (-1);
  }

  // Load the input file
  vtkSmartPointer<vtkPolyData> polydata;
  vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New ();
  reader->SetFileName (argv[obj_file_indices[0]]);
  reader->Update ();
  polydata = reader->GetOutput ();

  // Convert to VTK and save
  vtkSmartPointer<vtkPolyDataWriter> writer = vtkSmartPointer<vtkPolyDataWriter>::New ();
#if VTK_MAJOR_VERSION < 6
  writer->SetInput (polydata);
#else
  writer->SetInputData (polydata);
#endif
  writer->SetFileName (argv[vtk_file_indices[0]]);
  writer->Write ();
}

