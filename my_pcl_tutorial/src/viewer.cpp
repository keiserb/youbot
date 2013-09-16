/* \author Geoffrey Biggs */

#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl17/common/common_headers.h>
#include <pcl17/features/normal_3d.h>
#include <pcl17/io/pcd_io.h>
#include <pcl17/io/ply_io.h>
#include <pcl17/visualization/pcl_visualizer.h>
#include <pcl17/console/parse.h>
#include <pcl17/filters/voxel_grid.h>

// --------------
// -----Help-----
// --------------
void printUsage(const char* progName)
{
  std::cout << "\n\nUsage: " << progName << " [options]\n\n" << "Options:\n"
      << "-------------------------------------------\n" << "-h           this help\n"
      << "-s           Simple visualisation example\n" << "-r           RGB colour visualisation example\n"
      << "-c           Custom colour visualisation example\n" << "-n           Normals visualisation example\n"
      << "-a           Shapes visualisation example\n" << "-v           Viewports example\n"
      << "-i           Interaction Customization example\n" << "\n\n";
}

boost::shared_ptr<pcl17::visualization::PCLVisualizer> simpleVis(pcl17::PointCloud<pcl17::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl17::visualization::PCLVisualizer> viewer(new pcl17::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addPointCloud<pcl17::PointXYZRGB>(cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties(pcl17::visualization::PCL17_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  return (viewer);
}

boost::shared_ptr<pcl17::visualization::PCLVisualizer> rgbVis(pcl17::PointCloud<pcl17::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl17::visualization::PCLVisualizer> viewer(new pcl17::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  pcl17::visualization::PointCloudColorHandlerRGBField<pcl17::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl17::PointXYZRGB>(cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties(pcl17::visualization::PCL17_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  return (viewer);
}

boost::shared_ptr<pcl17::visualization::PCLVisualizer> customColourVis(
    pcl17::PointCloud<pcl17::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl17::visualization::PCLVisualizer> viewer(new pcl17::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  pcl17::visualization::PointCloudColorHandlerCustom<pcl17::PointXYZRGB> single_color(cloud, 0, 255, 0);
  viewer->addPointCloud<pcl17::PointXYZRGB>(cloud, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties(pcl17::visualization::PCL17_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  return (viewer);
}

boost::shared_ptr<pcl17::visualization::PCLVisualizer> normalsVis(pcl17::PointCloud<pcl17::PointXYZRGB>::ConstPtr cloud,
                                                                  pcl17::PointCloud<pcl17::Normal>::ConstPtr normals)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl17::visualization::PCLVisualizer> viewer(new pcl17::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  pcl17::visualization::PointCloudColorHandlerRGBField<pcl17::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl17::PointXYZRGB>(cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties(pcl17::visualization::PCL17_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addPointCloudNormals<pcl17::PointXYZRGB, pcl17::Normal>(cloud, normals, 10, 0.05, "normals");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  return (viewer);
}

boost::shared_ptr<pcl17::visualization::PCLVisualizer> shapesVis(pcl17::PointCloud<pcl17::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl17::visualization::PCLVisualizer> viewer(new pcl17::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  pcl17::visualization::PointCloudColorHandlerRGBField<pcl17::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl17::PointXYZRGB>(cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties(pcl17::visualization::PCL17_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();

  //------------------------------------
  //-----Add shapes at cloud points-----
  //------------------------------------
  viewer->addLine<pcl17::PointXYZRGB>(cloud->points[0], cloud->points[cloud->size() - 1], "line");
  viewer->addSphere(cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");

  //---------------------------------------
  //-----Add shapes at other locations-----
  //---------------------------------------
  pcl17::ModelCoefficients coeffs;
  coeffs.values.push_back(0.0);
  coeffs.values.push_back(0.0);
  coeffs.values.push_back(1.0);
  coeffs.values.push_back(0.0);
  viewer->addPlane(coeffs, "plane");
  coeffs.values.clear();
  coeffs.values.push_back(0.3);
  coeffs.values.push_back(0.3);
  coeffs.values.push_back(0.0);
  coeffs.values.push_back(0.0);
  coeffs.values.push_back(1.0);
  coeffs.values.push_back(0.0);
  coeffs.values.push_back(5.0);
  viewer->addCone(coeffs, "cone");

  return (viewer);
}

boost::shared_ptr<pcl17::visualization::PCLVisualizer> viewportsVis(
    pcl17::PointCloud<pcl17::PointXYZRGB>::ConstPtr cloud, pcl17::PointCloud<pcl17::Normal>::ConstPtr normals1,
    pcl17::PointCloud<pcl17::Normal>::ConstPtr normals2)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl17::visualization::PCLVisualizer> viewer(new pcl17::visualization::PCLVisualizer("3D Viewer"));
  viewer->initCameraParameters();

  int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor(0, 0, 0, v1);
  viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
  pcl17::visualization::PointCloudColorHandlerRGBField<pcl17::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl17::PointXYZRGB>(cloud, rgb, "sample cloud1", v1);

  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
  viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
  pcl17::visualization::PointCloudColorHandlerCustom<pcl17::PointXYZRGB> single_color(cloud, 0, 255, 0);
  viewer->addPointCloud<pcl17::PointXYZRGB>(cloud, single_color, "sample cloud2", v2);

  viewer->setPointCloudRenderingProperties(pcl17::visualization::PCL17_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
  viewer->setPointCloudRenderingProperties(pcl17::visualization::PCL17_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
  viewer->addCoordinateSystem(1.0);

  viewer->addPointCloudNormals<pcl17::PointXYZRGB, pcl17::Normal>(cloud, normals1, 10, 0.05, "normals1", v1);
  viewer->addPointCloudNormals<pcl17::PointXYZRGB, pcl17::Normal>(cloud, normals2, 10, 0.05, "normals2", v2);

  return (viewer);
}

unsigned int text_id = 0;
void keyboardEventOccurred(const pcl17::visualization::KeyboardEvent &event, void* viewer_void)
{
  boost::shared_ptr<pcl17::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<
      pcl17::visualization::PCLVisualizer> *>(viewer_void);
  if (event.getKeySym() == "r" && event.keyDown())
  {
    std::cout << "r was pressed => removing all text" << std::endl;

    char str[512];
    for (unsigned int i = 0; i < text_id; ++i)
    {
      sprintf(str, "text#%03d", i);
      viewer->removeShape(str);
    }
    text_id = 0;
  }
}

void mouseEventOccurred(const pcl17::visualization::MouseEvent &event, void* viewer_void)
{
  boost::shared_ptr<pcl17::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<
      pcl17::visualization::PCLVisualizer> *>(viewer_void);
  if (event.getButton() == pcl17::visualization::MouseEvent::LeftButton
      && event.getType() == pcl17::visualization::MouseEvent::MouseButtonRelease)
  {
    std::cout << "Left mouse button released at position (" << event.getX() << ", " << event.getY() << ")" << std::endl;

    char str[512];
    sprintf(str, "text#%03d", text_id++);
    viewer->addText("clicked here", event.getX(), event.getY(), str);
  }
}

boost::shared_ptr<pcl17::visualization::PCLVisualizer> interactionCustomizationVis()
{
  boost::shared_ptr<pcl17::visualization::PCLVisualizer> viewer(new pcl17::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(1.0);

  viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);
  viewer->registerMouseCallback(mouseEventOccurred, (void*)&viewer);

  return (viewer);
}

// --------------
// -----Main-----
// --------------
int main(int argc, char** argv)
{
  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------
  if (pcl17::console::find_argument(argc, argv, "-h") >= 0)
  {
    printUsage(argv[0]);
    return 0;
  }
  bool simple(false), rgb(false), custom_c(false), normals(false), shapes(false), viewports(false),
       interaction_customization(false);
  if (pcl17::console::find_argument(argc, argv, "-s") >= 0)
  {
    simple = true;
    std::cout << "Simple visualisation example\n";
  }
  else if (pcl17::console::find_argument(argc, argv, "-c") >= 0)
  {
    custom_c = true;
    std::cout << "Custom colour visualisation example\n";
  }
  else if (pcl17::console::find_argument(argc, argv, "-r") >= 0)
  {
    rgb = true;
    std::cout << "RGB colour visualisation example\n";
  }
  else if (pcl17::console::find_argument(argc, argv, "-n") >= 0)
  {
    normals = true;
    std::cout << "Normals visualisation example\n";
  }
  else if (pcl17::console::find_argument(argc, argv, "-a") >= 0)
  {
    shapes = true;
    std::cout << "Shapes visualisation example\n";
  }
  else if (pcl17::console::find_argument(argc, argv, "-v") >= 0)
  {
    viewports = true;
    std::cout << "Viewports example\n";
  }
  else if (pcl17::console::find_argument(argc, argv, "-i") >= 0)
  {
    interaction_customization = true;
    std::cout << "Interaction Customization example\n";
  }
  else
  {
    printUsage(argv[0]);
    return 0;
  }

  // ------------------------------------
  // -----Create example point cloud-----
  // ------------------------------------
  pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr basic_cloud_ptr(new pcl17::PointCloud<pcl17::PointXYZRGB>);
  pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr point_cloud_ptr(new pcl17::PointCloud<pcl17::PointXYZRGB>);
  pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr cloud_f(new pcl17::PointCloud<pcl17::PointXYZRGB>);
  pcl17::VoxelGrid<pcl17::PointXYZRGB > sor;
  //pcl17::PLYReader reader;
  //reader.read("box.ply",*point_cloud_ptr,0);
  //pcl17::PointXYZRGB point;

  if (pcl17::io::loadPCDFile<pcl17::PointXYZRGB>(argv[1], *point_cloud_ptr) == -1) //* load the file
  {
    PCL17_ERROR("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  basic_cloud_ptr = point_cloud_ptr;
  /*
  std::cout << "Loaded " << point_cloud_ptr->width * point_cloud_ptr->height << " data points" << std::endl;
  cout << "gobbel" << endl;
  sor.setInputCloud(point_cloud_ptr);
  std::cerr << "PointCloud before filtering: " << point_cloud_ptr->width * point_cloud_ptr->height << " data points ("
      << pcl17::getFieldsList(*point_cloud_ptr) << ").";
  std::cout << std::endl;
  sor.setLeafSize(0.01, 0.01, 0.01);
  sor.filter(*cloud_f);
  pcl17::io::savePCDFileASCII("BoxFiltered.pcd", *cloud_f); */
  /* for (size_t i = 0; i < point_cloud_ptr->points.size (); ++i)
   std::cout << "    " << point_cloud_ptr->points[i].x
   << " "    << point_cloud_ptr->points[i].y
   << " "    << point_cloud_ptr->points[i].z << std::endl;
   cout << point_cloud_ptr->width << endl;
   cout << point_cloud_ptr->height << endl;
   for(int i=0;i<point_cloud_ptr->width * point_cloud_ptr->height;i++)
   {
   point=point_cloud_ptr->points.at(i);
   point.r = 255;
   point.g = 0;
   point.b = 0;
   basic_cloud_ptr->points.at(i)=point;
   }
   cout << basic_cloud_ptr->width << endl;
   cout << basic_cloud_ptr->height << endl;
   pcl17::PCDWriter writer;
   writer.write<pcl17::PointXYZRGB> (argv[1], *basic_cloud_ptr, false);
   point_cloud_ptr=basic_cloud_ptr; */
  // ----------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.05-----
  // ----------------------------------------------------------------
  pcl17::NormalEstimation<pcl17::PointXYZRGB, pcl17::Normal> ne;
  ne.setInputCloud(point_cloud_ptr);
  pcl17::search::KdTree<pcl17::PointXYZRGB>::Ptr tree(new pcl17::search::KdTree<pcl17::PointXYZRGB>());
  ne.setSearchMethod(tree);
  pcl17::PointCloud<pcl17::Normal>::Ptr cloud_normals1(new pcl17::PointCloud<pcl17::Normal>);
  ne.setRadiusSearch(0.05);
  ne.compute(*cloud_normals1);

  // ---------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.1-----
  // ---------------------------------------------------------------
  pcl17::PointCloud<pcl17::Normal>::Ptr cloud_normals2(new pcl17::PointCloud<pcl17::Normal>);
  ne.setRadiusSearch(0.1);
  ne.compute(*cloud_normals2);

  boost::shared_ptr<pcl17::visualization::PCLVisualizer> viewer;
  if (simple)
  {
    viewer = simpleVis(basic_cloud_ptr);
  }
  else if (rgb)
  {
    viewer = rgbVis(point_cloud_ptr);
  }
  else if (custom_c)
  {
    viewer = customColourVis(basic_cloud_ptr);
  }
  else if (normals)
  {
    viewer = normalsVis(point_cloud_ptr, cloud_normals2);
  }
  else if (shapes)
  {
    viewer = shapesVis(point_cloud_ptr);
  }
  else if (viewports)
  {
    viewer = viewportsVis(point_cloud_ptr, cloud_normals1, cloud_normals2);
  }
  else if (interaction_customization)
  {
    viewer = interactionCustomizationVis();
  }

  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer->wasStopped())
  {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
}

