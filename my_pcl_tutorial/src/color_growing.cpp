#include <iostream>
#include <vector>
#include <pcl17/point_types.h>
#include <pcl17/io/pcd_io.h>
#include <pcl17/search/search.h>
#include <pcl17/search/kdtree.h>
#include <pcl17/visualization/cloud_viewer.h>
#include <pcl17/filters/passthrough.h>
#include <pcl17/segmentation/region_growing_rgb.h>

int main(int argc, char** argv)
{
  pcl17::search::Search<pcl17::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl17::search::Search<pcl17::PointXYZRGB> >(
      new pcl17::search::KdTree<pcl17::PointXYZRGB>);

  pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr cloud(new pcl17::PointCloud<pcl17::PointXYZRGB>);
  if (pcl17::io::loadPCDFile<pcl17::PointXYZRGB>("SceneFiltered.pcd", *cloud) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }
  pcl17::IndicesPtr indices(new std::vector<int>);
  pcl17::PassThrough<pcl17::PointXYZRGB> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 1.0);
  pass.filter(*indices);

  pcl17::RegionGrowingRGB<pcl17::PointXYZRGB> reg;
  reg.setInputCloud(cloud);
  reg.setIndices(indices);
  reg.setSearchMethod(tree);
  reg.setDistanceThreshold(10);
  reg.setPointColorThreshold(6);
  reg.setRegionColorThreshold(5);
  reg.setMinClusterSize(600);

  std::vector<pcl17::PointIndices> clusters;
  reg.extract(clusters);

  pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
  pcl17::visualization::CloudViewer viewer("Cluster viewer");
  viewer.showCloud(colored_cloud);
  while (!viewer.wasStopped())
  {
    boost::this_thread::sleep(boost::posix_time::microseconds(100));
  }

  return (0);
}
