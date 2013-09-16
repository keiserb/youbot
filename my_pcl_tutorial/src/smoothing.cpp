#include <pcl17/point_types.h>
#include <pcl17/io/pcd_io.h>
#include <pcl17/kdtree/kdtree_flann.h>
#include <pcl17/surface/mls.h>

int
main (int argc, char** argv)
{
  // Load input file into a PointCloud<T> with an appropriate type
  pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr cloud (new pcl17::PointCloud<pcl17::PointXYZRGB> ());
  // Load bun0.pcd -- should be available with the PCL archive in test
  pcl17::io::loadPCDFile ("SceneFiltered.pcd", *cloud);

  // Create a KD-Tree
  pcl17::search::KdTree<pcl17::PointXYZRGB>::Ptr tree (new pcl17::search::KdTree<pcl17::PointXYZRGB>);

  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl17::PointCloud<pcl17::PointNormal> mls_points;

  // Init object (second point type is for the normals, even if unused)
  pcl17::MovingLeastSquares<pcl17::PointXYZRGB, pcl17::PointNormal> mls;

  mls.setComputeNormals (true);

  // Set parameters
  mls.setInputCloud (cloud);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);

  // Reconstruct
  mls.process (mls_points);

  // Save output
  pcl17::io::savePCDFile ("bun0-mls.pcd", mls_points);
}
