#include <iostream>
#include <pcl17/ModelCoefficients.h>
#include <pcl17/io/pcd_io.h>
#include <pcl17/point_types.h>
#include <pcl17/sample_consensus/method_types.h>
#include <pcl17/sample_consensus/model_types.h>
#include <pcl17/segmentation/sac_segmentation.h>
#include <pcl17/filters/voxel_grid.h>
#include <pcl17/filters/extract_indices.h>

int
main (int argc, char** argv)
{
  sensor_msgs::PointCloud2::Ptr cloud_blob (new sensor_msgs::PointCloud2), cloud_filtered_blob (new sensor_msgs::PointCloud2);
  pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr cloud_filtered (new pcl17::PointCloud<pcl17::PointXYZRGB>), cloud_p (new pcl17::PointCloud<pcl17::PointXYZRGB>), cloud_f (new pcl17::PointCloud<pcl17::PointXYZRGB>);

  // Fill in the cloud data
  pcl17::PCDReader reader;
  reader.read ("SceneFiltered.pcd", *cloud_filtered_blob);

  std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;
  /*
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl17::VoxelGrid<sensor_msgs::PointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered_blob);
  */
  // Convert to the templated PointCloud
  pcl17::fromROSMsg (*cloud_filtered_blob, *cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

  // Write the downsampled version to disk
  pcl17::PCDWriter writer;
  writer.write<pcl17::PointXYZRGB> ("Scene_downsampled.pcd", *cloud_filtered, false);

  pcl17::ModelCoefficients::Ptr coefficients (new pcl17::ModelCoefficients ());
  pcl17::PointIndices::Ptr inliers (new pcl17::PointIndices ());
  // Create the segmentation object
  pcl17::SACSegmentation<pcl17::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl17::SACMODEL_PLANE);
  seg.setMethodType (pcl17::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);

  // Create the filtering object
  pcl17::ExtractIndices<pcl17::PointXYZRGB> extract;

  int i = 0, nr_points = (int) cloud_filtered->points.size ();
  // While 30% of the original cloud is still there
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    std::stringstream ss;
    ss << "Scene_" << i << ".pcd";
    writer.write<pcl17::PointXYZRGB> (ss.str (), *cloud_p, false);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    writer.write<pcl17::PointXYZRGB> ("object.pcd", *cloud_f, false);
    cloud_filtered.swap (cloud_f);
    i++;

  }
  return (0);
}

