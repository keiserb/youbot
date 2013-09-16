#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl17/ros/conversions.h>
#include <pcl17/io/pcd_io.h>
#include <pcl17/point_cloud.h>
#include <pcl17/point_types.h>
#include <pcl17/filters/voxel_grid.h>
#include <iostream>

ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  pcl17::PointCloud<pcl17::PointXYZRGB> cloud_c, cloud_f;
  sensor_msgs::PointCloud2 cloud_filtered;
  pcl17::fromROSMsg(*cloud, cloud_c);
  pcl17::io::savePCDFileASCII ("SceneUnfiltered.pcd", cloud_c);
  pcl17::VoxelGrid<sensor_msgs::PointCloud2> sor;
  sor.setInputCloud(cloud);
  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height << " data points (" << pcl17::getFieldsList(*cloud) << ").";
  std::cout << std::endl;
  sor.setLeafSize(0.01, 0.01, 0.01);
  sor.filter(cloud_filtered);
  pcl17::fromROSMsg(cloud_filtered, cloud_f);
  pcl17::io::savePCDFileASCII ("SceneFiltered.pcd", cloud_f);
  std::cerr << "PointCloud after filtering: " << cloud_filtered.width * cloud_filtered.height << " data points (" << pcl17::getFieldsList(cloud_filtered) << ").";
  std::cout << std::endl;
  // Publish the data
  pub.publish(cloud_filtered);
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

  // Spin
  ros::spin();
}
