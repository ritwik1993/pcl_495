
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/impl/conditional_euclidean_clustering.hpp>
#include <pcl/PCLPointCloud2.h>

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2 input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(input, pcl_pc);
  // Do data processing here...
  //output = *input;
  pcl::PointCloud<pcl::PointXYZI> conv_input;
  pcl::fromPCLPointCloud2(pcl_pc, conv_input);
  pcln::ConditionalEuclideanClustering<pcl::PointXYZI> cec (true);
  cec.setInputCloud (conv_input);
  cec.setConditionFunction (&enforceIntensitySimilarity);
  // Points within this distance from one another are going to need to validate the enforceIntensitySimilarity function to be part of the same cluster:
  cec.setClusterTolerance (0.09f);
  // Size constraints for the clusters:
  cec.setMinClusterSize (5);
  cec.setMaxClusterSize (30);
  // The resulting clusters (an array of pointindices):
  cec.segment (*clusters);
  // The clusters that are too small or too large in size can also be extracted separately:
  cec.getRemovedClusters (small_clusters, large_clusters);
   output=*input;
    // Publish the data.
    pub.publish (output);
}

bool
enforceIntensitySimilarity (const pcl::PointXYZI& point_a, const pcl::PointXYZI& point_b, float squared_distance)
{
  if (fabs (point_a.intensity - point_b.intensity) < 0.1f)
    return (true);
  else
    return (false);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_node");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("camera/depth/images", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}
