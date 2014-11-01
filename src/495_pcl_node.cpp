#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/Vertices.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/impl/conditional_euclidean_clustering.hpp>

#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/surface/mls.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include <boost/thread/thread.hpp>

ros::Publisher pub;

float deg2rad(float alpha)
{
  return (alpha * 0.017453293f);
}

void ransac(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  *cloud = *input;

  pcl::ModelCoefficients::Ptr floor_coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr floor_indices(new pcl::PointIndices());
  pcl::SACSegmentation<pcl::PointXYZRGB> floor_finder;
  floor_finder.setOptimizeCoefficients(true);
  floor_finder.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
  // floor_finder.setModelType (SACMODEL_PLANE);
  floor_finder.setMethodType(pcl::SAC_RANSAC);
  floor_finder.setMaxIterations(300);
  floor_finder.setAxis(Eigen::Vector3f(0, 0, 1));
  floor_finder.setDistanceThreshold(0.08);
  floor_finder.setEpsAngle(deg2rad(5));
  floor_finder.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(*cloud));
  floor_finder.segment(*floor_indices, *floor_coefficients);

  if (floor_indices->indices.size() > 0)
  {
    // Extract the floor plane inliers
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr floor_points(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ExtractIndices<pcl::PointXYZRGB> extractor;
    extractor.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(*cloud));
    extractor.setIndices(floor_indices);
    extractor.filter(*floor_points);
    extractor.setNegative(true);
    extractor.filter(*cloud);

    // Project the floor inliers
    pcl::ProjectInliers<pcl::PointXYZRGB> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(floor_points);
    proj.setModelCoefficients(floor_coefficients);
    proj.filter(*cloud_projected);

    floor_points->header.frame_id = "camera_link";
    floor_points->header.stamp = ros::Time::now().toNSec();
   }
}

void passthrough_z(const sensor_msgs::PointCloud2ConstPtr& input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_passthrough)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*input, *cloud);

    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(*cloud));
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 6.0);
    pass.filter (*cloud_passthrough);
}

void passthrough_y(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_passthrough)
{
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(*cloud_passthrough));
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (0.0, 5.0);
    pass.filter (*cloud_passthrough);
}

void passthrough_x(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_passthrough)
{
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(*cloud_passthrough));
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-2.0, 2.0);
    pass.filter (*cloud_passthrough);
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Do data processing here...
  
  // run pass through filter to shrink point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_passthrough(new pcl::PointCloud<pcl::PointXYZRGB>);
  passthrough_z(input, cloud_passthrough);
  passthrough_y(cloud_passthrough);
  passthrough_x(cloud_passthrough);

  // run ransac to find floor
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZRGB>);
  ransac(cloud_passthrough, cloud_projected);
  pub.publish(*cloud_projected);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "four95_pcl_node");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("camera/depth_registered/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}


/*
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  // Do data processing here...
  //output = *input;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr conv_input(new pcl::PointCloud<pcl::PointXYZRGB>);
  //pcl_conversions::fromROSMsg(input, conv_input);
  pcl::fromROSMsg(*input, *conv_input);

  pcl::ConditionalEuclideanClustering<pcl::PointXYZRGB> cec (true);
  cec.setInputCloud (conv_input);
  //cec.setConditionFunction (&enforceIntensitySimilarity);
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

bool enforceIntensitySimilarity (const pcl::PointXYZRGB& point_a, const pcl::PointXYZRGB& point_b, float squared_distance)
{
  if (fabs (point_a.intensity - point_b.intensity) < 0.1f)
	return (true);
  else
	return (false);
}
*/
