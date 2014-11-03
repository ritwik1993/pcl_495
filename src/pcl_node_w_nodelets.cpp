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

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

ros::Publisher pub;
ros::Publisher pub_obj;

float deg2rad(float alpha)
{
  return (alpha * 0.017453293f);
}

void ransac(const sensor_msgs::PointCloud2ConstPtr& input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*input, *cloud);

  pcl::ModelCoefficients::Ptr floor_coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr floor_indices(new pcl::PointIndices());
  pcl::SACSegmentation<pcl::PointXYZRGB> floor_finder;
  floor_finder.setOptimizeCoefficients(true);
  floor_finder.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
  // floor_finder.setModelType (SACMODEL_PLANE);
  floor_finder.setMethodType(pcl::SAC_RANSAC);
  floor_finder.setMaxIterations(300);
  floor_finder.setAxis(Eigen::Vector3f(0, 0, 1));
  floor_finder.setDistanceThreshold(0.008);
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

   // new euclidean clustering code
   // Creating the KdTree object for the search method of the extraction
   pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
   tree->setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(*cloud));
   
   std::vector<pcl::PointIndices> cluster_indices;
   pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
   ec.setClusterTolerance (0.02); // 20cm
   ec.setMinClusterSize (100);
   ec.setMaxClusterSize (25000);
   ec.setSearchMethod (tree);
   ec.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(*cloud));
   ec.extract (cluster_indices);
   
   int j = 0;
   
   for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
   {
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
     for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
     { 
       cloud_cluster->points.push_back (cloud_projected->points[*pit]); //*
     }
     cloud_cluster->width = cloud_cluster->points.size ();
     cloud_cluster->height = 1;
     cloud_cluster->is_dense = true;
     cloud_cluster->header.frame_id = "camera_link";
     cloud_cluster->header.stamp = ros::Time::now().toNSec();
     pub_obj.publish(*cloud_cluster);
     //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
     //std::stringstream ss;
     //ss << "cloud_cluster_" << j << ".pcd";
     j++;
   }
   

}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Do data processing here...
  // run ransac to find floor
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZRGB>);
  ransac(input, cloud_projected);
  pub.publish(*cloud_projected);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_node_w_nodelets");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("voxel_grid/output", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  pub_obj = nh.advertise<sensor_msgs::PointCloud2> ("objects", 1);

  // Spin
  ros::spin ();
}

