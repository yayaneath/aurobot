#include <iostream>

// Ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/passthrough.h>

// Time
#include <boost/chrono/thread_clock.hpp>

// Our libraries
#include <geograsp/GeoGrasp.h> // Grasping points calculator
#include <aurobot_utils/GraspConfiguration.h> // Custom message for publishing grasps
#include <aurobot_utils/SceneObjects.h> // Custom message for publishing a vector of grasps

const int ALLEGRO_GRIP_TIP = 28;

// Global variables needed for the callback function
pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Cloud viewer"));
ros::Publisher pub;

// callback signature
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr & inputCloudMsg) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromROSMsg(*inputCloudMsg, *cloud);

  // Save to file
  pcl::io::savePCDFileBinary("1-cloud.pcd", *cloud);

  // Remove background points
  pcl::PassThrough<pcl::PointXYZRGB> ptFilter;
  ptFilter.setInputCloud(cloud);
  ptFilter.setFilterFieldName("z");
  ptFilter.setFilterLimits(0.0, 1.5);
  ptFilter.filter(*cloud);

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZRGB> sacSegmentator;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPlane(new pcl::PointCloud<pcl::PointXYZRGB>());

  sacSegmentator.setModelType(pcl::SACMODEL_PLANE);
  sacSegmentator.setMethodType(pcl::SAC_RANSAC);
  sacSegmentator.setMaxIterations(50);
  sacSegmentator.setDistanceThreshold(0.01);
  sacSegmentator.setInputCloud(cloud);
  sacSegmentator.segment(*inliers, *coefficients);

  // Remove the planar inliers, extract the rest
  pcl::ExtractIndices<pcl::PointXYZRGB> indExtractor;
  indExtractor.setInputCloud(cloud);
  indExtractor.setIndices(inliers);
  indExtractor.setNegative(false);

  // Get the points associated with the planar surface
  indExtractor.filter(*cloudPlane);

  // Remove the planar inliers, extract the rest
  indExtractor.setNegative(true);
  indExtractor.filter(*cloud);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ecExtractor;
  ecExtractor.setClusterTolerance(0.01);
  ecExtractor.setMinClusterSize(500);
  //ecExtractor.setMaxClusterSize(25000);
  ecExtractor.setSearchMethod(tree);
  ecExtractor.setInputCloud(cloud);
  ecExtractor.extract(clusterIndices);

  if (clusterIndices.empty()) {
    // Visualize the result
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> planeColor(cloudPlane,
      0, 255, 0);

    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "Main cloud");
    viewer->addPointCloud<pcl::PointXYZRGB>(cloudPlane, planeColor, "Plane");

    viewer->spinOnce();
  }
  else {
    aurobot_utils::SceneObjects scene_msg;
    std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin();
    int objectNumber = 0;

    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> planeColor(cloudPlane);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloudPlane, planeColor, "Plane");

    for (it = clusterIndices.begin(); it != clusterIndices.end(); ++it) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

      for (std::vector<int>::const_iterator pit = it->indices.begin(); 
          pit != it->indices.end(); ++pit)
        objectCloud->points.push_back(cloud->points[*pit]);

      objectCloud->width = objectCloud->points.size();
      objectCloud->height = 1;
      objectCloud->is_dense = true;

      GeoGrasp graspPoints;
      graspPoints.setBackgroundCloud(cloudPlane);
      graspPoints.setObjectCloud(objectCloud);
      graspPoints.setGripTipSize(ALLEGRO_GRIP_TIP);
      
      boost::chrono::thread_clock::time_point objStart = boost::chrono::thread_clock::now();
      graspPoints.compute();
      boost::chrono::thread_clock::time_point objStop = boost::chrono::thread_clock::now();

      std::cout << "Elapsed time: " 
        << boost::chrono::duration_cast<boost::chrono::milliseconds>(objStop-objStart).count() << " ms\n";

      GraspConfiguration bestGrasp = graspPoints.getBestGrasp();

      // Center point axis
      Eigen::Vector3f firstPoint(bestGrasp.firstPoint.x, bestGrasp.firstPoint.y, bestGrasp.firstPoint.z);
      Eigen::Vector3f secondPoint(bestGrasp.secondPoint.x, bestGrasp.secondPoint.y, bestGrasp.secondPoint.z);

      // Visualize the result
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(objectCloud);

      std::string objectLabel = "";
      std::ostringstream converter;

      converter << objectNumber;
      objectLabel += converter.str();
      objectLabel += "-";

      pcl::ModelCoefficients objAxisCoeff = graspPoints.getObjectAxisCoeff();
      //viewer->addLine(objAxisCoeff, std::string(objectLabel + "Object axis vector"));
      //std::cout << "Obj axis: " << objAxisCoeff << "\n";

      viewer->addPointCloud<pcl::PointXYZRGB>(objectCloud, rgb, objectLabel + "Object");
      viewer->addSphere(bestGrasp.firstPoint, 0.01, 0, 0, 255,
        objectLabel + "First best grasp point");
      viewer->addSphere(bestGrasp.secondPoint, 0.01, 255, 0, 0,
        objectLabel + "Second best grasp point");
      
      viewer->addLine(bestGrasp.firstPoint, bestGrasp.secondPoint, 0, 255, 0, 
        objectLabel + "Grasping line");

      // Add the new grasp configuration to the scene message
      aurobot_utils::GraspConfiguration msg;
      msg.first_point_x = bestGrasp.firstPoint.x;
      msg.first_point_y = bestGrasp.firstPoint.y;
      msg.first_point_z = bestGrasp.firstPoint.z;
      msg.second_point_x = bestGrasp.secondPoint.x;
      msg.second_point_y = bestGrasp.secondPoint.y;
      msg.second_point_z = bestGrasp.secondPoint.z;
      msg.obj_axis_coeff_0 = objAxisCoeff.values[0];
      msg.obj_axis_coeff_1 = objAxisCoeff.values[1];
      msg.obj_axis_coeff_2 = objAxisCoeff.values[2];
      msg.obj_axis_coeff_3 = objAxisCoeff.values[3];
      msg.obj_axis_coeff_4 = objAxisCoeff.values[4];
      msg.obj_axis_coeff_5 = objAxisCoeff.values[5];
      pcl::toROSMsg<pcl::PointXYZRGB>(*objectCloud, msg.object_cloud);

      scene_msg.objects.push_back(msg);

      objectNumber++;
    }

    // Publish the grasp configurations found in the scene
    pub.publish(scene_msg);

    viewer->spinOnce();
    //viewer->spin();
  }
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "aurobot_camera_listener");

  viewer->initCameraParameters();
  viewer->addCoordinateSystem(0.1);

  ros::NodeHandle nh("~");
  std::string cloudTopic;
  
  nh.getParam("topic", cloudTopic);

  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(cloudTopic, 1, cloudCallback);
  pub = nh.advertise<aurobot_utils::SceneObjects>("/aurobot_utils/scene_objects", 1);

  ros::spin();

  return 0;
}
