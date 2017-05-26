#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>


ros::Publisher pub;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr & inputCloudMsg) {
  sensor_msgs::PointCloud2 outputCloudMsg;
  tf::TransformListener tfListener;
  tf::StampedTransform transform;

  tfListener.waitForTransform("/world", "/head_link", ros::Time(0), ros::Duration(3.0));
  tfListener.lookupTransform("/world", "/head_link", ros::Time(0), transform);

  pcl_ros::transformPointCloud("/world", transform, *inputCloudMsg, outputCloudMsg);

  pub.publish (outputCloudMsg);

  ROS_INFO_NAMED("mapper", "New RealSense cloud mapped");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "realsense_to_world_mapper");

  ros::NodeHandle n;
  ros::NodeHandle nh;
  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/cloud_pcd",///camera/depth_registered/points",
    1, cloudCallback);
 
  pub = nh.advertise<sensor_msgs::PointCloud2>("/camera/depth_registered/points_tfed", 30);

  ros::spin();

  return 0;
}
