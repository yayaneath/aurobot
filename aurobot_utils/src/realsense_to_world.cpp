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

  pub.publish(outputCloudMsg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "realsense_to_world");

  ros::NodeHandle nh("~");
  std::string cloudTopic;
  
  nh.getParam("topic", cloudTopic);

  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(cloudTopic, 1, cloudCallback);

  pub = nh.advertise<sensor_msgs::PointCloud2>("/camera/depth_registered/points_tfed", 1);

  ros::spin();

  return 0;
}
