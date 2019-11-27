#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "global_pose_publisher");

  ros::NodeHandle node;

  ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map");

  ros::Publisher globalPosePub = 
    node.advertise<geometry_msgs::PoseStamped>("/global_pose", 10);

  tf::TransformListener listener;
  uint32_t seq = 0;

  ros::Rate rate(100.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.header.seq = seq;
    msg.pose.position.x = transform.getOrigin().getX();
    msg.pose.position.y = transform.getOrigin().getY();
    msg.pose.position.z = transform.getOrigin().getZ();
    msg.pose.orientation.x = transform.getRotation().getX();
    msg.pose.orientation.y = transform.getRotation().getY();
    msg.pose.orientation.z = transform.getRotation().getZ();
    msg.pose.orientation.w = transform.getRotation().getW();
    globalPosePub.publish(msg);

    seq ++;
    rate.sleep();
  }
  return 0;
};