#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <stdlib.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "fake_bins_moves");
  ros::NodeHandle node;
  
  tf::TransformBroadcaster br;
  tf::Transform transform;
  
  while(node.ok()){
  transform.setOrigin(tf::Vector3(0.5+(double)rand()/RAND_MAX, (double)rand()/RAND_MAX,1.5));
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "torso_base_link", "bin1"));
  ros::Duration(1).sleep();
  
  transform.setOrigin(tf::Vector3(0.5+(double)rand()/RAND_MAX, (double)rand()/RAND_MAX,1.5));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "torso_base_link", "bin2"));
  
  transform.setOrigin(tf::Vector3(0.5+(double)rand()/RAND_MAX, (double)rand()/RAND_MAX,1.5));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "torso_base_link", "bin3"));
  
  //ros::Duration(5).sleep();
  }

  return 0;
}
