#include <ros/ros.h>
#include <vision/bottle_data.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
  
class bottleTFUpdate{

	private:
		ros::NodeHandle n;
		ros::Subscriber sub_left;
		ros::Subscriber sub_right;
		
		geometry_msgs::PoseStamped pose_left;
		geometry_msgs::PoseStamped pose_right;
		
		tf::TransformBroadcaster br;
		tf::Transform transform;
	
	public:
		std::string REFERENCE_FRAME;
		std::string bottle_left, bottle_right;
	
		bottleTFUpdate(){
			sub_left=n.subscribe("vision/bottle_data_arm_left",100, &bottleTFUpdate::bottleLeftObjectCallback,this);
			sub_right=n.subscribe("vision/bottle_data_arm_right",100, &bottleTFUpdate::bottleRightObjectCallback,this);
			bottle_left="bottle_left_arm";
			bottle_right="bottle_right_arm";
			REFERENCE_FRAME="torso_base_link";
		}
	
		void bottleLeftObjectCallback(const vision::bottle_data::ConstPtr& msg){
			pose_left=msg->P;
		}
		
		void bottleRightObjectCallback(const vision::bottle_data::ConstPtr& msg){
			pose_right=msg->P;
		}
	
		void leftTFPublisher(std::string bottle){
			transform.setOrigin(tf::Vector3(pose_left.pose.position.x,pose_left.pose.position.y,pose_left.pose.position.z));
 			tf::Quaternion q(pose_left.pose.orientation.x,pose_left.pose.orientation.y,pose_left.pose.orientation.z,pose_left.pose.orientation.w);
 			transform.setRotation(q);
 			try{
 				if(pose_left.pose.position.z>0.3)
 				br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),REFERENCE_FRAME,bottle));
 			}
 			catch (tf::TransformException ex){
 				ros::Duration(0.1).sleep();
 			}
 		}
 		
 		void rightTFPublisher(std::string bottle){
			transform.setOrigin(tf::Vector3(pose_right.pose.position.x,pose_right.pose.position.y,pose_right.pose.position.z));
 			tf::Quaternion q(pose_right.pose.orientation.x,pose_right.pose.orientation.y,pose_right.pose.orientation.z,pose_right.pose.orientation.w);
 			transform.setRotation(q);
 			try{
 				if(pose_left.pose.position.z>0.3)
 				br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),REFERENCE_FRAME,bottle));
 			}
 			catch (tf::TransformException ex){
 				ros::Duration(0.1).sleep();
 			}
 		}
};
 
int main(int argc, char** argv){
	ros::init(argc, argv, "bottles_TF_to_environment");
	ros::NodeHandle node; 
	bottleTFUpdate B;
	
	while(ros::ok()){
		ros::spinOnce(); 
		B.leftTFPublisher(B.bottle_left);
		B.rightTFPublisher(B.bottle_right);
		ros::Duration(1).sleep();
	}
	return 0; 	
}
