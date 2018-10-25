#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <vision/bottle_data.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
  
class bottleSceneUpdate{

	private:
		ros::NodeHandle n;
		ros::Subscriber sub_left;
		ros::Subscriber sub_right;
	
		float length,width;
		double r,p,y;
		std::string bottle_left,bottle_right;
		std::string REFERENCE_FRAME;
		
		geometry_msgs::PoseStamped pose;
		moveit_msgs::CollisionObject bottle_object;
		moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
		shape_msgs::SolidPrimitive primitive;
		
		tf::Quaternion q;
		tf::Matrix3x3 m;
	
	public:
		bottleSceneUpdate(){
			sub_left=n.subscribe("vision/bottle_data_arm_left",10, &bottleSceneUpdate::bottleLeftObjectCallback,this);
			sub_right=n.subscribe("vision/bottle_data_arm_right",10, &bottleSceneUpdate::bottleRightObjectCallback,this);
			primitive.type = primitive.CYLINDER;
			bottle_left="bottle_left_arm";
			bottle_right="bottle_right_arm";
			REFERENCE_FRAME="torso_base_link";
  			bottle_object.primitives.resize(1);
  			primitive.dimensions.resize(2);
		}
	
		void bottleLeftObjectCallback(const vision::bottle_data::ConstPtr& msg){
			length=msg->length;
			width=msg->width;
			pose=msg->P;
			if(pose.pose.position.z>0.3) //A very bad checkpoint wrong data
				bottleUpdater(bottle_left);
		}
		
		void bottleRightObjectCallback(const vision::bottle_data::ConstPtr& msg){
			length=msg->length;
			width=msg->width;
			pose=msg->P;
			if(pose.pose.position.z>0.3) //A very bad checkpoint to skip wrong data
				bottleUpdater(bottle_right);
		}
		
		void bottleUpdater(std::string bottle){
			
			bottle_object.header.frame_id = REFERENCE_FRAME; //"father frame"
  			bottle_object.id = bottle; //id
  			
  			primitive.dimensions[0] = length;
  			primitive.dimensions[1] = width/2;
  	 
  			bottle_object.primitives[0]=primitive;
  			bottle_object.operation = bottle_object.ADD; //add object to collitions 
			bottle_object.primitive_poses.resize(1);
			
			tf::Quaternion q(pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w);
			tf::Matrix3x3 m(q);
			m.getRPY(r,p,y);
			q.setRPY(r+1.5708,p,y);
  	
 			bottle_object.primitive_poses[0].position.x= pose.pose.position.x;
 			bottle_object.primitive_poses[0].position.y= pose.pose.position.y;
 			bottle_object.primitive_poses[0].position.z= pose.pose.position.z;
			bottle_object.primitive_poses[0].orientation.w= q.w();
 			bottle_object.primitive_poses[0].orientation.x= q.x();
 			bottle_object.primitive_poses[0].orientation.y= q.y();
 			bottle_object.primitive_poses[0].orientation.z= q.z();
 		
  			planning_scene_interface.applyCollisionObject(bottle_object); //add objects to planning interface*/
			//ros::Duration(0.5).sleep();
			ROS_INFO("%s published to robot's environment",bottle.c_str());
		}
};
 
int main(int argc, char** argv){
	ros::init(argc, argv, "bottles_to_environment");
	ros::NodeHandle node; 
	bottleSceneUpdate B;
  	ros::spin();  	
}
