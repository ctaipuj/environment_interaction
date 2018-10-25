  #include <moveit/planning_scene_interface/planning_scene_interface.h>
  #include <moveit_msgs/CollisionObject.h>

  #include <tf/transform_listener.h>


int main(int argc, char** argv){
	ros::init(argc, argv, "bins_tracker");
	ros::NodeHandle node; 
	
	tf::TransformListener listener; //tf listener & transformations
	tf::StampedTransform t_tool;
	
	moveit_msgs::CollisionObject tool_tag; //create the collition object
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  	tool_tag.header.frame_id = "base"; //"father frame"

  	tool_tag.id = "tool_tag"; //id

  	shape_msgs::SolidPrimitive primitive; 
  	primitive.type = primitive.BOX;  
  
  	primitive.dimensions.resize(3); 
  	primitive.dimensions[0] = 0.05;
  	primitive.dimensions[1] = 0.05;
  	primitive.dimensions[2] = 0.05;
  	tool_tag.primitives.resize(1);
  	tool_tag.primitives[0]=primitive;
  	tool_tag.operation = tool_tag.ADD; //add object to collitions 
	tool_tag.primitive_poses.resize(1);
  	
  	while(node.ok()){
  	
  		try{
			listener.lookupTransform("base", "Herramienta", ros::Time(0), t_tool);  //TODO change the strings according to frames IDs
		}
		
    		catch (tf::TransformException ex){
			ROS_WARN("%s",ex.what());
		}
  	
 		tool_tag.primitive_poses[0].position.x= t_tool.getOrigin().x();
 		tool_tag.primitive_poses[0].position.y= t_tool.getOrigin().y();
 		tool_tag.primitive_poses[0].position.z= t_tool.getOrigin().z();
		tool_tag.primitive_poses[0].orientation.w= t_tool.getRotation().w();
 		tool_tag.primitive_poses[0].orientation.x= t_tool.getRotation().x();
 		tool_tag.primitive_poses[0].orientation.y= t_tool.getRotation().y();
 		tool_tag.primitive_poses[0].orientation.z= t_tool.getRotation().z();

  		planning_scene_interface.applyCollisionObject(tool_tag); //add objects to planning interface*/
  		ros::Duration(0.5).sleep();
	}  	
}
