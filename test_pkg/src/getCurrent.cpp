#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "getCurrent");
	ros::NodeHandle n;
	ros::AsyncSpinner spinner(1);
    spinner.start();
	static const std::string PLANNING_GROUP = "manipulator";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());
    geometry_msgs::Pose current_pose;
    std::vector< double > current_rpy;
	// ros::Rate loop_rate(0.2);
	while (ros::ok())
	{
        current_pose = move_group.getCurrentPose().pose;
	    current_rpy = move_group.getCurrentRPY();
        ROS_INFO("current pose x : %lf",current_pose.position.x);
    	ROS_INFO("current pose y : %lf",current_pose.position.y);
    	ROS_INFO("current pose z : %lf",current_pose.position.z);
    	ROS_INFO("current r : %lf",current_rpy[0]);
    	ROS_INFO("current p : %lf",current_rpy[1]);
    	ROS_INFO("current y : %lf",current_rpy[2]);
    	ROS_INFO("current orientation x: %lf",current_pose.orientation.x);
    	ROS_INFO("current orientation y: %lf",current_pose.orientation.y);
    	ROS_INFO("current orientation z: %lf",current_pose.orientation.z);
    	ROS_INFO("current orientation w: %lf",current_pose.orientation.w);
        ros::Duration(5.0).sleep();
		// ros::spinOnce();
		// loop_rate.sleep();
	}

	return 0;
}