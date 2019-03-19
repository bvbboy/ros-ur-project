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
	static const std::string ROBOT1_PLANNING_GROUP = "robot1_manipulator";
    static const std::string ROBOT2_PLANNING_GROUP = "robot2_manipulator";
	moveit::planning_interface::MoveGroupInterface robot1_move_group(ROBOT1_PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface robot2_move_group(ROBOT2_PLANNING_GROUP);
    ROS_INFO("Reference frame: %s", robot1_move_group.getPlanningFrame().c_str());
    ROS_INFO("Reference frame: %s", robot2_move_group.getPlanningFrame().c_str());
	ROS_INFO("End effector link of robot1: %s", robot1_move_group.getEndEffectorLink().c_str());
	ROS_INFO("End effector link of robot2: %s", robot2_move_group.getEndEffectorLink().c_str());
	robot1_move_group.setEndEffectorLink("robot1_camera_eef_link");
	// robot1_move_group.setEndEffectorLink("robot1_gripper_eef_link");
	// robot1_move_group.setEndEffectorLink("camera_link");
	ROS_INFO("End effector link of robot1: %s", robot1_move_group.getEndEffectorLink().c_str());
    geometry_msgs::Pose robot1_current_pose,robot2_current_pose;
    std::vector< double > robot1_current_rpy,robot2_current_rpy;
	std::vector<double> robot1_current_joint_values;
	// ros::Rate loop_rate(0.2);
	while (ros::ok())
	{
        // robot1_current_pose = robot1_move_group.getCurrentPose().pose;
	    // robot1_current_rpy = robot1_move_group.getCurrentRPY();
		// robot1_current_joint_values = robot1_move_group.getCurrentJointValues();
        // ROS_INFO("robot1_current pose x : %lf",robot1_current_pose.position.x);
    	// ROS_INFO("robot1_current pose y : %lf",robot1_current_pose.position.y);
    	// ROS_INFO("robot1_current pose z : %lf",robot1_current_pose.position.z);
        // robot2_current_pose = robot2_move_group.getCurrentPose().pose;
        // robot2_current_rpy = robot2_move_group.getCurrentRPY();
        // ROS_INFO("robot2_current pose x : %lf",robot2_current_pose.position.x);
    	// ROS_INFO("robot2_current pose y : %lf",robot2_current_pose.position.y);
    	// ROS_INFO("robot2_current pose z : %lf",robot2_current_pose.position.z);
    	// ROS_INFO("robot1 current r : %lf",robot1_current_rpy[0]);
    	// ROS_INFO("robot1 current p : %lf",robot1_current_rpy[1]);
    	// ROS_INFO("robot1 current y : %lf",robot1_current_rpy[2]);
		// ROS_INFO("robot2 current r : %lf",robot2_current_rpy[0]);
    	// ROS_INFO("robot2 current p : %lf",robot2_current_rpy[1]);
    	// ROS_INFO("robot2 current y : %lf",robot2_current_rpy[2]);
		// ROS_INFO("robot1_current joint value 1 : %lf",robot1_current_joint_values[0]);
		
    	// ROS_INFO("current orientation x: %lf",current_pose.orientation.x);
    	// ROS_INFO("current orientation y: %lf",current_pose.orientation.y);
    	// ROS_INFO("current orientation z: %lf",current_pose.orientation.z);
    	// ROS_INFO("current orientation w: %lf",current_pose.orientation.w);

		// robot1_move_group.setEndEffectorLink("robot1_ee_link");
        // geometry_msgs::Pose robot1_current_pose = robot1_move_group.getCurrentPose().pose;
		// ros::Duration(0.1).sleep();
        // double X0 = robot1_current_pose.position.x;
        // double Y0 = robot1_current_pose.position.y;
		// ROS_INFO("ee_link x0 : %lf", X0);
		// ROS_INFO("ee_link y0 : %lf", Y0);
        // robot1_move_group.setEndEffectorLink("robot1_gripper_eef_link");
		ros::Duration(0.1).sleep();
        robot1_current_pose = robot1_move_group.getCurrentPose().pose;
		ROS_INFO("robot1_current pose x : %lf",robot1_current_pose.position.x);
    	ROS_INFO("robot1_current pose y : %lf",robot1_current_pose.position.y);
    	ROS_INFO("robot1_current pose z : %lf",robot1_current_pose.position.z);
		ROS_INFO("robot1 orientation x : %lf",robot1_current_pose.orientation.x);
    	ROS_INFO("robot1 orientation y : %lf",robot1_current_pose.orientation.y);
    	ROS_INFO("robot1 orientation z : %lf",robot1_current_pose.orientation.z);
		ROS_INFO("robot1 orientation w : %lf",robot1_current_pose.orientation.w);



        // double X1 = robot1_current_pose.position.x;
        // double Y1 = robot1_current_pose.position.y;
		// ROS_INFO("gripper_link x1 : %lf", X1);
		// ROS_INFO("gripper_link y1 : %lf", Y1);
        // double theta_0 = std::atan((Y1 - Y0) / (X1 - X0));
        // ROS_INFO("the angle of line between gripper and ee is %lf", theta_0);
        ros::Duration(2.0).sleep();
		// ros::spinOnce();
		// loop_rate.sleep();
	}

	return 0;
}