#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <object_recognition_msgs/GetObjectInformation.h>
#include "ur_modern_driver/RG2.h"

const double PI = 3.14159265359;
geometry_msgs::Pose obj_pose;
int num = 0;

void targetCallback(std_msgs::Float64 target_w)
{
	ur_modern_driver::RG2 srv;
	double target = target_w.data;
	srv.request.target_width = target;
	if(ros::service::call("rg2_gripper/control_width",srv))
	{
		ROS_INFO("current width: ",target);
	}
	else
	{
		ROS_ERROR("failed to call service");
	}
}

void printCurrentState(geometry_msgs::Pose current_pose, std::vector< double > current_rpy)
{
	
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
}

void poseCallback(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg)
{
	if (num < 1)
	{
		int obj_id = 0;
    	object_recognition_msgs::RecognizedObjectArray::_objects_type::const_iterator it;
		geometry_msgs::PoseStamped msg_obj_cam,msg_obj_pose;
    	for (it = msg->objects.begin(); it != msg->objects.end(); ++it)
    	{
        msg_obj_cam.header = msg->header;
        msg_obj_cam.header.stamp = ros::Time(0);
        msg_obj_cam.pose = it->pose.pose.pose;
		tf::TransformListener listener;
		listener.waitForTransform("base_link","camera_link",ros::Time(0),ros::Duration(2.0));
        listener.transformPose("base_link",msg_obj_cam,msg_obj_pose);
        obj_pose = msg_obj_pose.pose;
        ROS_INFO("POSITON X: %lf",obj_pose.position.x);
        ROS_INFO("POSITON Y: %lf",obj_pose.position.y);
        ROS_INFO("POSITON Z: %lf",obj_pose.position.z);
        ++obj_id;
    	}
		num++;
	}
}
int main(int argc, char *argv[])
{

	ros::init(argc, argv, "pick_place1");
  	ros::AsyncSpinner spinner(1);
  	spinner.start();

	ros::NodeHandle n;
	ros::Duration sleep_time(2.0);
  	sleep_time.sleep();

	static const std::string PLANNING_GROUP = "manipulator";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	move_group.setPlanningTime(30);
	move_group.setNumPlanningAttempts(10);
    move_group.setMaxVelocityScalingFactor(0.03);
	move_group.setMaxAccelerationScalingFactor(0.1);

	const robot_state::JointModelGroup *joint_model_group =
   		move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
	ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
	
	
	ros::ServiceClient client = n.serviceClient<ur_modern_driver::RG2>("rg2_gripper/control_width");
	ros::Subscriber sub = n.subscribe("target",1,targetCallback);
	ros::Subscriber pose_sub = n.subscribe<object_recognition_msgs::RecognizedObjectArray>("/recognized_object_array",1,poseCallback);

	geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;
	std::vector< double > current_rpy = move_group.getCurrentRPY();
	
	geometry_msgs::Pose ini_pose;
	ini_pose.orientation.y = -0.707;
 	ini_pose.orientation.z = -0.706;
	ini_pose.position.x = -0.318;
	ini_pose.position.y = -0.341;
 	ini_pose.position.z = 0.319;
	printCurrentState(current_pose,current_rpy);

	ros::Duration(2.0).sleep();
	ROS_INFO("set pre_grasp_pose");
	geometry_msgs::Pose target_pose1;
	target_pose1.position.x = obj_pose.position.x + 0.1;
	target_pose1.position.y = obj_pose.position.y - 0.009;
	target_pose1.position.z = obj_pose.position.z + 0.05;
	geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(PI/2,0,-PI);
	target_pose1.orientation = quat;
	// move_group.setPoseTarget(target_pose1);
	ROS_INFO("pre_grasp_pose x : %lf",target_pose1.position.x);
	ROS_INFO("pre_grasp_pose y : %lf",target_pose1.position.y);
	ROS_INFO("pre_grasp_pose z : %lf",target_pose1.position.z);

	current_pose = move_group.getCurrentPose().pose;
	std::vector<geometry_msgs::Pose> waypoints;
	waypoints.push_back(current_pose);
 	waypoints.push_back(target_pose1); 
	moveit_msgs::RobotTrajectory trajectory1;
 	const double jump_threshold = 10;
 	const double eef_step = 0.01;// change from 0.01 to 0.002
 	double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory1);
 	ROS_INFO_NAMED("tutorial", "Visualizing move2 (cartesian path) (%.2f%% acheived)", fraction * 100.0);
	moveit::planning_interface::MoveGroupInterface::Plan plan1;
	plan1.trajectory_ = trajectory1;
	move_group.execute(plan1);

	ROS_INFO("open gripper");
	ur_modern_driver::RG2 srv;
	float target_width;
	target_width = 90.0;
	srv.request.target_width = target_width;
	if(client.call(srv))
	{
		ROS_INFO("current width: ",target_width);
	}
	else
	{
		ROS_INFO("failed to call service");
	}
	ros::Duration(2.0).sleep();


	waypoints.clear();
	current_pose = move_group.getCurrentPose().pose;
	waypoints.push_back(current_pose);
  	geometry_msgs::Pose target_pose2 = current_pose;
  	target_pose2.position.x -= 0.10;
 	waypoints.push_back(target_pose2); 

	moveit_msgs::RobotTrajectory trajectory2;
	fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory2);
 	ROS_INFO_NAMED("tutorial", "Visualizing move2 (cartesian path) (%.2f%% acheived)", fraction * 100.0);
	moveit::planning_interface::MoveGroupInterface::Plan plan2;
	plan2.trajectory_ = trajectory2;
	move_group.execute(plan2);
	ros::Duration(2.0).sleep();

	current_pose = move_group.getCurrentPose().pose;
	current_rpy = move_group.getCurrentRPY();
	printCurrentState(current_pose,current_rpy);

	ROS_INFO("close gripper");
	target_width = 74.0;
	srv.request.target_width = target_width;
	client.call(srv);
	ros::Duration(2.0).sleep();


	ROS_INFO("move up");
	waypoints.clear();
	current_pose = move_group.getCurrentPose().pose;
	waypoints.push_back(current_pose);
	geometry_msgs::Pose target_pose3 = current_pose;
	target_pose3.position.z += 0.2;
	waypoints.push_back(target_pose3);
	target_pose3.position.y += 0.3;
	waypoints.push_back(target_pose3);
	target_pose3.position.z -= 0.2;
	waypoints.push_back(target_pose3);
	moveit_msgs::RobotTrajectory trajectory3;
	fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory3);
 	ROS_INFO_NAMED("tutorial", "Visualizing move3 (cartesian path) (%.2f%% acheived)", fraction * 100.0);
	moveit::planning_interface::MoveGroupInterface::Plan plan3;
	plan3.trajectory_ = trajectory3;
	move_group.execute(plan3);
	ros::Duration(2.0).sleep();
	current_pose = move_group.getCurrentPose().pose;
	current_rpy = move_group.getCurrentRPY();
	printCurrentState(current_pose,current_rpy);


	// ROS_INFO("set place_pose");
	// geometry_msgs::Pose target_pose4;
	// target_pose4.position.x = current_pose.position.x;
	// target_pose4.position.y = current_pose.position.y + 0.3;
	// target_pose4.position.z = current_pose.position.z;
	// geometry_msgs::Quaternion quate = tf::createQuaternionMsgFromRollPitchYaw(PI/2,0,-PI);
	// target_pose4.orientation = quate;
 	// move_group.setPoseTarget(target_pose4);

	// moveit::planning_interface::MoveGroupInterface::Plan plan4;
	// success = (move_group.plan(plan4) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	// ROS_INFO_NAMED("tutorial", "Visualizing move 4 (pose goal) %s", success ? "" : "FAILED");
	// move_group.execute(plan4);
	// ros::Duration(2.0).sleep();
	// current_pose = move_group.getCurrentPose().pose;
	// current_rpy = move_group.getCurrentRPY();
	// printCurrentState(current_pose,current_rpy);

	// ROS_INFO("move down");
	// waypoints.clear();
	// waypoints.push_back(move_group.getCurrentPose().pose);
	// geometry_msgs::Pose target_pose5 = move_group.getCurrentPose().pose;
	// target_pose5.position.z -= 0.20;
	// waypoints.push_back(target_pose5);
	// fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
 	// ROS_INFO_NAMED("tutorial", "Visualizing move 5 (cartesian path) (%.2f%% acheived)", fraction * 100.0);
	// ros::Duration(2.0).sleep();
	// moveit::planning_interface::MoveGroupInterface::Plan plan5;
	// plan5.trajectory_ = trajectory;
	// move_group.execute(plan5);
	// ros::Duration(2.0).sleep();
	// current_pose = move_group.getCurrentPose().pose;
	// current_rpy = move_group.getCurrentRPY();
	// printCurrentState(current_pose,current_rpy);

	ROS_INFO("open gripper");
	target_width = 90.0;
	srv.request.target_width = target_width;
	client.call(srv);
	ros::Duration(2.0).sleep();

	ROS_INFO("move up again");
	current_pose = move_group.getCurrentPose().pose;
	waypoints.clear();
	waypoints.push_back(current_pose);
	geometry_msgs::Pose target_pose6 = current_pose;
	target_pose6.position.z += 0.2;
	waypoints.push_back(target_pose6);
	waypoints.push_back(ini_pose);
	moveit_msgs::RobotTrajectory trajectory4;
	fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory4);
 	ROS_INFO_NAMED("tutorial", "Visualizing move 6 (cartesian path) (%.2f%% acheived)", fraction * 100.0);
	moveit::planning_interface::MoveGroupInterface::Plan plan6;
	plan6.trajectory_ = trajectory4;
	move_group.execute(plan6);
	ros::Duration(2.0).sleep();
	current_pose = move_group.getCurrentPose().pose;
	current_rpy = move_group.getCurrentRPY();
	printCurrentState(current_pose,current_rpy);

	ROS_INFO("close gripper");
	target_width = 74.0;
	srv.request.target_width = target_width;
	client.call(srv);
	ros::Duration(2.0).sleep();
	// ROS_INFO("move to initial pose");
 	// move_group.setPoseTarget(ini_pose);
	// moveit::planning_interface::MoveGroupInterface::Plan plan7;
	// success = (move_group.plan(plan7) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	// ROS_INFO_NAMED("tutorial", "Visualizing move 7 (pose goal) %s", success ? "" : "FAILED");
	// move_group.execute(plan7);
	// ros::Duration(2.0).sleep();
	// current_pose = move_group.getCurrentPose().pose;
	// current_rpy = move_group.getCurrentRPY();
	// printCurrentState(current_pose,current_rpy);

	return 0;
}