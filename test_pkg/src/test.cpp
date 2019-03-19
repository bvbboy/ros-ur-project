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
// ros::ServiceClient client;
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
const double PI = 3.14159265359;

geometry_msgs::Pose obj_pose;
int num = 0;

void targetCallback(std_msgs::Float64 target_w)
{
	ur_modern_driver::RG2 srv;
	double target = target_w.data;
	srv.request.target_width = target;
	// if(client.call(srv))
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

	ros::init(argc, argv, "testtest");
  	ros::AsyncSpinner spinner(1);
  	spinner.start();

	ros::NodeHandle n;
	ros::Duration sleep_time(5.0);
  	sleep_time.sleep();

	static const std::string PLANNING_GROUP = "manipulator";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	move_group.setPlanningTime(30);
	move_group.setNumPlanningAttempts(10);
	//move_group.setPoseReferenceFrame("base_link");
	const robot_state::JointModelGroup *joint_model_group =
   		move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
	ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
	
	// ros::Publisher planning_secne_diff_publisher = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
	ros::ServiceClient client = n.serviceClient<ur_modern_driver::RG2>("rg2_gripper/control_width");
	ros::Subscriber sub = n.subscribe("target",1,targetCallback);
	ros::Subscriber pose_sub = n.subscribe<object_recognition_msgs::RecognizedObjectArray>("/recognized_object_array",1,poseCallback);
	// ros::Rate loop_rate(10);
	// ROS_INFO("remove collision object");
	// moveit_msgs::CollisionObject collision_object;

	// std::vector<std::string> object_ids;
	// object_ids = planning_scene_interface.getKnownObjectNames();
    // // object_ids.push_back(collision_object.id);
    // planning_scene_interface.removeCollisionObjects(object_ids);
	// object_ids = planning_scene_interface.getKnownObjectNames();
	// ROS_INFO("size %i",object_ids.size());
	// ros::Duration(5.0).sleep();

	// ROS_INFO("add collision object");
	// collision_object.header.frame_id = move_group.getPlanningFrame();
	// // collision_object.header.frame_id = "world";
	// collision_object.id = "table";
	// shape_msgs::SolidPrimitive primitive;
    // primitive.type = primitive.BOX;
    // primitive.dimensions.resize(3);
    // primitive.dimensions[0] = 1.4;
    // primitive.dimensions[1] = 1.4;
    // primitive.dimensions[2] = 0.02;
    
	// geometry_msgs::Pose table_pose;
 	// table_pose.orientation.w = 1.0;
    // table_pose.position.x = 0.0;
    // table_pose.position.y = 0.0;
    // table_pose.position.z = -0.03;

	// collision_object.primitives.push_back(primitive);
    // collision_object.primitive_poses.push_back(table_pose);
    // collision_object.operation = collision_object.ADD;

	// std::vector<moveit_msgs::CollisionObject> collision_objects;
    // collision_objects.push_back(collision_object);
	// ROS_INFO_NAMED("tutorial", "Add an object into the world");
	// planning_scene_interface.addCollisionObjects(collision_objects);
	// ros::Duration(5.0).sleep();


	/**
	 * A count of how many messages we have sent. This is used to create
	 * a unique string for each message.
	 */
	// ROS_INFO("set target_pose1");
	// geometry_msgs::Pose target_pose1;
 	// target_pose1.orientation.w = 0.5;
	// target_pose1.position.x = 0.3;
	// target_pose1.position.y = 0.3;
 	// target_pose1.position.z = 0.4;
 	// move_group.setPoseTarget(target_pose1);

	// moveit::planning_interface::MoveGroupInterface::Plan plan1;
	// bool success = (move_group.plan(plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	// ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
	// move_group.setMaxVelocityScalingFactor(0.1);
    // // move_group.move();
	// move_group.execute(plan1);
	// ros::Duration(10.0).sleep();

	geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;
	std::vector< double > current_rpy = move_group.getCurrentRPY();
	
	// ROS_INFO("current pose x : %lf",current_pose.position.x);
	// ROS_INFO("current pose y : %lf",current_pose.position.y);
	// ROS_INFO("current pose z : %lf",current_pose.position.z);
	// ROS_INFO("current r : %lf",current_rpy[0]);
	// ROS_INFO("current p : %lf",current_rpy[1]);
	// ROS_INFO("current y : %lf",current_rpy[2]);
	// ROS_INFO("current orientation x: %lf",current_pose.orientation.x);
	// ROS_INFO("current orientation y: %lf",current_pose.orientation.y);
	// ROS_INFO("current orientation z: %lf",current_pose.orientation.z);
	// ROS_INFO("current orientation w: %lf",current_pose.orientation.w);

	// moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
	// std::vector<double> joint_group_positions;
  	// current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
	// joint_group_positions[0] = -1.0;  // radians
  	// move_group.setJointValueTarget(joint_group_positions);
	// moveit::planning_interface::MoveGroupInterface::Plan plan2;
	// success = (move_group.plan(plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  	// ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
	// ros::Duration(5.0).sleep();
	
	// std::vector<geometry_msgs::Pose> waypoints;
	// waypoints.push_back(move_group.getCurrentPose().pose);

  	// geometry_msgs::Pose target_pose3 = move_group.getCurrentPose().pose;

  	// target_pose3.position.z += 0.1;
  	// waypoints.push_back(target_pose3);  // up 

  	// target_pose3.position.y -= 0.1;
 	// waypoints.push_back(target_pose3);  // left

  	// target_pose3.position.z -= 0.15;
  	// target_pose3.position.y += 0.15;
  	// target_pose3.position.x -= 0.15;
  	// waypoints.push_back(target_pose3);  // down and right 

	// moveit_msgs::RobotTrajectory trajectory;
 	// const double jump_threshold = 0.0;
 	// const double eef_step = 0.01;
 	// double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
 	// ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (cartesian path) (%.2f%% acheived)", fraction * 100.0);
	// ros::Duration(5.0).sleep();
	// moveit::planning_interface::MoveGroupInterface::Plan plan3;
	// plan3.trajectory_ = trajectory;
	// move_group.execute(plan3);
	move_group.setMaxVelocityScalingFactor(0.1);
	move_group.setMaxAccelerationScalingFactor(0.1);
	
	geometry_msgs::Pose ini_pose;
	ini_pose.orientation.y = -0.707;
 	ini_pose.orientation.z = -0.706;
	ini_pose.position.x = -0.318;
	ini_pose.position.y = -0.341;
 	ini_pose.position.z = 0.319;
	printCurrentState(current_pose,current_rpy);

	ros::Duration(5.0).sleep();
	ROS_INFO("set target_pose1");
	geometry_msgs::Pose target_pose1;
	// target_pose1.orientation.x = 0.707;
 	// target_pose1.orientation.w = 0.706;
	// target_pose1.position.x = 0.355;
	// target_pose1.position.y = 0.507;
 	// target_pose1.position.z = 0.101;
	// move_group.setPositionTarget(target_pose1.position.x,target_pose1.position.y,target_pose1.position.z);
	// move_group.setRPYTarget(PI/2,0,PI/4,move_group.getEndEffectorLink().c_str());
	target_pose1.position.x = obj_pose.position.x + 0.1;
	target_pose1.position.y = obj_pose.position.y - 0.015;
	target_pose1.position.z = obj_pose.position.z + 0.06;
	// geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(PI/2,0,-3*PI/4);
	geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(PI/2,0,-PI);
	target_pose1.orientation = quat;
	move_group.setPoseTarget(target_pose1);
	ROS_INFO("target_pose1 x : %lf",target_pose1.position.x);
	ROS_INFO("target_pose1 y : %lf",target_pose1.position.y);
	ROS_INFO("target_pose1 z : %lf",target_pose1.position.z);

	moveit::planning_interface::MoveGroupInterface::Plan plan1;
	bool success = (move_group.plan(plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("tutorial", "Visualizing move 1 (pose goal) %s", success ? "" : "FAILED");

    // move_group.move();
	move_group.execute(plan1);
	current_pose = move_group.getCurrentPose().pose;
	current_rpy = move_group.getCurrentRPY();
	printCurrentState(current_pose,current_rpy);
	// ros::Duration(5.0).sleep();
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
		return 1;
	}

	ros::Duration(2.0).sleep();

	std::vector<geometry_msgs::Pose> waypoints;
	waypoints.push_back(move_group.getCurrentPose().pose);

  	geometry_msgs::Pose target_pose2 = move_group.getCurrentPose().pose;
	
  	target_pose2.position.x -= 0.1;
  	// target_pose2.position.y -= 0.1;
 	waypoints.push_back(target_pose2); 

	moveit_msgs::RobotTrajectory trajectory;
 	const double jump_threshold = 0.0;
 	const double eef_step = 0.01;
 	double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
 	ROS_INFO_NAMED("tutorial", "Visualizing move2 (cartesian path) (%.2f%% acheived)", fraction * 100.0);
	ros::Duration(5.0).sleep();
	moveit::planning_interface::MoveGroupInterface::Plan plan2;
	plan2.trajectory_ = trajectory;
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
	waypoints.push_back(move_group.getCurrentPose().pose);
	geometry_msgs::Pose target_pose3 = move_group.getCurrentPose().pose;
	target_pose3.position.z += 0.2;
	waypoints.push_back(target_pose3);
	fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
 	ROS_INFO_NAMED("tutorial", "Visualizing move3 (cartesian path) (%.2f%% acheived)", fraction * 100.0);
	ros::Duration(2.0).sleep();
	moveit::planning_interface::MoveGroupInterface::Plan plan3;
	plan3.trajectory_ = trajectory;
	move_group.execute(plan3);
	ros::Duration(2.0).sleep();
	current_pose = move_group.getCurrentPose().pose;
	current_rpy = move_group.getCurrentRPY();
	printCurrentState(current_pose,current_rpy);


	ROS_INFO("set target_pose4");
	geometry_msgs::Pose target_pose4;
	// target_pose4.orientation.x = 0.707;
 	// target_pose4.orientation.w = 0.706;
	// target_pose4.position.x = 0.417;
	// target_pose4.position.y = 0.142;
 	// target_pose4.position.z = 0.301;
	target_pose4.position.x = current_pose.position.x;
	target_pose4.position.y = current_pose.position.y + 0.3;
	target_pose4.position.z = current_pose.position.z;
	geometry_msgs::Quaternion quate = tf::createQuaternionMsgFromRollPitchYaw(PI/2,0,-PI);
	target_pose4.orientation = quate;
 	move_group.setPoseTarget(target_pose4);
	// move_group.setPositionTarget(target_pose4.position.x,target_pose4.position.y,target_pose4.position.z);
	// move_group.setRPYTarget(PI/2,0,-PI/4,move_group.getEndEffectorLink().c_str());

	moveit::planning_interface::MoveGroupInterface::Plan plan4;
	success = (move_group.plan(plan4) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("tutorial", "Visualizing move 4 (pose goal) %s", success ? "" : "FAILED");
	//move_group.setMaxVelocityScalingFactor(0.1);
	move_group.execute(plan4);
	ros::Duration(2.0).sleep();
	current_pose = move_group.getCurrentPose().pose;
	current_rpy = move_group.getCurrentRPY();
	printCurrentState(current_pose,current_rpy);

	ROS_INFO("move down");
	waypoints.clear();
	waypoints.push_back(move_group.getCurrentPose().pose);
	geometry_msgs::Pose target_pose5 = move_group.getCurrentPose().pose;
	target_pose5.position.z -= 0.20;
	waypoints.push_back(target_pose5);
	fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
 	ROS_INFO_NAMED("tutorial", "Visualizing move 5 (cartesian path) (%.2f%% acheived)", fraction * 100.0);
	ros::Duration(2.0).sleep();
	moveit::planning_interface::MoveGroupInterface::Plan plan5;
	plan5.trajectory_ = trajectory;
	move_group.execute(plan5);
	ros::Duration(2.0).sleep();
	current_pose = move_group.getCurrentPose().pose;
	current_rpy = move_group.getCurrentRPY();
	printCurrentState(current_pose,current_rpy);

	ROS_INFO("open gripper");
	target_width = 90.0;
	srv.request.target_width = target_width;
	client.call(srv);
	ros::Duration(2.0).sleep();

	ROS_INFO("move up again");
	waypoints.clear();
	waypoints.push_back(move_group.getCurrentPose().pose);
	geometry_msgs::Pose target_pose6 = move_group.getCurrentPose().pose;
	target_pose6.position.z += 0.2;
	waypoints.push_back(target_pose6);
	fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
 	ROS_INFO_NAMED("tutorial", "Visualizing move 6 (cartesian path) (%.2f%% acheived)", fraction * 100.0);
	ros::Duration(2.0).sleep();
	moveit::planning_interface::MoveGroupInterface::Plan plan6;
	plan6.trajectory_ = trajectory;
	move_group.execute(plan6);
	ros::Duration(2.0).sleep();
	current_pose = move_group.getCurrentPose().pose;
	current_rpy = move_group.getCurrentRPY();
	printCurrentState(current_pose,current_rpy);

	ROS_INFO("move to initial");
 	move_group.setPoseTarget(ini_pose);
	moveit::planning_interface::MoveGroupInterface::Plan plan7;
	success = (move_group.plan(plan7) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("tutorial", "Visualizing move 7 (pose goal) %s", success ? "" : "FAILED");
	//move_group.setMaxVelocityScalingFactor(0.1);
	move_group.execute(plan7);
	ros::Duration(2.0).sleep();
	current_pose = move_group.getCurrentPose().pose;
	current_rpy = move_group.getCurrentRPY();
	printCurrentState(current_pose,current_rpy);


	//ros::shutdown();
	// ros::spin();
	return 0;
}