#include "ros/ros.h"
#include "std_msgs/String.h"
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


const double PI = 3.14159265359; 

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "addCollision");
  	ros::AsyncSpinner spinner(1);
  	spinner.start();

	ros::NodeHandle n;
    static const std::string PLANNING_GROUP = "manipulator";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	move_group.setPlanningTime(30);
	move_group.setNumPlanningAttempts(10);
    move_group.setMaxVelocityScalingFactor(0.03);
	move_group.setMaxAccelerationScalingFactor(0.03);
	
	geometry_msgs::Pose ini_pose;
	ini_pose.orientation.y = -0.707;
 	ini_pose.orientation.z = -0.706;
	ini_pose.position.x = -0.318;
	ini_pose.position.y = -0.341;
 	ini_pose.position.z = 0.319;

    const robot_state::JointModelGroup *joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    move_group.setRandomTarget();
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    move_group.plan(plan);
    move_group.execute(plan);
    ros::Duration(2.0).sleep();
    // move_group.setRPYTarget(PI/2,0,PI/4,move_group.getEndEffectorLink().c_str());
    move_group.setPoseTarget(ini_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    move_group.plan(plan1);
    move_group.execute(plan1);
    ros::Duration(2.0).sleep();
    geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;
    ROS_INFO("pose x: %lf",current_pose.position.x);
    ROS_INFO("pose y: %lf",current_pose.position.y);
    ROS_INFO("pose z: %lf",current_pose.position.z);

    // move_group.setPoseReferenceFrame("rg2_eef_link");
    // current_pose = move_group.getCurrentPose().pose;
    // std::vector<geometry_msgs::Pose> waypoints;
    // waypoints.push_back(current_pose);
    // ROS_INFO("pose x: %lf",current_pose.position.x);
    // ROS_INFO("pose y: %lf",current_pose.position.y);
    // ROS_INFO("pose z: %lf",current_pose.position.z);
    // current_pose.position.x += 0.1;
    // ROS_INFO("after pose x: %lf",current_pose.position.x);
    // ROS_INFO("after pose y: %lf",current_pose.position.y);
    // ROS_INFO("after pose z: %lf",current_pose.position.z);
    // waypoints.push_back(current_pose);
    // moveit::planning_interface::MoveGroupInterface::Plan plan1;
    // moveit_msgs::RobotTrajectory trajectory;
 	// const double jump_threshold = 0.0;
 	// const double eef_step = 0.01;
 	// double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
 	// ROS_INFO_NAMED("tutorial", "Visualizing move (cartesian path) (%.2f%% acheived)", fraction * 100.0);
	// ros::Duration(5.0).sleep();
    // plan1.trajectory_ = trajectory;
    // move_group.execute(plan1);

    // move_group.setRPYTarget(PI/2,0,PI/4,move_group.getEndEffectorLink().c_str());
    // moveit::planning_interface::MoveGroupInterface::Plan plan;
    // move_group.plan(plan);
    // move_group.execute(plan);
    // namespace rvt = rviz_visual_tools;
    // moveit_visual_tools::MoveItVisualTools visual_tools("odom_combined");
    // visual_tools.deleteAllMarkers();
    // visual_tools.loadRemoteControl();
    // Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    // text_pose.translation().z() = 1.75;
    // visual_tools.publishText(text_pose, "Collision Demo", rvt::WHITE, rvt::XLARGE);
    // visual_tools.trigger();
    
    // ROS_INFO("remove collision object");
	// moveit_msgs::CollisionObject collision_object;
	// std::vector<std::string> object_ids;
	// object_ids = planning_scene_interface.getKnownObjectNames();
    // planning_scene_interface.removeCollisionObjects(object_ids);
	// // object_ids = planning_scene_interface.getKnownObjectNames();
	// // ROS_INFO("size %i",object_ids.size());
	// ros::Duration(2.0).sleep();

	// ROS_INFO("add collision object");
	// collision_object.header.frame_id = move_group.getPlanningFrame();
	// collision_object.id = "cube_collision";
	// shape_msgs::SolidPrimitive primitive;
    // primitive.type = primitive.BOX;
    // primitive.dimensions.resize(3);
    // primitive.dimensions[0] = 0.05;
    // primitive.dimensions[1] = 0.1;
    // primitive.dimensions[2] = 0.1;
    
	// geometry_msgs::Pose table_pose;
 	// table_pose.orientation.w = 1.0;
    // table_pose.position.x = 0.5;
    // table_pose.position.y = 0.2;
    // table_pose.position.z = 0.3;

	// collision_object.primitives.push_back(primitive);
    // collision_object.primitive_poses.push_back(table_pose);
    // collision_object.operation = collision_object.ADD;

	// std::vector<moveit_msgs::CollisionObject> collision_objects;
    // collision_objects.push_back(collision_object);
	// // ROS_INFO("Add an object into the world");
	// planning_scene_interface.addCollisionObjects(collision_objects);
	// ros::Duration(2.0).sleep();

    // ROS_INFO("set initial pose");
    // geometry_msgs::Pose ini_pose;
	// ini_pose.orientation.x = 0.707;
 	// ini_pose.orientation.w = 0.706;
	// ini_pose.position.x = 0.6;
	// ini_pose.position.y = 0.0;
 	// ini_pose.position.z = 0.3;
    // move_group.setPoseTarget(ini_pose);

    // moveit::planning_interface::MoveGroupInterface::Plan plan1;
	// bool success = (move_group.plan(plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	// ROS_INFO_NAMED("tutorial", "Visualizing move to initial pose %s", success ? "" : "FAILED");
	// move_group.setMaxVelocityScalingFactor(0.1);
	// move_group.setMaxAccelerationScalingFactor(0.1);
    // ros::Duration(5.0).sleep();

    // visual_tools.publishAxisLabeled(ini_pose, "initial_pose");
    // visual_tools.publishText(text_pose, "Move To Initial Pose", rvt::WHITE, rvt::XLARGE);
    // visual_tools.publishTrajectoryLine(plan1.trajectory_, joint_model_group);
    // visual_tools.trigger();
    // // visual_tools.prompt("next step");

    // ROS_INFO("set target pose");
    // geometry_msgs::Pose tar_pose;
 	// tar_pose.orientation.w = 1.0;
	// tar_pose.position.x = 0.6;
	// tar_pose.position.y = 0.5;
 	// tar_pose.position.z = 0.3;
    // move_group.setPoseTarget(tar_pose);
    
    // moveit::planning_interface::MoveGroupInterface::Plan plan2;
	// success = (move_group.plan(plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	// ROS_INFO_NAMED("tutorial", "Visualizing move to target pose %s", success ? "" : "FAILED");
    // ros::Duration(5.0).sleep();

    // visual_tools.deleteAllMarkers();
    // visual_tools.publishAxisLabeled(ini_pose, "start");
    // visual_tools.publishAxisLabeled(tar_pose, "goal");
    // visual_tools.publishText(text_pose, "Avoid Collision", rvt::WHITE, rvt::XLARGE);
    // visual_tools.publishTrajectoryLine(plan2.trajectory_, joint_model_group);
    // visual_tools.trigger();
    // // visual_tools.prompt("next step");

    // ROS_INFO("back to initial pose");
    // //move_group.setStartState(*move_group.getCurrentState());
    // move_group.setPoseTarget(ini_pose);
    // moveit::planning_interface::MoveGroupInterface::Plan plan3;
	// success = (move_group.plan(plan3) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	// ROS_INFO_NAMED("tutorial", "Visualizing move back to initial pose %s", success ? "" : "FAILED");
    // ros::Duration(5.0).sleep();

    // ros::shutdown();
    return 0;
}


    