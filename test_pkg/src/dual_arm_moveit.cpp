#include "ros/ros.h"
#include <std_msgs/String.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <boost/thread/thread.hpp>
#include <iostream>
#include <string>

const double PI = 3.14159265359;
const double jump_threshold = 0.0;
const double eef_step = 0.005;
static const std::string ROBOT1_PLANNING_GROUP = "robot1_manipulator";
static const std::string ROBOT2_PLANNING_GROUP = "robot2_manipulator";
static const std::string ARMS_PLANNING_GROUP = "dual_arms";


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
void thread_robot1(std::string str)
{
    std::cout<<str;
}
void thread_robot2(moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::MoveGroupInterface::Plan plan)
{
    move_group.asyncExecute(plan);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "dual_arm_moveit");
  	ros::AsyncSpinner spinner(2);
  	spinner.start();
    
    ros::NodeHandle n;
    moveit::planning_interface::MoveGroupInterface robot1_move_group(ROBOT1_PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface robot2_move_group(ROBOT2_PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface arms_move_group(ARMS_PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ROS_INFO("End effector link of robot1: %s", robot1_move_group.getEndEffectorLink().c_str());
	ROS_INFO("End effector link of robot2: %s", robot2_move_group.getEndEffectorLink().c_str());

	geometry_msgs::Pose robot1_current_pose = robot1_move_group.getCurrentPose().pose;
	std::vector<double> robot1_current_rpy = robot1_move_group.getCurrentRPY();
    // printCurrentState(robot1_current_pose,robot1_current_rpy);

	geometry_msgs::Pose robot2_current_pose = robot2_move_group.getCurrentPose().pose;
	std::vector<double> robot2_current_rpy = robot2_move_group.getCurrentRPY();
    printCurrentState(robot2_current_pose,robot2_current_rpy);

    ROS_INFO("Swith eef of robot1");
    robot1_move_group.setEndEffectorLink("robot1_suction_eef_link");
	ROS_INFO("End effector link of robot1: %s", robot1_move_group.getEndEffectorLink().c_str());
    ros::Duration(1.0).sleep();
    robot1_current_pose = robot1_move_group.getCurrentPose().pose;
    robot1_current_rpy = robot1_move_group.getCurrentRPY();
    printCurrentState(robot1_current_pose,robot1_current_rpy);

    std::vector<geometry_msgs::Pose> robot1_waypoints;
    robot1_waypoints.push_back(robot1_current_pose);
    geometry_msgs::Pose ini_pose;
	ini_pose.position.x = -0.41;
	ini_pose.position.y = 0.07;
 	ini_pose.position.z = 1.20;
    ini_pose.orientation.x = 0.0;
    ini_pose.orientation.y = 0.71;
    ini_pose.orientation.z = 0.0;
    ini_pose.orientation.w = 0.71;
    robot1_waypoints.push_back(ini_pose); 
    moveit_msgs::RobotTrajectory trajectory;
 	double fraction = robot1_move_group.computeCartesianPath(robot1_waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO("move to initial pose(cartesian path) (%.2f%% acheived)", fraction * 100.0);
    moveit::planning_interface::MoveGroupInterface::Plan robot1_plan;
	robot1_plan.trajectory_ = trajectory;
    std::cout<<trajectory.multi_dof_joint_trajectory.header.stamp.sec<<std::endl<<trajectory.multi_dof_joint_trajectory.header.stamp.nsec<<std::endl;


    std::vector<geometry_msgs::Pose> robot2_waypoints;
    robot2_waypoints.push_back(robot2_current_pose);
    geometry_msgs::Pose tar_pose;
	tar_pose.position.x = 1.327674;
	tar_pose.position.y = 0.224720;
 	tar_pose.position.z = 1.249712;
    tar_pose.orientation.x = -0.673816;
    tar_pose.orientation.y = 0.099909;
    tar_pose.orientation.z = 0.732074;
    tar_pose.orientation.w = 0.007581;
    robot2_waypoints.push_back(tar_pose); 
    moveit_msgs::RobotTrajectory robot2_trajectory;
    fraction = robot2_move_group.computeCartesianPath(robot2_waypoints, eef_step, jump_threshold, robot2_trajectory);
    ROS_INFO("move to target pose(cartesian path) (%.2f%% acheived)", fraction * 100.0);
    moveit::planning_interface::MoveGroupInterface::Plan robot2_plan;
	robot2_plan.trajectory_ = robot2_trajectory;
    std::cout<<trajectory.multi_dof_joint_trajectory.header.stamp.sec<<std::endl<<trajectory.multi_dof_joint_trajectory.header.stamp.nsec<<std::endl;

    // boost::thread task2(thread_robot2,std::ref(robot2_move_group),robot2_plan);
    // ros::Duration(0.1).sleep();
    // robot1_move_group.asyncExecute(robot1_plan);

    // task2.detach();
    



}