#include "ros/ros.h"
#include <std_msgs/String.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <ur_msgs/IOStates.h>
#include <ur_msgs/Digital.h>
#include <ur_msgs/SetIO.h>

#include <iostream>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "weitu.h"

geometry_msgs::Point robot1_wait_point; 
geometry_msgs::Quaternion robot1_wait_quat;

geometry_msgs::Point robot2_wait_point; 
geometry_msgs::Quaternion robot2_wait_quat;

geometry_msgs::Point upper_loc_record; 
geometry_msgs::Quaternion upper_quat_record;

geometry_msgs::Point lower_loc_record; 
geometry_msgs::Quaternion lower_quat_record;

geometry_msgs::Point controller_loc_record; 
geometry_msgs::Quaternion controller_quat_record;

geometry_msgs::Point controller_loc_record2; 
geometry_msgs::Quaternion controller_quat_record2;

int current_hole = 0;
int total_holes = 4;
std::vector<geometry_msgs::Point> hole_loc_record(total_holes);
std::vector<geometry_msgs::Quaternion> hole_quat_record(total_holes);

std::vector<geometry_msgs::Point> hole_loc_record2(total_holes);
std::vector<geometry_msgs::Quaternion> hole_quat_record2(total_holes);

bool use_cam = true;
double cam_z = 1.211108 - 1.211108 + 0.227 + 0.007;
std::vector<std::vector<double>> hole_detect_record_base(total_holes, {0, 0});
std::vector<std::vector<double>> delta_xy(total_holes);

void record_init(){
robot1_wait_point.x = 0.402874;
robot1_wait_point.y = -0.423946;
robot1_wait_point.z =  1.151709;
robot1_wait_quat = tf::createQuaternionMsgFromRollPitchYaw(-1.563320,0.025573,0.021157);

robot2_wait_point.x = 1.276787;
robot2_wait_point.y = -0.163849;
robot2_wait_point.z = 1.156983;
robot2_wait_quat = tf::createQuaternionMsgFromRollPitchYaw(2.061372,1.551123,-0.402698);

hole_loc_record[0].x =  0.777989;
hole_loc_record[0].y = -0.393309;
hole_loc_record[0].z =  1.125336;
hole_quat_record[0] = tf::createQuaternionMsgFromRollPitchYaw(-1.568676,0.023632,0.034409);

hole_loc_record[1].x = 0.924520;
hole_loc_record[1].y = -0.490436;
hole_loc_record[1].z = 1.125700;
hole_quat_record[1] = tf::createQuaternionMsgFromRollPitchYaw(-1.570561,0.025131,0.033755);

upper_loc_record.x =  0.752768;
upper_loc_record.y = -0.561114;
upper_loc_record.z =  0.984807 + 0.001;
upper_quat_record = tf::createQuaternionMsgFromRollPitchYaw(-1.568221,0.026935,0.025700);

lower_loc_record.x =  0.747596;
lower_loc_record.y = -0.238111;
lower_loc_record.z =  1.024590 - 0.001;
lower_quat_record = tf::createQuaternionMsgFromRollPitchYaw(-1.568186,0.020250,0.024984);

controller_loc_record.x =  -0.704858;
controller_loc_record.y =  -0.229268;
controller_loc_record.z =  1.015082;
controller_quat_record = tf::createQuaternionMsgFromRollPitchYaw(-1.576162,0.017326,-1.553648);

controller_loc_record2.x =  0.732074;
controller_loc_record2.y =  -0.043857;
controller_loc_record2.z =  1.019833;
controller_quat_record2 = tf::createQuaternionMsgFromRollPitchYaw(-1.548886,0.014081,1.616281);

hole_loc_record2[0].x = 0.697769;
hole_loc_record2[0].y = -0.148696;
hole_loc_record2[0].z = 1.022616 + 0.004;
hole_quat_record2[0] = tf::createQuaternionMsgFromRollPitchYaw(2.275835,1.551514,-0.194501);

hole_loc_record2[1].x = 0.812843;
hole_loc_record2[1].y = -0.266683;
hole_loc_record2[1].z = 1.026773 + 0.002;
hole_quat_record2[1] = tf::createQuaternionMsgFromRollPitchYaw(2.276561,1.554767,-0.194700);

// hole_loc_record2[2].x = ;
// hole_loc_record2[2].y = ;
// hole_loc_record2[2].z = ;
// hole_quat_record2[2] = tf::createQuaternionMsgFromRollPitchYaw(2.276561,1.554767,-0.194700);

// hole_loc_record2[3].x = ;
// hole_loc_record2[3].y = ;
// hole_loc_record2[3].z = ;
// hole_quat_record2[3] = tf::createQuaternionMsgFromRollPitchYaw(2.276561,1.554767,-0.194700);

hole_detect_record_base[0][0] = -0.010891;
hole_detect_record_base[0][1] = 0.006826;

hole_detect_record_base[1][0] = -0.005617;
hole_detect_record_base[1][1] = 0.002216;
}

    const double jump_threshold = 5;
    const double eef_step = 0.002;



// robot1当前点、目标点
bool robot1_go(geometry_msgs::Point p, geometry_msgs::Quaternion q, moveit::planning_interface::MoveGroupInterface& robot1_move_group){;
    double fraction = 0;
    std::vector<geometry_msgs::Pose> waypoints;

    geometry_msgs::Pose robot1_current_pose;
    robot1_current_pose = robot1_move_group.getCurrentPose().pose;
    waypoints.push_back(robot1_current_pose);
    geometry_msgs::Pose robot1_pose1;

    // robot1_pose1.position = robot1_current_pose.position;
	robot1_pose1.orientation = q;
    // waypoints.push_back(robot1_pose1);
    robot1_pose1.position = p;
    waypoints.push_back(robot1_pose1);
    moveit_msgs::RobotTrajectory trajectory1;
    
    fraction = robot1_move_group.computeCartesianPath(waypoints, eef_step, jump_threshold,trajectory1);
    if(fraction<0.9){
        return false;
    }
    
    ROS_WARN("Robot1 cartesian path. (%.2f%% achieved)",fraction*100.0);
    ros::Duration(1.0).sleep();
    moveit::planning_interface::MoveGroupInterface::Plan robot1_plan1;
    robot1_plan1.trajectory_ = trajectory1;
    robot1_move_group.execute(robot1_plan1);
    // ROS_INFO("robot1 is ready");
    ros::Duration(1.0).sleep();
    return true;
}

// robot2当前点、目标点z+0.1、目标点
bool robot1_go2(geometry_msgs::Point p, geometry_msgs::Quaternion q, moveit::planning_interface::MoveGroupInterface& robot1_move_group){
    double fraction;
    std::vector<geometry_msgs::Pose> waypoints;
    robot1_move_group.setMaxVelocityScalingFactor(0.1);
    waypoints.clear();
    geometry_msgs::Pose robot1_current_pose;
    robot1_current_pose = robot1_move_group.getCurrentPose().pose;
    waypoints.push_back(robot1_current_pose);
    geometry_msgs::Pose target_pose;
    target_pose.position = p;
	target_pose.orientation = q;
    geometry_msgs::Pose mid_pose;
    mid_pose = target_pose;
    mid_pose.position.z += 0.1;
    waypoints.push_back(mid_pose);
    waypoints.push_back(target_pose);
    moveit_msgs::RobotTrajectory trajectory3;
    
    fraction = robot1_move_group.computeCartesianPath(waypoints, eef_step, jump_threshold,trajectory3);
    if(fraction<0.9){
        return false;
    }
    
    ROS_WARN("Robot1 cartesian path. (%.2f%% achieved)",fraction*100.0);
    ros::Duration(1.0).sleep();
    moveit::planning_interface::MoveGroupInterface::Plan robot1_plan1;
    robot1_plan1.trajectory_ = trajectory3;
    robot1_move_group.execute(robot1_plan1);
    ros::Duration(1.0).sleep();
    return true;
}

// robot2当前点、目标点z+0.1、目标点
bool robot2_go(geometry_msgs::Point p, geometry_msgs::Quaternion q, moveit::planning_interface::MoveGroupInterface& robot2_move_group){
    double fraction;
    std::vector<geometry_msgs::Pose> waypoints;

    robot2_move_group.setMaxVelocityScalingFactor(0.1);
    waypoints.clear();
    geometry_msgs::Pose robot2_current_pose;
    robot2_current_pose = robot2_move_group.getCurrentPose().pose;
    waypoints.push_back(robot2_current_pose);
    geometry_msgs::Pose screw_hole_pose1;
    screw_hole_pose1.position = p;
	screw_hole_pose1.orientation = q;
    geometry_msgs::Pose mid_pose;
    mid_pose = screw_hole_pose1;
    mid_pose.position.z += 0.1;
    waypoints.push_back(mid_pose);
    waypoints.push_back(screw_hole_pose1);
    moveit_msgs::RobotTrajectory trajectory3;
    
    fraction = robot2_move_group.computeCartesianPath(waypoints, eef_step, jump_threshold,trajectory3);
    if(fraction<0.9){
        return false;
    }
    
    ROS_WARN("Robot2 cartesian path. (%.2f%% achieved)",fraction*100.0);
    ros::Duration(1.0).sleep();
    moveit::planning_interface::MoveGroupInterface::Plan robot2_plan1;
    robot2_plan1.trajectory_ = trajectory3;
    robot2_move_group.execute(robot2_plan1);
    // ROS_INFO("robot2 is ready");
    ros::Duration(1.0).sleep();
    return true;
}

// robot2当前点、当前点z+0.1、等待位置
bool robot2_go2(geometry_msgs::Point p, geometry_msgs::Quaternion q, moveit::planning_interface::MoveGroupInterface& robot2_move_group){
    double fraction;
    std::vector<geometry_msgs::Pose> waypoints;

    robot2_move_group.setMaxVelocityScalingFactor(0.1);
    waypoints.clear();
    geometry_msgs::Pose robot2_current_pose;
    robot2_current_pose = robot2_move_group.getCurrentPose().pose;
    waypoints.push_back(robot2_current_pose);
    geometry_msgs::Pose mid_pose;
    mid_pose = robot2_current_pose;
    mid_pose.position.z += 0.1;
    waypoints.push_back(mid_pose);
    geometry_msgs::Pose robot2_wait_pose;
    robot2_wait_pose.position = p;
    robot2_wait_pose.orientation = q;
    waypoints.push_back(robot2_wait_pose);
    moveit_msgs::RobotTrajectory trajectory3;
    
    fraction = robot2_move_group.computeCartesianPath(waypoints, eef_step, jump_threshold,trajectory3);
    if(fraction<0.9){
        return false;
    }
    
    ROS_WARN("Robot2 cartesian path. (%.2f%% achieved)",fraction*100.0);
    ros::Duration(1.0).sleep();
    moveit::planning_interface::MoveGroupInterface::Plan robot2_plan1;
    robot2_plan1.trajectory_ = trajectory3;
    robot2_move_group.execute(robot2_plan1);
    // ROS_INFO("robot2 is ready");
    ros::Duration(1.0).sleep();
    return true;
}

std::vector<double> get_delta_xy(std::vector<double> ref_xy, double cam_z, ros::ServiceClient& robot2_io_states_client){
    ur_msgs::SetIO robot2_io_states_srv;
    robot2_io_states_srv.request.fun = 1;
    robot2_io_states_srv.request.pin = 7;
    robot2_io_states_srv.request.state = 1.0;
    robot2_io_states_client.call(robot2_io_states_srv);
    ROS_INFO("OPEN LIGHT");

    std::vector<double> result;
    
    auto detect_xy = hole_detect::find_hole(cam_z);
    if(detect_xy.size()>0){
        // result.resize(2);
        // result[0] = detect_xy[0] - ref_xy[0];
        // result[1] = detect_xy[1] - ref_xy[1];
        result = detect_xy;
    }

    robot2_io_states_srv.request.state = 0.0;
    robot2_io_states_client.call(robot2_io_states_srv);

    if(result.size() > 0){
        ROS_INFO("base x : %lf", result[0]);
        ROS_INFO("base y : %lf", result[1]);
    }

    return result;
}

const double PI = 3.14159265359;
static const std::string ROBOT1_PLANNING_GROUP = "robot1_manipulator";
static const std::string ROBOT2_PLANNING_GROUP = "robot2_manipulator";
static const std::string ARMS_PLANNING_GROUP = "dual_arms";
std::vector<ur_msgs::Digital> robot1_digital_in, robot2_digital_in;
bool robot1_digital_in_flag = false;
bool robot2_digital_in_flag = false;

void robot1_io_callback(const ur_msgs::IOStates::ConstPtr& msg)
{
    robot1_digital_in = msg->digital_in_states;
    if (robot1_digital_in.size()) {
        robot1_digital_in_flag = true;
    } else {
        ROS_INFO("robot1_digital_in is null!");
        robot1_digital_in_flag = false;
    }
}
    
void robot2_io_callback(const ur_msgs::IOStates::ConstPtr& msg)
{
    robot2_digital_in = msg->digital_in_states;
    if (robot2_digital_in.size()) {
        robot2_digital_in_flag = true;
    } else {
        ROS_INFO("robot2_digital_in is null!");
        robot2_digital_in_flag = false;
    }
}

void printCurrentState(geometry_msgs::Pose current_pose)
{
	ROS_INFO("current pose x : %lf",current_pose.position.x);
	ROS_INFO("current pose y : %lf",current_pose.position.y);
	ROS_INFO("current pose z : %lf",current_pose.position.z);
}

void screw_hole(ros::ServiceClient& robot2_io_states_client){
    int setio_fun = 1;
    int setio_pin = 2;
    double setio_state = 1.0;
    ur_msgs::SetIO robot2_io_states_srv;
    robot2_io_states_srv.request.fun = setio_fun;
    robot2_io_states_srv.request.pin = setio_pin;
    robot2_io_states_srv.request.state = setio_state;
    robot2_io_states_client.call(robot2_io_states_srv);
    ros::Duration(1.0).sleep();
    robot2_io_states_srv.request.state = 0.0;
    robot2_io_states_client.call(robot2_io_states_srv);
    //2.吹气 DO1
    setio_fun = 1;
    setio_pin = 1;
    setio_state = 1.0;
    robot2_io_states_srv.request.fun = setio_fun;
    robot2_io_states_srv.request.pin = setio_pin;
    robot2_io_states_srv.request.state = setio_state;
    robot2_io_states_client.call(robot2_io_states_srv);
    ros::Duration(2.0).sleep();
    robot2_io_states_srv.request.state = 0.0;
    robot2_io_states_client.call(robot2_io_states_srv);
    //3.下压 DO3
    setio_fun = 1;
    setio_pin = 3;
    setio_state = 1.0;
    robot2_io_states_srv.request.fun = setio_fun;
    robot2_io_states_srv.request.pin = setio_pin;
    robot2_io_states_srv.request.state = setio_state;
    robot2_io_states_client.call(robot2_io_states_srv);
    ros::Duration(3.0).sleep();
    robot2_io_states_srv.request.state = 0.0;
    robot2_io_states_client.call(robot2_io_states_srv);

} 

void robot1_setio(int fun, int pin, double state, ros::ServiceClient& robot1_io_states_client)
{
    int setio_fun = fun;
    int setio_pin = pin;
    double setio_state = state;
    ur_msgs::SetIO robot1_io_states_srv;
    robot1_io_states_srv.request.fun = setio_fun;
    robot1_io_states_srv.request.pin = setio_pin;
    robot1_io_states_srv.request.state = setio_state;
    robot1_io_states_client.call(robot1_io_states_srv);
    ros::Duration(2.0).sleep();
    robot1_io_states_srv.request.state = 0.0;
    robot1_io_states_client.call(robot1_io_states_srv);
}

int main(int argc, char *argv[])
{
    record_init();
    ros::init(argc, argv, "pres2");
    ros::NodeHandle nh;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface robot1_move_group(ROBOT1_PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface robot2_move_group(ROBOT2_PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface arms_move_group(ARMS_PLANNING_GROUP);

    ROS_INFO("Reference frame of robot1: %s", robot1_move_group.getPlanningFrame().c_str());
    robot1_move_group.setEndEffectorLink("robot1_gripper_eef_link");
	ROS_INFO("End effector link of robot1: %s", robot1_move_group.getEndEffectorLink().c_str());
    ROS_INFO("End effector link of robot2: %s", robot2_move_group.getEndEffectorLink().c_str());
    ros::Duration(1.0).sleep();
    tf::TransformListener listener;
    std::string target_frame = "table_ground";
    ros::ServiceClient robot1_io_states_client, robot2_io_states_client;
    ur_msgs::SetIO robot1_io_states_srv, robot2_io_states_srv;
    std::string robot1_io_states_srv_name, robot2_io_states_srv_name;
    robot1_io_states_srv_name = "robot1/ur_driver/set_io";
    robot2_io_states_srv_name = "robot2/ur_driver/set_io";
    robot1_io_states_client = nh.serviceClient<ur_msgs::SetIO>(robot1_io_states_srv_name);
    robot2_io_states_client = nh.serviceClient<ur_msgs::SetIO>(robot2_io_states_srv_name);
    int setio_fun;
    int setio_pin;
    double setio_state;
    std::string robot1_io_states_topic, robot2_io_states_topic;
    ros::Subscriber robot1_io_states_sub, robot2_io_states_sub;
    robot1_io_states_topic = "robot1/ur_driver/io_states";
    robot2_io_states_topic = "robot2/ur_driver/io_states";
    robot1_io_states_sub = nh.subscribe<ur_msgs::IOStates>(robot1_io_states_topic,10,robot1_io_callback);
    robot2_io_states_sub = nh.subscribe<ur_msgs::IOStates>(robot2_io_states_topic,10,robot2_io_callback);
    ros::Publisher robot1_urscript_pub_;
    std::string robot1_urscript_topic;
    robot1_urscript_topic = "robot1/ur_driver/URScript";
    robot1_urscript_pub_ = nh.advertise<std_msgs::String>(robot1_urscript_topic,1);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    while(1)
    {

        //move up
        // geometry_msgs::Point robot1_wait_point2; 
        // robot1_wait_point2 = robot1_wait_point;
        // robot1_wait_point2.z += 0.02;
        // bool ret1 = true;
        // ret1 = robot1_go(robot1_wait_point2, robot1_wait_quat, robot1_move_group);
        // if(!ret1){
        //     std::cout << "wrong cartesian" << std::endl;
        //     return 0;
        // }
  //move to first hole
    current_hole = 0;
    bool ret1 = robot1_go(hole_loc_record[current_hole], hole_quat_record[current_hole], robot1_move_group);
    if(!ret1){
        std::cout << "wrong cartesian" << std::endl;
        return 0;
    }

    if(use_cam)
    delta_xy[current_hole] = get_delta_xy(hole_detect_record_base[current_hole], cam_z, robot2_io_states_client);

    //MOVE TO NEXT HOLE
    current_hole++;
    ret1 = robot1_go(hole_loc_record[current_hole], hole_quat_record[current_hole], robot1_move_group);
    if(!ret1){
        std::cout << "wrong cartesian" << std::endl;
        return 0;
    }

    if(use_cam)
    delta_xy[current_hole] = get_delta_xy(hole_detect_record_base[current_hole], cam_z, robot2_io_states_client);

    //move up
    geometry_msgs::Point upper_loc_record2;
    upper_loc_record2 = upper_loc_record;
    upper_loc_record2.z += 0.1;
    ret1 = robot1_go(upper_loc_record2, upper_quat_record, robot1_move_group);
    if(!ret1) {
        std::cout << "wrong cartesian" << std::endl;
        return 0;
    }

    robot1_setio(1,5,1.0,robot1_io_states_client);
    ROS_INFO("Open the gripper of robot1!");

    //移动到上盖
    ret1 = robot1_go2(upper_loc_record, upper_quat_record, robot1_move_group);
    if(!ret1) {
        std::cout << "wrong cartesian" << std::endl;
        return 0;
    }

    //close gripper
    robot1_setio(1,3,1.0,robot1_io_states_client);
    ROS_INFO("Close the gripper of robot1!");
    // ros::Duration(10.0).sleep();

    //move up
    ret1 = robot1_go(upper_loc_record2, upper_quat_record, robot1_move_group);
    if(!ret1) {
        std::cout << "wrong cartesian" << std::endl;
        return 0;
    }

    //移动到下盖
        ret1 = robot1_go2(lower_loc_record, lower_quat_record, robot1_move_group);
    if(!ret1) {
        std::cout << "wrong cartesian" << std::endl;
        return 0;
    }

    //open gripper
    robot1_setio(1,5,1.0,robot1_io_states_client);
    ROS_INFO("Open the gripper of robot1!");
    // ros::Duration(5.0).sleep();

    //move up
    geometry_msgs::Point lower_loc_record2;
    lower_loc_record2 = lower_loc_record;
    lower_loc_record2.z += 0.1;
    ret1 = robot1_go(lower_loc_record2, lower_quat_record, robot1_move_group);
    if(!ret1) {
        std::cout << "wrong cartesian" << std::endl;
        return 0;
    }

    //robot1 move away
    ret1 = robot1_go(robot1_wait_point, robot1_wait_quat, robot1_move_group);
    if(!ret1){
        std::cout << "wrong cartesian" << std::endl;
        return 0;
    }

    //robot2 move in 
    current_hole = 0;
    ret1 = robot2_go(hole_loc_record2[current_hole], hole_quat_record2[current_hole], robot2_move_group);
    if (!ret1) {
        std::cout << "wrong cartesian" << std::endl;
        return 0;       
    }

    
    screw_hole(robot2_io_states_client);

    //move to next hole
    current_hole++;
    ROS_INFO("Move to next hole!");
    ret1 = robot2_go(hole_loc_record2[current_hole], hole_quat_record2[current_hole], robot2_move_group);
    if (!ret1) {
        std::cout << "wrong cartesian" << std::endl;
        return 0;       
    }


    screw_hole(robot2_io_states_client);
    current_hole++;

    //robot2 move away
    ROS_INFO("robot2 move away");
    robot2_go2(robot2_wait_point, robot2_wait_quat, robot2_move_group);

    ros::Duration(2.0).sleep();

    //robot1 取飞控
    robot1_move_group.setMaxVelocityScalingFactor(1.0);
    std::vector<double> robot1_current_joint_values;
    robot1_current_joint_values = robot1_move_group.getCurrentJointValues();
    robot1_current_joint_values.at(0) = -2.732177;
    robot1_move_group.setJointValueTarget(robot1_current_joint_values);
    moveit::planning_interface::MoveGroupInterface::Plan robot1_switch_agv_plan;
    bool success = (robot1_move_group.plan(robot1_switch_agv_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("robot1 switch to another agv %s", success ? "":"Failed");
    robot1_move_group.execute(robot1_switch_agv_plan);

    delta_xy[0] = get_delta_xy(hole_detect_record_base[0], cam_z, robot2_io_states_client);

    ret1 = robot1_go2(controller_loc_record, controller_quat_record, robot1_move_group);
    if(!ret1){
        std::cout << "wrong cartesian" << std::endl;
        return 0;
    }
    //打开吸盘
    robot1_setio(1,7,1.0,robot1_io_states_client);

    //move up
    geometry_msgs::Point controller_loc_record3;
    controller_loc_record3 = controller_loc_record;
    controller_loc_record3.z += 0.2;
    ret1 = robot1_go(controller_loc_record3, controller_quat_record, robot1_move_group);
    if(!ret1) {
        std::cout << "wrong cartesian" << std::endl;
        return 0;
    }
    //robot1返回
    robot1_move_group.setMaxVelocityScalingFactor(1.0);
    robot1_current_joint_values = robot1_move_group.getCurrentJointValues();
    robot1_current_joint_values.at(0) = 0.026273;
    robot1_move_group.setJointValueTarget(robot1_current_joint_values);
    moveit::planning_interface::MoveGroupInterface::Plan robot1_switch_agv_plan2;
    success = (robot1_move_group.plan(robot1_switch_agv_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("robot1 switch to another agv %s", success ? "":"Failed");
    robot1_move_group.execute(robot1_switch_agv_plan2);

    //place controller
    geometry_msgs::Point controller_loc_record4;
    controller_loc_record4 = controller_loc_record2;
    controller_loc_record4.z += 0.15;
    ret1 = robot1_go(controller_loc_record4, controller_quat_record2, robot1_move_group);
    if(!ret1){
        std::cout << "wrong cartesian" << std::endl;
        return 0;
    }
    ros::Duration(1.0).sleep();
    ret1 = robot1_go(controller_loc_record2, controller_quat_record2, robot1_move_group);
    if(!ret1){
        std::cout << "wrong cartesian" << std::endl;
        return 0;
    }

    //关闭吸盘
    robot1_setio(1,6,1.0,robot1_io_states_client);

    //robot1返回
    ret1 = robot1_go(controller_loc_record4, controller_quat_record2, robot1_move_group);
    if(!ret1){
        std::cout << "wrong cartesian" << std::endl;
        return 0;
    }
    ret1 = robot1_go(robot1_wait_point, robot1_wait_quat, robot1_move_group);
    if(!ret1){
        std::cout << "wrong cartesian" << std::endl;
        return 0;
    }

    // //robot2打螺钉
    // ret1 = robot2_go(hole_loc_record2[current_hole], hole_quat_record2[current_hole], robot2_move_group);
    // if (!ret1) {
    //     std::cout << "wrong cartesian" << std::endl;
    //     return 0;       
    // }

    
    // screw_hole(robot2_io_states_client);

    // //move to next hole
    // current_hole++;
    // ROS_INFO("Move to next hole!");
    // ret1 = robot2_go(hole_loc_record2[current_hole], hole_quat_record2[current_hole], robot2_move_group);
    // if (!ret1) {
    //     std::cout << "wrong cartesian" << std::endl;
    //     return 0;       
    // }


    // screw_hole(robot2_io_states_client);
    // //robot2 move away
    // ROS_INFO("robot2 move away");
    // robot2_go2(robot2_wait_point, robot2_wait_quat, robot2_move_group);

    }

  


}