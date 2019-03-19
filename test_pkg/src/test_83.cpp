#include "ros/ros.h"
#include <moveit/move_group_interface/move_group.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <ur_msgs/SetIO.h>
#include <ur_msgs/IOStates.h>
#include <ur_msgs/Digital.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "weitu.h"
#include <iostream>
#include <csignal>
#include <cmath>

static int total_holes = 6;

static bool use_cam = true;
static bool init_data_cali = true;
static bool linear_cali = true;
static bool init_success = false;
static bool isscrew = true;

static std::vector<geometry_msgs::Point> hole_loc_record(total_holes);
static std::vector<geometry_msgs::Quaternion> hole_quat_record(total_holes);

static std::vector<geometry_msgs::Point> hole_loc_record2(total_holes);
static std::vector<geometry_msgs::Quaternion> hole_quat_record2(total_holes);

static geometry_msgs::Point robot1_wait_point;
static geometry_msgs::Quaternion robot1_wait_quat;

static geometry_msgs::Point robot1_grip_point;
static geometry_msgs::Quaternion robot1_grip_quat;
static geometry_msgs::Point robot1_grip_point2;
static geometry_msgs::Quaternion robot1_grip_quat2;

static geometry_msgs::Point robot2_wait_point;
static geometry_msgs::Quaternion robot2_wait_quat;

static double cam_z = 0.234;
static std::vector<std::vector<double>> hole_detect_record_base(total_holes, {0, 0});
static std::vector<std::vector<double>> delta_xy(total_holes, {0, 0});
static double linear_cali_mat[4] = {1, 0, 0, 1};
double rotation_mat[4] = {1, 0, 0, 1};
double translation_mat[2] = {0, 0};
cv::Mat r_mat = cv::Mat(2, 2, CV_64FC1);
cv::Mat t_mat = cv::Mat(2, 1, CV_64FC1);

static ros::ServiceClient robot1_io_states_client;
static ros::ServiceClient robot2_io_states_client;

static const double jump_threshold = 5;
static const double eef_step = 0.005;

bool robot1_digital_in_flag = false;
std::vector<ur_msgs::Digital> robot1_digital_in;
std::string robot1_io_states_topic = "robot1/ur_driver/io_states";
ros::Subscriber robot1_io_states_sub;
ur_msgs::SetIO robot1_io_states_srv;
std::string robot1_io_states_srv_name = "robot1/ur_driver/set_io";

double xa, ya, xb, yb;

void save_point_quat(geometry_msgs::Point &robot1_wait_point, std::string str_robot1_wait_point,
                     geometry_msgs::Quaternion &robot1_wait_quat,
                     std::string str_robot1_wait_quat, cv::FileStorage &fs)
{
    fs << (str_robot1_wait_point + "_x") << robot1_wait_point.x;
    fs << (str_robot1_wait_point + "_y") << robot1_wait_point.y;
    fs << (str_robot1_wait_point + "_z") << robot1_wait_point.z;
    fs << (str_robot1_wait_quat + "_x") << robot1_wait_quat.x;
    fs << (str_robot1_wait_quat + "_y") << robot1_wait_quat.y;
    fs << (str_robot1_wait_quat + "_z") << robot1_wait_quat.z;
    fs << (str_robot1_wait_quat + "_w") << robot1_wait_quat.w;
}

template <typename T>
void load_point_quat(geometry_msgs::Point &robot1_wait_point, std::string str_robot1_wait_point,
                     geometry_msgs::Quaternion &robot1_wait_quat,
                     std::string str_robot1_wait_quat, T &fs)
{
    robot1_wait_point.x = (double)(fs)[str_robot1_wait_point + "_x"];
    robot1_wait_point.y = (double)(fs)[str_robot1_wait_point + "_y"];
    robot1_wait_point.z = (double)(fs)[str_robot1_wait_point + "_z"];
    robot1_wait_quat.x = (double)(fs)[str_robot1_wait_quat + "_x"];
    robot1_wait_quat.y = (double)(fs)[str_robot1_wait_quat + "_y"];
    robot1_wait_quat.z = (double)(fs)[str_robot1_wait_quat + "_z"];
    robot1_wait_quat.w = (double)(fs)[str_robot1_wait_quat + "_w"];
}

void save_data()
{
    cv::FileStorage fs("data_83.yaml", cv::FileStorage::WRITE);
    save_point_quat(robot1_wait_point, "robot1_wait_point", robot1_wait_quat, "robot1_wait_quat", fs);
    save_point_quat(robot2_wait_point, "robot2_wait_point", robot2_wait_quat, "robot2_wait_quat", fs);

    fs << "hole_record"
       << "[";
    for (int i = 0; i < hole_loc_record.size(); i++)
    {
        fs << "{";
        save_point_quat(hole_loc_record[i], "loc", hole_quat_record[i], "quat", fs);
        save_point_quat(hole_loc_record2[i], "loc2", hole_quat_record2[i], "quat2", fs);
        fs << "}";
    }
    fs << "]";

    fs << "hole_detect_record_base"
       << "[";
    for (auto &p : hole_detect_record_base)
    {
        fs << "[";
        for (auto v : p)
        {
            fs << v;
        }
        fs << "]";
    }
    fs << "]";

    fs << "linear_cali_mat"
       << "[";
    fs << linear_cali_mat[0] << linear_cali_mat[1] << linear_cali_mat[2] << linear_cali_mat[3];
    fs << "]";

    save_point_quat(robot1_grip_point, "robot1_grip_point", robot1_grip_quat, "robot1_grip_quat", fs);
    save_point_quat(robot1_grip_point2, "robot1_grip_point2", robot1_grip_quat2, "robot1_grip_quat2", fs);
}

void load_data()
{
    cv::FileStorage fs("data_83.yaml", cv::FileStorage::READ);
    load_point_quat(robot1_wait_point, "robot1_wait_point", robot1_wait_quat, "robot1_wait_quat", fs);
    load_point_quat(robot2_wait_point, "robot2_wait_point", robot2_wait_quat, "robot2_wait_quat", fs);

    {
        cv::FileNode hole_record = fs["hole_record"];
        cv::FileNodeIterator it = hole_record.begin(), it_end = hole_record.end();
        for (int i = 0; it != it_end; ++it, i++)
        {
            auto fn = *it;
            load_point_quat(hole_loc_record[i], "loc", hole_quat_record[i], "quat", fn);
            load_point_quat(hole_loc_record2[i], "loc2", hole_quat_record2[i], "quat2", fn);
        }
    }

    {
        cv::FileNode hole_detect_record_base_ = fs["hole_detect_record_base"];
        cv::FileNodeIterator it = hole_detect_record_base_.begin(), it_end = hole_detect_record_base_.end();
        for (int i = 0; it != it_end; ++it, i++)
        {
            auto inner_seq = *it;
            auto inner_it = inner_seq.begin();
            auto inner_it_end = inner_seq.end();
            for (int j = 0; inner_it != inner_it_end; inner_it++, j++)
            {
                hole_detect_record_base[i][j] = (double)(*inner_it);
            }
        }
    }

    {
        cv::FileNode linear_cali_mat_ = fs["linear_cali_mat"];
        cv::FileNodeIterator it = linear_cali_mat_.begin(), it_end = linear_cali_mat_.end();
        for (int i = 0; it != it_end; ++it, i++)
        {
            linear_cali_mat[i] = (double)(*it);
        }
    }

    load_point_quat(robot1_grip_point, "robot1_grip_point", robot1_grip_quat, "robot1_grip_quat", fs);
    load_point_quat(robot1_grip_point2, "robot1_grip_point2", robot1_grip_quat2, "robot1_grip_quat2", fs);
}

bool cam_bot_go(geometry_msgs::Point p, geometry_msgs::Quaternion q, moveit::planning_interface::MoveGroupInterface& robot1_move_group)
{
    double fraction = 0;
    std::vector<geometry_msgs::Pose> waypoints;

    geometry_msgs::Pose robot1_pose1;
    robot1_pose1.orientation = q;
    robot1_pose1.position = p;
    waypoints.push_back(robot1_pose1);
    moveit_msgs::RobotTrajectory trajectory1;

    fraction = robot1_move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory1);
    if (fraction < 0.9)
    {
        return false;
    }

    ROS_WARN("Robot1 cartesian path. (%.2f%% achieved)", fraction * 100.0);
    // ros::Duration(1.0).sleep();
    moveit::planning_interface::MoveGroupInterface::Plan robot1_plan1;
    robot1_plan1.trajectory_ = trajectory1;
    robot1_move_group.execute(robot1_plan1);
    // ROS_INFO("robot1 is ready");
    // ros::Duration(1.0).sleep();
    return true;
}

bool screw_bot_go(geometry_msgs::Point p, geometry_msgs::Quaternion q, moveit::planning_interface::MoveGroupInterface& robot2_move_group)
{
    double fraction;
    std::vector<geometry_msgs::Pose> waypoints;

    robot2_move_group.setMaxVelocityScalingFactor(0.1);
    waypoints.clear();

    geometry_msgs::Pose robot2_current_pose = robot2_move_group.getCurrentPose().pose;
    robot2_current_pose = robot2_move_group.getCurrentPose().pose;
    robot2_current_pose.position.z += 0.1;
    waypoints.push_back(robot2_current_pose);

    geometry_msgs::Pose screw_hole_pose1;
    screw_hole_pose1.position = p;
    screw_hole_pose1.orientation = q;
    geometry_msgs::Pose mid_pose;
    mid_pose = screw_hole_pose1;
    mid_pose.position.z = robot2_current_pose.position.z;
    waypoints.push_back(mid_pose);
    waypoints.push_back(screw_hole_pose1);
    moveit_msgs::RobotTrajectory trajectory3;

    fraction = robot2_move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory3);
    if (fraction < 0.9)
    {
        return false;
    }

    ROS_WARN("Robot2 cartesian path. (%.2f%% achieved)", fraction * 100.0);
    // ros::Duration(1.0).sleep();
    moveit::planning_interface::MoveGroupInterface::Plan robot2_plan1;
    robot2_plan1.trajectory_ = trajectory3;
    robot2_move_group.execute(robot2_plan1);
    // ROS_INFO("robot2 is ready");
    // ros::Duration(1.0).sleep();
    return true;
}

bool screw_bot_go_wait(geometry_msgs::Point p, geometry_msgs::Quaternion q, moveit::planning_interface::MoveGroupInterface& robot2_move_group)
{
    double fraction;
    std::vector<geometry_msgs::Pose> waypoints;

    robot2_move_group.setMaxVelocityScalingFactor(0.1);
    waypoints.clear();
    geometry_msgs::Pose robot2_current_pose = robot2_move_group.getCurrentPose().pose;
    robot2_current_pose = robot2_move_group.getCurrentPose().pose;

    geometry_msgs::Pose mid_pose;
    mid_pose = robot2_current_pose;
    mid_pose.position.z += 0.1;
    waypoints.push_back(mid_pose);
    geometry_msgs::Pose robot2_wait_pose;
    robot2_wait_pose.position = p;
    robot2_wait_pose.orientation = q;
    waypoints.push_back(robot2_wait_pose);
    moveit_msgs::RobotTrajectory trajectory3;

    fraction = robot2_move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory3);
    if (fraction < 0.9)
    {
        return false;
    }

    ROS_WARN("Robot2 cartesian path. (%.2f%% achieved)", fraction * 100.0);
    // ros::Duration(1.0).sleep();
    moveit::planning_interface::MoveGroupInterface::Plan robot2_plan1;
    robot2_plan1.trajectory_ = trajectory3;
    robot2_move_group.execute(robot2_plan1);
    // ROS_INFO("robot2 is ready");
    // ros::Duration(1.0).sleep();
    return true;
}

std::vector<double> get_xy(double cam_z)
{
    ur_msgs::SetIO robot2_io_states_srv;
    robot2_io_states_srv.request.fun = 1;
    robot2_io_states_srv.request.pin = 7;
    robot2_io_states_srv.request.state = 1.0;
    robot2_io_states_client.call(robot2_io_states_srv);
    ROS_INFO("OPEN LIGHT");

    std::vector<double> result;

    auto detect_xy = hole_detect::find_hole(cam_z);
    if (detect_xy.size() > 0)
    {
        result = detect_xy;
    }

    robot2_io_states_srv.request.state = 0.0;
    robot2_io_states_client.call(robot2_io_states_srv);
    ROS_INFO("CLOSE LIGHT");
    // if (result.size() > 0)
    // {
    //     ROS_INFO("base x : %lf", result[0]);
    //     ROS_INFO("base y : %lf", result[1]);
    // }
    return result;
}

void screw_hole()
{
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
    ros::Duration(1.5).sleep();
    robot2_io_states_srv.request.state = 0.0;
    robot2_io_states_client.call(robot2_io_states_srv);
}

void open_gripper()
{
    int setio_fun = 1;
    int setio_pin = 5;
    double setio_state = 1.0;
    ur_msgs::SetIO robot1_io_states_srv;
    robot1_io_states_srv.request.fun = setio_fun;
    robot1_io_states_srv.request.pin = setio_pin;
    robot1_io_states_srv.request.state = setio_state;
    robot1_io_states_client.call(robot1_io_states_srv);
    ros::Duration(2.0).sleep();
    robot1_io_states_srv.request.state = 0.0;
    robot1_io_states_client.call(robot1_io_states_srv);
}

void close_gripper()
{
    int setio_fun = 1;
    int setio_pin = 3;
    double setio_state = 1.0;
    ur_msgs::SetIO robot1_io_states_srv;
    robot1_io_states_srv.request.fun = setio_fun;
    robot1_io_states_srv.request.pin = setio_pin;
    robot1_io_states_srv.request.state = setio_state;
    robot1_io_states_client.call(robot1_io_states_srv);
    ros::Duration(2.0).sleep();
    robot1_io_states_srv.request.state = 0.0;
    robot1_io_states_client.call(robot1_io_states_srv);
}

void SIGINT_handler(int signal)
{
    // close light
    // if (init_success)
    // {
    //     ur_msgs::SetIO robot2_io_states_srv;
    //     robot2_io_states_srv.request.fun = 1;
    //     robot2_io_states_srv.request.pin = 7;
    //     robot2_io_states_srv.request.state = 0.0;
    //     robot2_io_states_client.call(robot2_io_states_srv);
    // }
    exit(1);
}

std::vector<double> compute_delta(geometry_msgs::Point p)
{
    double x0 = p.x;
    double y0 = p.y;
    cv::Mat g_mat = cv::Mat(2, 1, CV_64FC1);
    cv::Mat g0_mat = cv::Mat(2, 1, CV_64FC1);
    g0_mat.at<double>(0, 0) = x0;
    g0_mat.at<double>(1, 0) = y0;
    g_mat = r_mat * g0_mat + t_mat;
    double x = g_mat.at<double>(0, 0);
    double y = g_mat.at<double>(1, 0);
    double delta_x = x - x0;
    double delta_y = y - y0;
    std::vector<double> result;
    result.push_back(delta_x);
    result.push_back(delta_y);
    return result;
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

int main(int argc, char *argv[])
{
    std::signal(SIGINT, SIGINT_handler);

    // at least open one window for waitkey
    auto show_1 = cv::imread("/home/a/catkin_ws/1.png");
    cv::imshow("1", show_1);

    load_data();

    ros::init(argc, argv, "dual_arm");
    ros::NodeHandle nh;
    
    const std::string ROBOT1_PLANNING_GROUP = "robot1_manipulator";
    const std::string ROBOT2_PLANNING_GROUP = "robot2_manipulator";
    moveit::planning_interface::MoveGroupInterface robot1_move_group(ROBOT1_PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface robot2_move_group(ROBOT2_PLANNING_GROUP);
    robot1_move_group.setEndEffectorLink("robot1_camera_eef_link");

    robot1_io_states_sub = nh.subscribe<ur_msgs::IOStates>(robot1_io_states_topic,10,robot1_io_callback);
    robot1_io_states_client = nh.serviceClient<ur_msgs::SetIO>("robot1/ur_driver/set_io");
    robot2_io_states_client = nh.serviceClient<ur_msgs::SetIO>("robot2/ur_driver/set_io");
    ros::AsyncSpinner spinner(2);
    spinner.start();

    init_success = false;

    init_data_cali = false;
    if (init_data_cali)
    {
        std::cout << "init calibrating..." << std::endl;

        bool cam_bot_cali = true;
        if (cam_bot_cali)
        {
            bool hole_cali = true;
            if (hole_cali) 
            {
            ROS_INFO("OPEN LIGHT");
            ur_msgs::SetIO robot2_io_states_srv;
            robot2_io_states_srv.request.fun = 1;
            robot2_io_states_srv.request.pin = 7;
            robot2_io_states_srv.request.state = 1.0;
            robot2_io_states_client.call(robot2_io_states_srv);

            for (int current_hole = 0; current_hole < total_holes; current_hole++)
            {
                ROS_INFO(("calibrating hole " + std::to_string(current_hole) + ", cam_robot go").c_str());
                geometry_msgs::Pose robot1_current_pose = robot1_move_group.getCurrentPose().pose;;
                bool ret1 = true;

                if (init_success)
                {
                    ret1 = cam_bot_go(hole_loc_record[current_hole], hole_quat_record[current_hole], robot1_move_group);
                    if (!ret1)
                    {
                        std::cout << "wrong cartesian" << std::endl;
                        return 0;
                    }
                }
                else
                {
                    ROS_INFO("no init pos provided, move robot by hand");
                }

                std::cout << "please align hole to center, \npress w a s d, q to break, \n1-9 for steps， others to refresh" << std::endl;

                {
                    weitu::Camera camera;
                    camera.open(0);

                    float bot_steps = 0.003;
                    while (1)
                    {
                        cv::Mat rgb = camera.get();
                        if (!rgb.empty())
                        {
                            cv::Mat smaller_rgb;
                            cv::pyrDown(rgb, smaller_rgb);
                            auto p = hole_detect::find(smaller_rgb);
                        }
                        char key = cv::waitKey(0);

                        robot1_current_pose = robot1_move_group.getCurrentPose().pose;
                        auto temp = robot1_current_pose.position;

                        if (key == '1')
                        {
                            bot_steps = 0.001;
                        }
                        else if (key == '2')
                        {
                            bot_steps = 0.002;
                        }
                        else if (key == '3')
                        {
                            bot_steps = 0.003;
                        }
                        else if (key == '4')
                        {
                            bot_steps = 0.004;
                        }
                        else if (key == '5')
                        {
                            bot_steps = 0.005;
                        }
                        else if (key == '6')
                        {
                            bot_steps = 0.006;
                        }
                        else if (key == '7')
                        {
                            bot_steps = 0.007;
                        }
                        else if (key == '8')
                        {
                            bot_steps = 0.008;
                        }
                        else if (key == '9')
                        {
                            bot_steps = 0.009;
                        }
                        else if (key == '0')
                        {
                            bot_steps = 0.010;
                        }
                        else if (key == 'w')
                        {
                            temp.y += bot_steps;
                        }
                        else if (key == 's')
                        {
                            temp.y -= bot_steps;
                        }
                        else if (key == 'a')
                        {
                            temp.x += bot_steps;
                        }
                        else if (key == 'd')
                        {
                            temp.x -= bot_steps;
                        }
                        else if (key == 'z')
                        {
                            temp.z += bot_steps;
                        }
                        else if (key == 'c')
                        {
                            temp.z -= bot_steps;
                        }
                        else if (key == 'q')
                        {
                            break;
                        }
                        else
                        {
                            continue;
                        }
                        ret1 = cam_bot_go(temp, robot1_current_pose.orientation, robot1_move_group);
                        if (!ret1)
                        {
                            std::cout << "wrong cartesian" << std::endl;
                            return 0;
                        }
                    }
                }

                robot1_current_pose = robot1_move_group.getCurrentPose().pose;

                hole_loc_record[current_hole] = robot1_current_pose.position;
                hole_quat_record[current_hole] = robot1_current_pose.orientation;

                auto detect_xy = hole_detect::find_hole(cam_z);
                while (1)
                {
                    std::cout << "press space to next step, or others to detect again" << std::endl;
                    char key = cv::waitKey(0);
                    if (key == ' ')
                    {
                        break;
                    }
                    else
                    {
                        detect_xy = hole_detect::find_hole(cam_z);
                    }
                }

                if (detect_xy.size() > 0)
                {
                    ROS_INFO(("hole_detect_record_base " + std::to_string(current_hole) +
                              "\nx: " + std::to_string(detect_xy[0]) +
                              "\ny: " + std::to_string(detect_xy[1]))
                                 .c_str());
                    hole_detect_record_base[current_hole] = detect_xy;
                }
                else
                {
                    std::cout << "nothing detect, wrong init of hole_detect_record_base" << std::endl;
                    return 0;
                }
            }
            save_data();
            load_data();
            if (linear_cali)
            {
                int current_hole = 1;
                std::cout << "linear calibrating..." << std::endl;
                double bound = 0.005;
                auto loc_temp = hole_loc_record[current_hole];

                cv::Mat cam_mat = cv::Mat(2, 2, CV_64FC1);
                cv::Mat phy_mat = cv::Mat(2, 2, CV_64FC1);

                // go 0.005, 0
                loc_temp.x += bound;
                bool ret1 = cam_bot_go(loc_temp, hole_quat_record[current_hole], robot1_move_group);
                if (!ret1)
                {
                    std::cout << "wrong cartesian" << std::endl;
                    return 0;
                }
                auto cam_delta_xy1 = get_xy(cam_z);
                cv::waitKey(2000);

                cam_delta_xy1[0] -= hole_detect_record_base[current_hole][0];
                cam_delta_xy1[1] -= hole_detect_record_base[current_hole][1];

                cam_mat.at<double>(0, 0) = cam_delta_xy1[0];
                cam_mat.at<double>(1, 0) = cam_delta_xy1[1];

                geometry_msgs::Pose robot1_current_pose = robot1_move_group.getCurrentPose().pose;;
                robot1_current_pose = robot1_move_group.getCurrentPose().pose;
                phy_mat.at<double>(0, 0) = robot1_current_pose.position.x - hole_loc_record[current_hole].x;
                phy_mat.at<double>(1, 0) = robot1_current_pose.position.y - hole_loc_record[current_hole].y;

                // go 0, 0.005
                loc_temp.x = hole_loc_record[current_hole].x;
                loc_temp.y += bound;
                ret1 = cam_bot_go(loc_temp, hole_quat_record[current_hole], robot1_move_group);
                if (!ret1)
                {
                    std::cout << "wrong cartesian" << std::endl;
                    return 0;
                }
                auto cam_delta_xy2 = get_xy(cam_z);
                cv::waitKey(2000);

                cam_delta_xy2[0] -= hole_detect_record_base[current_hole][0];
                cam_delta_xy2[1] -= hole_detect_record_base[current_hole][1];

                cam_mat.at<double>(0, 1) = cam_delta_xy2[0];
                cam_mat.at<double>(1, 1) = cam_delta_xy2[1];

                robot1_current_pose = robot1_move_group.getCurrentPose().pose;
                phy_mat.at<double>(0, 1) = robot1_current_pose.position.x - hole_loc_record[current_hole].x;
                phy_mat.at<double>(1, 1) = robot1_current_pose.position.y - hole_loc_record[current_hole].y;

                // relative, bot plus means hole minus
                cv::Mat linear_cv = (cam_mat.inv()) * (-phy_mat);

                linear_cali_mat[0] = linear_cv.at<double>(0, 0);
                linear_cali_mat[1] = linear_cv.at<double>(0, 1);
                linear_cali_mat[2] = linear_cv.at<double>(1, 0);
                linear_cali_mat[3] = linear_cv.at<double>(1, 1);

                std::cout << "cam_mat: \n" << cam_mat << std::endl;
                std::cout << "phy_mat: \n" << phy_mat << std::endl;

                std::cout << "liear_cali_mat: \n";
                std::cout << linear_cali_mat[0] << '\t' << linear_cali_mat[1] << '\n';
                std::cout << linear_cali_mat[2] << '\t' << linear_cali_mat[3] << '\n'
                          << std::endl;
            }
            save_data();
            load_data();
            robot2_io_states_srv.request.state = 0.0;
            robot2_io_states_client.call(robot2_io_states_srv);

            //消除hole_detect_record_base
            for (int current_hole = 0; current_hole < total_holes; current_hole++)
            {
                double cam_delta_x = hole_detect_record_base[current_hole][0];
                double cam_delta_y = hole_detect_record_base[current_hole][1];
                double delta_x = linear_cali_mat[0] * cam_delta_x + linear_cali_mat[1] * cam_delta_y;
                double delta_y = linear_cali_mat[2] * cam_delta_x + linear_cali_mat[3] * cam_delta_y;
                ROS_INFO("消除hole_detect_record_base x: %f", delta_x);
                ROS_INFO("消除hole_detect_record_base y: %f", delta_y);
                hole_loc_record[current_hole].x += delta_x;
                hole_loc_record[current_hole].y += delta_y;
                save_data();
            }
            }
            

            bool grip_cali = true;
            if (grip_cali) 
            {

                {
                ROS_INFO("calibrating gripper pos1");
                robot1_move_group.setEndEffectorLink("robot1_gripper_eef_link");
                bool ret1 = true;
                if (init_success)
                {
                    robot1_grip_point.z += 0.1;
                    ret1 = cam_bot_go(robot1_grip_point, robot1_grip_quat, robot1_move_group);
                    if (!ret1)
                    {
                        std::cout << "wrong cartesian" << std::endl;
                        return 0;
                    }
                }
                else
                {
                    ROS_INFO("no init pos provided, move robot by hand");
                }

                std::cout << "please move to grip pos, press space to next step" << std::endl;
                while (1)
                {
                    char key = cv::waitKey(0);
                    if (key == ' ')
                    {
                        break;
                    }
                }
                geometry_msgs::Pose robot1_current_pose = robot1_move_group.getCurrentPose().pose;
                ros::Duration(1.0).sleep();
                robot1_current_pose = robot1_move_group.getCurrentPose().pose;
                robot1_grip_point = robot1_current_pose.position;
                robot1_grip_quat = robot1_current_pose.orientation;
                }
                save_data();

                {
                ROS_INFO("calibrating gripper pos2");
                bool ret1 = true;
                if (init_success)
                {
                    robot1_grip_point.z += 0.1;
                    ret1 = cam_bot_go(robot1_grip_point, robot1_grip_quat, robot1_move_group);
                    if (!ret1)
                    {
                        std::cout << "wrong cartesian" << std::endl;
                        return 0;
                    }
                    ros::Duration(2.0).sleep();
                    robot1_grip_point2.z += 0.1;
                    ret1 = cam_bot_go(robot1_grip_point2, robot1_grip_quat2, robot1_move_group);
                    if (!ret1)
                    {
                        std::cout << "wrong cartesian" << std::endl;
                        return 0;
                    }
                }
                else
                {
                    ROS_INFO("no init pos provided, move robot by hand");
                }

                std::cout << "please move to grip pos, press space to next step" << std::endl;
                while (1)
                {
                    char key = cv::waitKey(0);
                    if (key == ' ')
                    {
                        break;
                    }
                }
                geometry_msgs::Pose robot1_current_pose = robot1_move_group.getCurrentPose().pose;
                ros::Duration(1.0).sleep();
                robot1_current_pose = robot1_move_group.getCurrentPose().pose;
                robot1_grip_point2 = robot1_current_pose.position;
                robot1_grip_quat2 = robot1_current_pose.orientation;
                }
                save_data();
            }
            
            {
                ROS_INFO("calibrating cam_robot wait pos");
                robot1_move_group.setEndEffectorLink("robot1_camera_eef_link");
                geometry_msgs::Pose robot1_current_pose = robot1_move_group.getCurrentPose().pose;
                bool ret1 = true;

                if (init_success)
                {
                    ret1 = cam_bot_go(robot1_wait_point, robot1_wait_quat, robot1_move_group);
                    if (!ret1)
                    {
                        std::cout << "wrong cartesian" << std::endl;
                        return 0;
                    }
                }
                else
                {
                    ROS_INFO("no init pos provided, move robot by hand");
                }

                std::cout << "please move to wanted wait pos, press space to next step" << std::endl;
                while (1)
                {
                    char key = cv::waitKey(0);
                    if (key == ' ')
                    {
                        break;
                    }
                }
                robot1_current_pose = robot1_move_group.getCurrentPose().pose;
                robot1_wait_point = robot1_current_pose.position;
                robot1_wait_quat = robot1_current_pose.orientation;
            }
            save_data();
        }

        bool screw_bot_cali = true;
        if (screw_bot_cali)
        {
            for (int current_hole = 2; current_hole < total_holes; current_hole++)
            {
                ROS_INFO(("calibrating hole " + std::to_string(current_hole) + ", screw_robot go").c_str());
                geometry_msgs::Pose robot2_current_pose  = robot2_move_group.getCurrentPose().pose;
                bool ret1 = true;

                if (init_success)
                {
                    ret1 = screw_bot_go(hole_loc_record2[current_hole], hole_quat_record2[current_hole], robot2_move_group);
                    if (!ret1)
                    {
                        std::cout << "wrong cartesian" << std::endl;
                        return 0;
                    }
                }
                else
                {
                    ROS_INFO("no init pos provided, move robot by hand");
                }

                std::cout << "press space to next step" << std::endl;
                while (1)
                {
                    char key = cv::waitKey(0);
                    if (key == ' ')
                    {
                        break;
                    }
                }
                robot2_current_pose = robot2_move_group.getCurrentPose().pose;
                ros::Duration(1.0).sleep();
                robot2_current_pose = robot2_move_group.getCurrentPose().pose;
                hole_loc_record2[current_hole] = robot2_current_pose.position;
                hole_quat_record2[current_hole] = robot2_current_pose.orientation;
            }
            save_data();

            {
                ROS_INFO("calibrating screw_robot wait pos");
                geometry_msgs::Pose robot2_current_pose = robot2_move_group.getCurrentPose().pose;
                bool ret1 = true;

                if (init_success)
                {
                    ret1 = screw_bot_go_wait(robot2_wait_point, robot2_wait_quat, robot2_move_group);
                    if (!ret1)
                    {
                        std::cout << "wrong cartesian" << std::endl;
                        return 0;
                    }
                }
                else
                {
                    ROS_INFO("no init pos provided, move robot by hand");
                }

                std::cout << "please move to wanted wait pos, press space to next step" << std::endl;
                while (1)
                {
                    char key = cv::waitKey(0);
                    if (key == ' ')
                    {
                        break;
                    }
                }
                robot2_current_pose = robot2_move_group.getCurrentPose().pose;
                robot2_wait_point = robot2_current_pose.position;
                robot2_wait_quat = robot2_current_pose.orientation;
            }
            save_data();
        }

        save_data();
        ROS_INFO("init data cali done");
        return 0;
    }

    // while (1)
    {
        // set 0 to move bot to waiting pos
        // total_holes = 0;
        bool agv_signal = true;
        while (agv_signal)
        {
            if(robot1_digital_in_flag){
                if(robot1_digital_in.at(0).state == 1){
                    ROS_INFO("AGVs arrive!");
                    break;
                }
            }
            ros::Duration(0.2).sleep();
        }
    
        ROS_INFO("computing rotation and translation matrix");
        load_data();
        int current_hole = 0;
        auto temp1 = hole_loc_record[current_hole];
        robot1_move_group.setEndEffectorLink("robot1_camera_eef_link");
        geometry_msgs::Pose robot1_current_pose = robot1_move_group.getCurrentPose().pose;
        if (!cam_bot_go(temp1, hole_quat_record[current_hole], robot1_move_group))
        {
            std::cout << "wrong cartesian" << std::endl;
            return 0;
        }
        if (use_cam)
        {
            auto cam_xy = get_xy(cam_z);
            ROS_INFO("cam_x: %f",cam_xy[0]);
            ROS_INFO("cam_y: %f",cam_xy[1]);
            cv::waitKey(1000);

            if (cam_xy.size() > 0)
            {
                double delta_xa = linear_cali_mat[0] * cam_xy[0] + linear_cali_mat[1] * cam_xy[1];
                double delta_ya = linear_cali_mat[2] * cam_xy[0] + linear_cali_mat[3] * cam_xy[1];
                robot1_current_pose = robot1_move_group.getCurrentPose().pose;
                xa = robot1_current_pose.position.x + delta_xa;
                ya = robot1_current_pose.position.y + delta_ya;
                ROS_INFO("xa: %f",xa);
                ROS_INFO("ya: %f",ya);
            }
        }
        robot1_current_pose = robot1_move_group.getCurrentPose().pose;
        double current_hole0_x = robot1_current_pose.position.x;
        double current_hole0_y = robot1_current_pose.position.y;
        current_hole++;

        temp1 = hole_loc_record[current_hole];
        if (!cam_bot_go(temp1, hole_quat_record[current_hole], robot1_move_group))
        {
            std::cout << "wrong cartesian" << std::endl;
            return 0;
        }
        if (use_cam)
        {
            auto cam_xy = get_xy(cam_z);
            ROS_INFO("cam_x: %f",cam_xy[0]);
            ROS_INFO("cam_y: %f",cam_xy[1]);
            cv::waitKey(1000);
            robot1_current_pose = robot1_move_group.getCurrentPose().pose;
            if (cam_xy.size() > 0)
            {
                double delta_xb = linear_cali_mat[0] * cam_xy[0] + linear_cali_mat[1] * cam_xy[1];
                double delta_yb = linear_cali_mat[2] * cam_xy[0] + linear_cali_mat[3] * cam_xy[1];
                robot1_current_pose = robot1_move_group.getCurrentPose().pose;
                xb = robot1_current_pose.position.x + delta_xb;
                yb = robot1_current_pose.position.y + delta_yb;
                ROS_INFO("xb: %f",xb);
                ROS_INFO("yb: %f",yb);
            }
        }
        robot1_current_pose = robot1_move_group.getCurrentPose().pose;
        double current_hole1_x = robot1_current_pose.position.x;
        double current_hole1_y = robot1_current_pose.position.y;
        double x0 = current_hole0_x - current_hole1_x;
        double y0 = current_hole0_y - current_hole1_y;
        double x1 = xa - xb;
        double y1 = ya - yb;
        // double x0 = hole_loc_record[0].x - hole_loc_record[1].x;
        // double y0 = hole_loc_record[0].y - hole_loc_record[1].y;
        ROS_INFO("x1: %f",x1);
        ROS_INFO("y1: %f",y1);
        ROS_INFO("x0: %f",x0);
        ROS_INFO("y0: %f",y0);
        // double cos_theta = (x0 * x1 + y0 * y1) / (x1 * x1 + y1 * y1);
        double sin_theta = (x0 * y1 - x1 * y0) / (x1 * x1 + y1 * y1);
        ROS_INFO("sin_theta: %f",sin_theta);
        double cos_theta = std::sqrt(1 - sin_theta * sin_theta);
        // sin_theta = sqrt(1 - cos_theta * cos_theta);
        rotation_mat[0] = cos_theta;
        rotation_mat[1] = -sin_theta;
        rotation_mat[2] = sin_theta;
        rotation_mat[3] = cos_theta;
        cv::Mat a_mat = cv::Mat(2, 1, CV_64FC1);
        cv::Mat a0_mat = cv::Mat(2, 1, CV_64FC1);
        a_mat.at<double>(0, 0) = xa;
        a_mat.at<double>(1, 0) = ya;
        r_mat.at<double>(0, 0) = cos_theta;
        r_mat.at<double>(0, 1) = -sin_theta;
        r_mat.at<double>(1, 0) = sin_theta;
        r_mat.at<double>(1, 1) = cos_theta;
        a0_mat.at<double>(0, 0) = hole_loc_record[0].x;
        a0_mat.at<double>(1, 0) = hole_loc_record[0].y;
        t_mat = a_mat - r_mat * a0_mat;
        translation_mat[0] = t_mat.at<double>(0, 0);
        translation_mat[1] = t_mat.at<double>(1, 0);
        ROS_INFO("translation_x: %f",translation_mat[0]);
        ROS_INFO("translation_y: %f",translation_mat[1]);

        //计算grip_point delta_x,y
        std::vector<double> grip_delta = compute_delta(robot1_grip_point);
        double grip_delta_x = grip_delta.at(0);
        double grip_delta_y = grip_delta.at(1);
        ROS_INFO("grip delta x: %f", grip_delta_x);
        ROS_INFO("grip delta x: %f", grip_delta_y);
        double alpha = std::asin(sin_theta);
        //计算gripper和ee连线与xy坐标系夹角theta_0
        // robot1_move_group.setEndEffectorLink("robot1_ee_link");
        // ros::Duration(0.1).sleep();
        // geometry_msgs::Pose robot1_current_pose = robot1_move_group.getCurrentPose().pose;
        // double X0 = robot1_current_pose.position.x;
        // double Y0 = robot1_current_pose.position.y;
        // robot1_move_group.setEndEffectorLink("robot1_gripper_eef_link");
        // ros::Duration(0.1).sleep();
        // robot1_current_pose = robot1_move_group.getCurrentPose().pose;
        // double X1 = robot1_current_pose.position.x;
        // double Y1 = robot1_current_pose.position.y;
        // double theta_0 = std::atan((Y1 - Y0) / (X1 - X0));
        ROS_INFO("the angle difference is %f", alpha);
        // double len = 125;
        // double DELTA_X = len * (std::cos(theta_0 + alpha) - std::cos(theta_0));
        // double DELTA_Y = len * (std::sin(theta_0 + alpha) - std::sin(theta_0));

        robot1_move_group.setEndEffectorLink("robot1_gripper_eef_link");
        ros::Duration(0.1).sleep();
        robot1_current_pose = robot1_move_group.getCurrentPose().pose;
        double X0 = robot1_current_pose.position.x;
        double Y0 = robot1_current_pose.position.y;
                

        //旋转alpha
        if (std::abs(alpha) > 0.017) 
        {
            std::vector<double> robot1_current_joint_values = robot1_move_group.getCurrentJointValues();
            robot1_current_joint_values.at(5) -= alpha;
            robot1_move_group.setJointValueTarget(robot1_current_joint_values);
            moveit::planning_interface::MoveGroupInterface::Plan rotation_plan;
            bool success = (robot1_move_group.plan(rotation_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO("rotation plan %s", success? "":"Failed");
            if (success) {
                robot1_move_group.execute(rotation_plan);
            } 

            robot1_current_pose = robot1_move_group.getCurrentPose().pose;
            double X1 = robot1_current_pose.position.x;
            double Y1 = robot1_current_pose.position.y;
            double DELTA_X = X1 - X0;
            double DELTA_Y = Y1 - Y0;
            ROS_INFO("DELTA_X of rotation: %f",DELTA_X);
            ROS_INFO("DELTA_Y of rotation: %f",DELTA_Y);
            //更新delta
            grip_delta_x -= DELTA_X;
            grip_delta_y -= DELTA_Y;
        }
        

        ROS_INFO("translation..");
        robot1_current_pose = robot1_move_group.getCurrentPose().pose;
        robot1_grip_point.x += grip_delta_x;
        robot1_grip_point.y += grip_delta_y;
        robot1_grip_point.z += 0.1;
        robot1_grip_quat = robot1_current_pose.orientation;
        bool ret1 = cam_bot_go(robot1_grip_point, robot1_grip_quat, robot1_move_group);
        if (!ret1)
        {
            std::cout << "wrong cartesian" << std::endl;
            return 0;
        }
        //open gripper
        open_gripper();
        ROS_INFO("open the gripper");
        robot1_grip_point.z -= 0.1;
        robot1_current_pose = robot1_move_group.getCurrentPose().pose;
        robot1_grip_quat = robot1_current_pose.orientation;
        ret1 = cam_bot_go(robot1_grip_point, robot1_grip_quat, robot1_move_group);
        if (!ret1)
        {
            std::cout << "wrong cartesian" << std::endl;
            return 0;
        }
        ros::Duration(1.5).sleep();
        //close gripper
        close_gripper();
        ROS_INFO("close the gripper");

        robot1_grip_point.z += 0.1;
        robot1_current_pose = robot1_move_group.getCurrentPose().pose;
        robot1_grip_quat = robot1_current_pose.orientation;
        ret1 = cam_bot_go(robot1_grip_point, robot1_grip_quat, robot1_move_group);
        if (!ret1)
        {
            std::cout << "wrong cartesian" << std::endl;
            return 0;
        }


        //move to upper
        std::vector<double> grip2_delta = compute_delta(robot1_grip_point2);
        double grip2_delta_x = grip2_delta.at(0);
        double grip2_delta_y = grip2_delta.at(1);
        robot1_grip_point2.x += grip2_delta_x;
        robot1_grip_point2.y += grip2_delta_y;
        robot1_grip_point2.z += 0.1;
        robot1_current_pose = robot1_move_group.getCurrentPose().pose;
        robot1_grip_quat = robot1_current_pose.orientation;
        ret1 = cam_bot_go(robot1_grip_point2, robot1_grip_quat, robot1_move_group);
        if (!ret1)
        {
            std::cout << "wrong cartesian" << std::endl;
            return 0;
        }
        robot1_grip_point2.z -= 0.1;
        ret1 = cam_bot_go(robot1_grip_point2, robot1_grip_quat, robot1_move_group);
        if (!ret1)
        {
            std::cout << "wrong cartesian" << std::endl;
            return 0;
        }
        //open gripper
        open_gripper();
        ROS_INFO("open the gripper");
        robot1_grip_point2.z += 0.2;
        robot1_current_pose = robot1_move_group.getCurrentPose().pose;
        robot1_grip_quat = robot1_current_pose.orientation;
        ret1 = cam_bot_go(robot1_grip_point2, robot1_grip_quat, robot1_move_group);
        if (!ret1)
        {
            std::cout << "wrong cartesian" << std::endl;
            return 0;
        }
        robot1_grip_point2.x -= 0.6;
        ret1 = cam_bot_go(robot1_grip_point2, robot1_grip_quat, robot1_move_group);
        if (!ret1)
        {
            std::cout << "wrong cartesian" << std::endl;
            return 0;
        }


        //robot1 move away
        // ROS_INFO("cam robot move to waiting point");
        // if (!cam_bot_go(robot1_wait_point, robot1_wait_quat, robot1_move_group))
        // {
        //     std::cout << "wrong cartesian" << std::endl;
        //     return 0;
        // }


        //compute delta x,y for holes 
        for (int current_hole = 2; current_hole < total_holes; current_hole++)
        {
            ROS_INFO(("hole " + std::to_string(current_hole) + ", delta x,y").c_str());
            auto temp1 = hole_loc_record[current_hole];
            std::vector<double> hole_delta = compute_delta(temp1);
            delta_xy[current_hole][0] = hole_delta.at(0);
            delta_xy[current_hole][1] = hole_delta.at(1);

            std::cout << "\n####################" << std::endl;
            std::cout << "delta x: " << hole_delta.at(0) * 1000 << " mm" << std::endl;
            std::cout << "delta y: " << hole_delta.at(1) * 1000 << " mm" << std::endl;
            std::cout << "####################\n"
                        << std::endl;
        }
        //中断
        // return 0;

        // delta theta
        // assert(total_holes%2==0);
        // for (int current_hole = 0; current_hole < total_holes; current_hole+=2)
        // {
        //     ROS_INFO(("hole " + std::to_string(current_hole) + ", delta theta").c_str());

        //     auto point_base = hole_loc_record[current_hole];
        //     auto point_delta = hole_loc_record[current_hole+1];

        //     double base_theta = 3.1415926/2;
        //     if(std::abs(point_delta.x - point_base.x) < std::numeric_limits<double>::eps * 1000){
        //         if((point_delta.y - point_base.y)<0){
        //             base_theta = -base_theta;
        //         }
        //     }else{
        //         base_theta = std::atan((point_delta.y - point_base.y)/(point_delta.x - point_base.x));
        //     }
            
        //     point_base.x += delta_xy[current_hole][0];
        //     point_base.y += delta_xy[current_hole][1];
            
        //     point_delta.x += delta_xy[current_hole+1][0];
        //     point_delta.y += delta_xy[current_hole+1][1];

        //     double delta_theta = 3.1415926/2;
        //     if(std::abs(point_delta.x - point_base.x) < std::numeric_limits<double>::eps * 1000){
        //         if((point_delta.y - point_base.y)<0){
        //             delta_theta = -base_theta;
        //         }
        //     }else{
        //         delta_theta = std::atan((point_delta.y - point_base.y)/(point_delta.x - point_base.x));
        //     }
        //     delta_theta -= base_theta; 
        //     ROS_INFO(("delta theta: " + std::to_string(delta_theta) + " rad").c_str());

        // }

        for (int current_hole = 5; current_hole >= 2; current_hole--)
        {
            ROS_INFO(("hole " + std::to_string(current_hole) + ", screw_robot move").c_str());

            auto point_ = hole_loc_record2[current_hole];
            point_.x += delta_xy[current_hole][0];
            point_.y += delta_xy[current_hole][1];

            if (!screw_bot_go(point_, hole_quat_record2[current_hole], robot2_move_group))
            {
                std::cout << "wrong cartesian" << std::endl;
                return 0;
            }
            ros::Duration(5.0).sleep();
            if (isscrew) {
                screw_hole();
            }
            
        }

        //robot2 move away
        ROS_INFO("screw robot move to waiting point");
        if (!screw_bot_go_wait(robot2_wait_point, robot2_wait_quat, robot2_move_group))
        {
            std::cout << "wrong cartesian" << std::endl;
            return 0;
        }

        ros::Duration(2.0).sleep();
        ROS_INFO("program ended");
    }
}