#include "ros/ros.h"
#include <moveit/move_group_interface/move_group.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <ur_msgs/SetIO.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "weitu.h"
#include <iostream>
#include <csignal>
#include <cmath>

static int total_holes = 2;

static bool use_cam = true;
static bool init_data_cali = true;
static bool linear_cali = true;
static bool init_success = false;

static std::vector<geometry_msgs::Point> hole_loc_record(total_holes);
static std::vector<geometry_msgs::Quaternion> hole_quat_record(total_holes);

static std::vector<geometry_msgs::Point> hole_loc_record2(total_holes);
static std::vector<geometry_msgs::Quaternion> hole_quat_record2(total_holes);

static geometry_msgs::Point robot1_wait_point;
static geometry_msgs::Quaternion robot1_wait_quat;

static geometry_msgs::Point robot2_wait_point;
static geometry_msgs::Quaternion robot2_wait_quat;

static double cam_z = 0.234;
static std::vector<std::vector<double>> hole_detect_record_base(total_holes, {0, 0});
static std::vector<std::vector<double>> delta_xy(total_holes, {0, 0});
static double linear_cali_mat[4] = {1, 0, 0, 1};

static ros::ServiceClient robot2_io_states_client;

static const double jump_threshold = 5;
static const double eef_step = 0.005;

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
    cv::FileStorage fs("data.yaml", cv::FileStorage::WRITE);
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
}

void load_data()
{
    cv::FileStorage fs("data.yaml", cv::FileStorage::READ);
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

int main(int argc, char *argv[])
{
    std::signal(SIGINT, SIGINT_handler);

    // at least open one window for waitkey
    auto show_1 = cv::imread("1.png");
    cv::imshow("1", show_1);

    load_data();

    ros::init(argc, argv, "dual_arm");
    ros::NodeHandle nh;
    
    const std::string ROBOT1_PLANNING_GROUP = "robot1_manipulator";
    const std::string ROBOT2_PLANNING_GROUP = "robot2_manipulator";
    moveit::planning_interface::MoveGroupInterface robot1_move_group(ROBOT1_PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface robot2_move_group(ROBOT2_PLANNING_GROUP);
    robot1_move_group.setEndEffectorLink("robot1_camera_eef_link");

    robot2_io_states_client = nh.serviceClient<ur_msgs::SetIO>("robot2/ur_driver/set_io");
    ros::AsyncSpinner spinner(2);
    spinner.start();

    init_success = true;

    init_data_cali = false;
    if (init_data_cali)
    {
        std::cout << "init calibrating..." << std::endl;

        bool cam_bot_cali = true;
        if (cam_bot_cali)
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
                        char key = cv::waitKey(100);

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
            if (linear_cali)
            {
                int current_hole = 0;
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
            {
                ROS_INFO("calibrating cam_robot wait pos");
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

            robot2_io_states_srv.request.state = 0.0;
            robot2_io_states_client.call(robot2_io_states_srv);
        }

        bool screw_bot_cali = false;
        if (screw_bot_cali)
        {
            for (int current_hole = 0; current_hole < total_holes; current_hole++)
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
    }

    // while (1)
    {
        // set 0 to move bot to waiting pos
        // total_holes = 0;

        for (int current_hole = 0; current_hole < total_holes; current_hole++)
        {
            ROS_INFO(("hole " + std::to_string(current_hole) + ", cam_robot move").c_str());
            auto temp1 = hole_loc_record[current_hole];
            // temp1.x += 0.006;
            // temp1.y += 0.004;

            if (!cam_bot_go(temp1, hole_quat_record[current_hole], robot1_move_group))
            {
                std::cout << "wrong cartesian" << std::endl;
                return 0;
            }

            if (use_cam)
            {
                auto cam_xy = get_xy(cam_z);
                cv::waitKey(1000);

                if (cam_xy.size() > 0)
                {

                    double cam_delta_x = cam_xy[0] - hole_detect_record_base[current_hole][0];
                    double cam_delta_y = cam_xy[1] - hole_detect_record_base[current_hole][1];

                    double delta_x, delta_y;
                    delta_x = linear_cali_mat[0] * cam_delta_x + linear_cali_mat[1] * cam_delta_y;
                    delta_y = linear_cali_mat[2] * cam_delta_x + linear_cali_mat[3] * cam_delta_y;

                    geometry_msgs::Pose robot1_current_pose = robot1_move_group.getCurrentPose().pose;
                    robot1_current_pose = robot1_move_group.getCurrentPose().pose;
                    double bot1_dx = robot1_current_pose.position.x - temp1.x;
                    double bot1_dy = robot1_current_pose.position.y - temp1.y;

                    delta_x += bot1_dx;
                    delta_y += bot1_dy;

                    delta_xy[current_hole][0] = delta_x;
                    delta_xy[current_hole][1] = delta_y;

                    std::cout << "\n####################" << std::endl;
                    std::cout << "delta x: " << delta_x*1000 << " mm" << std::endl;
                    std::cout << "delta y: " << delta_y*1000 << " mm" << std::endl;
                    std::cout << "####################\n"
                              << std::endl;
                }
                else
                {
                    std::cout << "no hole detected" << std::endl;
                }
            }
        }

        //robot1 move away
        ROS_INFO("cam robot move to waiting point");
        if (!cam_bot_go(robot1_wait_point, robot1_wait_quat, robot1_move_group))
        {
            std::cout << "wrong cartesian" << std::endl;
            return 0;
        }

        // delta theta
        assert(total_holes%2==0);
        for (int current_hole = 0; current_hole < total_holes; current_hole+=2)
        {
            ROS_INFO(("hole " + std::to_string(current_hole) + ", delta theta").c_str());

            auto point_base = hole_loc_record[current_hole];
            auto point_delta = hole_loc_record[current_hole+1];

            double base_theta = 3.1415926/2;
            if(std::abs(point_delta.x - point_base.x) < std::numeric_limits<double>::epsilon() * 1000){
                if((point_delta.y - point_base.y)<0){
                    base_theta = -base_theta;
                }
            }else{
                base_theta = std::atan((point_delta.y - point_base.y)/(point_delta.x - point_base.x));
            }
            
            point_base.x += delta_xy[current_hole][0];
            point_base.y += delta_xy[current_hole][1];
            
            point_delta.x += delta_xy[current_hole+1][0];
            point_delta.y += delta_xy[current_hole+1][1];

            double delta_theta = 3.1415926/2;
            if(std::abs(point_delta.x - point_base.x) < std::numeric_limits<double>::epsilon() * 1000){
                if((point_delta.y - point_base.y)<0){
                    delta_theta = -base_theta;
                }
            }else{
                delta_theta = std::atan((point_delta.y - point_base.y)/(point_delta.x - point_base.x));
            }
            delta_theta -= base_theta;

            if(delta_theta > 3.141593/2) delta_theta -= 3.141593;
            if(delta_theta < -3.141593/2) delta_theta += 3.141593;  

            std::cout << "\n####################" << std::endl;
            ROS_INFO(("delta theta: " + std::to_string(delta_theta*180/3.141593) + " deg").c_str());
            std::cout << "####################\n" << std::endl;

        }

        ROS_INFO("camera bot end");
        return 0;

        for (int current_hole = 0; current_hole < total_holes; current_hole++)
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
            screw_hole();
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
