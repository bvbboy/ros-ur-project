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

#include "ur_modern_driver/RG2.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

static int total_holes = 2;
static int grip_num = 2;
static bool use_cam = true;
static bool init_data_cali = true;

static std::vector<geometry_msgs::Point> loc_record1(total_holes);
static std::vector<geometry_msgs::Quaternion> quat_record1(total_holes);
static std::vector<geometry_msgs::Point> grip_loc_record1(grip_num);
static std::vector<geometry_msgs::Quaternion> grip_quat_record1(grip_num);

static std::vector<geometry_msgs::Point> loc_record2(total_holes);
static std::vector<geometry_msgs::Quaternion> quat_record2(total_holes);
static std::vector<geometry_msgs::Point> grip_loc_record2(grip_num);
static std::vector<geometry_msgs::Quaternion> grip_quat_record2(grip_num);

static geometry_msgs::Point wait_point1;
static geometry_msgs::Quaternion wait_quat1;
static geometry_msgs::Point wait_point2;
static geometry_msgs::Quaternion wait_quat2;

static std::vector<std::vector<double>> detect_record_base1(total_holes, {0, 0});
static std::vector<std::vector<double>> detect_record_base2(total_holes, {0, 0});
static std::vector<std::vector<double>> delta_xy1(total_holes, {0, 0});
static std::vector<std::vector<double>> delta_xy2(total_holes, {0, 0});
static double linear_cali_mat[4] = {1, 0, 0, 1};
double rotation_mat[4] = {1, 0, 0, 1};
double translation_mat[2] = {0, 0};
cv::Mat r_mat = cv::Mat(2, 2, CV_64FC1);
cv::Mat t_mat = cv::Mat(2, 1, CV_64FC1);

static double cam_z = 0.234;
static const double jump_threshold = 5;
static const double eef_step = 0.002;
double xa, ya, xb, yb;

bool digital_in_flag = false;
std::vector<ur_msgs::Digital> digital_in;
ros::Subscriber io_states_sub;
ur_msgs::SetIO io_states_srv;
ros::ServiceClient io_states_client;
std::string io_states_topic = "ur_driver/io_states";
std::string io_states_srv_name = "ur_driver/set_io";
ros::ServiceClient rg2_client;
std::string rg2_srv_name = "rg2_gripper/control_width";

void save_point_quat(geometry_msgs::Point &point, std::string str_point,
                     geometry_msgs::Quaternion &quat, std::string str_quat,
                     cv::FileStorage &fs)
{
    fs << (str_point + "_x") << point.x;
    fs << (str_point + "_y") << point.y;
    fs << (str_point + "_z") << point.z;
    fs << (str_quat + "_x") << quat.x;
    fs << (str_quat + "_y") << quat.y;
    fs << (str_quat + "_z") << quat.z;
    fs << (str_quat + "_w") << quat.w;
}

template <typename T>
void load_point_quat(geometry_msgs::Point &point, std::string str_point,
                     geometry_msgs::Quaternion &quat, std::string str_quat,
                     T &fs)
{
    point.x = (double)(fs)[str_point + "_x"];
    point.y = (double)(fs)[str_point + "_y"];
    point.z = (double)(fs)[str_point + "_z"];
    quat.x = (double)(fs)[str_quat + "_x"];
    quat.y = (double)(fs)[str_quat + "_y"];
    quat.z = (double)(fs)[str_quat + "_z"];
    quat.w = (double)(fs)[str_quat + "_w"];
}

void save_data()
{
    cv::FileStorage fs("/home/a/catkin_ws/data_mir.yaml", cv::FileStorage::WRITE);
    save_point_quat(wait_point1, "wait_point1", wait_quat1, "wait_quat1", fs);
    save_point_quat(wait_point2, "wait_point2", wait_quat2, "wait_quat2", fs);

    fs << "record1"
       << "[";
    for (int i = 0; i < loc_record1.size(); i++)
    {
        fs << "{";
        save_point_quat(loc_record1[i], "loc", quat_record1[i], "quat", fs);
        fs << "}";
    }
    fs << "]";

    fs << "grip_record1"
       << "[";
    for (int i = 0; i < grip_loc_record1.size(); i++)
    {
        fs << "{";
        save_point_quat(grip_loc_record1[i], "grip_loc", grip_quat_record1[i], "grip_quat", fs);
        fs << "}";
    }
    fs << "]";

    fs << "record2"
       << "[";
    for (int i = 0; i < loc_record2.size(); i++)
    {
        fs << "{";
        save_point_quat(loc_record2[i], "loc", quat_record2[i], "quat", fs);
        fs << "}";
    }
    fs << "]";

    fs << "grip_record2"
       << "[";
    for (int i = 0; i < grip_loc_record2.size(); i++)
    {
        fs << "{";
        save_point_quat(grip_loc_record2[i], "grip_loc", grip_quat_record2[i], "grip_quat", fs);
        fs << "}";
    }
    fs << "]";

    fs << "detect_record_base1"
       << "[";
    for (auto &p : detect_record_base1)
    {
        fs << "[";
        for (auto v : p)
        {
            fs << v;
        }
        fs << "]";
    }
    fs << "]";

    fs << "detect_record_base2"
       << "[";
    for (auto &p : detect_record_base2)
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
    cv::FileStorage fs("/home/a/catkin_ws/data_mir.yaml", cv::FileStorage::READ);
    load_point_quat(wait_point1, "wait_point1", wait_quat1, "wait_quat1", fs);
    load_point_quat(wait_point2, "wait_point2", wait_quat2, "wait_quat2", fs);

    {
        cv::FileNode record1 = fs["record1"];
        cv::FileNodeIterator it = record1.begin(), it_end = record1.end();
        for (int i = 0; it != it_end; ++it, i++)
        {
            auto fn = *it;
            load_point_quat(loc_record1[i], "loc", quat_record1[i], "quat", fn);
        }
    }

    {
        cv::FileNode grip_record1 = fs["grip_record1"];
        cv::FileNodeIterator it = grip_record1.begin(), it_end = grip_record1.end();
        for (int i = 0; it != it_end; ++it, i++)
        {
            auto fn = *it;
            load_point_quat(grip_loc_record1[i], "grip_loc", grip_quat_record1[i], "grip_quat", fn);
        }
    }

    {
        cv::FileNode record2 = fs["record2"];
        cv::FileNodeIterator it = record2.begin(), it_end = record2.end();
        for (int i = 0; it != it_end; ++it, i++)
        {
            auto fn = *it;
            load_point_quat(loc_record2[i], "loc", quat_record2[i], "quat", fn);
        }
    }

    {
        cv::FileNode grip_record2 = fs["grip_record2"];
        cv::FileNodeIterator it = grip_record2.begin(), it_end = grip_record2.end();
        for (int i = 0; it != it_end; ++it, i++)
        {
            auto fn = *it;
            load_point_quat(grip_loc_record2[i], "grip_loc", grip_quat_record2[i], "grip_quat", fn);
        }
    }

    {
        cv::FileNode detect_record_base1_ = fs["detect_record_base1"];
        cv::FileNodeIterator it = detect_record_base1_.begin(), it_end = detect_record_base1_.end();
        for (int i = 0; it != it_end; ++it, i++)
        {
            auto inner_seq = *it;
            auto inner_it = inner_seq.begin();
            auto inner_it_end = inner_seq.end();
            for (int j = 0; inner_it != inner_it_end; inner_it++, j++)
            {
                detect_record_base1[i][j] = (double)(*inner_it);
            }
        }
    }

    {
        cv::FileNode detect_record_base2_ = fs["detect_record_base2"];
        cv::FileNodeIterator it = detect_record_base2_.begin(), it_end = detect_record_base2_.end();
        for (int i = 0; it != it_end; ++it, i++)
        {
            auto inner_seq = *it;
            auto inner_it = inner_seq.begin();
            auto inner_it_end = inner_seq.end();
            for (int j = 0; inner_it != inner_it_end; inner_it++, j++)
            {
                detect_record_base2[i][j] = (double)(*inner_it);
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

bool robot_go(geometry_msgs::Point p, geometry_msgs::Quaternion q, moveit::planning_interface::MoveGroupInterface &move_group)
{
    double fraction = 0;
    std::vector<geometry_msgs::Pose> waypoints;

    geometry_msgs::Pose pose1;
    pose1.orientation = q;
    pose1.position = p;
    waypoints.push_back(pose1);
    moveit_msgs::RobotTrajectory trajectory1;

    fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory1);
    if (fraction < 0.9)
    {
        return false;
    }
    ROS_WARN("Robot cartesian path. (%.2f%% achieved)", fraction * 100.0);
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    plan1.trajectory_ = trajectory1;
    move_group.execute(plan1);
    return true;
}

bool robot_go2(geometry_msgs::Point p, geometry_msgs::Quaternion q, moveit::planning_interface::MoveGroupInterface &move_group)
{
    double fraction = 0;
    std::vector<geometry_msgs::Pose> waypoints;

    geometry_msgs::Pose pose1;
    pose1.orientation = q;
    pose1.position = p;
    geometry_msgs::Pose mid_pose;
    mid_pose = pose1;
    mid_pose.position.z += 0.05;
    waypoints.push_back(mid_pose);
    waypoints.push_back(pose1);

    moveit_msgs::RobotTrajectory trajectory2;

    fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory2);
    if (fraction < 0.9)
    {
        return false;
    }

    ROS_WARN("Robot cartesian path. (%.2f%% achieved)", fraction * 100.0);
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    plan1.trajectory_ = trajectory2;
    move_group.execute(plan1);
    return true;
}

bool robot_go3(geometry_msgs::Point p, geometry_msgs::Quaternion q, moveit::planning_interface::MoveGroupInterface &move_group)
{
    double fraction = 0;
    std::vector<geometry_msgs::Pose> waypoints;

    geometry_msgs::Pose pose1;
    pose1.orientation = q;
    pose1.position = p;
    geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;
    geometry_msgs::Pose mid_pose;
    mid_pose = current_pose;
    mid_pose.position.z += 0.05;
    waypoints.push_back(mid_pose);
    waypoints.push_back(pose1);

    moveit_msgs::RobotTrajectory trajectory3;
    fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory3);
    if (fraction < 0.9)
    {
        return false;
    }
    ROS_WARN("Robot cartesian path. (%.2f%% achieved)", fraction * 100.0);
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    plan1.trajectory_ = trajectory3;
    move_group.execute(plan1);
    return true;
}

std::vector<double> get_xy(double cam_z)
{
    std::vector<double> result;

    auto detect_xy = hole_detect::find_hole(cam_z);
    if (detect_xy.size() > 0)
    {
        result = detect_xy;
    }
    return result;
}

void robot_io_callback(const ur_msgs::IOStates::ConstPtr &msg)
{
    digital_in = msg->digital_in_states;
    if (digital_in.size())
    {
        digital_in_flag = true;
    }
    else
    {
        ROS_INFO("digital_in is null!");
        digital_in_flag = false;
    }
}

void robot_setio(int fun, int pin, double state)
{
    io_states_srv.request.fun = fun;
    io_states_srv.request.pin = pin;
    io_states_srv.request.state = state;
    io_states_client.call(io_states_srv);
    ros::Duration(2.0).sleep();
    io_states_srv.request.state = 0.0;
    io_states_client.call(io_states_srv);
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

void rg2_width(double target)
{
    ur_modern_driver::RG2 srv;
    double target_width = target;
    srv.request.target_width = target_width;
    if (rg2_client.call(srv))
    {
        ROS_INFO("current width: %lf", target_width);
    }
    else
    {
        ROS_INFO("failed to call service");
    }
}

int main(int argc, char *argv[])
{
    auto show_1 = cv::imread("/home/a/catkin_ws/1.png");
    cv::imshow("1", show_1);
    cv::waitKey(1);

    load_data();

    ros::init(argc, argv, "mir");
    ros::NodeHandle nh;
    const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    move_group.setEndEffectorLink("rg2_eef_link");
    io_states_sub = nh.subscribe<ur_msgs::IOStates>(io_states_topic, 10, robot_io_callback);
    io_states_client = nh.serviceClient<ur_msgs::SetIO>(io_states_srv_name);
    rg2_client = nh.serviceClient<ur_modern_driver::RG2>(rg2_srv_name);
    ros::AsyncSpinner spinner(2);
    spinner.start();

    geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;
    bool ret1 = true;
    bool init_success = true;
    bool mir_signal = true;

    init_data_cali = false;
    while (1)
    {
        std::cout << "\n>>> "
                  << "cali or not, default not, timeout 3s. (y n): " << std::endl;
        char key = cv::waitKey(5000);
        if (key == 'y')
            init_data_cali = true;
        else if (key == 'n')
            init_data_cali = false;
        break;
    }

    if (init_data_cali)
    {
        std::cout << "init calibrating...\n"
                  << std::endl;

        bool gongwei1_1 = false;
        bool gongwei1_2 = false;
        bool gongwei2_1 = false;
        bool gongwei2_2 = false;

        bool wait1_flag = false;
        bool wait2_flag = false;

        bool camera_flag = false;

        while (1)
        {

            bool break_flag = false;
            while (1)
            {
                std::cout << "\n>>> cali what?" << std::endl;
                std::cout << "1. gongwei1_1(AGV pos 1)" << std::endl;
                std::cout << "2. gongwei1_2(AGV pos 2)" << std::endl;
                std::cout << "3. gongwei2_1(table pos 1)" << std::endl;
                std::cout << "4. gongwei2_2(table pos 2)" << std::endl;
                std::cout << "5. camera" << std::endl;
                std::cout << "6. wait1(AGV)" << std::endl;
                std::cout << "7. wait2(table)" << std::endl;
                std::cout << "q to end" << std::endl;

                char key = cv::waitKey(0);
                if (key == 'q')
                    break_flag = true;
                else if (key == '1')
                    gongwei1_1 = true;
                else if (key == '2')
                    gongwei1_2 = true;
                else if (key == '3')
                    gongwei2_1 = true;
                else if (key == '4')
                    gongwei2_2 = true;
                else if (key == '5')
                    camera_flag = true;
                else if (key == '6')
                    wait1_flag = true;
                else if (key == '7')
                    wait2_flag = true;
                else
                    continue;
                break;
            }

            if (break_flag)
                break;

            load_data();

            if (gongwei1_1)
            {
                // //工位1-1
                ROS_INFO("calibrating 1-1");

                bool go_by_what = true;
                while (1)
                {
                    std::cout << "\n>>> "
                              << "go by record or hand? defalut r, timeout 3s. (r h): " << std::endl;
                    char key = cv::waitKey(5000);
                    if (key == 'r')
                        go_by_what = true;
                    else if (key == 'h')
                        go_by_what = false;
                    break;
                }

                bool ret1 = true;
                if (init_success && go_by_what)
                {
                    ret1 = robot_go2(loc_record2[0], quat_record2[0], move_group);
                    if (!ret1)
                    {
                        std::cout << "wrong cartesian 1" << std::endl;
                        return 0;
                    }
                }
                else
                {
                    ROS_INFO("no init pos provided, move robot by hand");
                }

                std::cout << "please move to wanted pos, press space to next step" << std::endl;
                while (1)
                {
                    char key = cv::waitKey(0);
                    if (key == ' ')
                    {
                        break;
                    }
                }
                current_pose = move_group.getCurrentPose().pose;
                loc_record2[0] = current_pose.position;
                quat_record2[0] = current_pose.orientation;
                save_data();
                gongwei1_1 = false;
            }

            if (wait1_flag)
            {
                //wait point1
                ROS_INFO("calibrating robot wait pos1");

                bool go_by_what = true;
                while (1)
                {
                    std::cout << "\n>>> "
                              << "go by record or hand? defalut r, timeout 3s. (r h): " << std::endl;
                    char key = cv::waitKey(5000);
                    if (key == 'r')
                        go_by_what = true;
                    else if (key == 'h')
                        go_by_what = false;
                    break;
                }

                if (init_success && go_by_what)
                {
                    ret1 = robot_go(wait_point1, wait_quat1, move_group);
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
                current_pose = move_group.getCurrentPose().pose;
                wait_point1 = current_pose.position;
                wait_quat1 = current_pose.orientation;
                save_data();
                wait1_flag = false;
            }

            // !!!!!!!!!
            // double z_2 = loc_record1[0].z;

            if (camera_flag)
            {
                //工位2-+1/+2

                bool go_by_what = true;
                while (1)
                {
                    std::cout << "\n>>> "
                              << "go by record or hand? defalut r, timeout 3s. (r h): " << std::endl;
                    char key = cv::waitKey(5000);
                    if (key == 'r')
                        go_by_what = true;
                    else if (key == 'h')
                        go_by_what = false;
                    break;
                }

                for (int current_loc = 0; current_loc < total_holes; current_loc++)
                {
                    geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;

                    bool ret1 = true;
                    if (init_success && go_by_what)
                    {

                        // loc_record1[current_loc].z = z_2;

                        ret1 = robot_go(loc_record1[current_loc], quat_record1[current_loc], move_group);
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

                    weitu::Camera camera;
                    camera.open(0);
                    float bot_steps = 0.003;

                    while (1)
                    {
                        cv::Mat rgb = camera.get();
                        if (!rgb.empty())
                        {
                            // cv::Mat smaller_rgb;
                            // cv::pyrDown(rgb, smaller_rgb);
                            // auto p = hole_detect::find(smaller_rgb);
                            int start_cols = rgb.cols / 4;
                            int start_rows = rgb.rows / 4;
                            cv::Rect roi(start_cols, start_rows, rgb.cols / 2, rgb.rows / 2);
                            auto p = hole_detect::find(rgb(roi));
                        }

                        char key = cv::waitKey(0);
                        current_pose = move_group.getCurrentPose().pose;
                        auto temp = current_pose.position;
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
                        ret1 = robot_go(temp, current_pose.orientation, move_group);
                        if (!ret1)
                        {
                            std::cout << "wrong cartesian" << std::endl;
                            return 0;
                        }
                    }
                    current_pose = move_group.getCurrentPose().pose;
                    loc_record1[current_loc] = current_pose.position;
                    quat_record1[current_loc] = current_pose.orientation;

                    // z_2 = current_pose.position.z;

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
                        ROS_INFO(("detect_record_base1 " + std::to_string(current_loc) +
                                  "\nx: " + std::to_string(detect_xy[0]) +
                                  "\ny: " + std::to_string(detect_xy[1]))
                                     .c_str());
                        detect_record_base1[current_loc] = detect_xy;
                    }
                    else
                    {
                        std::cout << "nothing detect, wrong init of detect_record_base" << std::endl;
                        return 0;
                    }
                }
                save_data();

                //线性校正
                bool linear_cali = true;
                if (linear_cali)
                {
                    int current_loc = 1;
                    std::cout << "linear calibrating..." << std::endl;
                    double bound = 0.010;
                    auto loc_temp = loc_record1[current_loc];
                    cv::Mat cam_mat = cv::Mat(2, 2, CV_64FC1);
                    cv::Mat phy_mat = cv::Mat(2, 2, CV_64FC1);

                    // go 0.005, 0
                    loc_temp.x += bound;
                    bool ret1 = robot_go(loc_temp, quat_record1[current_loc], move_group);
                    if (!ret1)
                    {
                        std::cout << "wrong cartesian" << std::endl;
                        return 0;
                    }
                    auto cam_delta_xy1 = get_xy(cam_z);
                    cv::waitKey(2000);
                    cam_delta_xy1[0] -= detect_record_base1[current_loc][0];
                    cam_delta_xy1[1] -= detect_record_base1[current_loc][1];

                    cam_mat.at<double>(0, 0) = cam_delta_xy1[0];
                    cam_mat.at<double>(1, 0) = cam_delta_xy1[1];

                    current_pose = move_group.getCurrentPose().pose;
                    phy_mat.at<double>(0, 0) = current_pose.position.x - loc_record1[current_loc].x;
                    phy_mat.at<double>(1, 0) = current_pose.position.y - loc_record1[current_loc].y;

                    // go 0, 0.005
                    loc_temp.x = loc_record1[current_loc].x;
                    loc_temp.y += bound;
                    ret1 = robot_go(loc_temp, quat_record1[current_loc], move_group);
                    if (!ret1)
                    {
                        std::cout << "wrong cartesian" << std::endl;
                        return 0;
                    }
                    auto cam_delta_xy2 = get_xy(cam_z);
                    cv::waitKey(2000);

                    cam_delta_xy2[0] -= detect_record_base1[current_loc][0];
                    cam_delta_xy2[1] -= detect_record_base1[current_loc][1];

                    cam_mat.at<double>(0, 1) = cam_delta_xy2[0];
                    cam_mat.at<double>(1, 1) = cam_delta_xy2[1];

                    current_pose = move_group.getCurrentPose().pose;
                    phy_mat.at<double>(0, 1) = current_pose.position.x - loc_record1[current_loc].x;
                    phy_mat.at<double>(1, 1) = current_pose.position.y - loc_record1[current_loc].y;

                    // relative, bot plus means hole minus
                    cv::Mat linear_cv = (cam_mat.inv()) * (-phy_mat);

                    linear_cali_mat[0] = linear_cv.at<double>(0, 0);
                    linear_cali_mat[1] = linear_cv.at<double>(0, 1);
                    linear_cali_mat[2] = linear_cv.at<double>(1, 0);
                    linear_cali_mat[3] = linear_cv.at<double>(1, 1);

                    std::cout << "cam_mat: \n"
                              << cam_mat << std::endl;
                    std::cout << "phy_mat: \n"
                              << phy_mat << std::endl;

                    std::cout << "liear_cali_mat: \n";
                    std::cout << linear_cali_mat[0] << '\t' << linear_cali_mat[1] << '\n';
                    std::cout << linear_cali_mat[2] << '\t' << linear_cali_mat[3] << '\n'
                              << std::endl;
                }
                save_data();

                //消除两个定位点detect_record_base
                ROS_INFO("消除两个定位点detect_record_base");
                for (int current_hole = 0; current_hole < total_holes; current_hole++)
                {
                    double cam_delta_x = detect_record_base1[current_hole][0];
                    double cam_delta_y = detect_record_base1[current_hole][1];
                    double delta_x = linear_cali_mat[0] * cam_delta_x + linear_cali_mat[1] * cam_delta_y;
                    double delta_y = linear_cali_mat[2] * cam_delta_x + linear_cali_mat[3] * cam_delta_y;
                    ROS_INFO("消除hole_detect_record_base x: %f", delta_x);
                    ROS_INFO("消除hole_detect_record_base y: %f", delta_y);
                    loc_record1[current_hole].x += delta_x;
                    loc_record1[current_hole].y += delta_y;
                    ROS_INFO("loc_record1 x: %f", loc_record1[current_hole].x);
                    ROS_INFO("loc_record1 y: %f", loc_record1[current_hole].y);
                }
                save_data();
                camera_flag = false;
            }

            bool grip_cali = true;
            //工位2-1,2-1‘

            if (gongwei2_1)
            {

                bool go_by_what = true;
                while (1)
                {
                    std::cout << "\n>>> "
                              << "go by record or hand? defalut r, timeout 3s. (r h): " << std::endl;
                    char key = cv::waitKey(5000);
                    if (key == 'r')
                        go_by_what = true;
                    else if (key == 'h')
                        go_by_what = false;
                    break;
                }

                if (grip_cali)
                {
                    ROS_INFO("calibrating gripper pose1");

                    while (1)
                    {
                        std::cout << "\n>>> "
                                  << "go to AGV pos first? default n, timeout 3s: " << std::endl;
                        char key = cv::waitKey(5000);
                        if (key == 'y')
                        {

                            std::cout << "rotate to AGV, press any to continue" << std::endl;
                            cv::waitKey(0);

                            if (!robot_go2(loc_record2[0], quat_record2[0], move_group))
                            {
                                std::cout << "wrong cartesian" << std::endl;
                                return 0;
                            }

                            std::cout << "grip and rotate to table, press any to continue" << std::endl;
                            cv::waitKey(0);
                        }
                        break;
                    }

                    if (init_success && go_by_what)
                    {
                        // grip_loc_record1[0].z = z_2;
                        ret1 = robot_go2(grip_loc_record1[0], grip_quat_record1[0], move_group);

                        // robot_go2(loc_record2[0], quat_record2[0], move_group);
                        // return 0;
                        if (!ret1)
                        {
                            std::cout << "############################" << std::endl;
                            std::cout << "wrong cartesian gipper1" << std::endl;
                            std::cout << grip_loc_record1[0].x << "\t" << grip_loc_record1[0].y << "\t" << grip_loc_record1[0].z << std::endl;

                            current_pose = move_group.getCurrentPose().pose;
                            std::cout << current_pose.position.x << "\t" << current_pose.position.y << "\t" << current_pose.position.z << std::endl;
                            std::cout << "############################" << std::endl;
                            return 0;
                        }
                    }
                    else
                    {
                        ROS_INFO("no init pos provided, move robot by hand");
                    }
                    std::cout << "please move to grip pos1, press space to next step" << std::endl;
                    while (1)
                    {
                        char key = cv::waitKey(0);
                        if (key == ' ')
                        {
                            break;
                        }
                    }
                    current_pose = move_group.getCurrentPose().pose;
                    grip_loc_record1[0] = current_pose.position;
                    grip_quat_record1[0] = current_pose.orientation;
                    save_data();

                    if (init_success && go_by_what)
                    {
                        // grip_loc_record2[0].z = z_2;
                        ret1 = robot_go3(grip_loc_record2[0], grip_quat_record2[0], move_group);
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
                    std::cout << "please move to grip pos1', press space to next step" << std::endl;
                    while (1)
                    {
                        char key = cv::waitKey(0);
                        if (key == ' ')
                        {
                            break;
                        }
                    }
                    current_pose = move_group.getCurrentPose().pose;
                    grip_loc_record2[0] = current_pose.position;
                    grip_quat_record2[0] = current_pose.orientation;
                    save_data();
                }
                save_data();
                gongwei2_1 = false;
            }

            if (wait2_flag)
            {
                ROS_INFO("calibrating robot wait pos2");

                bool go_by_what = true;
                while (1)
                {
                    std::cout << "\n>>> "
                              << "go by record or hand? defalut r, timeout 3s. (r h): " << std::endl;
                    char key = cv::waitKey(5000);
                    if (key == 'r')
                        go_by_what = true;
                    else if (key == 'h')
                        go_by_what = false;
                    break;
                }

                if (init_success && go_by_what)
                {
                    // wait_point2.z = z_2;
                    ret1 = robot_go(wait_point2, wait_quat2, move_group);
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

                std::cout << "please move to wanted wait pos2, press space to next step" << std::endl;
                while (1)
                {
                    char key = cv::waitKey(0);
                    if (key == ' ')
                    {
                        break;
                    }
                }
                current_pose = move_group.getCurrentPose().pose;
                wait_point2 = current_pose.position;
                wait_quat2 = current_pose.orientation;

                // // !!!!!!!!!
                // wait_point1.z = wait_point2.z;
                save_data();
                wait2_flag = false;
            }

            if (gongwei1_2)
            {
                ROS_INFO("calibrating 1-2");

                bool go_by_what = true;
                while (1)
                {
                    std::cout << "\n>>> "
                              << "go by record or hand? defalut r, timeout 3s. (r h): " << std::endl;
                    char key = cv::waitKey(5000);
                    if (key == 'r')
                        go_by_what = true;
                    else if (key == 'h')
                        go_by_what = false;
                    break;
                }

                ret1 = true;
                if (init_success && go_by_what)
                {
                    ret1 = robot_go2(loc_record2[1], quat_record2[1], move_group);
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

                std::cout << "please move to wanted pos, press space to next step" << std::endl;
                while (1)
                {
                    char key = cv::waitKey(0);
                    if (key == ' ')
                    {
                        break;
                    }
                }
                current_pose = move_group.getCurrentPose().pose;
                loc_record2[1] = current_pose.position;
                quat_record2[1] = current_pose.orientation;
                save_data();
                gongwei1_2 = false;
            }

            // //工位1-2

            //工位2-2,2-2’

            if (gongwei2_2)
            {
                ROS_INFO("calibrating gripper pose2");

                bool go_by_what = true;
                while (1)
                {
                    std::cout << "\n>>> "
                              << "go by record or hand? defalut r, timeout 3s. (r h): " << std::endl;
                    char key = cv::waitKey(5000);
                    if (key == 'r')
                        go_by_what = true;
                    else if (key == 'h')
                        go_by_what = false;
                    break;
                }

                while (1)
                {
                    std::cout << "\n>>> "
                              << "go to AGV pos first? default n, timeout 3s: " << std::endl;
                    char key = cv::waitKey(5000);
                    if (key == 'y')
                    {

                        std::cout << "rotate to AGV, press any to continue" << std::endl;
                        cv::waitKey(0);

                        if (!robot_go2(loc_record2[1], quat_record2[1], move_group))
                        {
                            std::cout << "wrong cartesian" << std::endl;
                            return 0;
                        }

                        std::cout << "grip and rotate to table, press any to continue" << std::endl;
                        cv::waitKey(0);
                    }
                    break;
                }

                if (init_success && go_by_what)
                {
                    // grip_loc_record1[1].z = z_2;
                    ret1 = robot_go2(grip_loc_record1[1], grip_quat_record1[1], move_group);

                    // ret1 = robot_go2(loc_record2[1], quat_record2[1], move_group);
                    // return 0;
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
                std::cout << "please move to grip pos2, press space to next step" << std::endl;
                while (1)
                {
                    char key = cv::waitKey(0);
                    if (key == ' ')
                    {
                        break;
                    }
                }
                current_pose = move_group.getCurrentPose().pose;
                grip_loc_record1[1] = current_pose.position;
                grip_quat_record1[1] = current_pose.orientation;
                save_data();

                if (init_success && go_by_what)
                {
                    // grip_loc_record2[1].z = z_2;
                    ret1 = robot_go3(grip_loc_record2[1], grip_quat_record2[1], move_group);
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
                std::cout << "please move to grip pos2', press space to next step" << std::endl;
                while (1)
                {
                    char key = cv::waitKey(0);
                    if (key == ' ')
                    {
                        break;
                    }
                }
                current_pose = move_group.getCurrentPose().pose;
                grip_loc_record2[1] = current_pose.position;
                grip_quat_record2[1] = current_pose.orientation;
                save_data();
                gongwei2_2 = false;
            }
            save_data();
        }

        return 0;
    }

    while (1)
    {
        std::cout << "\n>>> "
                  << "with MIR signal or not, default yes, timeout 3s. (y n): " << std::endl;
        char key = cv::waitKey(5000);
        if (key == 'y')
            mir_signal = true;
        else if (key == 'n')
            mir_signal = false;
        break;
    }

    while (true)
    {

        ros::Duration(1.0).sleep();
        rg2_width(60);
        ros::Duration(1.0).sleep();

        //第1次对mark板
        {
            load_data();

            ROS_INFO("wait for mir signal");
            while (mir_signal)
            {
                if (digital_in_flag)
                {
                    if (digital_in.at(0).state == 1)
                    {
                        ROS_INFO("AGVs arrive!");
                        break;
                    }
                }
                ros::Duration(0.2).sleep();
            }

            ROS_INFO("grip 1-1");
            if (!robot_go2(loc_record2[0], quat_record2[0], move_group))
            {
                std::cout << "wrong cartesian" << std::endl;
                return 0;
            }
            ros::Duration(1.0).sleep();
            rg2_width(41.5);
            ros::Duration(1.0).sleep();

            if (!robot_go3(wait_point1, wait_quat1, move_group))
            {
                std::cout << "wrong cartesian" << std::endl;
                return 0;
            }

            //转-180°
            move_group.setMaxVelocityScalingFactor(1.0);
            std::vector<double> current_joint_values;
            current_joint_values = move_group.getCurrentJointValues();
            current_joint_values.at(0) += 2.9;
            move_group.setJointValueTarget(current_joint_values);
            moveit::planning_interface::MoveGroupInterface::Plan switch_plan;
            bool success = (move_group.plan(switch_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO("switch plan %s", success ? "" : "Failed");
            if (success)
            {
                move_group.execute(switch_plan);
            }
            else
            {
                std::cout << "wrong cartesian" << std::endl;
                return 0;
            }

            ROS_INFO("computing rotation and translation matrix");
            int current_loc = 0;
            auto temp1 = loc_record1[current_loc];
            if (!robot_go(temp1, quat_record1[current_loc], move_group))
            {
                std::cout << "wrong cartesian" << std::endl;
                return 0;
            }
            ros::Duration(2.0).sleep();
            auto cam_xy = get_xy(cam_z);
            cv::waitKey(1000);
            if (cam_xy.size() > 0)
            {
                double delta_xa = linear_cali_mat[0] * cam_xy[0] + linear_cali_mat[1] * cam_xy[1];
                double delta_ya = linear_cali_mat[2] * cam_xy[0] + linear_cali_mat[3] * cam_xy[1];
                current_pose = move_group.getCurrentPose().pose;
                ROS_INFO("delta_x: %lf", delta_xa);
                ROS_INFO("delta_y: %lf", delta_ya);
                xa = current_pose.position.x + delta_xa;
                ya = current_pose.position.y + delta_ya;
            }
            current_pose = move_group.getCurrentPose().pose;
            double current_hole0_x = current_pose.position.x;
            double current_hole0_y = current_pose.position.y;
            // ROS_INFO("current_hole0_x: %lf", current_hole0_x);
            // ROS_INFO("current_hole0_y: %lf", current_hole0_y);

            current_loc++;
            temp1 = loc_record1[current_loc];
            if (!robot_go(temp1, quat_record1[current_loc], move_group))
            {
                std::cout << "wrong cartesian" << std::endl;
                return 0;
            }
            ros::Duration(2.0).sleep();
            cam_xy = get_xy(cam_z);
            cv::waitKey(1000);
            if (cam_xy.size() > 0)
            {
                double delta_xb = linear_cali_mat[0] * cam_xy[0] + linear_cali_mat[1] * cam_xy[1];
                double delta_yb = linear_cali_mat[2] * cam_xy[0] + linear_cali_mat[3] * cam_xy[1];
                current_pose = move_group.getCurrentPose().pose;
                ROS_INFO("delta_x: %lf", delta_xb);
                ROS_INFO("delta_y: %lf", delta_yb);
                xb = current_pose.position.x + delta_xb;
                yb = current_pose.position.y + delta_yb;
            }
            current_pose = move_group.getCurrentPose().pose;
            double current_hole1_x = current_pose.position.x;
            double current_hole1_y = current_pose.position.y;
            // ROS_INFO("current_hole0_x: %lf", current_hole0_x);
            // ROS_INFO("current_hole0_y: %lf", current_hole0_y);

            double x0 = current_hole0_x - current_hole1_x;
            double y0 = current_hole0_y - current_hole1_y;
            double x1 = xa - xb;
            double y1 = ya - yb;
            ROS_INFO("x1: %f", x1);
            ROS_INFO("y1: %f", y1);
            ROS_INFO("x0: %f", x0);
            ROS_INFO("y0: %f", y0);
            double sin_theta = (x0 * y1 - x1 * y0) / (x1 * x1 + y1 * y1);
            ROS_INFO("sin_theta: %f", sin_theta);
            double cos_theta = std::sqrt(1 - sin_theta * sin_theta);
            rotation_mat[0] = cos_theta;
            rotation_mat[1] = -sin_theta;
            rotation_mat[2] = sin_theta;
            rotation_mat[3] = cos_theta;

            cv::Mat a_mat = cv::Mat(2, 1, CV_64FC1);
            cv::Mat a0_mat = cv::Mat(2, 1, CV_64FC1);
            a_mat.at<double>(0, 0) = xb;
            a_mat.at<double>(1, 0) = yb;
            r_mat.at<double>(0, 0) = cos_theta;
            r_mat.at<double>(0, 1) = -sin_theta;
            r_mat.at<double>(1, 0) = sin_theta;
            r_mat.at<double>(1, 1) = cos_theta;
            std::cout << "rotation_mat: " << std::endl
                      << r_mat.at<double>(0, 0) << " " << r_mat.at<double>(0, 1) << std::endl;
            std::cout << r_mat.at<double>(1, 0) << r_mat.at<double>(1, 1) << std::endl;
            a0_mat.at<double>(0, 0) = current_hole1_x;
            a0_mat.at<double>(1, 0) = current_hole1_y;
            cv::Mat t1_mat = cv::Mat(2, 1, CV_64FC1);
            t1_mat = a_mat - r_mat * a0_mat;

            cv::Mat b_mat = cv::Mat(2, 1, CV_64FC1);
            cv::Mat b0_mat = cv::Mat(2, 1, CV_64FC1);
            b_mat.at<double>(0, 0) = xa;
            b_mat.at<double>(1, 0) = ya;
            b0_mat.at<double>(0, 0) = current_hole0_x;
            b0_mat.at<double>(1, 0) = current_hole0_y;
            cv::Mat t2_mat = cv::Mat(2, 1, CV_64FC1);
            t2_mat = a_mat - r_mat * a0_mat;
            ROS_INFO("t1_x: %lf", t1_mat.at<double>(0, 0));
            ROS_INFO("t1_y: %lf", t1_mat.at<double>(1, 0));
            ROS_INFO("t2_x: %lf", t2_mat.at<double>(0, 0));
            ROS_INFO("t2_x: %lf", t2_mat.at<double>(1, 0));

            translation_mat[0] = (t1_mat.at<double>(0, 0) + t2_mat.at<double>(0, 0)) / 2;
            translation_mat[1] = (t1_mat.at<double>(1, 0) + t2_mat.at<double>(1, 0)) / 2;
            ROS_INFO("translation_x: %f", translation_mat[0]);
            ROS_INFO("translation_y: %f", translation_mat[1]);
            t_mat.at<double>(0, 0) = translation_mat[0];
            t_mat.at<double>(1, 0) = translation_mat[1];

            //loc_record和相机测量比较
            std::vector<double> loc_delta = compute_delta(loc_record1[0]);
            double loc_delta_x1 = loc_delta.at(0);
            double loc_delta_y1 = loc_delta.at(1);
            ROS_INFO("loc delta1 x: %f", loc_delta_x1);
            ROS_INFO("loc delta1 y: %f", loc_delta_y1);
            loc_delta = compute_delta(loc_record1[1]);
            double loc_delta_x2 = loc_delta.at(0);
            double loc_delta_y2 = loc_delta.at(1);
            ROS_INFO("loc delta2 x: %f", loc_delta_x2);
            ROS_INFO("loc delta2 y: %f", loc_delta_y2);

            //计算grip_point1,2
            // auto actual_grip_loc1 = grip_loc_record1[0];
            // actual_grip_loc1.x -= (0.387174-0.308610);
            // actual_grip_loc1.y -= (0.004478+0.002420);
            // std::vector<double> grip_delta = compute_delta(actual_grip_loc1);
            std::vector<double> grip_delta = compute_delta(grip_loc_record2[0]);
            double grip_delta_x1 = grip_delta.at(0);
            double grip_delta_y1 = grip_delta.at(1);
            ROS_INFO("grip delta1 x: %f", grip_delta_x1);
            ROS_INFO("grip delta1 y: %f", grip_delta_y1);

            // auto actual_grip_loc2 = grip_loc_record1[1];
            // actual_grip_loc2.x -= (0.387174-0.308610);
            // actual_grip_loc2.y -= (0.004478+0.002420);
            // grip_delta = compute_delta(actual_grip_loc2);
            grip_delta = compute_delta(grip_loc_record2[1]);
            double grip_delta_x2 = grip_delta.at(0);
            double grip_delta_y2 = grip_delta.at(1);
            ROS_INFO("grip delta2 x: %f", grip_delta_x2);
            ROS_INFO("grip delta2 y: %f", grip_delta_y2);

            double alpha = std::asin(sin_theta);
            ROS_INFO("the angle difference is %f", alpha * 180 / 3.14159);

            // return 0;

            //旋转alpha
            if (std::abs(alpha) > 0)
            {
                std::vector<double> current_joint_values = move_group.getCurrentJointValues();
                current_joint_values.at(5) -= alpha;
                move_group.setJointValueTarget(current_joint_values);
                moveit::planning_interface::MoveGroupInterface::Plan rotation_plan;
                bool success = (move_group.plan(rotation_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                ROS_INFO("rotation plan %s", success ? "" : "Failed");
                if (success)
                {
                    move_group.execute(rotation_plan);
                }
            }

            ROS_INFO("translation..");
            current_pose = move_group.getCurrentPose().pose;
            grip_loc_record1[0].x += grip_delta_x1;
            grip_loc_record1[0].y += grip_delta_y1;
            // grip_loc_record1[0].y += 0.120 * sin_theta;
            grip_quat_record1[0] = current_pose.orientation;
            ret1 = robot_go2(grip_loc_record1[0], grip_quat_record1[0], move_group);
            if (!ret1)
            {
                std::cout << "wrong cartesian" << std::endl;
                return 0;
            }
            ros::Duration(2.0).sleep();
            rg2_width(60);
            ros::Duration(2.0).sleep();

            ret1 = robot_go3(wait_point2, grip_quat_record1[0], move_group);
            if (!ret1)
            {
                std::cout << "wrong cartesian" << std::endl;
                return 0;
            }

            grip_loc_record1[1].x += grip_delta_x2;
            grip_loc_record1[1].y += grip_delta_y2;
            // grip_loc_record1[1].y += 0.120 * sin_theta;
            ret1 = robot_go2(grip_loc_record1[1], grip_quat_record1[0], move_group);
            if (!ret1)
            {
                std::cout << "wrong cartesian" << std::endl;
                return 0;
            }
            ros::Duration(2.0).sleep();
            rg2_width(41.5);
            ros::Duration(2.0).sleep();

            // wait_point2.z += 0.15;
            ret1 = robot_go3(wait_point2, grip_quat_record1[0], move_group);
            if (!ret1)
            {
                std::cout << "wrong cartesian" << std::endl;
                return 0;
            }

            //转+180°
            current_joint_values = move_group.getCurrentJointValues();
            current_joint_values.at(0) -= 2.9;
            move_group.setJointValueTarget(current_joint_values);
            moveit::planning_interface::MoveGroupInterface::Plan switch_plan2;
            success = (move_group.plan(switch_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO("switch plan %s", success ? "" : "Failed");
            if (success)
            {
                move_group.execute(switch_plan2);
            }
            else
            {
                std::cout << "wrong cartesian" << std::endl;
                return 0;
            }

            ret1 = robot_go2(loc_record2[1], quat_record2[1], move_group);
            if (!ret1)
            {
                std::cout << "wrong cartesian" << std::endl;
                return 0;
            }
            ros::Duration(2.0).sleep();
            rg2_width(60);
            ros::Duration(2.0).sleep();

            wait_point1.z -= 0.1;
            ret1 = robot_go3(wait_point1, wait_quat1, move_group);
            if (!ret1)
            {
                std::cout << "wrong cartesian" << std::endl;
                return 0;
            }

            //发送agv离开信号DO0
            int setio_fun = 1;
            int setio_pin = 0;
            double setio_state = 1.0;
            robot_setio(setio_fun, setio_pin, setio_state);
            ROS_INFO("agv leave");
        }

        //第2次对mark板
        {
            load_data();
            while (mir_signal)
            {
                if (digital_in_flag)
                {
                    if (digital_in.at(0).state == 1)
                    {
                        ROS_INFO("AGVs arrive!");
                        break;
                    }
                }
                ros::Duration(0.2).sleep();
            }

            ROS_INFO("grip 1-2");
            if (!robot_go2(loc_record2[1], quat_record2[1], move_group))
            {
                std::cout << "wrong cartesian" << std::endl;
                return 0;
            }
            ros::Duration(2.0).sleep();
            rg2_width(41.5);
            ros::Duration(2.0).sleep();

            if (!robot_go3(wait_point1, wait_quat1, move_group))
            {
                std::cout << "wrong cartesian" << std::endl;
                return 0;
            }

            //转-180°
            move_group.setMaxVelocityScalingFactor(1.0);
            std::vector<double> current_joint_values;
            current_joint_values = move_group.getCurrentJointValues();
            current_joint_values.at(0) += 2.9;
            move_group.setJointValueTarget(current_joint_values);
            moveit::planning_interface::MoveGroupInterface::Plan switch_plan;
            bool success = (move_group.plan(switch_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO("switch plan %s", success ? "" : "Failed");
            if (success)
            {
                move_group.execute(switch_plan);
            }
            else
            {
                std::cout << "wrong cartesian" << std::endl;
                return 0;
            }

            ROS_INFO("computing rotation and translation matrix");
            int current_loc = 0;
            auto temp1 = loc_record1[current_loc];
            if (!robot_go(temp1, quat_record1[current_loc], move_group))
            {
                std::cout << "wrong cartesian" << std::endl;
                return 0;
            }
            ros::Duration(2.0).sleep();
            auto cam_xy = get_xy(cam_z);
            cv::waitKey(1000);
            if (cam_xy.size() > 0)
            {
                double delta_xa = linear_cali_mat[0] * cam_xy[0] + linear_cali_mat[1] * cam_xy[1];
                double delta_ya = linear_cali_mat[2] * cam_xy[0] + linear_cali_mat[3] * cam_xy[1];
                current_pose = move_group.getCurrentPose().pose;
                ROS_INFO("delta_x: %lf", delta_xa);
                ROS_INFO("delta_y: %lf", delta_ya);
                xa = current_pose.position.x + delta_xa;
                ya = current_pose.position.y + delta_ya;
            }
            current_pose = move_group.getCurrentPose().pose;
            double current_hole0_x = current_pose.position.x;
            double current_hole0_y = current_pose.position.y;
            // ROS_INFO("current_hole0_x: %lf", current_hole0_x);
            // ROS_INFO("current_hole0_y: %lf", current_hole0_y);

            current_loc++;
            temp1 = loc_record1[current_loc];
            if (!robot_go(temp1, quat_record1[current_loc], move_group))
            {
                std::cout << "wrong cartesian" << std::endl;
                return 0;
            }
            ros::Duration(2.0).sleep();
            cam_xy = get_xy(cam_z);
            cv::waitKey(1000);
            if (cam_xy.size() > 0)
            {
                double delta_xb = linear_cali_mat[0] * cam_xy[0] + linear_cali_mat[1] * cam_xy[1];
                double delta_yb = linear_cali_mat[2] * cam_xy[0] + linear_cali_mat[3] * cam_xy[1];
                current_pose = move_group.getCurrentPose().pose;
                ROS_INFO("delta_x: %lf", delta_xb);
                ROS_INFO("delta_y: %lf", delta_yb);
                xb = current_pose.position.x + delta_xb;
                yb = current_pose.position.y + delta_yb;
            }
            current_pose = move_group.getCurrentPose().pose;
            double current_hole1_x = current_pose.position.x;
            double current_hole1_y = current_pose.position.y;
            // ROS_INFO("current_hole0_x: %lf", current_hole0_x);
            // ROS_INFO("current_hole0_y: %lf", current_hole0_y);

            double x0 = current_hole0_x - current_hole1_x;
            double y0 = current_hole0_y - current_hole1_y;
            double x1 = xa - xb;
            double y1 = ya - yb;
            ROS_INFO("x1: %f", x1);
            ROS_INFO("y1: %f", y1);
            ROS_INFO("x0: %f", x0);
            ROS_INFO("y0: %f", y0);
            double sin_theta = (x0 * y1 - x1 * y0) / (x1 * x1 + y1 * y1);
            ROS_INFO("sin_theta: %f", sin_theta);
            double cos_theta = std::sqrt(1 - sin_theta * sin_theta);
            rotation_mat[0] = cos_theta;
            rotation_mat[1] = -sin_theta;
            rotation_mat[2] = sin_theta;
            rotation_mat[3] = cos_theta;

            cv::Mat a_mat = cv::Mat(2, 1, CV_64FC1);
            cv::Mat a0_mat = cv::Mat(2, 1, CV_64FC1);
            a_mat.at<double>(0, 0) = xb;
            a_mat.at<double>(1, 0) = yb;
            r_mat.at<double>(0, 0) = cos_theta;
            r_mat.at<double>(0, 1) = -sin_theta;
            r_mat.at<double>(1, 0) = sin_theta;
            r_mat.at<double>(1, 1) = cos_theta;
            std::cout << "rotation_mat: " << std::endl
                      << r_mat.at<double>(0, 0) << " " << r_mat.at<double>(0, 1) << std::endl;
            std::cout << r_mat.at<double>(1, 0) << r_mat.at<double>(1, 1) << std::endl;
            a0_mat.at<double>(0, 0) = current_hole1_x;
            a0_mat.at<double>(1, 0) = current_hole1_y;
            cv::Mat t1_mat = cv::Mat(2, 1, CV_64FC1);
            t1_mat = a_mat - r_mat * a0_mat;

            cv::Mat b_mat = cv::Mat(2, 1, CV_64FC1);
            cv::Mat b0_mat = cv::Mat(2, 1, CV_64FC1);
            b_mat.at<double>(0, 0) = xa;
            b_mat.at<double>(1, 0) = ya;
            b0_mat.at<double>(0, 0) = current_hole0_x;
            b0_mat.at<double>(1, 0) = current_hole0_y;
            cv::Mat t2_mat = cv::Mat(2, 1, CV_64FC1);
            t2_mat = a_mat - r_mat * a0_mat;
            ROS_INFO("t1_x: %lf", t1_mat.at<double>(0, 0));
            ROS_INFO("t1_y: %lf", t1_mat.at<double>(1, 0));
            ROS_INFO("t2_x: %lf", t2_mat.at<double>(0, 0));
            ROS_INFO("t2_x: %lf", t2_mat.at<double>(1, 0));

            translation_mat[0] = (t1_mat.at<double>(0, 0) + t2_mat.at<double>(0, 0)) / 2;
            translation_mat[1] = (t1_mat.at<double>(1, 0) + t2_mat.at<double>(1, 0)) / 2;
            ROS_INFO("translation_x: %f", translation_mat[0]);
            ROS_INFO("translation_y: %f", translation_mat[1]);
            t_mat.at<double>(0, 0) = translation_mat[0];
            t_mat.at<double>(1, 0) = translation_mat[1];

            //loc_record和相机测量比较
            std::vector<double> loc_delta = compute_delta(loc_record1[0]);
            double loc_delta_x1 = loc_delta.at(0);
            double loc_delta_y1 = loc_delta.at(1);
            ROS_INFO("loc delta1 x: %f", loc_delta_x1);
            ROS_INFO("loc delta1 y: %f", loc_delta_y1);
            loc_delta = compute_delta(loc_record1[1]);
            double loc_delta_x2 = loc_delta.at(0);
            double loc_delta_y2 = loc_delta.at(1);
            ROS_INFO("loc delta2 x: %f", loc_delta_x2);
            ROS_INFO("loc delta2 y: %f", loc_delta_y2);

            //计算grip_point1,2
            // auto actual_grip_loc1 = grip_loc_record1[0];
            // actual_grip_loc1.x -= (0.387174-0.308610);
            // actual_grip_loc1.y -= (0.004478+0.002420);
            // std::vector<double> grip_delta = compute_delta(actual_grip_loc1);
            std::vector<double> grip_delta = compute_delta(grip_loc_record2[0]);
            double grip_delta_x1 = grip_delta.at(0);
            double grip_delta_y1 = grip_delta.at(1);
            ROS_INFO("grip delta1 x: %f", grip_delta_x1);
            ROS_INFO("grip delta1 y: %f", grip_delta_y1);

            // auto actual_grip_loc2 = grip_loc_record1[1];
            // actual_grip_loc2.x -= (0.387174-0.308610);
            // actual_grip_loc2.y -= (0.004478+0.002420);
            // grip_delta = compute_delta(actual_grip_loc2);
            grip_delta = compute_delta(grip_loc_record2[1]);
            double grip_delta_x2 = grip_delta.at(0);
            double grip_delta_y2 = grip_delta.at(1);
            ROS_INFO("grip delta2 x: %f", grip_delta_x2);
            ROS_INFO("grip delta2 y: %f", grip_delta_y2);

            double alpha = std::asin(sin_theta);
            ROS_INFO("the angle difference is %f", alpha * 180 / 3.14159);

            // return 0;

            //旋转alpha
            if (std::abs(alpha) > 0)
            {
                std::vector<double> current_joint_values = move_group.getCurrentJointValues();
                current_joint_values.at(5) -= alpha;
                move_group.setJointValueTarget(current_joint_values);
                moveit::planning_interface::MoveGroupInterface::Plan rotation_plan;
                bool success = (move_group.plan(rotation_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                ROS_INFO("rotation plan %s", success ? "" : "Failed");
                if (success)
                {
                    move_group.execute(rotation_plan);
                }
            }

            ROS_INFO("translation..");
            current_pose = move_group.getCurrentPose().pose;
            grip_loc_record1[1].x += grip_delta_x2;
            grip_loc_record1[1].y += grip_delta_y2;
            // grip_loc_record1[0].y += 0.120 * sin_theta;
            grip_quat_record1[0] = current_pose.orientation;
            ret1 = robot_go2(grip_loc_record1[1], grip_quat_record1[0], move_group);
            if (!ret1)
            {
                std::cout << "wrong cartesian" << std::endl;
                return 0;
            }
            ros::Duration(2.0).sleep();
            rg2_width(60);
            ros::Duration(2.0).sleep();

            ret1 = robot_go3(wait_point2, grip_quat_record1[0], move_group);
            if (!ret1)
            {
                std::cout << "wrong cartesian" << std::endl;
                return 0;
            }

            grip_loc_record1[0].x += grip_delta_x1;
            grip_loc_record1[0].y += grip_delta_y1;
            // grip_loc_record1[1].y += 0.120 * sin_theta;
            ret1 = robot_go2(grip_loc_record1[0], grip_quat_record1[0], move_group);
            if (!ret1)
            {
                std::cout << "wrong cartesian" << std::endl;
                return 0;
            }
            ros::Duration(2.0).sleep();
            rg2_width(41.5);
            ros::Duration(2.0).sleep();

            // wait_point2.z += 0.15;
            ret1 = robot_go3(wait_point2, grip_quat_record1[0], move_group);
            if (!ret1)
            {
                std::cout << "wrong cartesian" << std::endl;
                return 0;
            }

            //转+180°
            current_joint_values = move_group.getCurrentJointValues();
            current_joint_values.at(0) -= 2.9;
            move_group.setJointValueTarget(current_joint_values);
            moveit::planning_interface::MoveGroupInterface::Plan switch_plan2;
            success = (move_group.plan(switch_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO("switch plan %s", success ? "" : "Failed");
            if (success)
            {
                move_group.execute(switch_plan2);
            }
            else
            {
                std::cout << "wrong cartesian" << std::endl;
                return 0;
            }

            ret1 = robot_go2(loc_record2[0], quat_record2[0], move_group);
            if (!ret1)
            {
                std::cout << "wrong cartesian" << std::endl;
                return 0;
            }
            ros::Duration(2.0).sleep();
            rg2_width(60);
            ros::Duration(2.0).sleep();

            wait_point1.z -= 0.1;
            ret1 = robot_go3(wait_point1, wait_quat1, move_group);
            if (!ret1)
            {
                std::cout << "wrong cartesian" << std::endl;
                return 0;
            }

            //发送agv离开信号DO0
            int setio_fun = 1;
            int setio_pin = 0;
            double setio_state = 1.0;
            robot_setio(setio_fun, setio_pin, setio_state);
            ROS_INFO("agv leave");

            ROS_INFO("finish");
        }
    }
    return 0;
}