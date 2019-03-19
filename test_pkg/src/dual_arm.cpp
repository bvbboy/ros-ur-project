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
#include "GenICam/CAPI/SDK.h"
#include "finder/finderpatternfinder.h"
#include "segmentation/seg.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

const double PI = 3.14159265359;
static const std::string ROBOT1_PLANNING_GROUP = "robot1_manipulator";
static const std::string ROBOT2_PLANNING_GROUP = "robot2_manipulator";
static const std::string ARMS_PLANNING_GROUP = "dual_arms";

namespace weitu{
#define INFINITE            0xFFFFFFFF  // Infinite timeout
#define CREATE_SUSPENDED    0x00000004

static int32_t GENICAM_connect(GENICAM_Camera *pGetCamera)
{
	int32_t isConnectSuccess;

	isConnectSuccess = pGetCamera->connect(pGetCamera, accessPermissionControl);

	if( isConnectSuccess != 0)
	{
		printf("connect cameral failed.\n");
		return -1;
	}
	
	return 0;
}

static int32_t GENICAM_CreateStreamSource(GENICAM_Camera *pGetCamera, GENICAM_StreamSource **ppStreamSource)
{
	int32_t isCreateStreamSource;
	GENICAM_StreamSourceInfo stStreamSourceInfo;


	stStreamSourceInfo.channelId = 0;
	stStreamSourceInfo.pCamera = pGetCamera;

	isCreateStreamSource = GENICAM_createStreamSource(&stStreamSourceInfo, ppStreamSource);
	
	if( isCreateStreamSource != 0)
	{
		printf("create stream obj  fail.\r\n");
		return -1;
	}
	
	return 0;
}

static int32_t GENICAM_startGrabbing(GENICAM_StreamSource *pStreamSource)
{
	int32_t isStartGrabbingSuccess;
	GENICAM_EGrabStrategy eGrabStrategy;

	eGrabStrategy = grabStrartegySequential;
	isStartGrabbingSuccess = pStreamSource->startGrabbing(pStreamSource, 0, eGrabStrategy);

	if( isStartGrabbingSuccess != 0)
	{
		printf("StartGrabbing  fail.\n");
		return -1;
	}
	
	return 0;
}

static int32_t GENICAM_stopGrabbing(GENICAM_StreamSource *pStreamSource)
{
	int32_t isStopGrabbingSuccess;

	isStopGrabbingSuccess = pStreamSource->stopGrabbing(pStreamSource);
	if( isStopGrabbingSuccess != 0)
	{
		printf("StopGrabbing  fail.\n");
		return -1;
	}
	
	return 0;
}

static int32_t modifyCamralExposureTime(GENICAM_Camera *pGetCamera)
{
	int32_t isExposureTimeSuccess;
	GENICAM_DoubleNode doubleNode;
	double exposureTimeValue;
	GENICAM_AcquisitionControl *pAcquisitionCtrl = NULL;
	GENICAM_AcquisitionControlInfo acquisitionControlInfo = {0};


	acquisitionControlInfo.pCamera = pGetCamera;

	isExposureTimeSuccess = GENICAM_createAcquisitionControl(&acquisitionControlInfo, &pAcquisitionCtrl);
	if( isExposureTimeSuccess != 0)
	{
		printf("ExposureTime  fail.\n");
		return -1;
	}
	
	exposureTimeValue = 0.0;
	doubleNode = pAcquisitionCtrl->exposureTime(pAcquisitionCtrl);

	isExposureTimeSuccess = doubleNode.getValue(&doubleNode, &exposureTimeValue);
	if( isExposureTimeSuccess != 0)
	{
		printf("get exposureTime fail.\n");
		return -1;
	}
	else
	{
		printf("before change ,exposureTime is %f\n",exposureTimeValue);
	}	
	
	doubleNode.setValue(&doubleNode, (exposureTimeValue + 2));
	if( isExposureTimeSuccess != 0)
	{
		printf("set exposureTime fail.\n");
		return -1;
	}

	doubleNode.getValue(&doubleNode, &exposureTimeValue);
	if( isExposureTimeSuccess != 0)
	{
		printf("get exposureTime fail.\n");
		return -1;
	}
	else
	{
		printf("after change ,exposureTime is %f\n",exposureTimeValue);
	}

	return 0;
}

static int32_t GENICAM_disconnect(GENICAM_Camera *pGetCamera)
{
	int32_t isDisconnectSuccess;

	isDisconnectSuccess = pGetCamera->disConnect(pGetCamera);
	if( isDisconnectSuccess != 0)
	{
		printf("disconnect fail.\n");
		return -1;
	}
	
	return 0;
}

class Camera{
public:
    bool open(uint32_t i=0);
    cv::Mat get();
    void close();
    ~Camera()
    {
        close();
    }
private:
    bool open_flag = false;

    GENICAM_Camera *pCamera = NULL;
    GENICAM_StreamSource *pStreamSource = NULL;
};

bool Camera::open(uint32_t i)
{
    int32_t ret;
    GENICAM_System *pSystem = NULL;
    GENICAM_Camera *pCameraList = NULL;
    uint32_t cameraCnt = 0;

    ret = GENICAM_getSystemInstance(&pSystem);
    if (-1 == ret)
    {
        printf("pSystem is null.\r\n");
        return false;
    }

    ret = pSystem->discovery(pSystem, &pCameraList, &cameraCnt, typeAll);
    if (-1 == ret)
    {
        printf("discovery device fail.\r\n");
        return false;
    }

    if(cameraCnt < i+1)
    {
        printf("no enough Camera is discovered.\r\n");
        return false;
    }

    pCamera = &pCameraList[i];
    // connect to camera
    //连接设备
    ret = GENICAM_connect(pCamera);
    if(ret != 0)
    {
        printf("connect cameral failed.\n");
        return false;
    }

    // create stream source instance
    //创建流对象
    ret = GENICAM_CreateStreamSource(pCamera, &pStreamSource);
    if(ret != 0)
    {
        printf("create stream obj  fail.\r\n");
        return false;
    }

    ret = GENICAM_startGrabbing(pStreamSource);
    if(ret != 0)
    {
        printf("StartGrabbing  fail.\n");
        return false;
    }
    open_flag = true;
    return true;
}

cv::Mat Camera::get()
{
    if(!open_flag){
        return cv::Mat();
    }

    int32_t ret = -1;
    GENICAM_Frame* pFrame;

    if(NULL == pStreamSource){
        return cv::Mat();
    }

    ret = pStreamSource->getFrame(pStreamSource, &pFrame, 1000);
    if (ret < 0){
        printf("getFrame  fail.\n");
        return cv::Mat();
    }

    ret = pFrame->valid(pFrame);
    if (ret < 0){
        printf("frame is invalid!\n");
        pFrame->release(pFrame);
        return cv::Mat();
    }
//    printf("get frame id = [%ld] successfully!\n", pFrame->getBlockId(pFrame));

    uint32_t rows = pFrame->getImageHeight(pFrame);
    uint32_t cols = pFrame->getImageWidth(pFrame);
    const void* img = pFrame->getImage(pFrame);
    char dest[rows*cols];
    std::memcpy(dest, img, sizeof dest);
    cv::Mat result = cv::Mat(rows, cols, CV_8UC1, dest);
    pFrame->release(pFrame);
    return result;
}

void Camera::close()
{
    if(open_flag){
        // stop grabbing from camera
        GENICAM_stopGrabbing(pStreamSource);

        //close camera
        GENICAM_disconnect(pCamera);

        // close stream
        pStreamSource->release(pStreamSource);
        open_flag = false;
    }
}
}

namespace qr_pattern {
std::vector<FinderPattern> find(cv::Mat src){
    cv::Mat graySrc;
    if(src.channels()>1){
       cv::cvtColor(src,graySrc,CV_BGR2GRAY);
    }else{
        graySrc = src;
    }
    graySrc.convertTo(graySrc,CV_8UC1);

    cv::Mat binarySrc;

//    cv::threshold(graySrc, binarySrc, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    cv::threshold(graySrc, binarySrc, 127, 255, CV_THRESH_BINARY);

    FinderPatternFinder finder(binarySrc);
    finder.find();

    std::vector<FinderPattern> pattern = finder.possibleCenters;
    return pattern;
}
}

namespace hole_detect {

template <typename T>
std::vector<size_t> sort_indexes(const std::vector<T> &v) {

  // initialize original index locations
  std::vector<size_t> idx(v.size());
  iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  sort(idx.begin(), idx.end(),
       [&v](size_t i1, size_t i2) {return v[i1].size() > v[i2].size();});

  return idx;
}

cv::Mat isCircle(std::vector<int>& idxs, cv::Mat& rgb,
                 double& radius, cv::Point& center, int rua=1){
    if(idxs.size()<1000 || idxs.size() > rgb.rows*rgb.cols/rua){
        return cv::Mat();
    }

    cv::Mat img = cv::Mat::zeros(rgb.rows, rgb.cols, CV_8UC1);
    int center_row = 0;
    int center_col = 0;
    int count = 0;
    for(auto idx: idxs){
        img.at<uchar>(idx/rgb.cols,idx%rgb.cols) = 255;
        center_row += idx/rgb.cols;
        center_col += idx%rgb.cols;
        count ++;
    }
    center_row /= count;
    center_col /= count;

    auto ker2 = cv::Mat::ones(3,3,CV_8UC1);
    cv::dilate(img, img, ker2);
    cv::dilate(img, img, ker2);
    cv::erode(img, img, ker2);
    cv::erode(img, img, ker2);

    cv::Mat img_copy = img.clone();
    cv::Mat img_copy_;
    cv::dilate(img_copy, img_copy_, ker2);
    cv::Mat edge = img_copy_ - img_copy;

    std::vector<double> dist;
    for(int i=0; i<edge.rows; i++){
        for(int j=0; j<edge.cols; j++){
            if(edge.at<uchar>(i,j)>0){
                dist.push_back(std::sqrt(
                                   (i-center_row)*(i-center_row)+
                                   (j-center_col)*(j-center_col)));
            }
        }
    }
    double mean = double(std::accumulate(dist.begin(), dist.end(), 0))
            /dist.size();
    double var = 0;
    for(auto d: dist){
        var += (d-mean)*(d-mean);
    }
    var = std::sqrt(var/dist.size());

    double ratio = 1-var/mean;

    if(ratio > 0.9){
        radius = mean;
        center = cv::Point(center_col, center_row);
        return img;
    }

    return cv::Mat();
}

cv::Point find(cv::Mat src){
    if(src.channels()<3){
        cv::cvtColor(src, src, CV_GRAY2BGR);
    }

    int wanted_rows = 256;

    cv::Mat circle;
    double radius = 0;
    cv::Point center;
    {
        cv::Mat rgb;
        cv::resize(src, rgb, {src.cols*wanted_rows/src.rows, wanted_rows});

        cv::Mat lab;
        cvtColor(rgb, lab, CV_BGR2Lab);
        seg_helper::min_span_tree::Graph graph(lab);
        Segmentation seg(lab, graph.mst_edges);
        auto lvs = seg.process();

        for(int i=int(lvs.size())-1;i>=0; i--){
            auto& lv = lvs[i];
            auto idx_sort = sort_indexes(lv);
            for(auto idx: idx_sort){
                auto part = lv[idx];

                circle = isCircle(part, rgb, radius, center, 4);
                if(!circle.empty()){
                    break;
                }
            }
            if(!circle.empty()){
                break;
            }
        }

        if(!circle.empty()){
            cv::resize(circle, circle, {src.cols, src.rows},
                       0, 0, cv::INTER_NEAREST);
            radius *= src.rows/wanted_rows;
            center *= src.rows/wanted_rows;
        }
    }

    if(!circle.empty()){
        cv::imshow("circle", circle);
        return center;
    }

    std::cout << "find nothing" << std::endl;
    return cv::Point(0,0);
}
}

class Timer
{
public:
    Timer() : beg_(clock_::now()) {}
    void reset() { beg_ = clock_::now(); }
    double elapsed() const {
        return std::chrono::duration_cast<second_>
            (clock_::now() - beg_).count(); }
    void out(std::string message = ""){
        double t = elapsed();
        std::cout << message << "  elasped time:" << t << "s" << std::endl;
        reset();
    }
private:
    typedef std::chrono::high_resolution_clock clock_;
    typedef std::chrono::duration<double, std::ratio<1> > second_;
    std::chrono::time_point<clock_> beg_;
};

class DualArm
{
public:
    ros::NodeHandle nh_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    //定义URbase坐标位置与世界坐标偏差
    double robot1_offset_x, robot1_offset_y, robot1_offset_z;
    double robot2_offset_x, robot2_offset_y, robot2_offset_z;
    std::string robot1_io_states_topic_, robot2_io_states_topic_;
    ros::Subscriber robot1_io_states_sub_, robot2_io_states_sub_;
    ros::Publisher robot1_urscript_pub_, robot2_urscript_pub_;
    ros::ServiceClient robot1_io_states_client_, robot2_io_states_client_;
    std::string robot1_io_states_srv_name_, robot2_io_states_srv_name_;
    ur_msgs::SetIO robot1_io_states_srv_, robot2_io_states_srv_;
    std::vector<ur_msgs::Digital> robot1_digital_in_, robot2_digital_in_;
    bool robot1_digital_in_flag = false;
    bool robot2_digital_in_flag = false;
    int setio_fun;
    int setio_pin;
    double setio_state;
    geometry_msgs::Pose robot1_current_pose_, robot2_current_pose_;
    std::vector<double> robot1_current_rpy_, robot2_current_rpy_;
    std::vector<double> robot1_current_joint_values, robot2_current_joint_values;
    geometry_msgs::Pose robot1_standby_pose, robot2_standby_pose;
    geometry_msgs::Quaternion robot1_quat_standby, robot2_quat_standby;
    moveit::planning_interface::MoveGroupInterface::Plan arms_standby_plan;
    moveit::planning_interface::MoveGroupInterface::Plan robot1_switch_eef_plan, robot1_switch_agv_plan, robot2_moveto_screw_plan;
    std::string robot1_urscript_topic, robot2_urscript_topic;
    std_msgs::String robot1_move, robot2_move;
    std::vector<geometry_msgs::Pose> waypoints;
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.005;
    double fraction;
    std::vector<geometry_msgs::PointStamped> screw_holes_estimate, screw_controller_estimate, screw_holes_cam, screw_holes_ground, screw_on_controller;
    std::string target_frame;
    geometry_msgs::Pose robot1_press_pose, robot2_screw_pose;
    // 飞控盒、上盖、下盖
    geometry_msgs::Pose flight_controller_pose, upper_cover_pose, lower_cover_pose;
    tf::TransformListener listener;

    DualArm()
    {

        robot1_io_states_topic_ = "robot1/ur_driver/io_states";
        robot2_io_states_topic_ = "robot2/ur_driver/io_states";
        robot1_io_states_srv_name_ = "robot1/ur_driver/set_io";
        robot2_io_states_srv_name_ = "robot2/ur_driver/set_io";
        robot1_urscript_topic = "robot1/ur_driver/URScript";
        robot2_urscript_topic = "robot2/ur_driver/URScript";
        target_frame = "table_ground";

        robot1_offset_x = 0.0;
        robot1_offset_y = 0.0; 
        robot1_offset_z = 0.68;
        robot2_offset_x = 1.50;
        robot2_offset_y = 0.0; 
        robot2_offset_z = 0.68;
        robot1_io_states_sub_ = nh_.subscribe<ur_msgs::IOStates>(robot1_io_states_topic_,10,&DualArm::robot1_io_callback,this);
        robot2_io_states_sub_ = nh_.subscribe<ur_msgs::IOStates>(robot2_io_states_topic_,10,&DualArm::robot2_io_callback,this);
        robot1_io_states_client_ = nh_.serviceClient<ur_msgs::SetIO>(robot1_io_states_srv_name_);
        robot2_io_states_client_ = nh_.serviceClient<ur_msgs::SetIO>(robot2_io_states_srv_name_);
        robot1_urscript_pub_ = nh_.advertise<std_msgs::String>(robot1_urscript_topic,1);
        robot2_urscript_pub_ = nh_.advertise<std_msgs::String>(robot2_urscript_topic,1);

        // robot1_quat_standby = tf::createQuaternionMsgFromRollPitchYaw(PI/2,PI/2,-PI/2);
        // robot1_standby_pose.orientation = robot1_quat_standby;



        //测试用URSCRIPT控制 robot1_move,robot2_move
        // robot1_move.data = "movej(p[0.4, -0.1, 0.5, 1.0, 3.0, 0.0], a=0.3, v=0.2)";
        // robot1_urscript_pub_.publish(robot1_move);

    }

    void robot1_io_callback(const ur_msgs::IOStates::ConstPtr& msg)
    {
        robot1_digital_in_ = msg->digital_in_states;
        if (robot1_digital_in_.size()) {
            robot1_digital_in_flag = true;
        } else {
            ROS_INFO("robot1_digital_in is null!");
            robot1_digital_in_flag = false;
        }
    }
    
    void robot2_io_callback(const ur_msgs::IOStates::ConstPtr& msg)
    {
        robot2_digital_in_ = msg->digital_in_states;
        if (robot2_digital_in_.size()) {
            robot2_digital_in_flag = true;
        } else {
            ROS_INFO("robot2_digital_in is null!");
            robot2_digital_in_flag = false;
        }
    }

    void robot1_setio(int fun, int pin, double state)
    {
        robot1_io_states_srv_.request.fun = fun;
        robot1_io_states_srv_.request.pin = pin;
        robot1_io_states_srv_.request.state = state;
        robot1_io_states_client_.call(robot1_io_states_srv_);
        ros::Duration(2.0).sleep();
        robot1_io_states_srv_.request.state = 0.0;
        robot1_io_states_client_.call(robot1_io_states_srv_);
    }

    void robot2_setio(int fun, int pin, double state)
    {
        robot2_io_states_srv_.request.fun = fun;
        robot2_io_states_srv_.request.pin = pin;
        robot2_io_states_srv_.request.state = state;
        robot2_io_states_client_.call(robot2_io_states_srv_);
        ros::Duration(2.0).sleep();
        robot2_io_states_srv_.request.state = 0.0;
        robot2_io_states_client_.call(robot2_io_states_srv_);
    }

};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "dual_arm");
    moveit::planning_interface::MoveGroupInterface robot1_move_group(ROBOT1_PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface robot2_move_group(ROBOT2_PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface arms_move_group(ARMS_PLANNING_GROUP);
    ROS_INFO("Reference frame of robot1: %s", robot1_move_group.getPlanningFrame().c_str());
    ROS_INFO("End effector link of robot1: %s", robot1_move_group.getEndEffectorLink().c_str());
    ROS_INFO("Reference frame of robot2: %s", robot2_move_group.getPlanningFrame().c_str());
    ROS_INFO("End effector link of robot2: %s", robot2_move_group.getEndEffectorLink().c_str());
    ROS_INFO("Reference frame of arms: %s", arms_move_group.getPlanningFrame().c_str());
    Timer timer;
    weitu::Camera camera;
    camera.open(0);
    timer.out("open camera");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    DualArm *da = new DualArm();


    // 移动robot1和robot2到等待位置 robot1_standby_pose, robot2_standby_pose
    da->waypoints.clear();
    da->robot1_current_pose_ = robot1_move_group.getCurrentPose().pose;
    da->waypoints.push_back(da->robot1_current_pose_);
    da->robot1_standby_pose.position.x = 0.0;
    da->robot1_standby_pose.position.y = 0.0;
    da->robot1_standby_pose.position.z = 0.8;
    da->robot1_standby_pose.orientation.w = 1.0;
    da->waypoints.push_back(da->robot1_standby_pose);
    da->fraction = robot1_move_group.computeCartesianPath(da->waypoints, da->eef_step, da->jump_threshold,da->trajectory);
    moveit::planning_interface::MoveGroupInterface::Plan robot1_standby_plan;
    robot1_standby_plan.trajectory_ = da->trajectory;
    robot1_move_group.execute(robot1_standby_plan);
    ROS_INFO("robot1 is ready");
    da->waypoints.clear();
    da->robot2_current_pose_ = robot2_move_group.getCurrentPose().pose;
    da->waypoints.push_back(da->robot2_current_pose_);
    da->robot2_standby_pose.position.x = 1.5;
    da->robot2_standby_pose.position.y = 0.0;
    da->robot2_standby_pose.position.z = 0.8;
    da->robot2_standby_pose.orientation.w = 1.0;
    da->waypoints.push_back(da->robot2_standby_pose);
    da->fraction = robot2_move_group.computeCartesianPath(da->waypoints, da->eef_step, da->jump_threshold,da->trajectory);
    moveit::planning_interface::MoveGroupInterface::Plan robot2_standby_plan;
    robot2_standby_plan.trajectory_ = da->trajectory;
    robot2_move_group.execute(robot2_standby_plan);
    ROS_INFO("robot2 is ready");

    while(ros::ok())
    {
        //robot1不断查询DI0口电平是否为高
        // while(!robot1_digital_in_flag || robot1_digital_in_.at(0).state == 0) 
        // {
        //     ros::Duration(1.0).sleep();
        // }
        while (1)
        {
            if(da->robot1_digital_in_flag){
                if(da->robot1_digital_in_.at(0).state == 1){
                    ROS_INFO("AGVs arrive!");
                    break;
                }
            }
            ros::Duration(0.2).sleep();
        }
        // 两台AGV定位 计算得到工件位置 flight_controller_pose, upper_cover_pose, lower_cover_pose
        //robot1_press_pose, robot2_screw_pose, screw_holes_estimate, screw_controller_estimate
        while(1) {
            cv::Mat img = camera.get();
            if(!img.empty()){
                cv::pyrDown(img, img);
                std::vector<FinderPattern> pattern = qr_pattern::find(img);
                cv::cvtColor(img, img, CV_GRAY2BGR);

                for(int i=0;i<pattern.size();i++){
                    int centerRow=int(pattern[i].getY());
                    int centerCol=int(pattern[i].getX());
                    cv::line(img, cv::Point(centerCol, centerRow-40), cv::Point(centerCol, centerRow+40)
                        , cv::Scalar(0, 0, 255), 2);
                    cv::line(img, cv::Point(centerCol-40, centerRow), cv::Point(centerCol+40, centerRow)
                        , cv::Scalar(0, 0, 255), 2);
                }

                cv::imshow("img", img);
                cv::waitKey(1000);

            }
            // 根据得到的img图像计算两台agv的位置和偏移程度
            

            // 将每一个工件在agv上的位置转化为世界坐标系下的位置以及各自UR坐标系下的位置


        }
        
        //设置robot1末端工具为gripper
        robot1_move_group.setEndEffectorLink("robot1_gripper_eef_link");
	    ROS_INFO("End effector link of robot1: %s", robot1_move_group.getEndEffectorLink().c_str());
        ros::Duration(1.0).sleep();

        //robot1移动到下盖正上方
        da->waypoints.clear();
        da->robot1_current_pose_ = robot1_move_group.getCurrentPose().pose;
        da->waypoints.push_back(da->robot1_current_pose_);
        geometry_msgs::Pose above_lower_cover_pose;
        above_lower_cover_pose = da->lower_cover_pose;
        above_lower_cover_pose.position.z += 0.2;
        da->waypoints.push_back(above_lower_cover_pose);
        moveit_msgs::RobotTrajectory robot1_moveto_lower_trajetory;
        da->fraction = robot1_move_group.computeCartesianPath(da->waypoints, da->eef_step, da->jump_threshold, robot1_moveto_lower_trajetory);
        if(da->fraction < 0.9) {
            ROS_WARN("Robot1 cartesian path. (%.2f%% achieved) Switch to URScript ",da->fraction*100.0);
            //换成用URScript控制机械臂
            double target_x = above_lower_cover_pose.position.x - da->robot1_offset_x;
            double target_y = above_lower_cover_pose.position.y - da->robot1_offset_y;
            double target_z = above_lower_cover_pose.position.z - da->robot1_offset_z;
            std::string str_target_x = std::to_string(target_x);
            std::string str_target_y = std::to_string(target_y);
            std::string str_target_z = std::to_string(target_z);
            da->robot1_move.data = "movej(p["+str_target_x+","+str_target_y+","+str_target_z+", 1.0, 3.0, 0.0], a=0.3, v=0.2)";
            da->robot1_urscript_pub_.publish(da->robot1_move);
            ros::Duration(2.0).sleep();
            ROS_INFO("Robot1 is above the lower cover!");
        } else {
            moveit::planning_interface::MoveGroupInterface::Plan robot1_above_lower_plan;
            robot1_above_lower_plan.trajectory_ = robot1_moveto_lower_trajetory;
            robot1_move_group.execute(robot1_above_lower_plan);
            ROS_INFO("Robot1 is above the lower cover!");
        }

        //打开手爪,调函数DO3
        da->setio_fun = 1;
        da->setio_pin = 3;
        da->setio_state = 1.0;
        da->robot1_setio(da->setio_fun,da->setio_pin,da->setio_state);
        ROS_INFO("Open the gripper of robot1!");
        //等待手爪打开完成信号DI5
        while (1)
        {
            if(da->robot1_digital_in_flag){
                if(da->robot1_digital_in_.at(5).state == 1){
                    ROS_INFO("Open gripper finished!");
                    break;
                }
            }
            ros::Duration(0.2).sleep();
        }
        //robot1竖直向下移动
        da->waypoints.clear();
        da->robot1_current_pose_ = robot1_move_group.getCurrentPose().pose;
        da->waypoints.push_back(da->robot1_current_pose_);
        da->waypoints.push_back(da->lower_cover_pose);
        moveit_msgs::RobotTrajectory robot1_moveto_lower_trajetory2;
        da->fraction = robot1_move_group.computeCartesianPath(da->waypoints, da->eef_step, da->jump_threshold, robot1_moveto_lower_trajetory2);
        if(da->fraction < 0.9) {
            ROS_WARN("Robot1 cartesian path. (%.2f%% achieved) Switch to URScript",da->fraction*100.0);
            //换成用URScript控制机械臂
            double target_x = da->lower_cover_pose.position.x - da->robot1_offset_x;
            double target_y = da->lower_cover_pose.position.y - da->robot1_offset_y;
            double target_z = da->lower_cover_pose.position.z - da->robot1_offset_z;
            std::string str_target_x = std::to_string(target_x);
            std::string str_target_y = std::to_string(target_y);
            std::string str_target_z = std::to_string(target_z);
            da->robot1_move.data = "movej(p["+str_target_x+","+str_target_y+","+str_target_z+", 1.0, 3.0, 0.0], a=0.3, v=0.2)";
            da->robot1_urscript_pub_.publish(da->robot1_move);
            ros::Duration(2.0).sleep();
            ROS_INFO("Robot1 is ready to grasp!");
        } else {
            moveit::planning_interface::MoveGroupInterface::Plan robot1_moveto_lower_plan;
            robot1_moveto_lower_plan.trajectory_ = robot1_moveto_lower_trajetory2;
            robot1_move_group.execute(robot1_moveto_lower_plan);
            ROS_INFO("Robot1 is ready to grasp!");
        }
        //关闭手爪DO5
        da->setio_fun = 1;
        da->setio_pin = 5;
        da->setio_state = 1.0;
        da->robot1_setio(da->setio_fun,da->setio_pin,da->setio_state);
        ROS_INFO("Close the gripper of robot1!");
        //等待手爪关闭完成信号DI6
        while (1)
        {
            if(da->robot1_digital_in_flag){
                if(da->robot1_digital_in_.at(6).state == 1){
                    ROS_INFO("Close gripper finished!");
                    break;
                }
            }
            ros::Duration(0.2).sleep();
        }
        //移动到上盖正上方
        ROS_INFO("Robot1 move to upper!");
        da->waypoints.clear();
        da->robot1_current_pose_ = robot1_move_group.getCurrentPose().pose;
        da->waypoints.push_back(da->robot1_current_pose_);
        geometry_msgs::Pose waypoint1 = da->robot1_current_pose_;
        waypoint1.position.z += 0.2;
        da->waypoints.push_back(waypoint1);
        waypoint1.position.y -= 0.325;
        da->waypoints.push_back(waypoint1);
        waypoint1.position.z -= 0.2;
        da->waypoints.push_back(waypoint1);
        da->fraction = robot1_move_group.computeCartesianPath(da->waypoints, da->eef_step, da->jump_threshold, da->trajectory);
        if(da->fraction < 0.9) {
            ROS_WARN("Robot1 cartesian path. (%.2f%% achieved) Switch to URScript",da->fraction*100.0);
            //换成用URScript控制机械臂(测试发多句)





            // da->robot1_move.data = "movej(p[0.4, -0.1, 0.5, 1.0, 3.0, 0.0], a=0.3, v=0.2)";
            // da->robot1_urscript_pub_.publish(da->robot1_moveto_upper);
            // ROS_INFO("Lower on upper!");
        } else {
            moveit::planning_interface::MoveGroupInterface::Plan robot1_moveto_upper_plan;
            robot1_moveto_upper_plan.trajectory_ = da->trajectory;
            robot1_move_group.execute(robot1_moveto_upper_plan);
            ROS_INFO("Lower on upper!");
        }
        //打开手爪DO3
        da->setio_fun = 1;
        da->setio_pin = 3;
        da->setio_state = 1.0;
        da->robot1_setio(da->setio_fun,da->setio_pin,da->setio_state);
        ROS_INFO("Open the gripper of robot1!");
        //等待手爪打开完成信号DI5
        while (1)
        {
            if(da->robot1_digital_in_flag){
                if(da->robot1_digital_in_.at(5).state == 1){
                    ROS_INFO("Open gripper finished!");
                    break;
                }
            }
            ros::Duration(0.2).sleep();
        }
        //robot1向上移动
        da->waypoints.clear();
        da->robot1_current_pose_ = robot1_move_group.getCurrentPose().pose;
        da->waypoints.push_back(da->robot1_current_pose_);
        waypoint1 = da->robot1_current_pose_;
        waypoint1.position.z += 0.2;
        da->waypoints.push_back(waypoint1);
        da->fraction = robot1_move_group.computeCartesianPath(da->waypoints, da->eef_step, da->jump_threshold, da->trajectory);
        if(da->fraction < 0.9) {
            ROS_WARN("Robot1 cartesian path. (%.2f%% achieved) Switch to URScript",da->fraction*100.0);
            //换成用URScript控制机械臂
            double target_x = waypoint1.position.x - da->robot1_offset_x;
            double target_y = waypoint1.position.y - da->robot1_offset_y;
            double target_z = waypoint1.position.z - da->robot1_offset_z;
            std::string str_target_x = std::to_string(target_x);
            std::string str_target_y = std::to_string(target_y);
            std::string str_target_z = std::to_string(target_z);
            da->robot1_move.data = "movej(p["+str_target_x+","+str_target_y+","+str_target_z+", 1.0, 3.0, 0.0], a=0.3, v=0.2)";
            da->robot1_urscript_pub_.publish(da->robot1_move);
            ros::Duration(2.0).sleep();
            ROS_INFO("Start to locate screw holes!");
        } else {
            moveit::planning_interface::MoveGroupInterface::Plan robot1_move_upper_plan;
            robot1_move_upper_plan.trajectory_ = da->trajectory;
            robot1_move_group.execute(robot1_move_upper_plan);
            ROS_INFO("Start to locate screw holes!");
        }
        //对螺丝孔定位(从远到近) camera.get()并对得到的图像计算，tf转换transformPoint()得到每个螺丝孔的具体坐标
        //move camera.get calculate transformPoint addtovector
        //切换robot1末端为camera
        robot1_move_group.setEndEffectorLink("robot1_camera_eef_link");
        ros::Duration(1.0).sleep();
	    ROS_INFO("End effector link of robot1: %s", robot1_move_group.getEndEffectorLink().c_str());
        
        for (int i=0; i<da->screw_holes_estimate.size(); i++)
        {
            geometry_msgs::Point screw_holes_estimate_loc = da->screw_holes_estimate.at(i).point;
            da->robot1_current_pose_ = robot1_move_group.getCurrentPose().pose;
            da->waypoints.clear();
            da->waypoints.push_back(da->robot1_current_pose_);
            geometry_msgs::Pose next_pose;
            next_pose.position.x = screw_holes_estimate_loc.x;
            next_pose.position.y = screw_holes_estimate_loc.y;
            next_pose.position.z = screw_holes_estimate_loc.z;
            next_pose.orientation.w = 1.0;
            da->waypoints.push_back(next_pose);
            moveit_msgs::RobotTrajectory nextmove_trajectory;
            da->fraction = robot1_move_group.computeCartesianPath(da->waypoints, da->eef_step, da->jump_threshold, nextmove_trajectory);
            moveit::planning_interface::MoveGroupInterface::Plan scan_screw_plan;
            scan_screw_plan.trajectory_ = nextmove_trajectory;
            robot1_move_group.execute(scan_screw_plan);
            ROS_INFO("Locate screw hole no.%d",i);
            //camera.get()





            //识别screw hole得到相机坐标系下的位置
            geometry_msgs::PointStamped screw_hole_cam_loc;
            da->screw_holes_cam.push_back(screw_hole_cam_loc);

            geometry_msgs::PointStamped screw_hole_ground_loc;
            da->listener.transformPoint(da->target_frame,screw_hole_cam_loc,screw_hole_ground_loc);
            da->screw_holes_ground.push_back(screw_hole_ground_loc);
            
        }


        //robot1按压，首先旋转末端工具，调整位置并竖直向下
        da->robot1_current_joint_values = robot1_move_group.getCurrentJointValues();
        double robot1_wrist_3_joint = da->robot1_current_joint_values.at(5);
        if (robot1_wrist_3_joint < 0 ) {
            da->robot1_current_joint_values.at(5) += PI;
        } else {
            da->robot1_current_joint_values.at(5) -= PI;
        }
        robot1_move_group.setJointValueTarget(da->robot1_current_joint_values);
        bool success = (robot1_move_group.plan(da->robot1_switch_eef_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO("Switch eef of robot1 %s", success ? "":"Failed");
        robot1_move_group.execute(da->robot1_switch_eef_plan);
        //切换robot1末端位置为suction
        robot1_move_group.setEndEffectorLink("robot1_suction_eef_link");
	    ROS_INFO("End effector link of robot1: %s", robot1_move_group.getEndEffectorLink().c_str());

        da->waypoints.clear();
        da->robot1_current_pose_ = robot1_move_group.getCurrentPose().pose;
        da->waypoints.push_back(da->robot1_current_pose_);
        da->waypoints.push_back(da->robot1_press_pose);
        moveit_msgs::RobotTrajectory robot1_press_trajectory;
        moveit::planning_interface::MoveGroupInterface::Plan robot1_press_plan;
        da->fraction = robot1_move_group.computeCartesianPath(da->waypoints, da->eef_step, da->jump_threshold, robot1_press_trajectory);
        if(da->fraction < 0.9) {
            ROS_WARN("Robot1 cartesian path. (%.2f%% achieved)  Switch to URScript",da->fraction*100.0);
            //换成用URScript控制机械臂
            double target_x = da->robot1_press_pose.position.x - da->robot1_offset_x;
            double target_y = da->robot1_press_pose.position.y - da->robot1_offset_y;
            double target_z = da->robot1_press_pose.position.z - da->robot1_offset_z;
            std::string str_target_x = std::to_string(target_x);
            std::string str_target_y = std::to_string(target_y);
            std::string str_target_z = std::to_string(target_z);
            da->robot1_move.data = "movej(p["+str_target_x+","+str_target_y+","+str_target_z+", 1.0, 3.0, 0.0], a=0.3, v=0.2)";
            da->robot1_urscript_pub_.publish(da->robot1_move);
            ros::Duration(2.0).sleep();
            ROS_INFO("Robot1 press on!");
        } else {
            robot1_press_plan.trajectory_ = robot1_press_trajectory;
            robot1_move_group.execute(robot1_press_plan);
            ROS_INFO("Robot1 press on!");
        }

        //robot2开始打螺钉 所有螺钉位置已知，首先移动到最近的螺钉孔上方，输出打螺钉信号
        //movetoscrewholesgroundat(0) setIO waitforiostate 总的用一个for循环 当i>2时robot1离开
        da->waypoints.clear();
        da->robot2_current_pose_ = robot2_move_group.getCurrentPose().pose;
        da->waypoints.push_back(da->robot2_current_pose_);
        da->waypoints.push_back(da->robot2_screw_pose);
        moveit_msgs::RobotTrajectory robot2_trajectory;
        da->fraction = robot2_move_group.computeCartesianPath(da->waypoints, da->eef_step, da->jump_threshold, robot2_trajectory);
        ROS_INFO("robot2 move to screw holes");
        if (da->fraction < 0.9) {
            ROS_WARN("Robot2 cartesian path. (%.2f%% achieved) Switch to URScript",da->fraction*100.0);
            //换成用URScript控制机械臂
            double target_x = da->robot2_screw_pose.position.x - da->robot2_offset_x;
            double target_y = da->robot2_screw_pose.position.y - da->robot2_offset_y;
            double target_z = da->robot2_screw_pose.position.z - da->robot2_offset_z;
            std::string str_target_x = std::to_string(target_x);
            std::string str_target_y = std::to_string(target_y);
            std::string str_target_z = std::to_string(target_z);
            da->robot2_move.data = "movej(p["+str_target_x+","+str_target_y+","+str_target_z+", 1.0, 3.0, 0.0], a=0.3, v=0.2)";
            da->robot2_urscript_pub_.publish(da->robot1_move);
            ros::Duration(2.0).sleep();
            ROS_INFO("Robot2 is ready to screw!");
            
        } else {
            da->robot2_moveto_screw_plan.trajectory_ = robot2_trajectory;
            robot2_move_group.execute(da->robot2_moveto_screw_plan);
            ROS_INFO("Robot2 is ready to screw!");
        }
        ros::Duration(2.0).sleep();
        //中间可能需要先向上再向下
        for (int i = 0; i < da->screw_holes_ground.size(); i++)
        {
            da->waypoints.clear();
            da->robot2_current_pose_ = robot2_move_group.getCurrentPose().pose;
            da->waypoints.push_back(da->robot2_current_pose_);
            double target_x = da->screw_holes_ground.at(i).point.x;
            double target_y = da->screw_holes_ground.at(i).point.y;
            double target_z = da->screw_holes_ground.at(i).point.z;
            da->robot2_screw_pose.position = da->screw_holes_ground.at(i).point;
            da->robot2_screw_pose.orientation.w = 1.0;
            da->waypoints.push_back(da->robot2_screw_pose);
            moveit_msgs::RobotTrajectory robot2_screw_trajectory;
            da->fraction = robot2_move_group.computeCartesianPath(da->waypoints, da->eef_step, da->jump_threshold, robot2_screw_trajectory);
            if (da->fraction < 0.9) {
            ROS_WARN("Robot2 cartesian path. (%.2f%% achieved)  Switch to URScript",da->fraction*100.0);
            //换成用URScript控制机械臂
            target_x -= da->robot2_offset_x;
            target_y -= da->robot2_offset_y;
            target_z -= da->robot2_offset_z;
            std::string str_target_x = std::to_string(target_x);
            std::string str_target_y = std::to_string(target_y);
            std::string str_target_z = std::to_string(target_z);
            da->robot2_move.data = "movej(p["+str_target_x+","+str_target_y+","+str_target_z+", 1.0, 3.0, 0.0], a=0.3, v=0.2)";
            da->robot2_urscript_pub_.publish(da->robot2_move);
            ROS_INFO("Robot2 is begin to screw no.%d",i);
            } else {
                da->robot2_moveto_screw_plan.trajectory_ = robot2_screw_trajectory;
                robot2_move_group.execute(da->robot2_moveto_screw_plan);
                ROS_INFO("Robot2 is begin to screw no.%d",i);
            }

            //打螺钉 1.分料
            da->setio_fun = 1;
            da->setio_pin = 1;
            da->setio_state = 1.0;
            da->robot2_io_states_srv_.request.fun = da->setio_fun;
            da->robot2_io_states_srv_.request.pin = da->setio_pin;
            da->robot2_io_states_srv_.request.state = da->setio_state;
            da->robot2_io_states_client_.call(da->robot2_io_states_srv_);
            ros::Duration(0.8).sleep();
            da->robot2_io_states_srv_.request.state = 0.0;
            da->robot2_io_states_client_.call(da->robot2_io_states_srv_);
            //2.吹气
            da->setio_fun = 1;
            da->setio_pin = 2;
            da->setio_state = 1.0;
            da->robot2_io_states_srv_.request.fun = da->setio_fun;
            da->robot2_io_states_srv_.request.pin = da->setio_pin;
            da->robot2_io_states_srv_.request.state = da->setio_state;
            da->robot2_io_states_client_.call(da->robot2_io_states_srv_);
            ros::Duration(2.0).sleep();
            da->robot2_io_states_srv_.request.state = 0.0;
            da->robot2_io_states_client_.call(da->robot2_io_states_srv_);
            //3.下压
            da->setio_fun = 1;
            da->setio_pin = 3;
            da->setio_state = 1.0;
            da->robot2_io_states_srv_.request.fun = da->setio_fun;
            da->robot2_io_states_srv_.request.pin = da->setio_pin;
            da->robot2_io_states_srv_.request.state = da->setio_state;
            da->robot2_io_states_client_.call(da->robot2_io_states_srv_);
            ros::Duration(2.0).sleep();
            da->robot2_io_states_srv_.request.state = 0.0;
            da->robot2_io_states_client_.call(da->robot2_io_states_srv_);
            ROS_INFO("start screwing no. %d",i);
            //等待吸钉完成信号
            while (1)
            {
                if(da->robot2_digital_in_flag){
                    if(da->robot2_digital_in_.at(1).state == 1){
                        ROS_INFO("screwing finished!");
                        break;
                    }
                }
                ros::Duration(1.0).sleep();
            }
            //打完2个后robot1离开，竖直向上转第一个轴
            if (i == 1){
                da->waypoints.clear();
                da->robot1_current_pose_ = robot1_move_group.getCurrentPose().pose;
                da->waypoints.push_back(da->robot1_current_pose_);
                geometry_msgs::Pose up_pose = da->robot1_current_pose_;
                up_pose.position.z += 0.4;
                da->waypoints.push_back(up_pose);
                moveit_msgs::RobotTrajectory robot1_up_trajectory;
                da->fraction = robot1_move_group.computeCartesianPath(da->waypoints, da->eef_step, da->jump_threshold, robot1_up_trajectory);
                moveit::planning_interface::MoveGroupInterface::Plan robot1_up_plan;
                robot1_up_plan.trajectory_ = robot1_up_trajectory;
                robot1_move_group.execute(robot1_up_plan);
                
                da->robot1_current_joint_values = robot1_move_group.getCurrentJointValues();
                double robot1_shoulder_pan_joint = da->robot1_current_joint_values.at(0);
                if (robot1_shoulder_pan_joint < 0 ) {
                    da->robot1_current_joint_values.at(0) += PI;
                } else {
                    da->robot1_current_joint_values.at(0) -= PI;
                }
                robot1_move_group.setJointValueTarget(da->robot1_current_joint_values);
                success = (robot1_move_group.plan(da->robot1_switch_agv_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                ROS_INFO("robot1 switch to another agv %s", success ? "":"Failed");
                robot1_move_group.execute(da->robot1_switch_agv_plan);
            }
            
        }
        //robot1吸飞控盒，涉及到定位精度的问题？
        da->robot1_current_pose_ = robot1_move_group.getCurrentPose().pose;
        da->waypoints.clear();
        da->waypoints.push_back(da->robot1_current_pose_);
        da->waypoints.push_back(da->flight_controller_pose);
        moveit_msgs::RobotTrajectory robot1_moveto_controller_trajectory;
        da->fraction = robot1_move_group.computeCartesianPath(da->waypoints, da->eef_step, da->jump_threshold, robot1_moveto_controller_trajectory);
        moveit::planning_interface::MoveGroupInterface::Plan robot1_moveto_controller_plan;
        robot1_moveto_controller_plan.trajectory_ = robot1_moveto_controller_trajectory;
        robot1_move_group.execute(robot1_moveto_controller_plan);
        
        //robot2移动到等待位置
        da->waypoints.clear();
        da->robot2_current_pose_ = robot2_move_group.getCurrentPose().pose;
        da->waypoints.push_back(da->robot2_current_pose_);
        da->waypoints.push_back(da->robot2_standby_pose);
        moveit_msgs::RobotTrajectory robot2_movehome_trajectory;
        da->fraction = robot2_move_group.computeCartesianPath(da->waypoints, da->eef_step, da->jump_threshold, robot2_movehome_trajectory);
        if (da->fraction < 0.9) {
            ROS_WARN("Robot2 cartesian path. (%.2f%% achieved) Switch to URScript",da->fraction*100.0);
            //换成用URScript控制机械臂
            double target_x = da->robot2_standby_pose.position.x - da->robot2_offset_x;
            double target_y = da->robot2_standby_pose.position.y - da->robot2_offset_y;
            double target_z = da->robot2_standby_pose.position.z - da->robot2_offset_z;
            std::string str_target_x = std::to_string(target_x);
            std::string str_target_y = std::to_string(target_y);
            std::string str_target_z = std::to_string(target_z);
            da->robot2_move.data = "movej(p["+str_target_x+","+str_target_y+","+str_target_z+", 1.0, 3.0, 0.0], a=0.3, v=0.2)";
            da->robot2_urscript_pub_.publish(da->robot2_move);
            ROS_INFO("Robot2 move home");
            
        } else {
            moveit::planning_interface::MoveGroupInterface::Plan robot2_movehome_plan;
            robot2_movehome_plan.trajectory_ = robot2_movehome_trajectory;
            robot2_move_group.execute(robot2_movehome_plan);
            ROS_INFO("Robot2 move home");
        }

        //robot1打开吸盘DO6
        da->setio_fun = 1;
        da->setio_pin = 6;
        da->setio_state = 1.0;
        da->robot1_setio(da->setio_fun,da->setio_pin,da->setio_state);
        ROS_INFO("open the suction");
        //robot1移动到下盖正上方
        da->robot1_current_joint_values = robot1_move_group.getCurrentJointValues();
        double robot1_shoulder_pan_joint = da->robot1_current_joint_values.at(0);
        if (robot1_shoulder_pan_joint < 0 ) {
            da->robot1_current_joint_values.at(0) += PI;
        } else {
            da->robot1_current_joint_values.at(0) -= PI;
        }
        robot1_move_group.setJointValueTarget(da->robot1_current_joint_values);
        success = (robot1_move_group.plan(da->robot1_switch_agv_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO("robot1 switch to another agv %s", success ? "":"Failed");
        robot1_move_group.execute(da->robot1_switch_agv_plan);
        
        da->robot1_current_pose_ = robot1_move_group.getCurrentPose().pose;
        da->waypoints.clear();
        da->waypoints.push_back(da->robot1_current_pose_);
        geometry_msgs::Pose above_upper_cover_pose = da->upper_cover_pose;
        above_upper_cover_pose.position.z += 0.2;
        da->waypoints.push_back(above_upper_cover_pose);
        da->waypoints.push_back(da->upper_cover_pose);
        moveit_msgs::RobotTrajectory robot1_moveto_upper_trajectory;
        da->fraction = robot1_move_group.computeCartesianPath(da->waypoints, da->eef_step, da->jump_threshold, robot1_moveto_upper_trajectory);
        moveit::planning_interface::MoveGroupInterface::Plan robot1_moveto_upper_plan;
        robot1_moveto_upper_plan.trajectory_ = robot1_moveto_upper_trajectory;
        robot1_move_group.execute(robot1_moveto_upper_plan);
        //robot1关闭吸盘DO7
        da->setio_fun = 1;
        da->setio_pin = 7;
        da->setio_state = 1.0;
        da->robot1_setio(da->setio_fun,da->setio_pin,da->setio_state);
        ROS_INFO("open the suction");
        ros::Duration(2.0).sleep();
        //螺丝孔定位
        //robot1上移
        da->robot1_current_pose_ = robot1_move_group.getCurrentPose().pose;
        da->waypoints.clear();
        da->waypoints.push_back(da->robot1_current_pose_);
        waypoint1 = da->robot1_current_pose_;
        waypoint1.position.z += 0.2;
        da->waypoints.push_back(waypoint1);
        moveit_msgs::RobotTrajectory robot1_moveup_trajectory;
        da->fraction = robot1_move_group.computeCartesianPath(da->waypoints, da->eef_step, da->jump_threshold, robot1_moveup_trajectory);
        moveit::planning_interface::MoveGroupInterface::Plan robot1_moveup_plan;
        robot1_moveup_plan.trajectory_ = robot1_moveup_trajectory;
        robot1_move_group.execute(robot1_moveup_plan);

        //robot1旋转末端为camera
        robot1_move_group.setEndEffectorLink("robot1_camera_eef_link");
        ros::Duration(1.0).sleep();
	    ROS_INFO("End effector link of robot1: %s", robot1_move_group.getEndEffectorLink().c_str());

        for (int i=0; i<da->screw_controller_estimate.size(); i++)
        {
            geometry_msgs::Point screw_controller_estimate_loc = da->screw_controller_estimate.at(i).point;
            da->robot1_current_pose_ = robot1_move_group.getCurrentPose().pose;
            da->waypoints.clear();
            da->waypoints.push_back(da->robot1_current_pose_);
            geometry_msgs::Pose next_pose;
            next_pose.position.x = screw_controller_estimate_loc.x;
            next_pose.position.y = screw_controller_estimate_loc.y;
            next_pose.position.z = screw_controller_estimate_loc.z;
            next_pose.orientation.w = 1.0;
            da->waypoints.push_back(next_pose);
            moveit_msgs::RobotTrajectory nextmove_trajectory;
            da->fraction = robot1_move_group.computeCartesianPath(da->waypoints, da->eef_step, da->jump_threshold, nextmove_trajectory);
            moveit::planning_interface::MoveGroupInterface::Plan scan_screw_plan;
            scan_screw_plan.trajectory_ = nextmove_trajectory;
            robot1_move_group.execute(scan_screw_plan);
            ROS_INFO("Locate screw hole no.%d",i);
            //camera.get()





            //识别screw hole得到相机坐标系下的位置
            geometry_msgs::PointStamped screw_hole_cam_loc;



            geometry_msgs::PointStamped screw_hole_ground_loc;
            da->listener.transformPoint(da->target_frame,screw_hole_cam_loc,screw_hole_ground_loc);
            da->screw_on_controller.push_back(screw_hole_ground_loc);
            
        }



        //robot1回等待位置
        da->waypoints.clear();
        da->robot1_current_pose_ = robot1_move_group.getCurrentPose().pose;
        da->waypoints.push_back(da->robot1_current_pose_);
        da->waypoints.push_back(da->robot1_standby_pose);
        moveit_msgs::RobotTrajectory robot1_movehome_trajectory;
        da->fraction = robot1_move_group.computeCartesianPath(da->waypoints, da->eef_step, da->jump_threshold, robot1_movehome_trajectory);
        if (da->fraction < 0.9) {
            ROS_WARN("Robot1 cartesian path. (%.2f%% achieved)  Switch to URScript",da->fraction*100.0);
            //换成用URScript控制机械臂
            double target_x = da->robot1_standby_pose.position.x - da->robot1_offset_x;
            double target_y = da->robot1_standby_pose.position.y - da->robot1_offset_y;
            double target_z = da->robot1_standby_pose.position.z - da->robot1_offset_z;
            std::string str_target_x = std::to_string(target_x);
            std::string str_target_y = std::to_string(target_y);
            std::string str_target_z = std::to_string(target_z);
            da->robot1_move.data = "movej(p["+str_target_x+","+str_target_y+","+str_target_z+", 1.0, 3.0, 0.0], a=0.3, v=0.2)";
            da->robot1_urscript_pub_.publish(da->robot2_move);
            ROS_INFO("Robot1 move home");
            
        } else {
            moveit::planning_interface::MoveGroupInterface::Plan robot1_movehome_plan;
            robot1_movehome_plan.trajectory_ = robot1_movehome_trajectory;
            robot1_move_group.execute(robot1_movehome_plan);
            ROS_INFO("Robot1 move home");
        }

        //robot2移动到飞控盒上方
        da->waypoints.clear();
        da->robot2_current_pose_ = robot2_move_group.getCurrentPose().pose;
        da->waypoints.push_back(da->robot2_current_pose_);
        da->waypoints.push_back(da->upper_cover_pose);
        moveit_msgs::RobotTrajectory robot2_moveto_controller_trajectory;
        da->fraction = robot2_move_group.computeCartesianPath(da->waypoints, da->eef_step, da->jump_threshold, robot2_moveto_controller_trajectory);
        if (da->fraction < 0.9) {
            ROS_WARN("Robot2 cartesian path. (%.2f%% achieved) Switch to URScript",da->fraction*100.0);
            //换成用URScript控制机械臂
            double target_x = da->upper_cover_pose.position.x - da->robot2_offset_x;
            double target_y = da->upper_cover_pose.position.y - da->robot2_offset_y;
            double target_z = da->upper_cover_pose.position.z - da->robot2_offset_z;
            std::string str_target_x = std::to_string(target_x);
            std::string str_target_y = std::to_string(target_y);
            std::string str_target_z = std::to_string(target_z);
            da->robot2_move.data = "movej(p["+str_target_x+","+str_target_y+","+str_target_z+", 1.0, 3.0, 0.0], a=0.3, v=0.2)";
            da->robot2_urscript_pub_.publish(da->robot2_move);
            ROS_INFO("Robot2 move to controller");

        } else {
            moveit::planning_interface::MoveGroupInterface::Plan robot2_moveto_controller_plan;
            robot2_moveto_controller_plan.trajectory_ = robot2_moveto_controller_trajectory;
            robot2_move_group.execute(robot2_moveto_controller_plan);
            ROS_INFO("Robot2 move to controller");
        }

        //开始打螺钉
        for (int i=0; i<da->screw_on_controller.size(); i++)
        {
            da->waypoints.clear();
            da->robot2_current_pose_ = robot2_move_group.getCurrentPose().pose;
            da->waypoints.push_back(da->robot2_current_pose_);
            geometry_msgs::Point hole_loc = da->screw_on_controller.at(i).point;
            geometry_msgs::Pose hole_pose;
            hole_pose.position = hole_loc;
            hole_pose.orientation.w = 1.0;
            da->waypoints.push_back(hole_pose);
            moveit_msgs::RobotTrajectory robot2_next_trajectory;
            da->fraction = robot2_move_group.computeCartesianPath(da->waypoints, da->eef_step, da->jump_threshold, robot2_next_trajectory);
            moveit::planning_interface::MoveGroupInterface::Plan robot2_next_plan;
            robot2_next_plan.trajectory_ = robot2_next_trajectory;
            robot2_move_group.execute(robot2_next_plan);
            ROS_INFO("Robot2 is ready to screw");
            
            //打螺钉 1.分料
            da->setio_fun = 1;
            da->setio_pin = 1;
            da->setio_state = 1.0;
            da->robot2_io_states_srv_.request.fun = da->setio_fun;
            da->robot2_io_states_srv_.request.pin = da->setio_pin;
            da->robot2_io_states_srv_.request.state = da->setio_state;
            da->robot2_io_states_client_.call(da->robot2_io_states_srv_);
            ros::Duration(0.8).sleep();
            da->robot2_io_states_srv_.request.state = 0.0;
            da->robot2_io_states_client_.call(da->robot2_io_states_srv_);
            //2.吹气
            da->setio_fun = 1;
            da->setio_pin = 2;
            da->setio_state = 1.0;
            da->robot2_io_states_srv_.request.fun = da->setio_fun;
            da->robot2_io_states_srv_.request.pin = da->setio_pin;
            da->robot2_io_states_srv_.request.state = da->setio_state;
            da->robot2_io_states_client_.call(da->robot2_io_states_srv_);
            ros::Duration(2.0).sleep();
            da->robot2_io_states_srv_.request.state = 0.0;
            da->robot2_io_states_client_.call(da->robot2_io_states_srv_);
            //3.下压
            da->setio_fun = 1;
            da->setio_pin = 3;
            da->setio_state = 1.0;
            da->robot2_io_states_srv_.request.fun = da->setio_fun;
            da->robot2_io_states_srv_.request.pin = da->setio_pin;
            da->robot2_io_states_srv_.request.state = da->setio_state;
            da->robot2_io_states_client_.call(da->robot2_io_states_srv_);
            ros::Duration(2.0).sleep();
            da->robot2_io_states_srv_.request.state = 0.0;
            da->robot2_io_states_client_.call(da->robot2_io_states_srv_);
            ROS_INFO("start screwing no. %d",i);
            //等待吸钉完成信号
            while (1)
            {
                if(da->robot2_digital_in_flag){
                    if(da->robot2_digital_in_.at(1).state == 1){
                        ROS_INFO("screwing finished!");
                        break;
                    }
                }
                ros::Duration(0.2).sleep();
            }

        }
        //robot2移动到等待位置
        da->waypoints.clear();
        da->robot2_current_pose_ = robot2_move_group.getCurrentPose().pose;
        da->waypoints.push_back(da->robot2_current_pose_);
        da->waypoints.push_back(da->robot2_standby_pose);
        da->fraction = robot2_move_group.computeCartesianPath(da->waypoints, da->eef_step, da->jump_threshold, robot2_movehome_trajectory);
        if (da->fraction < 0.9) {
            ROS_WARN("Robot2 cartesian path. (%.2f%% achieved) Switch to URScript",da->fraction*100.0);
            //换成用URScript控制机械臂
            double target_x = da->robot2_standby_pose.position.x - da->robot2_offset_x;
            double target_y = da->robot2_standby_pose.position.y - da->robot2_offset_y;
            double target_z = da->robot2_standby_pose.position.z - da->robot2_offset_z;
            std::string str_target_x = std::to_string(target_x);
            std::string str_target_y = std::to_string(target_y);
            std::string str_target_z = std::to_string(target_z);
            da->robot2_move.data = "movej(p["+str_target_x+","+str_target_y+","+str_target_z+", 1.0, 3.0, 0.0], a=0.3, v=0.2)";
            da->robot2_urscript_pub_.publish(da->robot2_move);
            ROS_INFO("Robot2 move home");

        } else {
            moveit::planning_interface::MoveGroupInterface::Plan robot2_home_plan;
            robot2_home_plan.trajectory_ = robot2_movehome_trajectory;
            robot2_move_group.execute(robot2_home_plan);
            ROS_INFO("Robot2 move home");
        }
        //robot1发送agv离开信号DO0
        da->setio_fun = 1;
        da->setio_pin = 0;
        da->setio_state = 1.0;
        da->robot1_setio(da->setio_fun,da->setio_pin,da->setio_state);
        ROS_INFO("inform agv leave");

    }
    // ros::waitForShutdown();
    return 0;
}
