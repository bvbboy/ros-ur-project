#include "ros/ros.h"
#include "find_object_2d/ObjectsStamped.h"
#include "std_msgs/Float32MultiArray.h"
#include <iostream>
#include <string>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <math.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl-1.7/pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.7/pcl/PCLPointCloud2.h>
#include <geometry_msgs/PointStamped.h>


using namespace std;
int u = 640;
int v = 360;
pcl::PointCloud<pcl::PointXYZ> cloud;
bool msg_flag = false;
bool obj_flag = false;
geometry_msgs::PointStamped point_cam, point_base;
std::string target_frame = "base_link";

void callBack(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    const std::vector<float> d = msg->data;
    if (d.size())
    {
        for(int i=0;i<d.size();i=i+12)
        {
            int id = (int)d[i];
            cout<<"id: "<<id<<endl;
            float objectWidth = d[i+1];
            float objectHeight = d[i+2];

            cv::Mat H = (cv::Mat_<float>(3,3) <<d[i+3], d[i+4], d[i+5],
                d[i+6], d[i+7], d[i+8],
                d[i+9], d[i+10], d[i+11]);
            // cout<<"H= "<<endl<<" "<<H<<endl<<endl;
            cv::Mat Ht;
            transpose(H,Ht);
            cout<<"Ht= "<<endl<<" "<<Ht<<endl<<endl;
            std::vector<cv::Point2f> obj_corners(4);
            obj_corners[0] = cvPoint(0,0);
            obj_corners[1] = cvPoint(objectWidth,0);
            obj_corners[2] = cvPoint(objectWidth,objectHeight);
            obj_corners[3] = cvPoint(0,objectHeight);
            for (int i=0; i<obj_corners.size();i++)
            {
                cout<<"obj_corners: "<<i<<obj_corners[i]<<endl;
            }
            std::vector<cv::Point2f> scene_corners(4);
            perspectiveTransform(obj_corners,scene_corners,Ht);
            cv::Point2f center;
            for (int i=0; i<scene_corners.size();i++)
            {
                center += scene_corners[i];
                cout<<"scene_corners: "<<i<<scene_corners[i]<<endl;
            }
            center.x = center.x/scene_corners.size();
            center.y = center.y/scene_corners.size();
            cout<<"center: "<<center<<endl;
            double theta1, theta2, theta;
            theta1 = atan((scene_corners[0].y-scene_corners[1].y)/(scene_corners[1].x-scene_corners[0].x))*180/M_PI;
            theta2 = atan((scene_corners[3].y-scene_corners[2].y)/(scene_corners[2].x-scene_corners[3].x))*180/M_PI;
            theta = (theta1+theta2)/2;
            cout<<"theta1= "<<theta1<<endl<<"theta2= "<<theta2<<endl<<"theta= "<<theta<<endl;
            //publish pose to a topic 
            u = (int)center.x;
            v = (int)center.y;
            obj_flag = true;
        }
    } else {
        cout<<"No object detected.\n";
        obj_flag = false;
    }
}

void pointCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    cloud.clear();
    pcl::fromROSMsg(*msg,cloud);
    msg_flag = true;
}
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sub_findobject2d");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<std_msgs::Float32MultiArray>("objects",10,callBack);
    ros::Subscriber points_sub = n.subscribe<sensor_msgs::PointCloud2>("camera/depth_registered/points",1,pointCallback);
    tf::TransformListener listener;
    ros::Rate rate(1);
    while (ros::ok())
    {
        ros::spinOnce();
        if (obj_flag && msg_flag)
        {
            int pos = v*cloud.width+u;
            ROS_INFO_STREAM(cloud.points[pos]);
            point_cam.header.frame_id = "/camera_link2";
            point_cam.header.stamp = ros::Time();
            point_cam.point.x = cloud.points[pos].x;
            point_cam.point.y = cloud.points[pos].y;
            point_cam.point.z = cloud.points[pos].z;
            listener.transformPoint(target_frame,point_cam,point_base);
            ROS_INFO_STREAM(point_base.point);
            msg_flag = false;
        }
        rate.sleep();
    }
    
    return 0;
}
