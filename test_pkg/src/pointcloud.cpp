#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl-1.7/pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.7/pcl/PCLPointCloud2.h>
#include <iostream>
#include <geometry_msgs/Point.h>
using namespace std;

pcl::PointCloud<pcl::PointXYZ> msg_;
int u = 640;
int v = 360;
bool msg_flag=false;
void pointCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    msg_.clear();
    // pcl::PointCloud<pcl::PointXYZ> msg_;
    pcl::fromROSMsg(*msg,msg_);
    // int width = msg_.width;
    // int heigth = msg_.height;
    // int position = v*msg.row_step+u*msg.point_step;
    cout<<"point cloud height:"<<msg_.height<<endl;
    cout<<"point cloud width:"<<msg_.width<<endl;
    cout<<"point cloud size:"<<msg_.points.size()<<endl;
    cout<<msg_.isOrganized()<<endl; 
    // pcl::PointXYZ center = msg_.at(u,v);
    // ROS_INFO_STREAM(center);

    msg_flag = true;
    
}
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pointcloud");
    ros::NodeHandle nh;
    ros::Subscriber points_sub = nh.subscribe<sensor_msgs::PointCloud2>("camera/depth_registered/points",1,pointCallback);
    ros::Rate rate(1);
    while (ros::ok())
    {
        ros::spinOnce();
        // int u = 640;
        // int v = 360;
        // int pos = v*msg_.width+u;
        // ROS_INFO_STREAM(msg_.points[pos]);
        if (msg_flag)
        {
            int pos = v*msg_.width+u;
            ROS_INFO_STREAM(msg_.points[pos]);
            msg_flag = false;
        }

        // int position = v*msg_.row_step+u*msg_.point_step;
        // int posX = position + msg_.fields[0].offset;
        // int posY = position + msg_.fields[1].offset;
        // int posZ = position + msg_.fields[2].offset;
        // float X = 0.0;
        // float Y = 0.0;
        // float Z = 0.0;
        // X = msg_.data[posX];
        // Y = msg_.data[posY];
        // Z = msg_.data[posZ];
        // cout<<"x: "<<X<<endl<<"y: "<<Y<<endl<<"z:"<<Z<<endl;
        // pcl::PointXYZ center = msg_.at(u,v);
        rate.sleep();
    }
    
    
    return 0;
}