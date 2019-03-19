#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <moveit_msgs/CollisionObject.h>
#include <visualization_msgs/Marker.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <object_recognition_msgs/GetObjectInformation.h>
#include <iostream>
using namespace std;

class ObjectPose
{
public:
    ObjectPose()
    {
        sub = n.subscribe<object_recognition_msgs::RecognizedObjectArray>(topic,1,&ObjectPose::callback,this);
        marker_pub = n.advertise<visualization_msgs::Marker>(topic2,2);
    }

private:
    ros::NodeHandle n;
    std::string topic = "/recognized_object_array";
    std::string topic2 = "visualization_marker";
    std::string target_frame = "base_link";
    tf::TransformListener listener;
    ros::Subscriber sub;
    ros::Publisher marker_pub;
    geometry_msgs::PoseStamped msg_obj_cam,msg_obj_pose;
    visualization_msgs::Marker marker;
    int obj_id;

    // bool getMeshFromDB(object_recognition_msgs::GetObjectInformation &obj_info)
    // {
    //     ros::ServiceClient get_model_mesh_srv;
    //     std::string get_model_mesh_srv_name("get_object_info");
    //     ros::Time start_time = ros::Time::now();
    //     while ( !ros::service::waitForService(get_model_mesh_srv_name, ros::Duration(2.0)) )
    //     {
    //         ROS_INFO("Waiting for %s service to come up", get_model_mesh_srv_name.c_str());
    //         if (!n.ok() || ros::Time::now() - start_time >= ros::Duration(5.0))
    //             return false;
    //     }
    //     get_model_mesh_srv = n.serviceClient<object_recognition_msgs::GetObjectInformation>(get_model_mesh_srv_name,false);
    //     if ( !get_model_mesh_srv.call(obj_info) )
    //     {
    //         ROS_ERROR("Get model mesh service service call failed altogether");
    //         return false;
    //     }
    //   return true;
    // }

    void addMesh()
    {
        marker.header.frame_id = "/base_link";
        marker.header.stamp = ros::Time::now();
        marker.ns = "mesh1";
        marker.id = obj_id;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.mesh_resource = "package://test_pkg/meshes/soda_can1.blend";
        marker.action = visualization_msgs::Marker::ADD;
        geometry_msgs::Pose obj_pose = msg_obj_pose.pose;
        marker.pose.position.x = obj_pose.position.x;
        marker.pose.position.y = obj_pose.position.y;
        marker.pose.position.z = obj_pose.position.z;
        marker.pose.orientation.x = obj_pose.orientation.x;
        marker.pose.orientation.y = obj_pose.orientation.y;
        marker.pose.orientation.z = obj_pose.orientation.z;
        marker.pose.orientation.w = obj_pose.orientation.w;
        marker.scale.x = 0.03;
        marker.scale.y = 0.03;
        marker.scale.z = 0.03;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();
        marker_pub.publish(marker);
    }

    void callback(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg)
    {
        try
        {
            obj_id = 0;
            object_recognition_msgs::RecognizedObjectArray::_objects_type::const_iterator it;
            for (it = msg->objects.begin(); it != msg->objects.end(); ++it)
            {
                msg_obj_cam.header = msg->header;
                msg_obj_cam.header.stamp = ros::Time(0);
                msg_obj_cam.pose = it->pose.pose.pose;
                listener.transformPose(target_frame,msg_obj_cam,msg_obj_pose);
                geometry_msgs::Pose obj_pose = msg_obj_pose.pose;
                // std::stringstream ss;
                // ss << obj_id << "_";
                ROS_INFO("POSITON X: %lf",obj_pose.position.x);
                ROS_INFO("POSITON Y: %lf",obj_pose.position.y);
                ROS_INFO("POSITON Z: %lf",obj_pose.position.z);
                ROS_INFO("QUTATERNION X: %lf",obj_pose.orientation.x);
                ROS_INFO("QUTATERNION Y: %lf",obj_pose.orientation.y);
                ROS_INFO("QUTATERNION Z: %lf",obj_pose.orientation.z);
                ROS_INFO("QUTATERNION W: %lf",obj_pose.orientation.w);
                object_recognition_msgs::GetObjectInformation obj_info;
                // obj_info = it->type;
                // if (getMeshFromDB(obj_info))
                // {
                //     cout<<"obj_name:"<<obj_info.response.information.name.c_str()<<endl;
                // }
                // else
                // {
                //     cout<<"no object"<<endl;
                // }
                ++obj_id;
                cout<<"obj_id:"<<obj_id;
                addMesh();
            }
        
         }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    
    }
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "getPose");
    ObjectPose *op = new ObjectPose();
    ros::Rate rate(2);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

