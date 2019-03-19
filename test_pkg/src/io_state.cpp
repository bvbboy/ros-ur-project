#include <ros/ros.h>
#include <iostream>
#include "ur_msgs/IOStates.h"
#include "ur_msgs/Digital.h"
#include "ur_msgs/SetIO.h"

using namespace std;
std::vector<ur_msgs::Digital> digital_in;
bool digital_flag = false;
uint pin;
bool state;
int fun = 1;
// int pinout = 3;
int pinout = 0;
float stateout_h = 1.0;
float stateout_l = 0.0;

void iostateCallback(const ur_msgs::IOStates::ConstPtr& msg)
{
    digital_in = msg->digital_in_states;
    if (digital_in.size())
    {
        digital_flag = true;
    }
    else {
        cout<<"Digital_in not found"<<endl;
        digital_flag = false;
    }
}
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "io_state");
    ros::NodeHandle nh;
    ros::Subscriber io_state_sub = nh.subscribe<ur_msgs::IOStates>("robot1/ur_driver/io_states",1,iostateCallback);
    // ros::ServiceClient io_state_client = nh.serviceClient<ur_msgs::SetIO>("robot1/ur_driver/set_io");
    ros::ServiceClient io_state_client = nh.serviceClient<ur_msgs::SetIO>("robot2/ur_driver/set_io");
    ur_msgs::SetIO srv;
    ros::Rate rate(0.1);
    int count = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        if (digital_flag)
        {
            for (int i=0;i<digital_in.size();i++)
            {
                pin = digital_in.at(i).pin;
                state = digital_in.at(i).state;
                cout<<"pin: "<<pin<<"  state: "<<state<<endl;
            }
        }
        if (count%2 == 0)
        {
            srv.request.fun = fun;
            srv.request.pin = pinout;
            srv.request.state = stateout_h;
            io_state_client.call(srv);
        } else {
            srv.request.fun = fun;
            srv.request.pin = pinout;
            srv.request.state = stateout_l;
            io_state_client.call(srv);
        }
        count = count+1;
        rate.sleep();
    }
    
    
    return 0;
}
