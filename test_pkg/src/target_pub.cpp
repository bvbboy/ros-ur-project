#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <stdio.h>

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char *argv[])
{


	ros::init(argc, argv, "target");

	ros::NodeHandle n;


	ros::Publisher chatter_pub = n.advertise<std_msgs::Float64>("target", 10);

	ros::Rate loop_rate(0.2);


	int count = 0;
	while (ros::ok())
	{

		std_msgs::Float64 target_width;
		// target_width = rand()%(90-20);
		sscanf(argv[1],"%lf",&target_width.data);

		ROS_INFO("target_width: %lf", target_width.data);


		chatter_pub.publish(target_width);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;
}