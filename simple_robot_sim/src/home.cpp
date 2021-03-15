#include "ros/ros.h"
#include "ros/console.h"
#include <iostream>
#include "std_msgs/Float64MultiArray.h"
#include "cmath"

int main(int argc, char **argv)
{
    ros::init(argc,argv,"home");
    ros::NodeHandle nh;

    ros::Publisher home_advert = nh.advertise<std_msgs::Float64MultiArray>("joint_command", 1000);

    ros::Rate rate(1);

    while (ros::ok())
    {
        std_msgs::Float64MultiArray msg;
        // Set Robot Home Angles (theta1, theta2, theta3, ...)
        std::vector<float> angle = {0, 0, -20*M_PI/180, 40*M_PI/180, -20*M_PI/180, 0};
        
        msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        msg.layout.dim[0].size = angle.size();
        msg.layout.dim[0].stride = 1;
        msg.layout.dim[0].label = "theta";
        msg.data.clear();
        msg.data.insert(msg.data.end(), angle.begin(), angle.end());

        home_advert.publish(msg);

        //ROS_INFO("Spinning...");

        ros::spinOnce();
    }
    

}