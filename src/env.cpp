#include <commands_listener.hpp>
#include <vector>
#include <ctime>
#include <fstream>
#include <iostream>
#include <string>
#include <map>
#include <unistd.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <std_msgs/Int64.h>
#include <std_srvs/SetBool.h>
#include <nav_msgs/Odometry.h>

class Environment
{

private:
    int counter;
    int victim_position;
    ros::Publisher reward_publisher;
    ros::Subscriber drone_positon_subscriber;
    ros::ServiceServer reset_service;

public:
    Environment(ros::NodeHandle nh)
    {
        counter = 0;

        reward_publisher = nh.advertise<std_msgs::Int64>("/number_count", 10);
        drone_positon_subscriber = nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 1000,
                                                                    &Environment::pose_cb, this);
    }

    void callback_number(const std_msgs::Int64 &msg)
    {
        counter += msg.data;
        std_msgs::Int64 new_msg;
        new_msg.data = counter;
        reward_publisher.publish(new_msg);
    }

    void pose_cb(const nav_msgs::Odometry::ConstPtr &msg)
    {

        nav_msgs::Odometry current_pos_g;
        current_pos_g = *msg;
        geometry_msgs::Point current_pos_loc;
        current_pos_loc = enu_2_local(current_pose_g);
        ROS_INFO("cordinates pos d: %f %f", current_pos_loc.x, current_pos_loc.y);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "env");
    ros::NodeHandle nh;
    Environment nc = Environment(nh);
    ros::spin();
}