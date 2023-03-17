#include <commands_listener.hpp>
#include <vector>
#include <ctime>
#include <fstream>
#include <iostream>
#include <string>
#include <map>
#include <unistd.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

float avg_image_p_time = 0;
float counter = 1;
std::map<std::string, int[1]> FrameMap;
bool hover;

void detection_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
{
    for (int i = 0; i < msg->bounding_boxes.size(); i++)
    {
        if (msg->bounding_boxes[i].Class == "person")
        {

            ROS_INFO("Person found. calculating average time");
            hover = true;
        }
        // FrameMap.insert(msg->header.frame_id,{msg->header.stamp.toSec(),0});
        ros::Duration t = msg->header.stamp - msg->image_header.stamp;
        double sum = counter * avg_image_p_time + (double)t.toNSec();
        counter++;
        avg_image_p_time = sum / counter;
        ROS_INFO("time differnce is %f", avg_image_p_time);
    };
}

int main(int argc, char **argv)
{
    // initialize ros
    ros::init(argc, argv, "command_node");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/darknet_ros/bounding_boxes", 1, detection_cb);

    // initialize control publisher/subscribers
    init_publisher_subscriber(n);

    // wait for FCU connection
    wait4connect();

    // wait for used to switch to mode GUIDED
    wait4start();

    // create local reference frame
    initialize_local_frame();

    // request takeoff
    takeoff(3);

    ros::Rate rate(2.0);
    int counter = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}
