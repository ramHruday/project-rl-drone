#include <commands_listener.hpp>
#include <vector>
#include <ctime>
#include <fstream>
#include <iostream>
#include <string>
#include <map>
#include <unistd.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

std::map<std::string, int[1]> FrameMap;
bool found;
std::string DETECTION_OBJECT = "person";
std::map<int, int> reward_state_action;
int active_state = 0;

void detection_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
{
    for (int i = 0; i < msg->bounding_boxes.size(); i++)
    {

        if (msg->bounding_boxes[i].Class == "person")
        {

            found = true;
        }
    };
}

int search_mode(int increment, int speed, int limit)
{

    int segments = 0;
    ros::Rate rate(2);
    while (ros::ok() && !found)
    {
        ros::spinOnce();
        rate.sleep();
        if (check_waypoint_reached(.3) == 1)
        {
            std::vector<double> b = get_current_posing();

            if (b[1] == limit)
            {
                break;
            }
            set_speed(speed);

            for (size_t i = -1; i <= 1; i++)
            {
                if (check_waypoint_reached(.3) == 1)
                {
                    set_destination(0, b[1], 2, i * 30);
                }
            }

            set_destination(0, segments + increment, 3, 0);
            segments++;
        }
    }

    return 0;
}

int main(int argc, char **argv)
{
    // initialize ros
    ros::init(argc, argv, "gnc_node");
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

    search_mode(5, 5, 10);

    if (found)
    {
        sub.shutdown();
        wait4Land();
    }
}
