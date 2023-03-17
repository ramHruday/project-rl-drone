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
bool found;
std::string DETECTION_OBJECT = "person";
std::map<std::string, std::array<int, 2>> command_map;

void detection_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
{
    for (int i = 0; i < msg->bounding_boxes.size(); i++)
    {

        double t = msg->header.stamp.toSec() - msg->image_header.stamp.toSec();
        double sum = counter * avg_image_p_time + t;
        counter++;
        avg_image_p_time = sum / counter;

        if (msg->bounding_boxes[i].Class == "person")
        {

            ROS_INFO("Person found. calculating average time");
            found = true;
        }
    };
    std::cout << std::fixed << avg_image_p_time << counter << std::endl;
}

void generate_report(std::string report_name)
{
    command_map = getCommandMap();
    std::ofstream myfile;
    myfile.open(report_name);
    myfile << "<!DOCTYPE html><html><head></head><style>table {  font-family: arial, sans-serif;  border-collapse: collapse;  width: 100%;}td, th {  border: 1px solid #dddddd;  text-align: left;  padding: 8px;}tr:nth-child(even) {  background-color: #dddddd;}</style><body> <table><thead> <tr> <th>Index</th>   <th>Command</th>   <th>Count</th> <th>Battery</th>  </tr></thead>"; // starting html
    int counter = 1;
    std::map<std::string, std::array<int, 2>>::iterator it = command_map.begin();
    // Iterate over the map using Iterator till end.
    while (it != command_map.end())
    {
        // Accessing KEY from element pointed by it.
        std::string command = it->first;
        // Accessing VALUE from element pointed by it.
        int count = it->second[0];
        int battery = it->second[1];

        myfile << "<tbody><tr><td>" << counter
               << "</td><td>" << command
               << "</td><td>" << count
               << "</td><td>" << battery
               << "</ td></ tr></tbody>";

        it++;
        counter++;
    }

    myfile << "</body></html>";
    myfile.close();
}

int search_mode(int increment, int speed, int limit)
{

    int counter = 0;
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

            set_destination(0, counter + increment, 3, 0);
            counter++;
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

        generate_report("report_hover.html");
        wait4Land();
    }
}
