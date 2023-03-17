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
int processed_images_count = 1;
std::map<std::string, int[1]> FrameMap;
bool found;
bool giveup;
std::string DETECTION_OBJECT = "person";
std::map<std::string, std::array<int, 2>> commandMap;
int image_count;

void detection_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
{
    for (int i = 0; i < msg->bounding_boxes.size(); i++)
    {

        double t = msg->header.stamp.toSec() - msg->image_header.stamp.toSec();
        double sum = processed_images_count * avg_image_p_time + t;
        processed_images_count++;
        avg_image_p_time = sum / processed_images_count;

        if (msg->bounding_boxes[i].Class == "person")
        {

            ROS_INFO("Person found. calculating average time");
            found = true;
        }
    };
    std::cout << std::fixed << avg_image_p_time << processed_images_count << std::endl;
}

void generate_report(std::string report_name)
{
    commandMap = getCommandMap();
    std::ofstream myfile;
    myfile.open(report_name);
    myfile << "<!DOCTYPE html><html><head></head><style>table {  font-family: arial, sans-serif;  border-collapse: collapse;  width: 100%;}td, th {  border: 1px solid #dddddd;  text-align: left;  padding: 8px;}tr:nth-child(even) {  background-color: #dddddd;}</style><body> <table><thead> <tr> <th>Index</th>   <th>Command</th>   <th>Count</th> <th>Battery</th>  </tr></thead>"; // starting html
    int counter = 1;
    std::map<std::string, std::array<int, 2>>::iterator it = commandMap.begin();
    // Iterate over the map using Iterator till end.
    while (it != commandMap.end())
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

void generateBatteryUsageForImages(std::string report_name, float start_bat, float end_bat, std::string mission_type, double image_rate)
{
    std::ofstream myfile;
    myfile.open(report_name);
    myfile << "<!DOCTYPE html><html><head></head><body>"; // starting html
    if (mission_type == "with_images")
    {
        myfile << "<div>Raw image count  " << image_count << "</div>";
        // myfile << "<div>Processed image count " << processed_images_count << "</div>";
        myfile << "<div>Image rate " << image_rate << " image/sec </div>";

        // myfile
        //     << "<div>Size of  " << image_size << "Images"
        //     << "</div>";

        myfile << "<div>Images, the battery usage is </div>";
    }
    else
    {
        myfile << "<div>Raw image count  " << image_count << "</div>";
        myfile << "<div>Without Images, the battery usage is </div> ";
    }

    myfile << (start_bat - end_bat) << "%";

    myfile << "</body></html>";
    myfile.close();
}

int search_mode(int increment, int speed, int limit)
{

    int counter = 0;
    ros::Rate rate(2);
    while (ros::ok() && (!found || !giveup))
    {
        ros::spinOnce();
        rate.sleep();
        if (check_waypoint_reached(.3) == 1)
        {
            std::vector<double> b = get_current_posing();

            if (getTotalDistance() >= limit)
            {
                giveup = true;
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
    ros::init(argc, argv, "command_node");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/darknet_ros/bounding_boxes", 1, detection_cb);

    // initialize control publisher/subscribers
    init_publisher_subscriber(n, true);

    // wait for FCU connection
    wait4connect();

    // wait for used to switch to mode GUIDED
    wait4start();

    // create local reference frame
    initialize_local_frame();

    // get battery state before the mission
    float battery_start = get_battery_state().percentage;

    // mission here
    takeoff(3);
    search_mode(5, 5, 10);

    // when trip ends
    if (found || giveup)
    {
        sub.shutdown();

        auto commandMap = getCommandMap();
        double image_rate;
        image_count = commandMap["/webcam/image_raw"][0];
        float battery_end = get_battery_state().percentage;
        double secs = (double)getImageRate();
        if (secs > 0)
        {
            image_rate = image_count / secs;
        }
        generateBatteryUsageForImages("with_image_report.html", battery_start, battery_end, "with_images", image_rate);
        generate_report("command_report.html");
        wait4Land();
    }
}
