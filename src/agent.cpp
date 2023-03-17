/*
 AUTHOR: Rama Hruday Bandaru
 FILENAME: agent.cpp
 SPECIFICATION: controls agent in our RL problem i.e drone navigation, will extend to hold sensor data in later submissions
 FOR: CS 5392 Reinforcement Learning Section 01
*/

#include <commands_listener.hpp>

int move_the_drone(int speed, int increment, int limit)
{

    int segments = 0;
    ros::Rate rate(2);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
        if (check_waypoint_reached(.3) == 1)
        {
            std::vector<double> drone_pos = get_current_posing();

            if (drone_pos[1] >= limit)
            {
                break;
            }

            set_speed(speed);

            for (size_t i = -1; i <= 1; i++)
            {
                if (check_waypoint_reached(.3) == 1)
                {
                    set_destination(0, drone_pos[1], 2, i * 30);
                }
            }

            set_destination(0, segments + increment, 3, 0);
            segments++;
        }
    }

    return 0;
}

/*
    NAME: main
    PARAMETERS: argc, **argv  => command line arguments, classic way of ROS initliastion
    PURPOSE: Intiates Drone navigation related commands
    PRECONDITION: Gazebo, Drone simulation and MAVROS must be started before this file is RUN
    POSTCONDITION: the function spins out a ROS node "drone_nav" which is used to pass commands to the Drone
*/
int main(int argc, char **argv)
{
    // initialize ros
    ros::init(argc, argv, "drone_nav");
    ros::NodeHandle n;

    // initialize control publisher/subscribers
    init_publisher_subscriber(n, false);

    // wait for FCU connection
    wait4connect();

    // wait for user to switch to mode GUIDED
    wait4start();

    // the intital position of drone is set as origin.
    initialize_local_frame();

    // request drone to takeoff 3m high in air
    takeoff(3);

    move_the_drone(5, 5, 10);

    wait4Land();
}
