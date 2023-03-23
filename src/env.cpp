/*
 AUTHOR: Rama Hruday Bandaru
 FILENAME: env.cpp
 SPECIFICATION: Intiates RL Environment class, Environment class observes the drone position, will be extended to supply rewards by observing the proximity of the plane.
 FOR: CS 5392 Reinforcement Learning Section 01
*/

#include <commands_listener.hpp>
#include <std_msgs/Int64.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

/*
NAME: Environment
PURPOSE: To replicate RL environment, i.e observe the agent and reward it accordingly
INVARIANTS:
*/

class Environment
{

private:
    ros::Subscriber drone_pos_sub;       // subscriber to drone's position
    geometry_msgs::Point drone_position; // geometry coordinates of the drone

    int[3] REWARDS = {-1, 0, 10};

public:
    Environment(ros::NodeHandle nh, int[3] victim_position, int inc)
    {
        // drone position subscriber initiated
        drone_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 10,
                                                         &Environment::pos_callback, this);

        // publish reward here
        reward_pub = controlnode.advertise<std::int>("/reward", 10);
    }

    /*
     NAME: pos_callback
     PARAMETERS: &msg , Pointer to the reading of Odometry channel
     PURPOSE: prints the current position of drone {x,y,z}
     PRECONDITION: the parameter should be a odometry reading
     POSTCONDITION: the function updates the drone position in the environment class and prints it out
    */
    void pos_callback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        nav_msgs::Odometry odo_msg;
        odo_msg = *msg;
        drone_position = odo_msg.pose.pose.position;
        ROS_INFO("coordinates pos d: %f %f %f", drone_position.x, drone_position.y, drone_position.z);
        double[2] polar = get_polar_coordinates();
        if (polar[0] < 10)
        {
            reward_pub.publish(REWARDS[1])
        }
        else if (polar[0] > 20)
        {
            reward_pub.publish(REWARDS[0])
        }
    }

    /*
    NAME: get_polar_coordinates
    PARAMETERS: no params
    PURPOSE: to get polar coordinates with respect to person from the drone r,O
    PRECONDITION: drone simulation and person must be standing in the gazebo world
    POSTCONDITION: the function returns polar coordniates, later translated as state of the drone, no direct effect on the movement or navigation
    */
    double[2] get_polar_coordinates()
    {
        double r = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
        double theta = asin(z / r);
        return {r, theta};
    }
};

/*
    NAME: main
    PARAMETERS: argc, **argv  => command line arguments, classic way of ROS initliastion
    PURPOSE: prints the current position of drone {x,y,z}
    PRECONDITION: drone simulation and MAVROS must be started before this file is RUN
    POSTCONDITION: the function spins out a ROS node "env" which which is equiped with drone position observer
*/
int main(int argc, char **argv)
{
    int INCREMENT = 5;
    ros::init(argc, argv, "env");
    ros::NodeHandle nh;

    int[3] victim_pos = {20, 20, 0} // x,y,z cordinates of the victim

    Environment nc = Environment(nh, victim_pos, INCREMENT);
    ros::spin();
}