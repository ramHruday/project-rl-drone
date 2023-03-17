/*
 AUTHOR: Rama Hruday Bandaru
 FILENAME: env.cpp
 SPECIFICATION: Intiates RL Environment class, Environment class observes the drone position, will be extended to supply rewards by observing the proximity of the plane.
 FOR: CS 5392 Reinforcement Learning Section 01
*/

#include <commands_listener.hpp>
#include <std_msgs/Int64.h>
#include <nav_msgs/Odometry.h>

/*
NAME: Environment
PURPOSE: To replicate RL environment, i.e observe the agent and reward it accordingly
INVARIANTS:
*/

class Environment
{

private:
    ros::Subscriber drone_positon_subscriber; // a ros subsriber which subsribes to topic MAVROS_GLOBALPOSITION_LOCAL
    geometry_msgs::Point drone_position;      // a geometry point of {x,y,z} i.e cordinates of the drone

public:
    Environment(ros::NodeHandle nh)
    {
        // everytime a msg appears on the topic MAVROS_GLOBALPOSITION_LOCAL, pos_callback is called passing the msg. 10 is limit of msg to be qeued
        drone_positon_subscriber = nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 10,
                                                                    &Environment::pos_callback, this);
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
        ROS_INFO("cordinates pos d: %f %f %f", drone_position.x, drone_position.y, drone_position.z);
    }
};

// command line or remapping arguments

/*
    NAME: main
    PARAMETERS: argc, **argv  => command line arguments, classic way of ROS initliastion
    PURPOSE: prints the current position of drone {x,y,z}
    PRECONDITION: drone simulation and MAVROS must be started before this file is RUN
    POSTCONDITION: the function spins out a ROS node "env" which which is equiped with drone position observer
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "env");
    ros::NodeHandle nh;
    Environment nc = Environment(nh);
    ros::spin();
}