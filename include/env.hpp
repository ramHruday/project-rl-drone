#include <utilities.hpp>

class Environment
{

private:
    ros::Subscriber drone_pos_sub;       // subscriber to drone's position
    geometry_msgs::Point drone_position; // geometry coordinates of the drone
    ros::Publisher reward_pub;

public:
    int REWARDS[3] = {-1, 0, 10};
    int victim_position[3] = {20, 20, 0}; // x,y,z cordinates of the victim
    Environment(int inc)
    {
        int INCREMENT = 5;
        ros::init(argc, argv, "env");
        ros::NodeHandle nh;

        Environment nc = Environment(nh, INCREMENT);
        ros::spin();
        // drone position subscriber initiated
        drone_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 10,
                                                         &Environment::pos_callback, this);

        // publish reward here
        reward_pub = nh.advertise<std_msgs::Int8>("/reward", 10);
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
        // ROS_INFO("coordinates pos d: %f %f %f", drone_position.x, drone_position.y, drone_position.z);
        std::pair<double, double> polar = get_polar_coordinates();
        std_msgs::Int8 new_msg;
        if (polar.first < 10)
        {
            new_msg.data = 10;
        }
        else if (polar.first > 20)
        {
            new_msg.data = (int)-1 * polar.first;
        }
        else
        {
            new_msg.data = (int)1 / 10 * polar.first;
        }
        ROS_INFO("Polar co-ordinates %f %f", polar.first, polar.second);
        ROS_INFO("Polar co-ordinates %d", new_msg.data);
        reward_pub.publish(new_msg);
    }

    /*
    NAME: get_polar_coordinates
    PARAMETERS: no params
    PURPOSE: to get polar coordinates with respect to person from the drone r,O
    PRECONDITION: drone simulation and person must be standing in the gazebo world
    POSTCONDITION: the function returns polar coordniates, later translated as state of the drone, no direct effect on the movement or navigation
    */
    std::pair<double, double> get_polar_coordinates()
    {
        double r = sqrt(pow(drone_position.x - victim_position[0], 2) + pow(drone_position.y - victim_position[1], 2) + pow(drone_position.z - victim_position[2], 2));
        double theta = asin(drone_position.z / r);
        return std::make_pair(r, theta);
    }
};
