/*
 AUTHOR: Rama Hruday Bandaru
 FILENAME: agent.cpp
 SPECIFICATION: controls agent in our RL problem i.e drone navigation, will extend to hold sensor data in later submissions
 FOR: CS 5392 Reinforcement Learning Section 01
*/

#include <commands_listener.hpp>

pair<int, int> largest_value_in_map(
    map<int, int> sampleMap)
{

    // Reference variable to help find
    // the entry with the highest value
    pair<int, int> entryWithMaxValue = make_pair(0, 0);

    // Iterate in the map to find the required entry
    map<int, int>::iterator currentEntry;
    for (currentEntry = sampleMap.begin();
         currentEntry != sampleMap.end();
         ++currentEntry)
    {

        // If this entry's value is more
        // than the max value
        // Set this entry as the max
        if (currentEntry->second > entryWithMaxValue.second)
        {

            entryWithMaxValue = make_pair(
                currentEntry->first,
                currentEntry->second);
        }
    }

    return entryWithMaxValue;
}

/*
NAME: Drone
PURPOSE: To replicate RL Drone, i.e observe the agent and reward it accordingly
INVARIANTS:
*/

class Drone
{

private:
    ros::Subscriber drone_pos_sub;       // subscriber to drone's position
    geometry_msgs::Point drone_position; // geometry coordinates of the drone

public:
    Drone(ros::NodeHandle nh, int[3] victim_position, int inc)
    {

        // initialize control publisher/subscribers
        init_publisher_subscriber(n, false);

        // wait for FCU connection
        wait4connect();

        // wait for user to switch to mode GUIDED
        wait4start();

        // the intital position of drone is set as origin.
        initialize_local_frame();

        std::map<std::string, std::map<int, int>> QTable;
        int action[3] = {0, 1, 2};
        std::int reward;

        // drone position subscriber initiated
        drone_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 10,
                                                         &Drone::pos_callback, this);
        reward_sub = nh.subscribe<std::int>("/reward", 10,
                                            &Drone::reward_callback, this);

        // initiate random QTable
        // iterate over possible states from victim position using inc
        for (int i = 0; i < victim_position[0]; i += inc)
        {
            for (int j = 0; j < victim_position[1]; j += inc)
            {
                for (int k in &Drone::action)
                {
                    std::map<int, int> random_action_reward_map;
                    int random_v = std::rand() % (3 + 1 - 1) + 1;
                    random_action_reward_map.insert({k, random_v});
                    QTable.insert({std::to_string(i) + std::to_string(j), random_action_reward_map});
                }
            }
        }
    }

    /*
     NAME: pos_callback
     PARAMETERS: &msg , Pointer to the reading of Odometry channel
     PURPOSE: prints the current position of drone {x,y,z}
     PRECONDITION: the parameter should be a odometry reading
     POSTCONDITION: the function updates the drone position in the Drone class and prints it out
    */
    void pos_callback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        nav_msgs::Odometry odo_msg;
        odo_msg = *msg;
        drone_position = odo_msg.pose.pose.position;
        ROS_INFO("co-ordinates pos d: %f %f %f", drone_position.x, drone_position.y, drone_position.z);
    }

    void reward_callback(const std::int &msg)
    {
        reward = *msg;
        std::cout << reward;
    }
    int get_reward()
    {
        return reward;
    }

    void record_the_reward(int i, int j, int action)
    {
        std::map<int, int> random_action_reward_map;
        random_action_reward_map.insert({k, reward});
        QTable.insert({std::to_string(i) + std::to_string(j), random_action_reward_map});
    }

    int get_best_action(std::string state)
    {
        pair<int, int> action_with_max_val = largest_qval_in_map(QTable[state]);
        return entryWithMaxValue.first
    }
    void act(int i)
    {
        switch (i)
        {
        case 0:
            move_the_drone(10, drone_position.x - 5, drone_position.y, drone_position.z) break;
        case 1:
            move_the_drone(10, drone_position.x, drone_position.y + 5, drone_position.z) break;
        case 2:
            move_the_drone(10, drone_position.x + 5, drone_position.y, drone_position.z) break;
        default:
            // code block
        }
    }

    /*
     NAME: move_the_drone
     PARAMETERS: speed (Speed of the drone), x, y, z (Coordinates)
     PURPOSE: moves the drone to x,y,z point with velocity "speed m/s"
     PRECONDITION: drone must be switched ON
     POSTCONDITION: the function moves the drone to x,y,z point in the space with velocity "speed m/s"
    */
    void move_the_drone(int speed, int x, int y, int z)
    {

        ros::Rate rate(2);
        while (ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
            if (check_waypoint_reached(.3) == 1)
            {
                set_speed(speed);

                set_destination(x, y, z, 0);
            }
        }
    }
};

/*
    NAME: main
    PARAMETERS: argc, **argv  => command line arguments, classic way of ROS initliastion
    PURPOSE: Intiates Drone navigation related commands
    PRECONDITION: Gazebo, Drone simulation and MAVROS must be started before this file is RUN
    POSTCONDITION: the function spins out a ROS node "drone_nav" which is used to pass commands to the Drone
*/
int main(int argc, char **argv)
{
    // increment
    int INCREMENT = 5;

    // actions 0 -Left, 1 - Forward, 2- Right
    int action[3] = {0, 1, 2};
    int[3] initial_pos = {0, 0, 0};

    // initialize ros
    ros::init(argc, argv, "drone_nav");
    ros::NodeHandle n;

    Drone drone = Drone(nh, victim_pos, INCREMENT);

    for (int i = 0; i < 20; i += INCREMENT)
    {
        for (int j = 0; j < 20; j += INCREMENT)
        {
            int a = drone->get_best_action(std::to_string(i) + std::to_string(j));
            drone->act(a);
            int reward = drone.get_reward();
            ROS_INFO("reward is  %d", reward);
        }
    }

    wait4Land();
}
