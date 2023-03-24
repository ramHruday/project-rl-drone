/*
 AUTHOR: Rama Hruday Bandaru
 FILENAME: agent.cpp
 SPECIFICATION: controls agent in our RL problem i.e drone navigation, will extend to hold sensor data in later submissions
 FOR: CS 5392 Reinforcement Learning Section 01
*/

#include <commands_listener.hpp>
#include <future>

std::string state_maker(int i, int j)
{
    return std::to_string(i) + std::to_string(j);
}

std::pair<int, int> largest_qval_in_map(
    std::map<int, int> sampleMap)
{

    // Reference variable to help find
    // the entry with the highest value
    std::pair<int, int> entryWithMaxValue = std::make_pair(0, 0);

    // Iterate in the map to find the required entry
    std::map<int, int>::iterator currentEntry;
    for (currentEntry = sampleMap.begin();
         currentEntry != sampleMap.end();
         ++currentEntry)
    {

        // If this entry's value is more
        // than the max value
        // Set this entry as the max
        if (currentEntry->second > entryWithMaxValue.second)
        {

            entryWithMaxValue = std::make_pair(
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
    ros::Subscriber reward_sub;          // subscribes to rewards topic advertised by teh enviroment

public:
    //  Q-learning table
    std::map<std::string, std::map<int, int>> QTable;

    // Action space, next release action space would be recieved from ENV class.
    int action[3] = {0, 1, 2};

    // Params for reward and a store for reward_msg
    int reward_msg;
    int reward;

    // Height for the drone, in this release height is kept constant
    int height;

    // alpha and gamma values for the Q-learning
    float alpha;
    float gamma;

    // the reset parameters for drone
    bool reset_flag = false;
    int reset_counter = 0;

    Drone(ros::NodeHandle nh, int victim_position[3], int inc, int h, int al, float g)
    {

        // drone position subscriber initiated
        drone_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 10,
                                                         &Drone::pos_callback, this);
        reward_sub = nh.subscribe("/reward", 10,
                                  &Drone::callback_number, this);
        height = h;
        alpha = al;
        gamma = g;

        // initiate random QTable and iterate over possible states from victim position using inc
        for (int i = 0; i < victim_position[0]; i += inc)
        {
            for (int j = 0; j < victim_position[1]; j += inc)
            {
                std::map<int, int> random_action_reward_map;
                for (int k : action)
                {

                    int random_v = (rand() % 5) + 1;
                    random_action_reward_map.insert({k, random_v});
                }
                QTable.insert({std::to_string(i) + std::to_string(j), random_action_reward_map});
            }
        }
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
        if (!reset_flag)
        {
            drone_position = odo_msg.pose.pose.position;
        }
        reset_counter++;
        if (reset_counter > 10)
        {
            reset_flag = false;
            reset_counter = 0;
        }
        // ROS_INFO("coordinates pos d: %f %f %f", drone_position.x, drone_position.y, drone_position.z);
    }

    /*
    NAME: callback_number
    PARAMETERS: &msg , Pointer to the reading of reward channel
    PURPOSE: stores current published reward into reward_msgs, this is not reward accepted by the drone
   */
    void callback_number(const std_msgs::Int8 &msg)
    {
        std::cout << msg.data;
        reward_msg = msg.data;
    }

    /*
    NAME: get_reward
    PARAMETERS: na
    PURPOSE: actual reward acceptance by the drone
   */
    int get_reward()
    {
        reward += reward_msg;
        std::cout << "reward is " << reward_msg << std::endl;
        return reward;
    }

    /*
        NAME: get_best_action
        PARAMETERS: state
        PURPOSE: gets the best action for a state i.e max Q value from Qtable map
    */
    std::pair<int, int> get_best_action(std::string state)

    {
        return largest_qval_in_map(QTable[state]);
    }

    /*
        NAME: act
        PARAMETERS: action
        PURPOSE: if action is 0 the drone moves left, 1- front, 2 - right
    */
    int act(int action)
    {
        switch (action)
        {
        case 0:
            std::cout << "Move LEFT" << std::endl;
            move_the_drone(10, drone_position.x - 5, drone_position.y, height);
            break;
        case 1:
            std::cout << "Move FRONT" << std::endl;

            move_the_drone(10, drone_position.x, drone_position.y + 5, height);
            break;
        default:
            std::cout << "Move RIGHT" << std::endl;
            move_the_drone(10, drone_position.x + 5, drone_position.y, height);
            break;
        }
        return 1;
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

                set_destination(x, y, 3, 10);

                break;
            }
        }
    };

    /*
            NAME: reset
            PARAMETERS: na
            PURPOSE: resets the drone in to start state for the next iteration
        */
    void reset()
    {
        reset_flag = true;
        move_the_drone(20, 0, 0, height);
        drone_position.x = 0;
        drone_position.y = 0;
        drone_position.z = 0;
    }

    /*
        NAME: set_Q_value
        PARAMETERS: state - current state of the drone, action - action took by the drone, next_state - next state of the drone,reward
        PURPOSE: sets the Q-value according to Q-learning algorithm
    */
    void set_Q_value(std::string state, std::string next_state, int action, int reward)
    {
        std::map<int, int> action_reward_pair;

        // Q(s,a) = Q(s,a) + x(reward + max(Q(s',a') - Q(s,a)))
        int Q_s_a = QTable[state][action];
        Q_s_a = Q_s_a + alpha * (reward + gamma * largest_qval_in_map(QTable[next_state]).second - Q_s_a);
        action_reward_pair.insert({action, Q_s_a});
        QTable.insert({state, action_reward_pair});
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
    // initiate random library for random number generation (Used in inital Qvalue)
    srand(time(0));

    // increment of drone travel
    int INCREMENT = 5;

    // height of the drone flight
    int height = 3;

    // actions 0 -Left, 1 - Forward, 2- Right
    int action[3] = {0, 1, 2};
    int initial_pos[3] = {0, 0, 0};

    // alpha gamma values for Q_learning
    float alpha = 0.8;
    float gamma = 0.5;

    // Threshold for iterations
    int thresh = 100;

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

    // mission here
    takeoff(height);

    // Victim position
    int victim_pos[3] = {20, 20, 0};

    // counter for iterations
    int iterations = 0;

    // Initiate Drone class
    Drone drone = Drone(n, victim_pos, INCREMENT, height, alpha, gamma);
    while (iterations < thresh)
    {
        drone.reset();
        for (int i = 0; i < victim_pos[0]; i += INCREMENT)
        {
            for (int j = 0; j < victim_pos[1]; j += INCREMENT)
            {
                // the current state of the drone is xy cordinates of the waypoints
                std::string current_state = state_maker(i, j);

                // best action and Qvalue pair for a given state
                std::pair<int, int> s_a_pair = drone.get_best_action(current_state);

                int current_action = s_a_pair.first;
                std::cout << "Current Action  " << current_action << ", Qval " << s_a_pair.second << std::endl;

                // Ansync call to move the drone via Drone.act(), passing current action as parameter
                std::future<int> ft = std::async(std::launch::async, &Drone::act, drone, current_action);
                int v = ft.get(); // code to finish the ansync, no significance for v

                // accept reward into the drone system/class
                int reward = drone.get_reward();

                // Run the Q-value setting algorithm here passing down current state, next state, action and reward
                if (j < victim_pos[1])
                {
                    drone.set_Q_value(current_state, state_maker(i, j + 1), current_action, reward);
                }
            }
        }
        iterations++;
    }

    // Once drone completes the iteration , wait for user command to drone for landing.
    wait4Land();
}
