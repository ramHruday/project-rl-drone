/*
 AUTHOR: Rama Hruday Bandaru, Vamsi Krishna Kasineni,
 FILENAME: agent.cpp
 SPECIFICATION: controls agent in our RL problem i.e drone navigation, will extend to hold sensor data in later submissions
 FOR: CS 5392 Reinforcement Learning Section 01
*/

#include <future>
#include <drone.hpp>
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
    float alpha = 0.9;
    float gamma = 0.5;

    // alpha gamma values for backward_learning
    float alphab = 0.7;
    float gammab = 0.55;

    // Threshold for iterations
    int thresh = 2;

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
    Drone drone = Drone(n, victim_pos, INCREMENT, height, alpha, gamma, 0.9, alphab, gammab);

    std::vector<std::string> discrete_state_space = drone.states;
    while (iterations < thresh)
    {
        std::map<std::string, std::map<int, int>> actionHistory;
        for (size_t i = 0; i < discrete_state_space.size() - 1; ++i)
        {
            // the current state of the drone is xy cordinates of the waypoints
            std::string current_state = discrete_state_space[i];

            // best action and Qvalue pair for a given state
            std::pair<int, float> a_qval = drone.get_best_action(current_state);

            int current_action = a_qval.first;
            std::cout << "Current Action  " << current_action << ", Qval " << a_qval.second << std::endl;
            // Ansync call to move the drone via Drone.act(), passing current action as parameter
            std::future<int> ft = std::async(std::launch::async, &Drone::act, drone, current_action);

            int v = ft.get(); // code to finish the ansync, no significance for v

            // accept reward into the drone system/class
            int reward = drone.get_reward();
            std::map<int, int> action_reward_map;
            action_reward_map.insert({current_action, reward});
            actionHistory.insert({current_state, action_reward_map});

            // Run the Q-value setting algorithm here passing down current state, next state, action and reward
            if (i < discrete_state_space.size() - 1)
            {
                std::string next_state = discrete_state_space[i + 1];
                std::pair<int, float> a_qval2 = drone.get_action(current_state);

                drone.set_SARSA_value(current_state, next_state, current_action, reward, a_qval2.first);
            }
        }
        for (size_t i = discrete_state_space.size() - 1; i > 0; --i)
        {
            // the current state of the drone is xy cordinates of the waypoints
            std::string current_state = discrete_state_space[i];

            // Run the Q-value setting algorithm here passing down current state, next state, action and reward
            if (i > 0)
            {
                std::string next_state = discrete_state_space[i - 1];
                std::pair<int, float> a_qval2 = drone.get_action(current_state);
                drone.set_BL_Q_value(current_state, next_state, actionHistory[current_state][0], actionHistory[current_state][1], a_qval2.first);
            }
        }
        iterations++;
        drone.reset();
    }

    //  prints the New Qtable
    printQtable(drone.QTable);
    // Once drone completes the iteration , wait for user command to drone for landing.
    wait4Land();
}
