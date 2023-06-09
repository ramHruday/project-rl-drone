/*
 AUTHOR: Rama Hruday Bandaru, Vamsi Krishna Kasineni,
 FILENAME: agent.cpp
 SPECIFICATION: controls agent in our RL problem i.e drone navigation, will extend to hold sensor data in later submissions
 FOR: CS 5392 Reinforcement Learning Section 01
*/
#include <drone.hpp>
#include <env.hpp>
#include <future>

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

    // Threshold for iterations
    int thresh = 20000;

    // Victim position
    int victim_pos[3] = {20, 20, 0};

    // counter for iterations
    int iterations = 0;

    // Initiate Env class
    Environment Env = Environment(argc, argv, INCREMENT);

    // Initiate Drone class
    Drone drone = Drone(argc, argv, victim_pos, INCREMENT, height, alpha, gamma, 0.9);

    std::vector<std::string> discrete_state_space = drone.states;
    while (iterations < thresh)
    {
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

            // Run the Q-value setting algorithm here passing down current state, next state, action and reward
            if (i < discrete_state_space.size() - 1)
            {
                std::string next_state = discrete_state_space[i + 1];
                std::pair<int, float> a_qval2 = drone.get_action(current_state);

                drone.set_Q_value_SARSA(current_state, next_state, current_action, reward, a_qval2.first);
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
