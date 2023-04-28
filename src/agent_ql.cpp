/*
 AUTHOR: Rama Hruday Bandaru, Vamsi Krishna Kasineni,
 FILENAME: agent_bl.cpp
 SPECIFICATION: controls agent in our RL problem i.e drone navigation, implements Q-Learinng algorithm
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
    srand(time(0));
    // increment
    int INCREMENT = 5;

    // height
    int height = 3;

    // actions 0 -Left, 1 - Forward, 2- Right
    int action[3] = {0, 1, 2};

    // initial coordinates
    int initial_pos[3] = {0, 0, 0};
    //
    float alpha = 0.8;
    float gamma = 0.5;

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

    int victim_pos[3] = {20, 20, 0};
    int iterations = 0;
    Drone drone = Drone(n, victim_pos, INCREMENT, height, alpha, gamma, 0.9, 0, 0);
    while (iterations < 3)
    {
        drone.reset();
        for (int i = 0; i < victim_pos[0]; i += INCREMENT)
        {
            for (int j = 0; j < victim_pos[1]; j += INCREMENT)
            {
                std::string current_state = state_maker(i, j);
                std::pair<int, int> s_a_pair = drone.get_best_action(current_state);

                int current_action = s_a_pair.first;
                std::cout << "Current Action  " << current_action << ", Qval " << s_a_pair.second << std::endl;

                std::future<int> ft = std::async(std::launch::async, &Drone::act, drone, current_action);
                int v = ft.get(); // code to finish the ansync, no significance for v

                int reward = drone.get_reward();
                if (j < victim_pos[1])
                {
                    drone.set_Q_value(current_state, state_maker(i, j + 1), current_action, reward);
                }
            }
        }
        iterations++;
    }

    wait4Land();
}
