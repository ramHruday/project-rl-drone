#include <iostream>
#include <vector>
#include <cmath>
#include <random>

using namespace std;

// Define the environment and the action set
vector<vector<double>> environment = {{0.2, 0.2}, {0.8, 0.8}, {0.2, 0.8}, {0.8, 0.2}};
vector<vector<int>> action_set = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};

// Define the SARSA function
void sarsa(double alpha, double gamma, double epsilon, int num_episodes) {
    // Initialize the Q-table
    vector<vector<double>> q_table(4, vector<double>(4, 0.0));
    
    // Initialize the random number generator
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<double> dis(0.0, 1.0);
    
    // Run the SARSA algorithm for the given number of episodes
    for (int i = 0; i < num_episodes; i++) {
        // Choose a random starting state
        int current_state = floor(dis(gen) * 4);
        
        // Choose an action based on the epsilon-greedy policy
        int action;
        if (dis(gen) < epsilon) {
            action = floor(dis(gen) * 4);
        } else {
            int best_action = 0;
            double best_q = q_table[current_state][0];
            for (int j = 1; j < 4; j++) {
                if (q_table[current_state][j] > best_q) {
                    best_action = j;
                    best_q = q_table[current_state][j];
                }
            }
            action = best_action;
        }
        
        // Loop until a terminal state is reached
        while (true) {
            // Execute the chosen action and observe the reward and new state
            vector<double> new_state = {environment[current_state][0] + action_set[action][0], environment[current_state][1] + action_set[action][1]};
            double reward = (new_state[0] == 0.2 && new_state[1] == 0.2) ? -1.0 : ((new_state[0] == 0.8 && new_state[1] == 0.8) ? 1.0 : 0.0);
            
            // Choose the next action based on the epsilon-greedy policy
            int next_action;
            if (dis(gen) < epsilon) {
                next_action = floor(dis(gen) * 4);
            } else {
                int best_action = 0;
                double best_q = q_table[floor(new_state[0] * 2)][floor(new_state[1] * 2)];
                for (int j = 1; j < 4; j++) {
                    if (q_table[floor(new_state[0] * 2)][floor(new_state[1] * 2)][j] > best_q) {
                        best_action = j;
                        best_q = q_table[floor(new_state[0] * 2)][floor(new_state[1] * 2)][j];
                    }
                }
                next_action = best_action;
            }
            
            // Update the Q-value for the current state and action
            q_table[current_state][action] += alpha * (reward + gamma * q_table[f
