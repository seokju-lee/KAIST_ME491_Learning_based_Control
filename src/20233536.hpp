#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

Eigen::VectorXd value(1 << 12);
Eigen::Vector<int, (1 << 12)> policy;

// Count the number of the box
int box_count(Eigen::Vector<int, 12> state){
  int cnt = 0;
  int k;

  for (int i = 0; i < 4; i++){
    if ((i == 1) | (i == 2)){
      k = 9 - i;
    }
    else{
      k = i + 6;
    }
    if ((state(i) == state(i + 2)) && (state(i + 2) == state(k)) && (state(k) == state(k + 2))){
      cnt++;
    }
  }
  return cnt;
}

// Check the state where user can put the line
std::vector<int> free_space(Eigen::Vector<int, 12> state){
  std::vector<int> free_space;
  for (int i = 0; i < 12; i++){
    if (state(i) == 0){
      free_space.push_back(i);
    }
  }
  return free_space;
}

// Convert the binary to decimal for finding the state in whold states
int find_state_index(Eigen::Vector<int, 12> state){
  int index = 0;
  for (int i = 0; i < state.size(); i++){
    if (state(i) == 1){
      index += pow(2, state.size() - 1 - i);
    }
  }
  return index;
}

// Convert decimal to binary for finding the state
Eigen::Vector<int, 12> find_state(int s){
  Eigen::Vector<int, 12> state;
  for(int i = 11; i >= 0; i--){
    state(11 - i) = (s >> i) & 1;
  }
  return state;
}

// Compute expected value using recursive method
double expected(Eigen::Vector<int, 12> state){
  int reward, cur_box, next_box, cnt;
  double gamma = 1;
  double sum;
  Eigen::Vector<int, 12> next_state;

  // Terminate state
  if(free_space(state).size() == 0){
    return 0;
  }

  sum = 0;
  cnt = 0;
  cur_box = box_count(state);
  // for all action
  for(int a = 0; a < 12; a++){
    // possible action(free space)
    if(state(a) == 0){
      next_state = state;
      next_state(a) = 1;
      next_box = box_count(next_state);
      reward = next_box - cur_box;
      // If reward is positive, then user will have one extra move
      if (reward > 0){
        sum += expected(next_state);
      }
      // If reward is 0, then opponent has turn
      else{
        sum += gamma*value(find_state_index(next_state));
      }
      cnt++;
    }
  }
  // average value
  return sum / cnt;
}

// Policy iteration
void policy_iteration(){
  bool policy_stable = false;
  bool value_converge = false;
  double q_value, max_q;
  int optim_policy;
  double gamma = 1;
  // Initialize value
  value.setZero(1 << 12);
  // Initialize policy
  for(int s = 0; s < policy.size(); s++){
    Eigen::Vector<int, 12> state = find_state(s);
    for(int a = 0; a < 12; a++){
      if(state(a) == 0){
        // Policy is initalized to minimum index for doing action
        policy(s) = a;
        break;
      }
    }
  }
  // Repeat until policy and value are converged
  while(!policy_stable){
    policy_stable = true;
    // Repeate until value are converged
    while(!value_converge){
      value_converge = true;
      // Policy evaluation
      for(int s = 0; s < policy.size(); s++){
        Eigen::Vector<int, 12> state = find_state(s);
        Eigen::Vector<int, 12> next_state = state;
        next_state(policy(s)) = 1;
        int reward = box_count(next_state) - box_count(state);
        if(reward > 0){
          if(value(s) != reward + gamma*value(find_state_index(next_state))){
            value(s) = reward + gamma*value(find_state_index(next_state));
            value_converge = false;
          }
        }
        else{
          double expected_ = expected(next_state);
          if(value(s) != expected_){
            value(s) = expected_;
            value_converge = false;
          }
        }
      }
    }
    value_converge = false;
    // Policy improvement
    for(int s = 0; s < policy.size(); s++){
      max_q = -1;
      Eigen::Vector<int, 12> state = find_state(s);
      // for all action
      for(int a = 0; a < 12; a++){
        if(state(a) == 0){
          Eigen::Vector<int, 12> next_state = state;
          next_state(a) = 1;
          int reward = box_count(next_state) - box_count(state);
          if(reward > 0){
            q_value = reward + gamma*value(find_state_index(next_state));
          }
          else{
            q_value = expected(next_state);
          }
          // Find argmax Q
          if(q_value > max_q){
            max_q = q_value;
            optim_policy = a;
          }
        }
      }
      if(policy(s) != optim_policy){
        policy(s) = optim_policy;
        policy_stable = false;
      }
    }
  }
}

/// DO NOT CHANGE THE NAME AND FORMAT OF THIS FUNCTION
double getOptimalValue(const Eigen::Vector<int, 12> &state)
{
  // return the optimal value given the state
  /// TODO
  policy_iteration();
  return value(find_state_index(state)); // return optimal value
}

/// DO NOT CHANGE THE NAME AND FORMAT OF THIS FUNCTION
int getOptimalAction(const Eigen::Vector<int, 12> &state)
{
  // return one of the optimal actions given the state.
  // the action should be represented as a state index, at which a line will be drawn.
  /// TODO
  policy_iteration();
  return policy(find_state_index(state)); // return optimal action
}
