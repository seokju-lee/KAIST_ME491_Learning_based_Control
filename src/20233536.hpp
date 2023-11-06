#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

// The function for checking the results of the game
Eigen::Vector2d check_win(Eigen::Matrix3d state){
  double value = 0;
  double v_d = 0;
  Eigen::Vector2d cw;
  // Check rows and columns
  for (int i = 0; i < 3; i++){
    if(((state(0,i)==1)&&(state(1,i)==1)&&(state(2,i)==1))|((state(i,0)==1)&&(state(i,1)==1)&&(state(i,2)==1))){
      value = 1;
    }
    if(((state(0,i)==-1)&&(state(1,i)==-1)&&(state(2,i)==-1))|((state(i,0)==-1)&&(state(i,1)==-1)&&(state(i,2)==-1))){
      v_d = -1;
    }
  }
  // Check diagonals
  for (int i = 0; i < 2; i++){
    if ((state(0,2*i) == 1)&&(state(1,1) == 1)&&(state(2,2-2*i) == 1)){
      value = 1;
    }
    if ((state(0,2*i) == -1)&&(state(1,1) == -1)&&(state(2,2-2*i) == -1)){
      v_d = -1;
    } 
  }

  // Check draw case
  if ((value == 0) && (std::count(&state(0,0), &state(0,0)+9, 0) == 0)){
    value = 0.5;
  }

  // Convert the value when the user loses
  if (v_d == -1){
    value = 0;
  }

  cw << v_d, value;

  return cw; 
}

// The function for finding the free space which is marked 0
std::vector<Eigen::Index> find_free_space(Eigen::Matrix3d state){
  std::vector<Eigen::Index> idxs;
  for (Eigen::Index i = 0; i < state.size(); i++){
    if(state(i) == 0){
      idxs.push_back(i);
    }
  }
  return idxs;
}

// The function for value iteration using recursive method
double valueiter(Eigen::Matrix3d state, double gamma){
  double value, expected, expected_max;
  std::vector<Eigen::Index> free_index;
  Eigen::Matrix3d cur_state, next_state;
  expected_max = 0;

  // Set the index of the free space
  free_index = find_free_space(state);

  // When the game is terminated
  if((check_win(state)(1) != 0) | (check_win(state)(0) == -1)){
    value = check_win(state)(1);
    return value;
  }

  // The case in which the user does action
  for (int i = 0; i < 3; i++){
    for (int j = 0; j < 3; j++){
      // In free space
      if(state(i,j) == 0){
        cur_state = state;
        cur_state(i,j) = 1;
        // Check whether the game is terminated
        if((check_win(cur_state)(1) != 0) | (check_win(cur_state)(0) == -1)){
          value = gamma*check_win(cur_state)(1);
          return value;
        }
        // Set initial value
        expected = 0;
        // The case in which the opponent does action
        for (int k = 0; k < 3; k++){
          for (int l = 0; l < 3; l++){
            // In free space
            if(cur_state(k,l) == 0){
              next_state = cur_state;
              next_state(k,l) = -1;
              // When the opponent does action, the value must be computed
              expected += valueiter(next_state, gamma);
            }
          }
        }
        // Finding maximum value
        if(expected > expected_max){
          expected_max = expected;
        }
      }
    }
  }
  // Discount maximum value
  return gamma*expected_max/(free_index.size()-1);
}

/// DO NOT CHANGE THE NAME AND FORMAT OF THIS FUNCTION
double getOptimalValue(Eigen::Matrix3d state){
  double value;
  // Plug in the state and discount factor
  value = valueiter(state, 0.98);

  return value; // return optimal value
}
