

#ifndef PARKINGLOTPLANNING_SRC_LOCAL_PLANNER_H
#define PARKINGLOTPLANNING_SRC_LOCAL_PLANNER_H

#endif //PARKINGLOTPLANNING_SRC_LOCAL_PLANNER_H

#include <iostream>
#include <vector>
#include <fstream>
#include <unordered_map>
#include <math.h>

#include "state.h"
#include "map.h"
#include "util.h"


#define MAP_WIDTH 40   // for temp use
#define MAP_HEIGHT 28

const double RANDOM_STEP = M_PI / 20;

void local_planner(CarState &start_state, CarState &goal_state,
                   vector<CarState>& plan);

class RRT_Tree {
public:
  int size;
  CarState start_pos;
  CarState goal_pos;
  unordered_map<int, vector<int>> graph;
  unordered_map<int, CarState> node_map;

  RRT_Tree(CarState start, CarState goal);
  void add_node(int id);
  void add_node_from_primitives(int id, CarState& curr_state);
  void sample_node(int id, CarState& rand_state);
  void sample_node_from_primitives(int id, CarState& rand_state, CarState&
  curr_state);
  void extend(CarState& rand_state);
  void nearest_neighbor(CarState& rand_state, CarState& nearest);
  void print_tree();
  double calculate_distance(CarState& from_state, CarState& to_state);
};

